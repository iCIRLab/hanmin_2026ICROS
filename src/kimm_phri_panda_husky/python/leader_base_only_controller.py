#!/usr/bin/env python3
"""Leader 13-actuator base-only controller for the dual cooperative scene."""

import csv
import math
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Pose2D
from mujoco_ros_msgs.msg import JointSet
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension


def clamp(value, limit):
    return max(-limit, min(limit, value))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(w, x, y, z):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def parse_float_list(value, expected, default):
    if isinstance(value, (list, tuple)):
        data = [float(x) for x in value]
    else:
        data = [float(x) for x in str(value).replace(",", " ").split() if x]
    if len(data) != expected:
        return list(default)
    return data


OPENROBOTS_SITE = "/opt/openrobots/lib/python3.8/site-packages"
DEFAULT_PANDA_URDF_L = (
    "/home/ryoo/mmcm_ws/src/kimm_robots_description/"
    "franka_panda_description/robots/panda_arm_hand_l.urdf"
)


class LeaderBaseOnlyController:
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate", 100.0))
        self.duration = float(rospy.get_param("~duration", 0.0))
        self.output = rospy.get_param("~output", "/tmp/mm_verify_logs/leader_base_only_controller.csv")

        self.joint_state_topic = rospy.get_param(
            "~joint_state_topic", "/coop_scene/mujoco_ros/mujoco_ros_interface/joint_states"
        )
        self.sim_time_topic = rospy.get_param(
            "~sim_time_topic", "/coop_scene/mujoco_ros/mujoco_ros_interface/sim_time"
        )
        self.base_ref_topic = rospy.get_param("~base_ref_topic", "/leader/base_ref")
        self.command_topic = rospy.get_param("~command_topic", "/leader/joint_command")

        self.qpos_offset = int(rospy.get_param("~qpos_offset", 0))
        self.qvel_offset = int(rospy.get_param("~qvel_offset", 0))
        self.arm_qpos_offset = int(rospy.get_param("~arm_qpos_offset", 11))
        self.arm_qvel_offset = int(rospy.get_param("~arm_qvel_offset", 10))
        self.object_qpos_offset = int(rospy.get_param("~object_qpos_offset", 40))
        self.object_qvel_offset = int(rospy.get_param("~object_qvel_offset", 38))

        self.k_forward = float(rospy.get_param("~k_forward", 10.0))
        self.k_turn = float(rospy.get_param("~k_turn", 18.0))
        self.k_lateral_turn = float(rospy.get_param("~k_lateral_turn", 4.0))
        self.kd_forward = float(rospy.get_param("~kd_forward", 1.5))
        self.kd_turn = float(rospy.get_param("~kd_turn", 0.8))
        self.force_limit = float(rospy.get_param("~force_limit", 18.0))
        self.yaw_align_threshold = float(rospy.get_param("~yaw_align_threshold", 0.35))
        self.position_deadband = float(rospy.get_param("~position_deadband", 0.03))
        self.max_xy_error = float(rospy.get_param("~max_xy_error", 2.5))

        self.hold_arm = bool(rospy.get_param("~hold_arm", True))
        self.kp_arm = float(rospy.get_param("~kp_arm", 2.0))
        self.kd_arm = float(rospy.get_param("~kd_arm", 0.4))
        self.arm_torque_limit = float(rospy.get_param("~arm_torque_limit", 6.0))
        self.arm_hold_q = None
        self.use_pinocchio_gravity = bool(rospy.get_param("~use_pinocchio_gravity", False))
        self.panda_urdf = rospy.get_param("~panda_urdf", DEFAULT_PANDA_URDF_L)
        self.gravity_scale = float(rospy.get_param("~gravity_scale", 1.0))
        self.ramp_gravity_with_lift = bool(rospy.get_param("~ramp_gravity_with_lift", False))
        self.ee_z_force = float(rospy.get_param("~ee_z_force", 0.0))
        self.current_ee_z_force = 0.0
        self.pin = None
        self.pin_model = None
        self.pin_data = None
        self.pin_frame_id = None
        self.pin_status = "disabled"
        self.init_pinocchio()

        self.lift_enabled = bool(rospy.get_param("~lift_enabled", False))
        self.lift_start_delay = float(rospy.get_param("~lift_start_delay", 0.5))
        self.lift_duration = max(1e-6, float(rospy.get_param("~lift_duration", 1.0)))
        self.lift_joint_delta = parse_float_list(
            rospy.get_param("~lift_joint_delta", "0 -0.05 0 -0.05 0 0.04 0"),
            7,
            [0.0, -0.05, 0.0, -0.05, 0.0, 0.04, 0.0],
        )
        self.use_object_z_lift_control = bool(rospy.get_param("~use_object_z_lift_control", False))
        self.object_lift_height = float(rospy.get_param("~object_lift_height", 0.05))
        self.object_lift_kp = float(rospy.get_param("~object_lift_kp", 180.0))
        self.object_lift_kd = float(rospy.get_param("~object_lift_kd", 8.0))
        self.object_lift_force_min = float(rospy.get_param("~object_lift_force_min", 0.0))
        self.object_lift_force_max = float(rospy.get_param("~object_lift_force_max", 45.0))
        self.use_jacobian_lift_target = bool(rospy.get_param("~use_jacobian_lift_target", True))
        self.jacobian_lift_gain = float(rospy.get_param("~jacobian_lift_gain", 0.8))
        self.jacobian_lift_damping = float(rospy.get_param("~jacobian_lift_damping", 0.05))
        self.jacobian_lift_max_delta = abs(float(rospy.get_param("~jacobian_lift_max_delta", 0.35)))
        self.current_object_z_error = 0.0

        self.finger_command = float(rospy.get_param("~finger_command", -1.0))
        self.finger_command_limit = abs(float(rospy.get_param("~finger_command_limit", 3.0)))

        self.latest_state = None
        self.base_ref = None
        self.sim_time = 0.0
        self.start_wall = rospy.Time.now()
        self.first_sample_wall = None
        self.first_object = None
        self.rows = []
        self.command_count = 0

        rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_cb)
        rospy.Subscriber(self.sim_time_topic, Float32, self.sim_time_cb)
        rospy.Subscriber(self.base_ref_topic, Pose2D, self.base_ref_cb)
        self.command_pub = rospy.Publisher(self.command_topic, JointSet, queue_size=1)
        self.debug_pub = rospy.Publisher("~debug", Float32MultiArray, queue_size=10)
        rospy.loginfo(
            "leader_base_only_controller target=%s command=%s output=%s pin=%s",
            self.base_ref_topic,
            self.command_topic,
            self.output,
            self.pin_status,
        )

    def init_pinocchio(self):
        if not self.use_pinocchio_gravity:
            return
        try:
            if os.path.isdir(OPENROBOTS_SITE) and OPENROBOTS_SITE not in sys.path:
                sys.path.insert(0, OPENROBOTS_SITE)
            import pinocchio as pin

            model = pin.buildModelFromUrdf(self.panda_urdf)
            link7_id = model.getFrameId("panda_link7")
            link7_frame = model.frames[link7_id]
            placement = pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.1654]))
            frame_id = model.addFrame(
                pin.Frame(
                    "mujoco_ee_grasp_frame",
                    link7_frame.parentJoint,
                    link7_id,
                    placement,
                    pin.FrameType.OP_FRAME,
                )
            )
            self.pin = pin
            self.pin_model = model
            self.pin_data = model.createData()
            self.pin_frame_id = frame_id
            self.pin_status = "available:%s" % pin.__version__
        except Exception as exc:
            self.pin_status = "unavailable:%s:%s" % (type(exc).__name__, exc)
            rospy.logwarn("Pinocchio gravity unavailable: %s", self.pin_status)

    def pinocchio_compensation(self, q):
        if self.pin is None:
            return [0.0] * 7, 0.0, 0.0
        q_np = np.array(q, dtype=float)
        gravity_scale = self.gravity_scale
        if self.ramp_gravity_with_lift:
            gravity_scale *= self.lift_scale()
        tau = gravity_scale * self.pin.computeGeneralizedGravity(
            self.pin_model, self.pin_data, q_np
        )
        ee_force = self.current_ee_z_force
        if abs(ee_force) > 1e-9:
            self.pin.forwardKinematics(self.pin_model, self.pin_data, q_np)
            self.pin.updateFramePlacements(self.pin_model, self.pin_data)
            jac = self.pin.computeFrameJacobian(
                self.pin_model,
                self.pin_data,
                q_np,
                self.pin_frame_id,
                self.pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            wrench_linear = np.array([0.0, 0.0, ee_force])
            tau = tau + jac[:3, :].T.dot(wrench_linear)
        tau = np.asarray(tau).reshape(-1)[:7]
        return tau.tolist(), float(np.max(np.abs(tau))), ee_force

    def jacobian_lift_delta(self, q):
        if (
            self.pin is None
            or not self.use_jacobian_lift_target
            or abs(self.current_object_z_error) < 1e-6
        ):
            return [0.0] * 7
        q_np = np.array(q, dtype=float)
        self.pin.forwardKinematics(self.pin_model, self.pin_data, q_np)
        self.pin.updateFramePlacements(self.pin_model, self.pin_data)
        jac = self.pin.computeFrameJacobian(
            self.pin_model,
            self.pin_data,
            q_np,
            self.pin_frame_id,
            self.pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        jz = np.asarray(jac[2, :7]).reshape(7)
        denom = float(jz.dot(jz) + self.jacobian_lift_damping ** 2)
        if denom <= 1e-12:
            return [0.0] * 7
        dz = self.jacobian_lift_gain * self.current_object_z_error
        dq = jz * (dz / denom)
        dq = np.clip(dq, -self.jacobian_lift_max_delta, self.jacobian_lift_max_delta)
        return dq.tolist()

    def joint_state_cb(self, msg):
        self.latest_state = msg

    def sim_time_cb(self, msg):
        self.sim_time = float(msg.data)

    def base_ref_cb(self, msg):
        self.base_ref = msg

    def extract_base_pose(self):
        q = self.latest_state.position
        v = self.latest_state.velocity
        i = self.qpos_offset
        j = self.qvel_offset
        yaw = yaw_from_quaternion(q[i + 3], q[i + 4], q[i + 5], q[i + 6])
        return {
            "x": q[i],
            "y": q[i + 1],
            "z": q[i + 2],
            "yaw": yaw,
            "vx_world": v[j],
            "wz": v[j + 5],
        }

    def extract_object_pose(self):
        q = self.latest_state.position
        v = self.latest_state.velocity
        i = self.object_qpos_offset
        j = self.object_qvel_offset
        yaw = yaw_from_quaternion(q[i + 3], q[i + 4], q[i + 5], q[i + 6])
        vz = v[j + 2] if len(v) > j + 2 else 0.0
        return {"x": q[i], "y": q[i + 1], "z": q[i + 2], "yaw": yaw, "vz": vz}

    def compute_base_command(self, base, target):
        dx = target.x - base["x"]
        dy = target.y - base["y"]
        xy_error = math.hypot(dx, dy)
        if xy_error > self.max_xy_error:
            return self.zero_command(dx, dy, xy_error, wrap_angle(target.theta - base["yaw"]), True)

        c = math.cos(base["yaw"])
        s = math.sin(base["yaw"])
        ex_body = c * dx + s * dy
        ey_body = -s * dx + c * dy
        desired_heading = math.atan2(dy, dx) if xy_error > self.position_deadband else target.theta
        yaw_error = wrap_angle(desired_heading - base["yaw"])
        target_yaw_error = wrap_angle(target.theta - base["yaw"])
        vx_body = c * base["vx_world"] + s * 0.0

        forward_raw = self.k_forward * ex_body - self.kd_forward * vx_body
        if abs(yaw_error) > self.yaw_align_threshold:
            forward_raw = 0.0
        turn_raw = self.k_turn * yaw_error + self.k_lateral_turn * ey_body - self.kd_turn * base["wz"]
        forward = clamp(forward_raw, self.force_limit)
        turn = clamp(turn_raw, self.force_limit)
        left = clamp(forward - turn, self.force_limit)
        right = clamp(forward + turn, self.force_limit)
        saturated = (
            abs(forward_raw) > self.force_limit
            or abs(turn_raw) > self.force_limit
            or abs(forward - turn) > self.force_limit
            or abs(forward + turn) > self.force_limit
        )
        return {
            "left": left,
            "right": right,
            "forward": forward,
            "turn": turn,
            "dx": dx,
            "dy": dy,
            "ex_body": ex_body,
            "ey_body": ey_body,
            "xy_error": xy_error,
            "yaw_error": yaw_error,
            "target_yaw_error": target_yaw_error,
            "saturated": saturated,
            "gated": False,
        }

    def zero_command(self, dx, dy, xy_error, target_yaw_error, gated):
        return {
            "left": 0.0,
            "right": 0.0,
            "forward": 0.0,
            "turn": 0.0,
            "dx": dx,
            "dy": dy,
            "ex_body": 0.0,
            "ey_body": 0.0,
            "xy_error": xy_error,
            "yaw_error": 0.0,
            "target_yaw_error": target_yaw_error,
            "saturated": False,
            "gated": gated,
        }

    def lift_scale(self):
        if not self.lift_enabled or self.first_sample_wall is None:
            return 0.0
        elapsed = (rospy.Time.now() - self.first_sample_wall).to_sec() - self.lift_start_delay
        if elapsed <= 0.0:
            return 0.0
        return min(1.0, elapsed / self.lift_duration)

    def update_object_lift_force(self, obj):
        if not self.use_object_z_lift_control or self.first_object is None:
            self.current_ee_z_force = self.ee_z_force * self.lift_scale()
            return self.first_object["z"] + self.object_lift_height if self.first_object else float("nan"), 0.0
        scale = self.lift_scale()
        target_z = self.first_object["z"] + self.object_lift_height * scale
        z_error = target_z - obj["z"]
        self.current_object_z_error = z_error
        lift_feedforward = self.ee_z_force * scale
        raw_force = lift_feedforward + self.object_lift_kp * z_error - self.object_lift_kd * obj.get("vz", 0.0)
        self.current_ee_z_force = clamp(
            raw_force,
            max(abs(self.object_lift_force_min), abs(self.object_lift_force_max)),
        )
        self.current_ee_z_force = max(self.object_lift_force_min, min(self.object_lift_force_max, self.current_ee_z_force))
        return target_z, z_error

    def arm_hold_command(self):
        if not self.hold_arm:
            return [0.0] * 7, 0.0, 0.0, 0.0, 0.0, False
        q = list(self.latest_state.position[self.arm_qpos_offset : self.arm_qpos_offset + 7])
        dq = list(self.latest_state.velocity[self.arm_qvel_offset : self.arm_qvel_offset + 7])
        if self.arm_hold_q is None:
            self.arm_hold_q = q
        lift_scale = self.lift_scale()
        target = [
            self.arm_hold_q[i] + lift_scale * self.lift_joint_delta[i]
            for i in range(7)
        ]
        jac_lift_delta = self.jacobian_lift_delta(q)
        target = [target[i] + jac_lift_delta[i] for i in range(7)]
        ff_cmd, ff_max, ee_force = self.pinocchio_compensation(q)
        raw_cmd = [
            self.kp_arm * (target[i] - q[i]) - self.kd_arm * dq[i] + ff_cmd[i]
            for i in range(7)
        ]
        cmd = [clamp(value, self.arm_torque_limit) for value in raw_cmd]
        return (
            cmd,
            math.sqrt(sum((target[i] - q[i]) ** 2 for i in range(7))),
            lift_scale,
            ff_max,
            ee_force,
            any(abs(raw_cmd[i]) > self.arm_torque_limit for i in range(7)),
        )

    def publish_command(self, base_cmd):
        command = [0.0] * 13
        command[0] = command[2] = base_cmd["left"]
        command[1] = command[3] = base_cmd["right"]
        arm_cmd, arm_error, lift_scale, ff_max, ee_force, arm_saturated = self.arm_hold_command()
        command[4:11] = arm_cmd
        finger = clamp(self.finger_command, self.finger_command_limit)
        command[11] = finger
        command[12] = finger
        msg = JointSet()
        msg.header.stamp = rospy.Time.now()
        msg.time = self.sim_time
        msg.MODE = 1
        msg.position = [0.0] * 13
        msg.torque = command
        self.command_pub.publish(msg)
        self.command_count += 1
        return (
            max(abs(x) for x in command),
            arm_error,
            max(abs(x) for x in arm_cmd),
            lift_scale,
            finger,
            ff_max,
            ee_force,
            arm_saturated,
        )

    def make_row(self, base, obj, cmd):
        if self.first_sample_wall is None:
            self.first_sample_wall = rospy.Time.now()
        if self.first_object is None:
            self.first_object = obj
        object_target_z, object_z_error = self.update_object_lift_force(obj)
        (
            max_cmd,
            arm_error,
            arm_max,
            lift_scale,
            finger,
            ff_max,
            ee_force,
            arm_saturated,
        ) = self.publish_command(cmd)
        target = self.base_ref
        return {
            "wall_time": (rospy.Time.now() - self.start_wall).to_sec(),
            "sim_time": self.sim_time,
            "target_x": target.x,
            "target_y": target.y,
            "target_yaw": target.theta,
            "base_x": base["x"],
            "base_y": base["y"],
            "base_yaw": base["yaw"],
            "dx": cmd["dx"],
            "dy": cmd["dy"],
            "ex_body": cmd["ex_body"],
            "ey_body": cmd["ey_body"],
            "xy_error": cmd["xy_error"],
            "yaw_error": cmd["yaw_error"],
            "target_yaw_error": cmd["target_yaw_error"],
            "forward_cmd": cmd["forward"],
            "turn_cmd": cmd["turn"],
            "left_cmd": cmd["left"],
            "right_cmd": cmd["right"],
            "max_abs_command": max_cmd,
            "arm_hold_error": arm_error,
            "arm_max_cmd": arm_max,
            "lift_scale": lift_scale,
            "finger_command": finger,
            "pinocchio_status": self.pin_status,
            "gravity_ff_max": ff_max,
            "ee_z_force": ee_force,
            "arm_saturated": int(arm_saturated),
            "saturated": int(cmd["saturated"]),
            "gated": int(cmd["gated"]),
            "command_count": self.command_count,
            "object_x": obj["x"],
            "object_y": obj["y"],
            "object_z": obj["z"],
            "object_yaw": obj["yaw"],
            "object_target_z": object_target_z,
            "object_z_error": object_z_error,
            "object_drift_xy": math.hypot(obj["x"] - self.first_object["x"], obj["y"] - self.first_object["y"]),
            "object_drift_z": obj["z"] - self.first_object["z"],
            "object_drift_yaw": wrap_angle(obj["yaw"] - self.first_object["yaw"]),
        }

    def publish_debug(self, row):
        msg = Float32MultiArray()
        msg.layout.dim.append(
            MultiArrayDimension(
                label="xy_error,yaw_error,target_yaw_error,left,right,max_abs_command,saturated",
                size=7,
                stride=7,
            )
        )
        msg.data = [
            row["xy_error"],
            row["yaw_error"],
            row["target_yaw_error"],
            row["left_cmd"],
            row["right_cmd"],
            row["max_abs_command"],
            float(row["saturated"]),
        ]
        self.debug_pub.publish(msg)

    def step(self):
        if self.latest_state is None or self.base_ref is None:
            rospy.logwarn_throttle(2.0, "waiting for joint_states and leader base ref")
            return False
        if len(self.latest_state.position) < self.object_qpos_offset + 7:
            rospy.logerr_throttle(2.0, "unexpected joint_states position length=%d", len(self.latest_state.position))
            return False
        if len(self.latest_state.velocity) < self.arm_qvel_offset + 7:
            rospy.logerr_throttle(2.0, "unexpected joint_states velocity length=%d", len(self.latest_state.velocity))
            return False
        base = self.extract_base_pose()
        obj = self.extract_object_pose()
        cmd = self.compute_base_command(base, self.base_ref)
        row = self.make_row(base, obj, cmd)
        self.rows.append(row)
        self.publish_debug(row)
        rospy.loginfo_throttle(
            1.0,
            "leader_base_only xy=%.3f yaw=%.3f cmd=(%.2f, %.2f) obj=%.4f",
            row["xy_error"],
            row["yaw_error"],
            row["left_cmd"],
            row["right_cmd"],
            row["object_drift_xy"],
        )
        return True

    def write_csv(self):
        if not self.rows:
            return
        directory = os.path.dirname(self.output)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(self.output, "w", newline="") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)

    def print_summary(self):
        rospy.loginfo("leader_base_only_controller summary")
        rospy.loginfo("  samples: %d", len(self.rows))
        rospy.loginfo("  output: %s", self.output)
        if not self.rows:
            return
        first = self.rows[0]
        last = self.rows[-1]
        sat = sum(row["saturated"] for row in self.rows) / float(len(self.rows))
        rospy.loginfo("  xy error: %.4f -> %.4f m", first["xy_error"], last["xy_error"])
        rospy.loginfo("  yaw error: %.4f -> %.4f rad", first["yaw_error"], last["yaw_error"])
        rospy.loginfo("  max command: %.3f, saturation ratio: %.3f", max(row["max_abs_command"] for row in self.rows), sat)
        rospy.loginfo(
            "  final object drift xy/z/yaw: %.4f m / %.4f m / %.4f rad",
            last["object_drift_xy"],
            last["object_drift_z"],
            last["object_drift_yaw"],
        )

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            had_sample = self.step()
            if (
                had_sample
                and self.duration > 0.0
                and self.first_sample_wall is not None
                and (rospy.Time.now() - self.first_sample_wall).to_sec() >= self.duration
            ):
                break
            rate.sleep()
        self.write_csv()
        self.print_summary()


def main():
    rospy.init_node("leader_base_only_controller")
    LeaderBaseOnlyController().spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
