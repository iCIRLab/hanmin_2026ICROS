#!/usr/bin/env python3
"""
Follower baseline controller for the dual fixed-object scene.

The controller publishes one 26-actuator JointSet command so follower base and
right Panda arm commands do not fight on the same MuJoCo command topic.
It is a verification baseline, not the final Pinocchio/HQP controller.
"""

import csv
import math
import os
import sys

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


def vector_norm(values):
    return math.sqrt(sum(v * v for v in values))


class FollowerBaseArmHomeController:
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate", 100.0))
        self.duration = float(rospy.get_param("~duration", 0.0))
        self.output = rospy.get_param(
            "~output", "/tmp/mm_verify_logs/follower_base_arm_home_controller.csv"
        )

        self.joint_state_topic = rospy.get_param(
            "~joint_state_topic",
            "/coop_scene/mujoco_ros/mujoco_ros_interface/joint_states",
        )
        self.sim_time_topic = rospy.get_param(
            "~sim_time_topic", "/coop_scene/mujoco_ros/mujoco_ros_interface/sim_time"
        )
        self.command_topic = rospy.get_param(
            "~command_topic", "/coop_scene/mujoco_ros/mujoco_ros_interface/joint_set"
        )
        self.follower_base_topic = rospy.get_param(
            "~follower_base_topic", "/follower/base_target"
        )

        self.command_size = int(rospy.get_param("~command_size", 26))
        self.right_qpos_offset = int(rospy.get_param("~right_qpos_offset", 20))
        self.right_qvel_offset = int(rospy.get_param("~right_qvel_offset", 19))
        self.right_arm_qpos_offset = int(rospy.get_param("~right_arm_qpos_offset", 31))
        self.right_arm_qvel_offset = int(rospy.get_param("~right_arm_qvel_offset", 29))
        self.object_qpos_offset = int(rospy.get_param("~object_qpos_offset", 40))

        self.k_forward = float(rospy.get_param("~k_forward", 10.0))
        self.k_turn = float(rospy.get_param("~k_turn", 35.0))
        self.k_lateral_turn = float(rospy.get_param("~k_lateral_turn", 5.0))
        self.kd_forward = float(rospy.get_param("~kd_forward", 1.5))
        self.kd_turn = float(rospy.get_param("~kd_turn", 0.8))
        self.base_force_limit = float(rospy.get_param("~base_force_limit", 30.0))
        self.yaw_align_threshold = float(rospy.get_param("~yaw_align_threshold", 0.35))
        self.position_deadband = float(rospy.get_param("~position_deadband", 0.03))
        self.max_xy_error = float(rospy.get_param("~max_xy_error", 2.5))

        default_home = [0.0, 0.0, 0.0, -math.pi / 2.0, 0.0, math.pi / 2.0, -math.pi / 4.0]
        self.home_posture = list(rospy.get_param("~home_posture", default_home))
        if len(self.home_posture) != 7:
            raise ValueError("~home_posture must contain 7 joint values")
        self.seed_mode = rospy.get_param("~seed_mode", "home")
        self.cm_seed_table_path = rospy.get_param("~cm_seed_table_path", "")
        self.seed_source = "home_posture_param"
        if self.seed_mode == "cm":
            loaded = self.load_seed_posture(self.cm_seed_table_path, prefer_cm=True)
            if loaded is not None:
                self.home_posture, self.seed_source = loaded
                rospy.loginfo(
                    "using CM seed posture from %s source=%s",
                    self.cm_seed_table_path,
                    self.seed_source,
                )
            else:
                rospy.logwarn(
                    "seed_mode=cm requested but no valid seed found in %s; using home posture",
                    self.cm_seed_table_path,
                )
        elif self.seed_mode == "baseline_keyframe":
            loaded = self.load_seed_posture(self.cm_seed_table_path, prefer_cm=False)
            if loaded is not None:
                self.home_posture, self.seed_source = loaded
                rospy.loginfo(
                    "using baseline seed posture from %s source=%s",
                    self.cm_seed_table_path,
                    self.seed_source,
                )
        self.use_initial_arm_as_home = bool(rospy.get_param("~use_initial_arm_as_home", False))
        self.kp_arm = float(rospy.get_param("~kp_arm", 6.0))
        self.kd_arm = float(rospy.get_param("~kd_arm", 0.8))
        self.arm_torque_limit = float(rospy.get_param("~arm_torque_limit", 12.0))

        self.latest_state = None
        self.sim_time = 0.0
        self.base_target = None
        self.start_wall = rospy.Time.now()
        self.first_sample_wall = None
        self.first_object = None
        self.initial_posture_error = None
        self.rows = []
        self.command_count = 0

        rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_cb)
        rospy.Subscriber(self.sim_time_topic, Float32, self.sim_time_cb)
        rospy.Subscriber(self.follower_base_topic, Pose2D, self.base_target_cb)
        self.command_pub = rospy.Publisher(self.command_topic, JointSet, queue_size=1)
        self.debug_pub = rospy.Publisher("~debug", Float32MultiArray, queue_size=10)

        rospy.loginfo(
            "follower_base_arm_home_controller target=%s command=%s output=%s",
            self.follower_base_topic,
            self.command_topic,
            self.output,
        )

    def load_seed_posture(self, path, prefer_cm):
        if not path or not os.path.exists(path):
            return None
        with open(path) as f:
            rows = list(csv.DictReader(f))
        if not rows:
            return None

        def valid(row):
            return all("q%d" % i in row for i in range(1, 8))

        rows = [row for row in rows if valid(row)]
        if not rows:
            return None
        if prefer_cm:
            cm_rows = [row for row in rows if "cm" in row.get("seed_type", "").lower()]
            rows = cm_rows or rows
            key = "total_cost" if "total_cost" in rows[0] else "cost"

            def score(row):
                try:
                    return float(row.get(key, "-inf"))
                except ValueError:
                    return float("-inf")

            row = max(rows, key=score)
        else:
            baseline_rows = [
                row for row in rows if "baseline" in row.get("seed_type", "").lower()
            ]
            row = baseline_rows[0] if baseline_rows else rows[0]
        q = [float(row["q%d" % i]) for i in range(1, 8)]
        return q, row.get("seed_source", row.get("seed_type", "seed_table"))

    def joint_state_cb(self, msg):
        self.latest_state = msg

    def sim_time_cb(self, msg):
        self.sim_time = float(msg.data)

    def base_target_cb(self, msg):
        self.base_target = msg

    def extract_base_pose(self):
        q = self.latest_state.position
        v = self.latest_state.velocity
        i = self.right_qpos_offset
        j = self.right_qvel_offset
        yaw = yaw_from_quaternion(q[i + 3], q[i + 4], q[i + 5], q[i + 6])
        return {
            "x": q[i],
            "y": q[i + 1],
            "z": q[i + 2],
            "yaw": yaw,
            "vx_world": v[j],
            "wz": v[j + 5],
        }

    def extract_arm_state(self):
        q = self.latest_state.position
        v = self.latest_state.velocity
        qi = self.right_arm_qpos_offset
        vi = self.right_arm_qvel_offset
        return list(q[qi : qi + 7]), list(v[vi : vi + 7])

    def extract_object_pose(self):
        q = self.latest_state.position
        i = self.object_qpos_offset
        yaw = yaw_from_quaternion(q[i + 3], q[i + 4], q[i + 5], q[i + 6])
        return {"x": q[i], "y": q[i + 1], "z": q[i + 2], "yaw": yaw}

    def compute_base_command(self, base, target):
        dx = target.x - base["x"]
        dy = target.y - base["y"]
        xy_error = math.hypot(dx, dy)
        if xy_error > self.max_xy_error:
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
                "target_yaw_error": wrap_angle(target.theta - base["yaw"]),
                "saturated": False,
                "gated": True,
            }

        c = math.cos(base["yaw"])
        s = math.sin(base["yaw"])
        ex_body = c * dx + s * dy
        ey_body = -s * dx + c * dy
        desired_heading = math.atan2(dy, dx) if xy_error > self.position_deadband else target.theta
        yaw_error = wrap_angle(desired_heading - base["yaw"])
        target_yaw_error = wrap_angle(target.theta - base["yaw"])

        vx_body = c * base["vx_world"]
        forward_raw = self.k_forward * ex_body - self.kd_forward * vx_body
        if abs(yaw_error) > self.yaw_align_threshold:
            forward_raw = 0.0
        turn_raw = (
            self.k_turn * yaw_error
            + self.k_lateral_turn * ey_body
            - self.kd_turn * base["wz"]
        )

        forward = clamp(forward_raw, self.base_force_limit)
        turn = clamp(turn_raw, self.base_force_limit)
        left = clamp(forward - turn, self.base_force_limit)
        right = clamp(forward + turn, self.base_force_limit)
        saturated = (
            abs(forward_raw) > self.base_force_limit
            or abs(turn_raw) > self.base_force_limit
            or abs(forward - turn) > self.base_force_limit
            or abs(forward + turn) > self.base_force_limit
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

    def compute_arm_command(self, q_arm, dq_arm):
        error = [self.home_posture[i] - q_arm[i] for i in range(7)]
        raw = [self.kp_arm * error[i] - self.kd_arm * dq_arm[i] for i in range(7)]
        cmd = [clamp(value, self.arm_torque_limit) for value in raw]
        saturated = any(abs(raw[i]) > self.arm_torque_limit for i in range(7))
        posture_error = vector_norm(error)
        max_joint_error = max(abs(value) for value in error)
        return {
            "error": error,
            "command": cmd,
            "posture_error": posture_error,
            "max_joint_error": max_joint_error,
            "command_norm": vector_norm(cmd),
            "saturated": saturated,
        }

    def publish_command(self, base_cmd, arm_cmd):
        command = [0.0] * self.command_size
        command[13] = command[15] = base_cmd["left"]
        command[14] = command[16] = base_cmd["right"]
        for i in range(7):
            command[17 + i] = arm_cmd["command"][i]

        msg = JointSet()
        msg.header.stamp = rospy.Time.now()
        msg.time = self.sim_time
        msg.MODE = 1
        msg.position = [0.0] * self.command_size
        msg.torque = command
        self.command_pub.publish(msg)
        self.command_count += 1
        return max(abs(x) for x in command)

    def publish_debug(self, row):
        msg = Float32MultiArray()
        msg.layout.dim.append(
            MultiArrayDimension(
                label="xy_error,target_yaw_error,posture_error,max_joint_error,base_max_cmd,arm_cmd_norm,base_sat,arm_sat",
                size=8,
                stride=8,
            )
        )
        msg.data = [
            row["xy_error"],
            row["target_yaw_error"],
            row["posture_error"],
            row["max_joint_error"],
            row["base_max_abs_command"],
            row["arm_command_norm"],
            float(row["base_saturated"]),
            float(row["arm_saturated"]),
        ]
        self.debug_pub.publish(msg)

    def make_row(self, base, q_arm, dq_arm, obj, base_cmd, arm_cmd):
        if self.first_sample_wall is None:
            self.first_sample_wall = rospy.Time.now()
        if self.first_object is None:
            self.first_object = obj
        if self.initial_posture_error is None:
            self.initial_posture_error = arm_cmd["posture_error"]

        target = self.base_target
        max_cmd = self.publish_command(base_cmd, arm_cmd)
        row = {
            "wall_time": (rospy.Time.now() - self.start_wall).to_sec(),
            "sim_time": self.sim_time,
            "target_x": target.x,
            "target_y": target.y,
            "target_yaw": target.theta,
            "base_x": base["x"],
            "base_y": base["y"],
            "base_yaw": base["yaw"],
            "dx": base_cmd["dx"],
            "dy": base_cmd["dy"],
            "ex_body": base_cmd["ex_body"],
            "ey_body": base_cmd["ey_body"],
            "xy_error": base_cmd["xy_error"],
            "yaw_error": base_cmd["yaw_error"],
            "target_yaw_error": base_cmd["target_yaw_error"],
            "base_forward_cmd": base_cmd["forward"],
            "base_turn_cmd": base_cmd["turn"],
            "base_left_cmd": base_cmd["left"],
            "base_right_cmd": base_cmd["right"],
            "base_max_abs_command": max(abs(base_cmd["left"]), abs(base_cmd["right"])),
            "base_saturated": int(base_cmd["saturated"]),
            "base_gated": int(base_cmd["gated"]),
            "posture_error": arm_cmd["posture_error"],
            "posture_error_delta": arm_cmd["posture_error"] - self.initial_posture_error,
            "max_joint_error": arm_cmd["max_joint_error"],
            "arm_command_norm": arm_cmd["command_norm"],
            "arm_max_abs_command": max(abs(value) for value in arm_cmd["command"]),
            "arm_saturated": int(arm_cmd["saturated"]),
            "max_abs_command": max_cmd,
            "command_count": self.command_count,
            "seed_mode": self.seed_mode,
            "seed_source": self.seed_source,
            "cm_seed_table_path": self.cm_seed_table_path,
            "object_x": obj["x"],
            "object_y": obj["y"],
            "object_z": obj["z"],
            "object_yaw": obj["yaw"],
            "object_drift_xy": math.hypot(
                obj["x"] - self.first_object["x"], obj["y"] - self.first_object["y"]
            ),
            "object_drift_z": obj["z"] - self.first_object["z"],
            "object_drift_yaw": wrap_angle(obj["yaw"] - self.first_object["yaw"]),
        }
        for i in range(7):
            row["q%d" % (i + 1)] = q_arm[i]
            row["dq%d" % (i + 1)] = dq_arm[i]
            row["q_home%d" % (i + 1)] = self.home_posture[i]
            row["q_error%d" % (i + 1)] = arm_cmd["error"][i]
            row["arm_cmd%d" % (i + 1)] = arm_cmd["command"][i]
        return row

    def step(self):
        if self.latest_state is None or self.base_target is None:
            rospy.logwarn_throttle(2.0, "waiting for joint_states and follower base target")
            return False
        if len(self.latest_state.position) < self.object_qpos_offset + 7:
            rospy.logerr_throttle(
                2.0, "unexpected joint_states position length=%d", len(self.latest_state.position)
            )
            return False
        if len(self.latest_state.velocity) < self.right_arm_qvel_offset + 7:
            rospy.logerr_throttle(
                2.0, "unexpected joint_states velocity length=%d", len(self.latest_state.velocity)
            )
            return False

        base = self.extract_base_pose()
        q_arm, dq_arm = self.extract_arm_state()
        if self.use_initial_arm_as_home and not self.rows:
            self.home_posture = list(q_arm)
            rospy.logwarn("using initial right arm posture as home reference")
        obj = self.extract_object_pose()
        base_cmd = self.compute_base_command(base, self.base_target)
        arm_cmd = self.compute_arm_command(q_arm, dq_arm)
        row = self.make_row(base, q_arm, dq_arm, obj, base_cmd, arm_cmd)
        self.rows.append(row)
        self.publish_debug(row)
        rospy.loginfo_throttle(
            1.0,
            "base_arm_home xy=%.3f target_yaw=%.3f posture=%.3f arm_cmd=%.2f obj=%.4f",
            row["xy_error"],
            row["target_yaw_error"],
            row["posture_error"],
            row["arm_command_norm"],
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
        rospy.loginfo("follower_base_arm_home_controller summary")
        rospy.loginfo("  samples: %d", len(self.rows))
        rospy.loginfo("  output: %s", self.output)
        if not self.rows:
            return
        first = self.rows[0]
        last = self.rows[-1]
        base_sat = sum(row["base_saturated"] for row in self.rows)
        arm_sat = sum(row["arm_saturated"] for row in self.rows)
        rospy.loginfo("  xy error: %.4f -> %.4f m", first["xy_error"], last["xy_error"])
        rospy.loginfo(
            "  target yaw error: %.4f -> %.4f rad",
            first["target_yaw_error"],
            last["target_yaw_error"],
        )
        rospy.loginfo(
            "  posture error: %.4f -> %.4f rad",
            first["posture_error"],
            last["posture_error"],
        )
        rospy.loginfo(
            "  max command: base %.3f, arm %.3f",
            max(row["base_max_abs_command"] for row in self.rows),
            max(row["arm_max_abs_command"] for row in self.rows),
        )
        rospy.loginfo(
            "  saturation ratio: base %.3f, arm %.3f",
            float(base_sat) / float(len(self.rows)),
            float(arm_sat) / float(len(self.rows)),
        )
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
        return 0 if self.rows else 2


def main():
    rospy.init_node("follower_base_arm_home_controller")
    return FollowerBaseArmHomeController().spin()


if __name__ == "__main__":
    sys.exit(main())
