#!/usr/bin/env python3
"""
Follower base-only baseline controller for the dual fixed-object scene.

This node intentionally commands only the right/follower Husky wheels. It is a
small verification controller for non-holonomic yaw alignment and forward assist
before adding arm posture or whole-body control.
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


class FollowerBaseOnlyController:
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate", 100.0))
        self.duration = float(rospy.get_param("~duration", 0.0))
        self.output = rospy.get_param(
            "~output", "/tmp/mm_verify_logs/follower_base_only_controller.csv"
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
        self.object_qpos_offset = int(rospy.get_param("~object_qpos_offset", 40))

        self.k_forward = float(rospy.get_param("~k_forward", 10.0))
        self.k_turn = float(rospy.get_param("~k_turn", 12.0))
        self.k_lateral_turn = float(rospy.get_param("~k_lateral_turn", 3.0))
        self.kd_forward = float(rospy.get_param("~kd_forward", 1.5))
        self.kd_turn = float(rospy.get_param("~kd_turn", 0.8))
        self.force_limit = float(rospy.get_param("~force_limit", 12.0))
        self.yaw_align_threshold = float(rospy.get_param("~yaw_align_threshold", 0.35))
        self.position_deadband = float(rospy.get_param("~position_deadband", 0.03))
        self.max_xy_error = float(rospy.get_param("~max_xy_error", 2.5))

        self.latest_state = None
        self.sim_time = 0.0
        self.base_target = None
        self.start_wall = rospy.Time.now()
        self.first_sample_wall = None
        self.first_object = None
        self.rows = []
        self.command_count = 0

        rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_cb)
        rospy.Subscriber(self.sim_time_topic, Float32, self.sim_time_cb)
        rospy.Subscriber(self.follower_base_topic, Pose2D, self.base_target_cb)
        self.command_pub = rospy.Publisher(self.command_topic, JointSet, queue_size=1)
        self.debug_pub = rospy.Publisher("~debug", Float32MultiArray, queue_size=10)

        rospy.loginfo(
            "follower_base_only_controller target=%s command=%s output=%s",
            self.follower_base_topic,
            self.command_topic,
            self.output,
        )

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

    def extract_object_pose(self):
        q = self.latest_state.position
        i = self.object_qpos_offset
        yaw = yaw_from_quaternion(q[i + 3], q[i + 4], q[i + 5], q[i + 6])
        return {"x": q[i], "y": q[i + 1], "z": q[i + 2], "yaw": yaw}

    def compute_command(self, base, target):
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

        if xy_error > self.position_deadband:
            desired_heading = math.atan2(dy, dx)
        else:
            desired_heading = target.theta
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

    def publish_command(self, left, right):
        command = [0.0] * self.command_size
        command[13] = command[15] = left
        command[14] = command[16] = right

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
                label="xy_error,yaw_error,target_yaw_error,forward,turn,left,right,max_abs_command,saturated",
                size=9,
                stride=9,
            )
        )
        msg.data = [
            row["xy_error"],
            row["yaw_error"],
            row["target_yaw_error"],
            row["forward_cmd"],
            row["turn_cmd"],
            row["left_cmd"],
            row["right_cmd"],
            row["max_abs_command"],
            float(row["saturated"]),
        ]
        self.debug_pub.publish(msg)

    def make_row(self, base, obj, cmd):
        if self.first_sample_wall is None:
            self.first_sample_wall = rospy.Time.now()
        if self.first_object is None:
            self.first_object = obj

        target = self.base_target
        max_cmd = self.publish_command(cmd["left"], cmd["right"])
        row = {
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
            "saturated": int(cmd["saturated"]),
            "gated": int(cmd["gated"]),
            "command_count": self.command_count,
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
        if len(self.latest_state.velocity) < self.right_qvel_offset + 6:
            rospy.logerr_throttle(
                2.0, "unexpected joint_states velocity length=%d", len(self.latest_state.velocity)
            )
            return False

        base = self.extract_base_pose()
        obj = self.extract_object_pose()
        cmd = self.compute_command(base, self.base_target)
        row = self.make_row(base, obj, cmd)
        self.rows.append(row)
        self.publish_debug(row)
        rospy.loginfo_throttle(
            1.0,
            "follower_base_only err_xy=%.3f yaw=%.3f cmd=(%.2f, %.2f) obj_drift=%.4f",
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
        rospy.loginfo("follower_base_only_controller summary")
        rospy.loginfo("  samples: %d", len(self.rows))
        rospy.loginfo("  output: %s", self.output)
        if not self.rows:
            return
        first = self.rows[0]
        last = self.rows[-1]
        saturated = sum(row["saturated"] for row in self.rows)
        rospy.loginfo(
            "  xy error: %.4f -> %.4f m", first["xy_error"], last["xy_error"]
        )
        rospy.loginfo(
            "  yaw error: %.4f -> %.4f rad", first["yaw_error"], last["yaw_error"]
        )
        rospy.loginfo(
            "  target yaw error: %.4f -> %.4f rad",
            first["target_yaw_error"],
            last["target_yaw_error"],
        )
        rospy.loginfo(
            "  max command: %.3f, saturation ratio: %.3f",
            max(row["max_abs_command"] for row in self.rows),
            float(saturated) / float(len(self.rows)),
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
    rospy.init_node("follower_base_only_controller")
    return FollowerBaseOnlyController().spin()


if __name__ == "__main__":
    sys.exit(main())
