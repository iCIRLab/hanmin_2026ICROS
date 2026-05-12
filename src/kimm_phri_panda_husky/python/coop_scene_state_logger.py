#!/usr/bin/env python3
"""
Log dual Husky-Panda MuJoCo scene state for fixed-object verification.

This node is intentionally read-only. It samples the coop scene joint_states
topic, extracts the left base, right base, and transport object freejoint qpos,
then writes a CSV and a concise summary for verification gates.
"""

import csv
import math
import os
import sys

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


def yaw_from_quaternion(w, x, y, z):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def distance_xy(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def mean(values):
    return sum(values) / len(values) if values else float("nan")


class CoopSceneStateLogger:
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.duration = float(rospy.get_param("~duration", 5.0))
        self.output = rospy.get_param(
            "~output", "/tmp/mm_verify_logs/coop_scene_state.csv"
        )
        self.min_samples = int(rospy.get_param("~min_samples", 10))
        self.joint_state_topic = rospy.get_param(
            "~joint_state_topic",
            "/coop_scene/mujoco_ros/mujoco_ros_interface/joint_states",
        )
        self.sim_time_topic = rospy.get_param(
            "~sim_time_topic",
            "/coop_scene/mujoco_ros/mujoco_ros_interface/sim_time",
        )

        self.left_index = int(rospy.get_param("~left_index", 0))
        self.right_index = int(rospy.get_param("~right_index", 20))
        self.object_index = int(rospy.get_param("~object_index", 40))

        self.expected_left = tuple(rospy.get_param("~expected_left", [-7.0, 0.85, -0.12]))
        self.expected_right = tuple(rospy.get_param("~expected_right", [-7.0, -0.85, -0.12]))
        self.expected_object = tuple(
            rospy.get_param("~expected_object", [-6.562, 0.0, 0.9976])
        )

        self.latest_joint_state = None
        self.latest_sim_time = float("nan")
        self.rows = []
        self.first_sample = None
        self.sample_start_wall = None
        self.callback_count = 0
        self.first_callback_wall = None
        self.last_callback_wall = None
        self.start_wall = rospy.Time.now()

        rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_cb)
        rospy.Subscriber(self.sim_time_topic, Float32, self.sim_time_cb)

        rospy.loginfo(
            "coop_scene_state_logger output=%s duration=%.2fs joint_state=%s",
            self.output,
            self.duration,
            self.joint_state_topic,
        )

    def joint_state_cb(self, msg):
        now = rospy.Time.now().to_sec()
        if self.first_callback_wall is None:
            self.first_callback_wall = now
        self.last_callback_wall = now
        self.callback_count += 1
        self.latest_joint_state = msg

    def sim_time_cb(self, msg):
        self.latest_sim_time = msg.data

    def extract_freejoint(self, positions, start):
        if len(positions) < start + 7:
            return None
        q = positions[start : start + 7]
        return {
            "x": q[0],
            "y": q[1],
            "z": q[2],
            "qw": q[3],
            "qx": q[4],
            "qy": q[5],
            "qz": q[6],
            "yaw": yaw_from_quaternion(q[3], q[4], q[5], q[6]),
        }

    def make_row(self):
        if self.latest_joint_state is None:
            return None

        positions = list(self.latest_joint_state.position)
        left = self.extract_freejoint(positions, self.left_index)
        right = self.extract_freejoint(positions, self.right_index)
        obj = self.extract_freejoint(positions, self.object_index)
        if left is None or right is None or obj is None:
            rospy.logerr_throttle(
                2.0,
                "joint_states length=%d is too short for configured indices",
                len(positions),
            )
            return None

        now = rospy.Time.now()
        row = {
            "wall_time": (now - self.start_wall).to_sec(),
            "sim_time": self.latest_sim_time,
            "left_x": left["x"],
            "left_y": left["y"],
            "left_z": left["z"],
            "left_yaw": left["yaw"],
            "right_x": right["x"],
            "right_y": right["y"],
            "right_z": right["z"],
            "right_yaw": right["yaw"],
            "object_x": obj["x"],
            "object_y": obj["y"],
            "object_z": obj["z"],
            "object_qw": obj["qw"],
            "object_qx": obj["qx"],
            "object_qy": obj["qy"],
            "object_qz": obj["qz"],
            "object_yaw": obj["yaw"],
            "left_keyframe_dxy": distance_xy(self.expected_left, (left["x"], left["y"])),
            "right_keyframe_dxy": distance_xy(self.expected_right, (right["x"], right["y"])),
            "object_keyframe_dxy": distance_xy(self.expected_object, (obj["x"], obj["y"])),
            "object_keyframe_dz": obj["z"] - self.expected_object[2],
            "left_grasp_error": float("nan"),
            "right_grasp_error": float("nan"),
        }

        if self.first_sample is None:
            self.first_sample = row
            self.sample_start_wall = now

        first = self.first_sample
        row["left_drift_xy"] = math.hypot(
            row["left_x"] - first["left_x"], row["left_y"] - first["left_y"]
        )
        row["right_drift_xy"] = math.hypot(
            row["right_x"] - first["right_x"], row["right_y"] - first["right_y"]
        )
        row["object_drift_xy"] = math.hypot(
            row["object_x"] - first["object_x"], row["object_y"] - first["object_y"]
        )
        row["object_drift_z"] = row["object_z"] - first["object_z"]
        row["object_drift_yaw"] = row["object_yaw"] - first["object_yaw"]
        return row

    def sample_once(self):
        row = self.make_row()
        if row is None:
            rospy.logwarn_throttle(2.0, "waiting for coop scene joint_states")
            return
        self.rows.append(row)

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

    def joint_state_rate(self):
        if self.callback_count < 2 or self.first_callback_wall is None:
            return float("nan")
        elapsed = self.last_callback_wall - self.first_callback_wall
        if elapsed <= 0.0:
            return float("nan")
        return float(self.callback_count - 1) / elapsed

    def print_summary(self):
        rospy.loginfo("coop_scene_state_logger summary")
        rospy.loginfo("  samples: %d", len(self.rows))
        rospy.loginfo("  output: %s", self.output)
        rospy.loginfo("  observed joint_states rate: %.2f Hz", self.joint_state_rate())
        if not self.rows:
            return

        last = self.rows[-1]
        rospy.loginfo(
            "  final drift from first sample: left %.4fm, right %.4fm, object %.4fm",
            last["left_drift_xy"],
            last["right_drift_xy"],
            last["object_drift_xy"],
        )
        rospy.loginfo(
            "  final object z/yaw drift: %.4fm / %.4frad",
            last["object_drift_z"],
            last["object_drift_yaw"],
        )
        rospy.loginfo(
            "  mean object height: %.4fm",
            mean([row["object_z"] for row in self.rows]),
        )
        rospy.logwarn(
            "  EE-object grasp error is not available from joint_states alone; CSV fields are NaN."
        )

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.sample_once()
            if (
                self.duration > 0.0
                and self.sample_start_wall is not None
                and (rospy.Time.now() - self.sample_start_wall).to_sec() >= self.duration
            ):
                break
            rate.sleep()

        self.write_csv()
        self.print_summary()
        if len(self.rows) < self.min_samples:
            rospy.logerr("not enough samples: %d < %d", len(self.rows), self.min_samples)
            return 2
        return 0


def main():
    rospy.init_node("coop_scene_state_logger")
    code = CoopSceneStateLogger().spin()
    sys.exit(code)


if __name__ == "__main__":
    main()
