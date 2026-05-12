#!/usr/bin/env python3
"""
Log leader/follower reference tracking metrics for early coop verification.

This node is intentionally read-only. It subscribes to follower target topics,
actual MuJoCo state topics, and optional cooperative proxy metrics, then writes
a CSV plus a concise terminal summary.
"""

import csv
import math
import os
import sys

import rospy
from geometry_msgs.msg import Pose2D, Transform
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


DEFAULT_METRIC_LABELS = [
    "mu_proxy",
    "F_proxy",
    "B_proxy",
    "cost_proxy",
    "base_err_proxy",
    "yaw_err_proxy",
    "ee_err_proxy",
    "reach",
    "base_step",
]


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def transform_distance(a, b):
    return math.sqrt(
        (a.translation.x - b.translation.x) ** 2
        + (a.translation.y - b.translation.y) ** 2
        + (a.translation.z - b.translation.z) ** 2
    )


def mean(values):
    return sum(values) / len(values) if values else float("nan")


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else float("nan")


class CoopReferenceLogger:
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.duration = float(rospy.get_param("~duration", 10.0))
        self.output = rospy.get_param(
            "~output", "/tmp/mm_verify_logs/coop_reference_metrics.csv"
        )
        self.require_actual = bool(rospy.get_param("~require_actual", True))
        self.min_samples = int(rospy.get_param("~min_samples", 10))

        self.base_target_topic = rospy.get_param("~base_target_topic", "/follower/base_target")
        self.ee_target_topic = rospy.get_param("~ee_target_topic", "/follower/ee_target")
        self.base_state_topic = rospy.get_param(
            "~base_state_topic", "/ns1/mujoco_ros/mujoco_ros_interface/base_state"
        )
        self.ee_state_topic = rospy.get_param(
            "~ee_state_topic", "/ns1/mujoco_ros/mujoco_ros_interface/ee_state"
        )
        self.metric_topic = rospy.get_param("~metric_topic", "/follower/coop_metrics")

        self.base_target = None
        self.ee_target = None
        self.base_state = None
        self.base_velocity = (float("nan"), float("nan"), float("nan"))
        self.ee_state = None
        self.metric_values = []
        self.metric_labels = DEFAULT_METRIC_LABELS[:]

        self.rows = []
        self.start_time = rospy.Time.now()

        rospy.Subscriber(self.base_target_topic, Pose2D, self.base_target_cb)
        rospy.Subscriber(self.ee_target_topic, Transform, self.ee_target_cb)
        rospy.Subscriber(self.base_state_topic, JointState, self.base_state_cb)
        rospy.Subscriber(self.ee_state_topic, Transform, self.ee_state_cb)
        rospy.Subscriber(self.metric_topic, Float32MultiArray, self.metric_cb)

        rospy.loginfo(
            "coop_reference_logger output=%s duration=%.2fs require_actual=%s",
            self.output,
            self.duration,
            self.require_actual,
        )

    def base_target_cb(self, msg):
        self.base_target = msg

    def ee_target_cb(self, msg):
        self.ee_target = msg

    def base_state_cb(self, msg):
        if len(msg.position) >= 3:
            self.base_state = (msg.position[0], msg.position[1], msg.position[2])
        if len(msg.velocity) >= 3:
            self.base_velocity = (msg.velocity[0], msg.velocity[1], msg.velocity[2])

    def ee_state_cb(self, msg):
        self.ee_state = msg

    def metric_cb(self, msg):
        if msg.layout.dim and msg.layout.dim[0].label:
            labels = [label.strip() for label in msg.layout.dim[0].label.split(",")]
            if len(labels) == len(msg.data):
                self.metric_labels = labels
        self.metric_values = list(msg.data)

    def ready(self):
        if self.base_target is None or self.ee_target is None:
            return False
        if self.require_actual and (self.base_state is None or self.ee_state is None):
            return False
        return True

    def sample_once(self):
        if not self.ready():
            rospy.logwarn_throttle(2.0, "waiting for target and actual state topics")
            return

        now = rospy.Time.now()
        t = (now - self.start_time).to_sec()

        if self.base_state is None:
            actual_x = actual_y = actual_yaw = float("nan")
            base_err = yaw_err = float("nan")
        else:
            actual_x, actual_y, actual_yaw = self.base_state
            base_err = math.hypot(
                self.base_target.x - actual_x,
                self.base_target.y - actual_y,
            )
            yaw_err = abs(wrap_angle(self.base_target.theta - actual_yaw))

        ee_err = transform_distance(self.ee_target, self.ee_state) if self.ee_state else float("nan")

        row = {
            "time": t,
            "base_target_x": self.base_target.x,
            "base_target_y": self.base_target.y,
            "base_target_yaw": self.base_target.theta,
            "base_actual_x": actual_x,
            "base_actual_y": actual_y,
            "base_actual_yaw": actual_yaw,
            "base_actual_vx": self.base_velocity[0],
            "base_actual_vy": self.base_velocity[1],
            "base_actual_wz": self.base_velocity[2],
            "base_err_xy": base_err,
            "yaw_err": yaw_err,
            "ee_target_x": self.ee_target.translation.x,
            "ee_target_y": self.ee_target.translation.y,
            "ee_target_z": self.ee_target.translation.z,
            "ee_actual_x": self.ee_state.translation.x if self.ee_state else float("nan"),
            "ee_actual_y": self.ee_state.translation.y if self.ee_state else float("nan"),
            "ee_actual_z": self.ee_state.translation.z if self.ee_state else float("nan"),
            "ee_err_xyz": ee_err,
        }

        for idx, value in enumerate(self.metric_values):
            label = self.metric_labels[idx] if idx < len(self.metric_labels) else "metric_%d" % idx
            row["metric_" + label] = value

        self.rows.append(row)

    def write_csv(self):
        if not self.rows:
            return
        directory = os.path.dirname(self.output)
        if directory:
            os.makedirs(directory, exist_ok=True)

        fieldnames = list(self.rows[0].keys())
        for row in self.rows[1:]:
            for key in row.keys():
                if key not in fieldnames:
                    fieldnames.append(key)

        with open(self.output, "w", newline="") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.rows)

    def print_summary(self):
        base_errors = [row["base_err_xy"] for row in self.rows if not math.isnan(row["base_err_xy"])]
        yaw_errors = [row["yaw_err"] for row in self.rows if not math.isnan(row["yaw_err"])]
        ee_errors = [row["ee_err_xyz"] for row in self.rows if not math.isnan(row["ee_err_xyz"])]
        costs = [row["metric_cost_proxy"] for row in self.rows if "metric_cost_proxy" in row]

        rospy.loginfo("coop_reference_logger summary")
        rospy.loginfo("  samples: %d", len(self.rows))
        rospy.loginfo("  output: %s", self.output)
        if base_errors:
            rospy.loginfo(
                "  base_err_xy mean/rms/max: %.4f / %.4f / %.4f m",
                mean(base_errors),
                rms(base_errors),
                max(base_errors),
            )
        if yaw_errors:
            rospy.loginfo(
                "  yaw_err mean/rms/max: %.4f / %.4f / %.4f rad",
                mean(yaw_errors),
                rms(yaw_errors),
                max(yaw_errors),
            )
        if ee_errors:
            rospy.loginfo(
                "  ee_err_xyz mean/rms/max: %.4f / %.4f / %.4f m",
                mean(ee_errors),
                rms(ee_errors),
                max(ee_errors),
            )
        if costs:
            rospy.loginfo(
                "  cost_proxy mean/min/max: %.4f / %.4f / %.4f",
                mean(costs),
                min(costs),
                max(costs),
            )

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.sample_once()
            if self.duration > 0.0 and (rospy.Time.now() - self.start_time).to_sec() >= self.duration:
                break
            rate.sleep()

        self.write_csv()
        self.print_summary()

        if len(self.rows) < self.min_samples:
            rospy.logerr("not enough samples: %d < %d", len(self.rows), self.min_samples)
            return 2
        return 0


def main():
    rospy.init_node("coop_reference_logger")
    code = CoopReferenceLogger().spin()
    sys.exit(code)


if __name__ == "__main__":
    main()
