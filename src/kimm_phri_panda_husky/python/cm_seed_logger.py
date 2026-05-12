#!/usr/bin/env python3
"""Log baseline/CM-seed compatible records for cooperative transport experiments."""

import csv
import os
import sys

import rospy
from geometry_msgs.msg import Pose2D, Transform
from std_msgs.msg import Float32MultiArray


METRIC_LABELS = [
    "mu_proxy",
    "F_proxy",
    "B_proxy",
    "cost_proxy",
    "base_err",
    "yaw_err",
    "ee_err",
    "reach",
    "base_step",
]


class CMSeedLogger:
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.duration = float(rospy.get_param("~duration", 8.0))
        self.output = rospy.get_param("~output", "/tmp/mm_verify_logs/cm_seed_log.csv")
        self.seed_type = rospy.get_param("~seed_type", "baseline_home")
        self.seed_source = rospy.get_param("~seed_source", "home_posture_prior")
        self.cm_table_path = rospy.get_param("~cm_table_path", "")

        self.base_target = None
        self.ee_target = None
        self.metrics = {}
        self.rows = []
        self.start_time = rospy.Time.now()

        rospy.Subscriber("/follower/base_target", Pose2D, self.base_cb)
        rospy.Subscriber("/follower/ee_target", Transform, self.ee_cb)
        rospy.Subscriber("/follower/coop_metrics", Float32MultiArray, self.metric_cb)

        rospy.loginfo(
            "cm_seed_logger seed_type=%s output=%s cm_table=%s",
            self.seed_type,
            self.output,
            self.cm_table_path if self.cm_table_path else "none",
        )

    def base_cb(self, msg):
        self.base_target = msg

    def ee_cb(self, msg):
        self.ee_target = msg

    def metric_cb(self, msg):
        labels = METRIC_LABELS
        if msg.layout.dim and msg.layout.dim[0].label:
            parsed = [x.strip() for x in msg.layout.dim[0].label.split(",")]
            if len(parsed) == len(msg.data):
                labels = parsed
        self.metrics = {label: msg.data[i] for i, label in enumerate(labels)}

    def ready(self):
        return self.base_target is not None and self.ee_target is not None and bool(self.metrics)

    def sample_once(self):
        if not self.ready():
            rospy.logwarn_throttle(2.0, "waiting for follower target and coop metrics")
            return
        t = (rospy.Time.now() - self.start_time).to_sec()
        mu = float(self.metrics.get("mu_proxy", float("nan")))
        f_value = float(self.metrics.get("F_proxy", float("nan")))
        b_value = float(self.metrics.get("B_proxy", float("nan")))
        cost = float(self.metrics.get("cost_proxy", float("nan")))
        row = {
            "time": t,
            "seed_type": self.seed_type,
            "seed_source": self.seed_source,
            "cm_table_path": self.cm_table_path,
            "seed_score": cost,
            "mu": mu,
            "F": f_value,
            "B": b_value,
            "cost": cost,
            "base_target_x": self.base_target.x,
            "base_target_y": self.base_target.y,
            "base_target_yaw": self.base_target.theta,
            "ee_target_x": self.ee_target.translation.x,
            "ee_target_y": self.ee_target.translation.y,
            "ee_target_z": self.ee_target.translation.z,
            "reach": self.metrics.get("reach", float("nan")),
            "base_step": self.metrics.get("base_step", float("nan")),
        }
        self.rows.append(row)

    def write_csv(self):
        if not self.rows:
            return
        directory = os.path.dirname(self.output)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(self.output, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.sample_once()
            if self.duration > 0.0 and (rospy.Time.now() - self.start_time).to_sec() >= self.duration:
                break
            rate.sleep()
        self.write_csv()
        rospy.loginfo("cm_seed_logger wrote %d samples to %s", len(self.rows), self.output)
        return 0 if self.rows else 2


def main():
    rospy.init_node("cm_seed_logger")
    return CMSeedLogger().spin()


if __name__ == "__main__":
    sys.exit(main())
