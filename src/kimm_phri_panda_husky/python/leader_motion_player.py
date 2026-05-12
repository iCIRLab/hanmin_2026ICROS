#!/usr/bin/env python3
"""
Prescribed leader motion player for cooperative transport experiments.

This node intentionally does not control a robot. It publishes a deterministic
leader base, end-effector, and object reference trajectory so follower/planner
code can be verified before dual-robot MuJoCo coupling is introduced.
"""

import math

import rospy
from geometry_msgs.msg import Pose2D, Transform


def yaw_to_quaternion(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def rotate_xy(x, y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


class LeaderMotionPlayer:
    def __init__(self):
        self.scenario = rospy.get_param("~scenario", "straight")
        self.rate_hz = float(rospy.get_param("~rate", 100.0))
        self.duration = float(rospy.get_param("~duration", 10.0))
        self.loop = bool(rospy.get_param("~loop", True))

        self.x0 = float(rospy.get_param("~x0", 0.0))
        self.y0 = float(rospy.get_param("~y0", 0.0))
        self.yaw0 = float(rospy.get_param("~yaw0", 0.0))
        self.speed = float(rospy.get_param("~speed", 0.05))
        self.yaw_rate = float(rospy.get_param("~yaw_rate", 0.15))

        self.ee_offset_x = float(rospy.get_param("~ee_offset_x", 0.6))
        self.ee_offset_y = float(rospy.get_param("~ee_offset_y", 0.0))
        self.ee_offset_z = float(rospy.get_param("~ee_offset_z", 0.9))

        self.object_offset_x = float(rospy.get_param("~object_offset_x", 0.3))
        self.object_offset_y = float(rospy.get_param("~object_offset_y", 0.0))
        self.object_offset_z = float(rospy.get_param("~object_offset_z", 0.9))

        base_topic = rospy.get_param("~base_topic", "/leader/base_ref")
        ee_topic = rospy.get_param("~ee_topic", "/leader/ee_ref")
        object_topic = rospy.get_param("~object_topic", "/leader/object_ref")

        self.base_pub = rospy.Publisher(base_topic, Pose2D, queue_size=10)
        self.ee_pub = rospy.Publisher(ee_topic, Transform, queue_size=10)
        self.object_pub = rospy.Publisher(object_topic, Transform, queue_size=10)

        self.start_time = rospy.Time.now()

        rospy.loginfo(
            "leader_motion_player scenario=%s rate=%.1fHz duration=%.2fs loop=%s",
            self.scenario,
            self.rate_hz,
            self.duration,
            self.loop,
        )
        rospy.loginfo("publishing base=%s ee=%s object=%s", base_topic, ee_topic, object_topic)

    def elapsed(self):
        t = (rospy.Time.now() - self.start_time).to_sec()
        if self.loop and self.duration > 0.0:
            return math.fmod(t, self.duration)
        return min(t, self.duration) if self.duration > 0.0 else t

    def base_pose_at(self, t):
        if self.scenario == "straight":
            return self.x0 + self.speed * t, self.y0, self.yaw0

        if self.scenario == "turn":
            if abs(self.yaw_rate) < 1e-9:
                return self.x0 + self.speed * t, self.y0, self.yaw0

            yaw = self.yaw0 + self.yaw_rate * t
            radius = self.speed / self.yaw_rate
            x = self.x0 + radius * (math.sin(yaw) - math.sin(self.yaw0))
            y = self.y0 - radius * (math.cos(yaw) - math.cos(self.yaw0))
            return x, y, wrap_angle(yaw)

        if self.scenario == "s_curve":
            x = self.x0 + self.speed * t
            y = self.y0 + 0.25 * math.sin(2.0 * math.pi * t / max(self.duration, 1e-3))
            yaw = math.atan2(
                0.25 * (2.0 * math.pi / max(self.duration, 1e-3))
                * math.cos(2.0 * math.pi * t / max(self.duration, 1e-3)),
                self.speed,
            )
            return x, y, wrap_angle(yaw)

        rospy.logwarn_throttle(2.0, "unknown scenario '%s'; using straight", self.scenario)
        return self.x0 + self.speed * t, self.y0, self.yaw0

    def transform_from_base_offset(self, base_x, base_y, base_yaw, ox, oy, oz):
        wx, wy = rotate_xy(ox, oy, base_yaw)
        msg = Transform()
        msg.translation.x = base_x + wx
        msg.translation.y = base_y + wy
        msg.translation.z = oz
        qx, qy, qz, qw = yaw_to_quaternion(base_yaw)
        msg.rotation.x = qx
        msg.rotation.y = qy
        msg.rotation.z = qz
        msg.rotation.w = qw
        return msg

    def publish_once(self):
        t = self.elapsed()
        x, y, yaw = self.base_pose_at(t)

        base = Pose2D()
        base.x = x
        base.y = y
        base.theta = yaw

        ee = self.transform_from_base_offset(
            x, y, yaw, self.ee_offset_x, self.ee_offset_y, self.ee_offset_z
        )
        obj = self.transform_from_base_offset(
            x, y, yaw, self.object_offset_x, self.object_offset_y, self.object_offset_z
        )

        self.base_pub.publish(base)
        self.ee_pub.publish(ee)
        self.object_pub.publish(obj)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.publish_once()
            rate.sleep()


def main():
    rospy.init_node("leader_motion_player")
    LeaderMotionPlayer().spin()


if __name__ == "__main__":
    main()
