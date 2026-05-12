#!/usr/bin/env python3
"""Merge leader/follower 13-actuator JointSet commands into one 26-actuator command.

This node is the command arbitration layer for the dual Husky-Panda scene. It
keeps leader and follower controllers independent while ensuring that only one
publisher writes to the MuJoCo joint_set topic.
"""

import sys

import rospy
from mujoco_ros_msgs.msg import JointSet
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension


def fit_command(values, size):
    data = list(values)[:size]
    if len(data) < size:
        data.extend([0.0] * (size - len(data)))
    return data


class DualJointSetMerger:
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate", 100.0))
        self.command_size_each = int(rospy.get_param("~command_size_each", 13))
        self.output_size = int(rospy.get_param("~output_size", 26))
        self.timeout = float(rospy.get_param("~timeout", 0.25))
        self.mode = int(rospy.get_param("~mode", 1))
        self.zero_on_timeout = bool(rospy.get_param("~zero_on_timeout", True))
        self.start_delay = float(rospy.get_param("~start_delay", 0.0))
        self.start_wall_time = rospy.Time.now()

        self.leader_topic = rospy.get_param("~leader_command_topic", "/leader/joint_command")
        self.follower_topic = rospy.get_param("~follower_command_topic", "/follower/joint_command")
        self.output_topic = rospy.get_param(
            "~output_topic", "/coop_scene/mujoco_ros/mujoco_ros_interface/joint_set"
        )
        self.sim_time_topic = rospy.get_param(
            "~sim_time_topic", "/coop_scene/mujoco_ros/mujoco_ros_interface/sim_time"
        )

        self.sim_time = 0.0
        self.leader = [0.0] * self.command_size_each
        self.follower = [0.0] * self.command_size_each
        self.leader_stamp = rospy.Time(0)
        self.follower_stamp = rospy.Time(0)
        self.publish_count = 0

        rospy.Subscriber(self.leader_topic, JointSet, self.leader_cb)
        rospy.Subscriber(self.follower_topic, JointSet, self.follower_cb)
        rospy.Subscriber(self.sim_time_topic, Float32, self.sim_time_cb)
        self.pub = rospy.Publisher(self.output_topic, JointSet, queue_size=1)
        self.debug_pub = rospy.Publisher("~debug", Float32MultiArray, queue_size=10)

        rospy.loginfo(
            "dual_joint_set_merger leader=%s follower=%s output=%s",
            self.leader_topic,
            self.follower_topic,
            self.output_topic,
        )

    def sim_time_cb(self, msg):
        self.sim_time = float(msg.data)

    def leader_cb(self, msg):
        self.leader = fit_command(msg.torque, self.command_size_each)
        self.leader_stamp = rospy.Time.now()

    def follower_cb(self, msg):
        self.follower = fit_command(msg.torque, self.command_size_each)
        self.follower_stamp = rospy.Time.now()

    def fresh_or_zero(self, command, stamp):
        if stamp == rospy.Time(0):
            return [0.0] * self.command_size_each, False
        age = (rospy.Time.now() - stamp).to_sec()
        if age > self.timeout and self.zero_on_timeout:
            return [0.0] * self.command_size_each, False
        return command, age <= self.timeout

    def publish_debug(self, leader_fresh, follower_fresh, max_abs):
        msg = Float32MultiArray()
        msg.layout.dim.append(
            MultiArrayDimension(
                label="leader_fresh,follower_fresh,max_abs_command,publish_count",
                size=4,
                stride=4,
            )
        )
        msg.data = [
            1.0 if leader_fresh else 0.0,
            1.0 if follower_fresh else 0.0,
            max_abs,
            float(self.publish_count),
        ]
        self.debug_pub.publish(msg)

    def step(self):
        if self.start_delay > 0.0:
            elapsed = (rospy.Time.now() - self.start_wall_time).to_sec()
            if elapsed < self.start_delay:
                return

        leader, leader_fresh = self.fresh_or_zero(self.leader, self.leader_stamp)
        follower, follower_fresh = self.fresh_or_zero(self.follower, self.follower_stamp)
        torque = fit_command(leader + follower, self.output_size)

        msg = JointSet()
        msg.header.stamp = rospy.Time.now()
        msg.time = self.sim_time
        msg.MODE = self.mode
        msg.position = [0.0] * self.output_size
        msg.torque = torque
        self.pub.publish(msg)
        self.publish_count += 1

        max_abs = max([abs(x) for x in torque] or [0.0])
        self.publish_debug(leader_fresh, follower_fresh, max_abs)
        rospy.loginfo_throttle(
            1.0,
            "dual_joint_set_merger max=%.3f leader_fresh=%s follower_fresh=%s",
            max_abs,
            leader_fresh,
            follower_fresh,
        )

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


def main():
    rospy.init_node("dual_joint_set_merger")
    DualJointSetMerger().spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
