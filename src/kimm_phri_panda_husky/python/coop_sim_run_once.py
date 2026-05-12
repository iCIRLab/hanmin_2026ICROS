#!/usr/bin/env python3
"""Publish a one-shot MuJoCo sim_run command for coop scene verification."""

import sys

import rospy
from std_msgs.msg import Bool


def main():
    rospy.init_node("coop_sim_run_once")
    topic = rospy.get_param(
        "~topic", "/coop_scene/mujoco_ros/mujoco_ros_interface/sim_run"
    )
    delay = float(rospy.get_param("~delay", 2.0))
    value = bool(rospy.get_param("~value", True))
    latch_seconds = float(rospy.get_param("~latch_seconds", 1.0))

    pub = rospy.Publisher(topic, Bool, queue_size=1, latch=True)
    rospy.sleep(delay)
    pub.publish(Bool(data=value))
    rospy.loginfo("published sim_run=%s to %s", value, topic)
    rospy.sleep(latch_seconds)
    return 0


if __name__ == "__main__":
    sys.exit(main())
