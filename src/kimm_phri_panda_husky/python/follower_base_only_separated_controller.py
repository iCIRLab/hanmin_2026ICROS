#!/usr/bin/env python3
"""Follower 13-actuator base-only controller for the dual cooperative scene."""

from leader_base_only_controller import LeaderBaseOnlyController

import rospy


class FollowerBaseOnlySeparatedController(LeaderBaseOnlyController):
    def __init__(self):
        if not rospy.has_param("~base_ref_topic"):
            rospy.set_param("~base_ref_topic", "/follower/base_target")
        if not rospy.has_param("~command_topic"):
            rospy.set_param("~command_topic", "/follower/joint_command")
        if not rospy.has_param("~qpos_offset"):
            rospy.set_param("~qpos_offset", 20)
        if not rospy.has_param("~qvel_offset"):
            rospy.set_param("~qvel_offset", 19)
        if not rospy.has_param("~arm_qpos_offset"):
            rospy.set_param("~arm_qpos_offset", 31)
        if not rospy.has_param("~arm_qvel_offset"):
            rospy.set_param("~arm_qvel_offset", 29)
        if not rospy.has_param("~output"):
            rospy.set_param(
                "~output", "/tmp/mm_verify_logs/follower_base_only_separated_controller.csv"
            )
        if not rospy.has_param("~panda_urdf"):
            rospy.set_param(
                "~panda_urdf",
                "/home/ryoo/mmcm_ws/src/kimm_robots_description/franka_panda_description/robots/panda_arm_hand_r.urdf",
            )
        super(FollowerBaseOnlySeparatedController, self).__init__()


def main():
    rospy.init_node("follower_base_only_separated_controller")
    FollowerBaseOnlySeparatedController().spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
