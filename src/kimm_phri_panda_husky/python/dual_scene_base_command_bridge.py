#!/usr/bin/env python3
"""
Minimal base command bridge for dual Husky-Panda cooperative transport demos.

This is not a whole-body controller. It tracks prescribed leader/follower base
references with low, saturated wheel torques so the dual fixed-object scene can
be exercised before integrating the HQP/CM controller.
"""

import math
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


class DualSceneBaseCommandBridge:
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate", 100.0))
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
        self.leader_base_topic = rospy.get_param("~leader_base_topic", "/leader/base_ref")
        self.follower_base_topic = rospy.get_param(
            "~follower_base_topic", "/follower/base_target"
        )

        self.enable_leader = bool(rospy.get_param("~enable_leader", True))
        self.enable_follower = bool(rospy.get_param("~enable_follower", True))
        self.command_size = int(rospy.get_param("~command_size", 26))
        self.kx = float(rospy.get_param("~kx", 16.0))
        self.ky = float(rospy.get_param("~ky", 5.0))
        self.kyaw = float(rospy.get_param("~kyaw", 10.0))
        self.kd_forward = float(rospy.get_param("~kd_forward", 2.0))
        self.kd_yaw = float(rospy.get_param("~kd_yaw", 1.0))
        self.force_limit = float(rospy.get_param("~force_limit", 18.0))
        self.max_xy_error = float(rospy.get_param("~max_xy_error", 2.0))

        self.latest_state = None
        self.sim_time = 0.0
        self.leader_ref = None
        self.follower_ref = None
        self.command_count = 0

        rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_cb)
        rospy.Subscriber(self.sim_time_topic, Float32, self.sim_time_cb)
        rospy.Subscriber(self.leader_base_topic, Pose2D, self.leader_ref_cb)
        rospy.Subscriber(self.follower_base_topic, Pose2D, self.follower_ref_cb)
        self.command_pub = rospy.Publisher(self.command_topic, JointSet, queue_size=1)
        self.debug_pub = rospy.Publisher("~debug", Float32MultiArray, queue_size=10)

        rospy.loginfo(
            "dual_scene_base_command_bridge command=%s leader=%s follower=%s",
            self.command_topic,
            self.leader_base_topic,
            self.follower_base_topic,
        )

    def joint_state_cb(self, msg):
        self.latest_state = msg

    def sim_time_cb(self, msg):
        self.sim_time = float(msg.data)

    def leader_ref_cb(self, msg):
        self.leader_ref = msg

    def follower_ref_cb(self, msg):
        self.follower_ref = msg

    def base_pose(self, offset):
        q = self.latest_state.position
        v = self.latest_state.velocity
        yaw = yaw_from_quaternion(q[offset + 3], q[offset + 4], q[offset + 5], q[offset + 6])
        wz_index = 5 if offset == 0 else 24
        vx_index = 0 if offset == 0 else 19
        return (q[offset], q[offset + 1], yaw, v[vx_index], v[wz_index])

    def wheel_command(self, pose, ref):
        x, y, yaw, vx_world, wz = pose
        dx = ref.x - x
        dy = ref.y - y
        if math.hypot(dx, dy) > self.max_xy_error:
            return 0.0, 0.0, 0.0, 0.0

        c = math.cos(yaw)
        s = math.sin(yaw)
        ex_body = c * dx + s * dy
        ey_body = -s * dx + c * dy
        yaw_err = wrap_angle(ref.theta - yaw)
        vx_body = c * vx_world

        forward = clamp(self.kx * ex_body - self.kd_forward * vx_body, self.force_limit)
        turn = clamp(self.kyaw * yaw_err + self.ky * ey_body - self.kd_yaw * wz, self.force_limit)
        left = clamp(forward - turn, self.force_limit)
        right = clamp(forward + turn, self.force_limit)
        return left, right, math.hypot(dx, dy), abs(yaw_err)

    def publish_debug(self, leader_xy, leader_yaw, follower_xy, follower_yaw, max_cmd):
        msg = Float32MultiArray()
        msg.layout.dim.append(
            MultiArrayDimension(
                label="leader_xy_err,leader_yaw_err,follower_xy_err,follower_yaw_err,max_abs_command,command_count",
                size=6,
                stride=6,
            )
        )
        msg.data = [
            leader_xy,
            leader_yaw,
            follower_xy,
            follower_yaw,
            max_cmd,
            float(self.command_count),
        ]
        self.debug_pub.publish(msg)

    def step(self):
        if self.latest_state is None or self.leader_ref is None or self.follower_ref is None:
            rospy.logwarn_throttle(2.0, "waiting for joint_states and base references")
            return
        if len(self.latest_state.position) < 47 or len(self.latest_state.velocity) < 44:
            rospy.logerr_throttle(2.0, "unexpected joint_states layout")
            return

        command = [0.0] * self.command_size
        leader_xy = leader_yaw = follower_xy = follower_yaw = 0.0

        if self.enable_leader:
            l_left, l_right, leader_xy, leader_yaw = self.wheel_command(
                self.base_pose(0), self.leader_ref
            )
            command[0] = command[2] = l_left
            command[1] = command[3] = l_right

        if self.enable_follower:
            r_left, r_right, follower_xy, follower_yaw = self.wheel_command(
                self.base_pose(20), self.follower_ref
            )
            command[13] = command[15] = r_left
            command[14] = command[16] = r_right

        msg = JointSet()
        msg.header.stamp = rospy.Time.now()
        msg.time = self.sim_time
        msg.MODE = 1
        msg.position = [0.0] * self.command_size
        msg.torque = command
        self.command_pub.publish(msg)
        self.command_count += 1
        max_cmd = max(abs(x) for x in command)
        self.publish_debug(leader_xy, leader_yaw, follower_xy, follower_yaw, max_cmd)
        rospy.loginfo_throttle(
            1.0,
            "base bridge max=%.3f leader_err=%.3f follower_err=%.3f",
            max_cmd,
            leader_xy,
            follower_xy,
        )

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


def main():
    rospy.init_node("dual_scene_base_command_bridge")
    DualSceneBaseCommandBridge().spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
