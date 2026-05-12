#!/usr/bin/env python3
"""
Minimal dual-scene hold controller for the fixed-object MuJoCo verification gate.

The controller does not use the single-robot HQP stack. It reads the dual scene
joint_states message, stores the first actuated joint positions as hold targets,
and publishes a saturated low-gain torque command for the 26 MuJoCo motors.
"""

import math
import sys

import rospy
from mujoco_ros_msgs.msg import JointSet
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray, MultiArrayDimension


def clamp(value, limit):
    return max(-limit, min(limit, value))


def finite_sequence(values):
    return all(math.isfinite(v) for v in values)


class DualSceneHoldController:
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate", 100.0))
        self.joint_state_topic = rospy.get_param(
            "~joint_state_topic",
            "/coop_scene/mujoco_ros/mujoco_ros_interface/joint_states",
        )
        self.command_topic = rospy.get_param(
            "~command_topic",
            "/coop_scene/mujoco_ros/mujoco_ros_interface/joint_set",
        )
        self.sim_time_topic = rospy.get_param(
            "~sim_time_topic",
            "/coop_scene/mujoco_ros/mujoco_ros_interface/sim_time",
        )
        self.sim_run_topic = rospy.get_param(
            "~sim_run_topic", "/coop_scene/mujoco_ros/mujoco_ros_interface/sim_run"
        )

        self.mode = int(rospy.get_param("~mode", 1))
        self.command_size = int(rospy.get_param("~command_size", 26))
        self.autostop_on_unsafe = bool(rospy.get_param("~autostop_on_unsafe", True))

        self.kp_wheel = float(rospy.get_param("~kp_wheel", 20.0))
        self.kd_wheel = float(rospy.get_param("~kd_wheel", 5.0))
        self.limit_wheel = float(rospy.get_param("~limit_wheel", 20.0))

        self.kp_arm = float(rospy.get_param("~kp_arm", 0.0))
        self.kd_arm = float(rospy.get_param("~kd_arm", 0.0))
        self.limit_arm = float(rospy.get_param("~limit_arm", 5.0))

        self.kp_finger = float(rospy.get_param("~kp_finger", 0.0))
        self.kd_finger = float(rospy.get_param("~kd_finger", 0.2))
        self.limit_finger = float(rospy.get_param("~limit_finger", 5.0))

        # qpos/qvel indices from the current dual scene layout.
        self.left_pos_indices = list(range(7, 20))
        self.right_pos_indices = list(range(27, 40))
        self.left_vel_indices = list(range(6, 19))
        self.right_vel_indices = list(range(25, 38))
        self.actuator_labels = [
            "l_f_l",
            "l_f_r",
            "l_l_l",
            "l_l_r",
            "l_panda_joint1",
            "l_panda_joint2",
            "l_panda_joint3",
            "l_panda_joint4",
            "l_panda_joint5",
            "l_panda_joint6",
            "l_panda_joint7",
            "l_panda_finger_joint1",
            "l_panda_finger_joint2",
            "r_f_l",
            "r_f_r",
            "r_r_l",
            "r_r_r",
            "r_panda_joint1",
            "r_panda_joint2",
            "r_panda_joint3",
            "r_panda_joint4",
            "r_panda_joint5",
            "r_panda_joint6",
            "r_panda_joint7",
            "r_panda_finger_joint1",
            "r_panda_finger_joint2",
        ]

        self.latest_state = None
        self.hold_target = None
        self.latest_time = 0.0
        self.max_abs_command = 0.0
        self.saturated_count = 0
        self.command_count = 0

        rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_cb)
        rospy.Subscriber(self.sim_time_topic, Float32, self.sim_time_cb)
        self.command_pub = rospy.Publisher(self.command_topic, JointSet, queue_size=1)
        self.sim_run_pub = rospy.Publisher(self.sim_run_topic, Bool, queue_size=1, latch=True)
        self.debug_pub = rospy.Publisher(
            "~debug", Float32MultiArray, queue_size=10
        )

        rospy.loginfo(
            "dual_scene_hold_controller command=%s joint_state=%s mode=%d size=%d",
            self.command_topic,
            self.joint_state_topic,
            self.mode,
            self.command_size,
        )
        if self.kp_arm == 0.0 and self.kd_arm == 0.0:
            rospy.logwarn(
                "arm hold is disabled by default; current verified gate only damps/locks wheel motors"
            )

    def joint_state_cb(self, msg):
        self.latest_state = msg

    def sim_time_cb(self, msg):
        self.latest_time = float(msg.data)

    def current_actuated_state(self):
        if self.latest_state is None:
            return None, None
        qpos = list(self.latest_state.position)
        qvel = list(self.latest_state.velocity)
        if len(qpos) < 40 or len(qvel) < 38:
            rospy.logerr_throttle(
                2.0,
                "joint_states too short: position=%d velocity=%d",
                len(qpos),
                len(qvel),
            )
            return None, None
        q = [qpos[i] for i in self.left_pos_indices + self.right_pos_indices]
        dq = [qvel[i] for i in self.left_vel_indices + self.right_vel_indices]
        return q, dq

    def limit_for_index(self, idx):
        local = idx % 13
        if local < 4:
            return self.limit_wheel
        if local < 11:
            return self.limit_arm
        return self.limit_finger

    def gains_for_index(self, idx):
        local = idx % 13
        if local < 4:
            return self.kp_wheel, self.kd_wheel
        if local < 11:
            return self.kp_arm, self.kd_arm
        return self.kp_finger, self.kd_finger

    def compute_command(self, q, dq):
        if self.hold_target is None:
            self.hold_target = list(q)
            rospy.loginfo("captured dual-scene hold target for %d actuators", len(q))

        command = []
        saturated = 0
        for idx, (qi, dqi, qref) in enumerate(zip(q, dq, self.hold_target)):
            kp, kd = self.gains_for_index(idx)
            limit = self.limit_for_index(idx)
            raw = kp * (qref - qi) - kd * dqi
            cmd = clamp(raw, limit)
            if abs(raw) > limit:
                saturated += 1
            command.append(cmd)
        return command, saturated

    def publish_debug(self, command, saturated):
        max_abs = max(abs(x) for x in command) if command else 0.0
        l2 = math.sqrt(sum(x * x for x in command))
        msg = Float32MultiArray()
        msg.layout.dim.append(
            MultiArrayDimension(
                label="max_abs_command,l2_command,saturated_count,saturation_fraction,command_count",
                size=5,
                stride=5,
            )
        )
        msg.data = [
            float(max_abs),
            float(l2),
            float(saturated),
            float(saturated) / max(float(len(command)), 1.0),
            float(self.command_count),
        ]
        self.debug_pub.publish(msg)

    def stop_sim(self):
        if self.autostop_on_unsafe:
            self.sim_run_pub.publish(Bool(data=False))

    def publish_command(self, command):
        msg = JointSet()
        msg.header.stamp = rospy.Time.now()
        msg.time = self.latest_time
        msg.MODE = self.mode
        msg.position = [0.0] * self.command_size
        msg.torque = command
        self.command_pub.publish(msg)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and self.latest_state is None:
            rospy.logwarn_throttle(2.0, "waiting for dual scene joint_states")
            rate.sleep()

        while not rospy.is_shutdown():
            q, dq = self.current_actuated_state()
            if q is None or dq is None:
                rate.sleep()
                continue

            if len(q) != self.command_size or len(dq) != self.command_size:
                rospy.logerr(
                    "command size mismatch q=%d dq=%d expected=%d",
                    len(q),
                    len(dq),
                    self.command_size,
                )
                self.stop_sim()
                return 2
            if not finite_sequence(q) or not finite_sequence(dq):
                rospy.logerr("non-finite joint state detected; stopping hold controller")
                self.stop_sim()
                return 3

            command, saturated = self.compute_command(q, dq)
            if not finite_sequence(command):
                rospy.logerr("non-finite command detected; stopping simulation")
                self.stop_sim()
                return 4

            self.publish_command(command)
            self.command_count += 1
            self.saturated_count += saturated
            self.max_abs_command = max(self.max_abs_command, max(abs(x) for x in command))
            self.publish_debug(command, saturated)
            rospy.loginfo_throttle(
                1.0,
                "hold command max=%.3f saturated=%d/%d",
                max(abs(x) for x in command),
                saturated,
                len(command),
            )
            rate.sleep()
        return 0


def main():
    rospy.init_node("dual_scene_hold_controller")
    code = DualSceneHoldController().spin()
    sys.exit(code)


if __name__ == "__main__":
    main()
