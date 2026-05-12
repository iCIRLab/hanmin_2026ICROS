#!/usr/bin/env python3
"""
Follower cooperative target planner for early leader/follower verification.

The node consumes prescribed leader references and publishes follower base/EE
targets plus lightweight proxy metrics. It does not command the existing HQP
controller yet; this keeps the first integration step observable and reversible.
"""

import math

import rospy
from geometry_msgs.msg import Pose2D, Transform
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def rotate_xy(x, y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


def distance2(x0, y0, x1, y1):
    return math.hypot(x1 - x0, y1 - y0)


class FollowerCoopPlanner:
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate", 50.0))
        self.side = rospy.get_param("~side", "right")
        if self.side not in ("left", "right"):
            rospy.logwarn("unknown side '%s'; using right", self.side)
            self.side = "right"

        self.lateral_distance = float(rospy.get_param("~lateral_distance", 1.2))
        self.base_backoff = float(rospy.get_param("~base_backoff", 0.35))
        self.ee_forward_offset = float(rospy.get_param("~ee_forward_offset", 0.0))
        self.ee_lateral_distance = float(rospy.get_param("~ee_lateral_distance", 0.6))
        self.ee_height = float(rospy.get_param("~ee_height", 0.9))
        self.preferred_arm_reach = float(rospy.get_param("~preferred_arm_reach", 0.75))
        self.max_arm_reach = float(rospy.get_param("~max_arm_reach", 1.1))
        self.formation_distance = float(rospy.get_param("~formation_distance", -1.0))
        self.base_nominal_speed = float(rospy.get_param("~base_nominal_speed", 0.4))
        self.publish_only_when_ready = bool(rospy.get_param("~publish_only_when_ready", True))

        self.leader_base_topic = rospy.get_param("~leader_base_topic", "/leader/base_ref")
        self.leader_ee_topic = rospy.get_param("~leader_ee_topic", "/leader/ee_ref")
        self.leader_object_topic = rospy.get_param("~leader_object_topic", "/leader/object_ref")
        self.follower_base_state_topic = rospy.get_param(
            "~follower_base_state_topic",
            "/ns1/mujoco_ros/mujoco_ros_interface/base_state",
        )
        self.follower_ee_state_topic = rospy.get_param(
            "~follower_ee_state_topic",
            "/ns1/mujoco_ros/mujoco_ros_interface/ee_state",
        )

        self.follower_base_target_topic = rospy.get_param(
            "~follower_base_target_topic", "/follower/base_target"
        )
        self.follower_ee_target_topic = rospy.get_param(
            "~follower_ee_target_topic", "/follower/ee_target"
        )
        self.metric_topic = rospy.get_param("~metric_topic", "/follower/coop_metrics")

        self.leader_base = None
        self.leader_ee = None
        self.leader_object = None
        self.follower_base = None
        self.follower_ee = None
        self.last_base_target = None

        rospy.Subscriber(self.leader_base_topic, Pose2D, self.leader_base_cb)
        rospy.Subscriber(self.leader_ee_topic, Transform, self.leader_ee_cb)
        rospy.Subscriber(self.leader_object_topic, Transform, self.leader_object_cb)
        rospy.Subscriber(self.follower_base_state_topic, JointState, self.follower_base_cb)
        rospy.Subscriber(self.follower_ee_state_topic, Transform, self.follower_ee_cb)

        self.base_target_pub = rospy.Publisher(
            self.follower_base_target_topic, Pose2D, queue_size=10
        )
        self.ee_target_pub = rospy.Publisher(
            self.follower_ee_target_topic, Transform, queue_size=10
        )
        self.metric_pub = rospy.Publisher(self.metric_topic, Float32MultiArray, queue_size=10)

        rospy.loginfo(
            "follower_coop_planner side=%s lateral=%.3f base_backoff=%.3f",
            self.side,
            self.lateral_distance,
            self.base_backoff,
        )
        rospy.loginfo(
            "subscribed leader base=%s ee=%s object=%s",
            self.leader_base_topic,
            self.leader_ee_topic,
            self.leader_object_topic,
        )
        rospy.loginfo(
            "publishing follower base=%s ee=%s metrics=%s",
            self.follower_base_target_topic,
            self.follower_ee_target_topic,
            self.metric_topic,
        )

    def leader_base_cb(self, msg):
        self.leader_base = msg

    def leader_ee_cb(self, msg):
        self.leader_ee = msg

    def leader_object_cb(self, msg):
        self.leader_object = msg

    def follower_base_cb(self, msg):
        if len(msg.position) >= 3:
            self.follower_base = (msg.position[0], msg.position[1], msg.position[2])

    def follower_ee_cb(self, msg):
        self.follower_ee = msg

    def ready(self):
        if self.leader_base is None:
            return False
        if self.leader_object is None:
            return False
        if self.publish_only_when_ready and (self.follower_base is None or self.follower_ee is None):
            return False
        return True

    def compute_targets(self):
        leader_yaw = self.leader_base.theta
        side_sign = 1.0 if self.side == "left" else -1.0

        obj_x = self.leader_object.translation.x
        obj_y = self.leader_object.translation.y
        obj_z = self.leader_object.translation.z

        bx_local = -self.base_backoff
        by_local = side_sign * self.lateral_distance
        bx_world, by_world = rotate_xy(bx_local, by_local, leader_yaw)

        base_target = Pose2D()
        base_target.x = obj_x + bx_world
        base_target.y = obj_y + by_world
        base_target.theta = wrap_angle(leader_yaw)

        ex_local = self.ee_forward_offset
        ey_local = side_sign * self.ee_lateral_distance
        ex_world, ey_world = rotate_xy(ex_local, ey_local, leader_yaw)

        ee_target = Transform()
        ee_target.translation.x = obj_x + ex_world
        ee_target.translation.y = obj_y + ey_world
        ee_target.translation.z = obj_z if obj_z > 0.0 else self.ee_height
        qx, qy, qz, qw = yaw_to_quaternion(leader_yaw)
        ee_target.rotation.x = qx
        ee_target.rotation.y = qy
        ee_target.rotation.z = qz
        ee_target.rotation.w = qw

        return base_target, ee_target

    def compute_metrics(self, base_target, ee_target):
        if self.follower_base is None:
            base_err = 0.0
            yaw_err = 0.0
            base_step = 0.0
        else:
            base_err = distance2(
                self.follower_base[0],
                self.follower_base[1],
                base_target.x,
                base_target.y,
            )
            yaw_err = abs(wrap_angle(base_target.theta - self.follower_base[2]))
            if self.last_base_target is None:
                base_step = 0.0
            else:
                base_step = distance2(
                    self.last_base_target.x,
                    self.last_base_target.y,
                    base_target.x,
                    base_target.y,
                )

        if self.follower_ee is None:
            ee_err = 0.0
        else:
            ee_err = math.sqrt(
                (ee_target.translation.x - self.follower_ee.translation.x) ** 2
                + (ee_target.translation.y - self.follower_ee.translation.y) ** 2
                + (ee_target.translation.z - self.follower_ee.translation.z) ** 2
            )

        reach = distance2(base_target.x, base_target.y, ee_target.translation.x, ee_target.translation.y)
        reach_error = abs(reach - self.preferred_arm_reach)
        mu_proxy = max(0.0, 1.0 - reach_error / max(self.max_arm_reach, 1e-6))

        relation_target = (
            self.formation_distance
            if self.formation_distance > 0.0
            else math.hypot(self.base_backoff, self.lateral_distance)
        )
        relation_actual = distance2(
            self.leader_base.x, self.leader_base.y, base_target.x, base_target.y
        )
        f_proxy = math.exp(-abs(relation_actual - relation_target))

        b_proxy = math.exp(-(base_err + 0.2 * yaw_err + base_step / max(self.base_nominal_speed, 1e-6)))
        cost_proxy = mu_proxy * f_proxy * b_proxy

        return [
            mu_proxy,
            f_proxy,
            b_proxy,
            cost_proxy,
            base_err,
            yaw_err,
            ee_err,
            reach,
            base_step,
        ]

    def publish_metrics(self, metrics):
        msg = Float32MultiArray()
        msg.layout.dim.append(
            MultiArrayDimension(
                label="mu_proxy,F_proxy,B_proxy,cost_proxy,base_err,yaw_err,ee_err,reach,base_step",
                size=len(metrics),
                stride=len(metrics),
            )
        )
        msg.data = [float(x) for x in metrics]
        self.metric_pub.publish(msg)

    def publish_once(self):
        if not self.ready():
            rospy.logwarn_throttle(2.0, "waiting for leader and follower state topics")
            return

        base_target, ee_target = self.compute_targets()
        metrics = self.compute_metrics(base_target, ee_target)

        self.base_target_pub.publish(base_target)
        self.ee_target_pub.publish(ee_target)
        self.publish_metrics(metrics)
        self.last_base_target = base_target

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.publish_once()
            rate.sleep()


def main():
    rospy.init_node("follower_coop_planner")
    FollowerCoopPlanner().spin()


if __name__ == "__main__":
    main()
