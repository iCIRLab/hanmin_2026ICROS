#!/usr/bin/env python3
"""
Automated mode verification for kimm_phri_panda_husky.

Usage (simulation must already be running):
    rosrun kimm_phri_panda_husky test_modes.py

Runs each mode sequentially, records base & EE state,
checks pass/fail criteria, and prints a summary table.
"""

import rospy
import math
import time
import sys
from std_msgs.msg import Int16
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform

# ──────────────────────────────────────────────
# Global state holders
# ──────────────────────────────────────────────
base_state = {"x": 0.0, "y": 0.0, "yaw": 0.0}
ee_state = {"x": 0.0, "y": 0.0, "z": 0.0}
state_received = {"base": False, "ee": False}


def base_cb(msg):
    base_state["x"] = msg.position[0]
    base_state["y"] = msg.position[1]
    base_state["yaw"] = msg.position[2]
    state_received["base"] = True


def ee_cb(msg):
    ee_state["x"] = msg.translation.x
    ee_state["y"] = msg.translation.y
    ee_state["z"] = msg.translation.z
    state_received["ee"] = True


def send_mode(pub, mode, settle=1.0):
    pub.publish(Int16(data=mode))
    rospy.sleep(settle)


def wait_motion(duration):
    rospy.sleep(duration)


def snap():
    """Return a snapshot of current state."""
    return {
        "base_x": base_state["x"],
        "base_y": base_state["y"],
        "base_yaw": base_state["yaw"],
        "ee_x": ee_state["x"],
        "ee_y": ee_state["y"],
        "ee_z": ee_state["z"],
    }


def angle_diff(a, b):
    """Signed shortest angle difference (a - b), result in [-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


# ──────────────────────────────────────────────
# Test definitions
# ──────────────────────────────────────────────

def test_mode_1(pub):
    """Mode 1 (home): arm should reach home posture."""
    send_mode(pub, 1)
    wait_motion(5.0)
    after = snap()
    # Just check EE is at a reasonable height (home is ~0.3-0.6m z)
    ok = 0.1 < after["ee_z"] < 1.1
    return ok, f"EE z={after['ee_z']:.3f}"


def test_mode_43(pub):
    """Mode 43 (base CW 90deg): yaw should decrease by pi/2."""
    send_mode(pub, 1); wait_motion(5.0)
    before = snap()
    send_mode(pub, 43)
    wait_motion(10.0)
    after = snap()
    dyaw = angle_diff(after["base_yaw"], before["base_yaw"])
    xy_drift = math.sqrt((after["base_x"] - before["base_x"])**2 +
                         (after["base_y"] - before["base_y"])**2)
    ok = abs(dyaw - (-math.pi / 2)) < 0.2 and xy_drift < 0.3
    return ok, f"dyaw={math.degrees(dyaw):.1f}deg (expect -90), drift={xy_drift:.3f}m"


def test_mode_44(pub):
    """Mode 44 (base CCW 90deg): yaw should increase by pi/2."""
    send_mode(pub, 1); wait_motion(5.0)
    before = snap()
    send_mode(pub, 44)
    wait_motion(10.0)
    after = snap()
    dyaw = angle_diff(after["base_yaw"], before["base_yaw"])
    xy_drift = math.sqrt((after["base_x"] - before["base_x"])**2 +
                         (after["base_y"] - before["base_y"])**2)
    ok = abs(dyaw - (math.pi / 2)) < 0.2 and xy_drift < 0.3
    return ok, f"dyaw={math.degrees(dyaw):.1f}deg (expect +90), drift={xy_drift:.3f}m"


def test_mode_61(pub):
    """Mode 61 (wholebody): EE +0.1x with mobile contributing."""
    send_mode(pub, 1); wait_motion(5.0)
    before = snap()
    send_mode(pub, 61)
    wait_motion(8.0)
    after = snap()
    dx_ee = after["ee_x"] - before["ee_x"]
    dx_base = after["base_x"] - before["base_x"]
    ok = abs(dx_ee - 0.1) < 0.05 and dx_base > 0.01
    return ok, f"EE dx={dx_ee:.3f} (expect ~0.1), base dx={dx_base:.3f} (expect >0)"


def test_mode_62(pub):
    """Mode 62 (arm-only): EE +0.1x with mobile hold."""
    send_mode(pub, 1); wait_motion(5.0)
    before = snap()
    send_mode(pub, 62)
    wait_motion(8.0)
    after = snap()
    dx_ee = after["ee_x"] - before["ee_x"]
    dx_base = after["base_x"] - before["base_x"]
    ok = abs(dx_ee - 0.1) < 0.05 and abs(dx_base) < 0.02
    return ok, f"EE dx={dx_ee:.3f} (expect ~0.1), base dx={dx_base:.3f} (expect ~0)"


# ──────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────

ALL_TESTS = [
    ("Mode  1  (home)",        test_mode_1),
    ("Mode 43  (base CW 90)",  test_mode_43),
    ("Mode 44  (base CCW90)",  test_mode_44),
    ("Mode 61  (wholebody)",   test_mode_61),
    ("Mode 62  (arm-only)",    test_mode_62),
]


def main():
    rospy.init_node("test_modes", anonymous=True)

    ctrl_pub = rospy.Publisher(
        "/ns1/mujoco_ros/mujoco_ros_interface/ctrl_type",
        Int16, queue_size=1,
    )
    rospy.Subscriber(
        "/ns1/mujoco_ros/mujoco_ros_interface/base_state",
        JointState, base_cb,
    )
    rospy.Subscriber(
        "/ns1/mujoco_ros/mujoco_ros_interface/ee_state",
        Transform, ee_cb,
    )

    rospy.sleep(2.0)  # wait for connections

    if not state_received["base"] or not state_received["ee"]:
        rospy.logwarn("No state received yet — is the simulation running?")
        rospy.sleep(3.0)

    # gravity first to ensure clean start
    send_mode(ctrl_pub, 0)
    rospy.sleep(1.0)

    results = []
    for name, fn in ALL_TESTS:
        rospy.loginfo(f"=== Running: {name} ===")
        try:
            ok, detail = fn(ctrl_pub)
        except Exception as e:
            ok, detail = False, f"EXCEPTION: {e}"
        status = "PASS" if ok else "FAIL"
        results.append((name, status, detail))
        rospy.loginfo(f"  {status}: {detail}")

        # return to gravity between tests
        send_mode(ctrl_pub, 0)
        rospy.sleep(2.0)

    # Summary
    print("\n" + "=" * 65)
    print(f"{'Test':<25} {'Result':<8} {'Detail'}")
    print("-" * 65)
    for name, status, detail in results:
        mark = "OK" if status == "PASS" else "XX"
        print(f"[{mark}] {name:<22} {detail}")
    print("=" * 65)

    passed = sum(1 for _, s, _ in results if s == "PASS")
    total = len(results)
    print(f"\n{passed}/{total} passed")

    if passed < total:
        sys.exit(1)


if __name__ == "__main__":
    main()
