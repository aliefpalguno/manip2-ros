#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
manip2_matlab_bridge.py

- No direct access to Dynamixel.
- Bridges MATLAB <-> /manip2/command_deg.
- Feeds MATLAB with /ros/current_joint_state_deg based on /joint_states.
- Respects /manip2/lockout (safety takeover).
"""

import math, ast
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse

# ---------- param helpers ----------
def _ensure_list(obj):
    if isinstance(obj, list):
        return obj
    if isinstance(obj, tuple):
        return list(obj)
    if isinstance(obj, str):
        try:
            return ast.literal_eval(obj)
        except Exception:
            pass
    return [obj]

def _to_list_floats(obj):
    return [float(x) for x in _ensure_list(obj)]

def _to_list2_floats(obj):
    m = _ensure_list(obj)
    out = []
    for row in m:
        out.append([float(x) for x in _ensure_list(row)])
    return out

DEFAULT_DEG_LIMITS = [[-90,90],[-30,36],[-105,0],[-75,75],[-90,90]]

class Manip2MatlabBridge:
    def __init__(self):
        self.deg_limits = _to_list2_floats(
            rospy.get_param("~urdf_limits_deg", DEFAULT_DEG_LIMITS)
        )
        self.agent_enabled = True

        # Publishers
        self.cmd_pub = rospy.Publisher(
            "/manip2/command_deg", Float64MultiArray, queue_size=1
        )
        self.feedback_pub = rospy.Publisher(
            "/ros/current_joint_state_deg", Float64MultiArray, queue_size=1
        )
        self.agent_state_pub = rospy.Publisher(
            "/matlab_perceived/joint_states", JointState, queue_size=1
        )

        # Subscribers
        rospy.Subscriber(
            "/matlab/joint_command_deg",
            Float64MultiArray,
            self.on_agent_command,
            queue_size=1,
        )
        rospy.Subscriber(
            "/joint_states",
            JointState,
            self.on_joint_state,
            queue_size=10,
        )
        rospy.Subscriber(
            "/matlab/perceived_joint_state",
            Float64MultiArray,
            self.on_perceived_state,
            queue_size=1,
        )

        # Services
        self.srv_agent_enable = rospy.Service(
            "/manip2/agent_enable", Trigger, self._srv_agent_enable
        )
        self.srv_agent_disable = rospy.Service(
            "/manip2/agent_disable", Trigger, self._srv_agent_disable
        )
        self.srv_target_reached = rospy.Service(
            "/manip2/target_reached", Trigger, self._srv_target_reached
        )

        rospy.loginfo("manip2_matlab_bridge running (no direct DXL access).")

    # ========== Callbacks ==========

    def on_agent_command(self, msg: Float64MultiArray):
        """Forward RL action to /manip2/command_deg if allowed."""
        if not self.agent_enabled:
            rospy.logwarn_throttle(
                1.0, "Agent disabled; ignoring /matlab/joint_command_deg."
            )
            return

        if rospy.get_param("/manip2/lockout", False):
            rospy.logwarn_throttle(
                1.0, "Safety lockout active; ignoring /matlab/joint_command_deg."
            )
            return

        vals = _to_list_floats(msg.data)
        if len(vals) != 5:
            rospy.logwarn("Expected 5 joints, got %d", len(vals))
            return

        # Optional: clip to URDF limits (hardware driver also clips, so this is extra padding)
        for i in range(min(5, len(self.deg_limits))):
            lo, hi = self.deg_limits[i]
            if vals[i] < lo or vals[i] > hi:
                rospy.logwarn(
                    "Joint j%d %.1f° outside [%.1f, %.1f], clipping.",
                    i + 1, vals[i], lo, hi,
                )
                vals[i] = min(max(vals[i], lo), hi)

        out = Float64MultiArray()
        out.data = vals
        self.cmd_pub.publish(out)

    def on_joint_state(self, msg: JointState):
        """Convert /joint_states (rad) → /ros/current_joint_state_deg for MATLAB."""
        if not msg.position:
            return
        deg = [math.degrees(p) for p in msg.position]
        out = Float64MultiArray()
        out.data = deg
        self.feedback_pub.publish(out)

    def on_perceived_state(self, msg: Float64MultiArray):
        """Forward agent's perceived joint state (deg) → JointState for RViz."""
        vals = _to_list_floats(msg.data)
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = ["joint1", "joint2", "joint3", "joint4", "joint5"]
        js.position = [math.radians(x) for x in vals]
        self.agent_state_pub.publish(js)

    # ========== Services ==========

    def _srv_agent_enable(self, _req):
        self.agent_enabled = True
        return TriggerResponse(success=True, message="Agent control enabled.")

    def _srv_agent_disable(self, _req):
        self.agent_enabled = False
        return TriggerResponse(success=True, message="Agent control disabled.")

    def _srv_target_reached(self, _req):
        # MATLAB calls this just to be polite.
        rospy.loginfo("Target reached acknowledged (from MATLAB).")
        return TriggerResponse(success=True, message="OK")

def main():
    rospy.init_node("manip2_matlab_bridge", anonymous=False)
    node = Manip2MatlabBridge()
    try:
        rospy.spin()
    finally:
        pass

if __name__ == "__main__":
    main()