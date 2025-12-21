#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import rospy

from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray

import tf2_ros
from trac_ik_python.trac_ik import IK


def rad2deg(x): return x * 180.0 / math.pi


class Manip2IKBridgeTRACIK:
    def __init__(self):
        # --- params ---
        self.base_link = rospy.get_param("~base_link", "base_link")
        self.tip_link  = rospy.get_param("~tip_link",  "end_effector")

        # Your /user/target_position is in cm (from manip2_command_gateway.py)
        self.scale = float(rospy.get_param("~scale", 0.01))  # cm -> m

        # Bounds: if you “don’t measure orientation”, allow it to float
        # Small XYZ bound means “hit position tightly”
        self.pos_bound_m = float(rospy.get_param("~pos_bound_m", 0.005))  # 5mm
        # Big rotation bounds means “orientation not important”
        self.rot_bound_rad = float(rospy.get_param("~rot_bound_rad", math.pi))

        self.tf_timeout_s = float(rospy.get_param("~tf_timeout_s", 0.05))
        self.min_interval_ms = float(rospy.get_param("~min_interval_ms", 50.0))  # simple spam limiter

        # --- TF (to get current EE orientation) ---
        self.tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        # --- TRAC-IK init ---
        # By default TRAC-IK reads URDF from /robot_description.
        # (You can also pass urdf_string=... per docs)
        self.ik = IK(self.base_link, self.tip_link)

        rospy.loginfo("[IK/TRAC] chain=%s->%s joints=%s",
                      self.base_link, self.tip_link, list(self.ik.joint_names))

        # --- ROS I/O ---
        self.pub_cmd = rospy.Publisher("/manip2/command_deg", Float64MultiArray, queue_size=1)
        self.sub_target = rospy.Subscriber("/user/target_position", Point, self.on_target, queue_size=1)

        self._last_cmd_time = 0.0

    def _get_current_orientation(self):
        """
        Returns (qx,qy,qz,qw). If TF fails, returns identity quaternion.
        """
        try:
            tf = self.tf_buf.lookup_transform(
                self.base_link,
                self.tip_link,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout_s)
            )
            q = tf.transform.rotation
            return (q.x, q.y, q.z, q.w)
        except Exception as e:
            rospy.logwarn_throttle(1.0, "[IK/TRAC] TF lookup failed (%s). Using identity quat.", str(e))
            return (0.0, 0.0, 0.0, 1.0)

    def on_target(self, msg: Point):
        now = time.time()
        if (now - self._last_cmd_time) * 1000.0 < self.min_interval_ms:
            return
        self._last_cmd_time = now

        # Convert cm -> m
        x = float(msg.x) * self.scale
        y = float(msg.y) * self.scale
        z = float(msg.z) * self.scale

        # Seed state: zeros is “fine”, but current joint state seed is better.
        # TRAC-IK python wrapper exposes number_of_joints; seed must match length. :contentReference[oaicite:2]{index=2}
        seed = [0.0] * self.ik.number_of_joints

        qx, qy, qz, qw = self._get_current_orientation()

        # Call TRAC-IK
        # get_ik(seed, x,y,z, qx,qy,qz,qw, bx,by,bz, brx,bry,brz) :contentReference[oaicite:3]{index=3}
        sol = self.ik.get_ik(
            seed,
            x, y, z,
            qx, qy, qz, qw,
            self.pos_bound_m, self.pos_bound_m, self.pos_bound_m,
            self.rot_bound_rad, self.rot_bound_rad, self.rot_bound_rad
        )

        if sol is None:
            rospy.logwarn("[IK/TRAC] IK failed for target(m)=%.3f %.3f %.3f", x, y, z)
            return

        # TRAC-IK returns radians. Convert to degrees for manip2_dxl_bridge_telem.cpp
        degs = [rad2deg(v) for v in sol]

        if len(degs) != 5:
            rospy.logwarn("[IK/TRAC] Expected 5 joints, got %d. joints=%s", len(degs), list(self.ik.joint_names))
            return

        out = Float64MultiArray(data=degs)
        self.pub_cmd.publish(out)

        rospy.loginfo("[IK/TRAC] OK target(m)=%.3f %.3f %.3f -> deg=%s",
                      x, y, z, ["%.1f" % d for d in degs])


if __name__ == "__main__":
    rospy.init_node("manip2_ik_bridge", anonymous=False)
    Manip2IKBridgeTRACIK()
    rospy.loginfo("[IK/TRAC] Ready. sub=/user/target_position pub=/manip2/command_deg")
    rospy.spin()
