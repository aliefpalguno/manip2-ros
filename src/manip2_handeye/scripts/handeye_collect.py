#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger, TriggerResponse

import tf2_ros
import tf.transformations as tr

bridge = CvBridge()

# Global buffers
latest_img_msg = None
camera_K = None
camera_D = None

# Collected samples
R_g2b_list = []   # gripper->base rotation
t_g2b_list = []   # gripper->base translation
R_t2c_list = []   # target->cam rotation
t_t2c_list = []   # target->cam translation


def cam_info_cb(msg):
    global camera_K, camera_D
    if camera_K is None:
        camera_K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        camera_D = np.array(msg.D, dtype=np.float64)
        rospy.loginfo("Camera intrinsics received.")


def image_cb(msg):
    global latest_img_msg
    latest_img_msg = msg


def transform_to_matrix(transform):
    t = transform.transform.translation
    q = transform.transform.rotation

    T = np.dot(
        tr.translation_matrix((t.x, t.y, t.z)),
        tr.quaternion_matrix((q.x, q.y, q.z, q.w))
    )
    return T


def capture_cb(req):
    global R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list

    if latest_img_msg is None or camera_K is None:
        return TriggerResponse(
            success=False,
            message="No image or camera info yet."
        )

    # Get parameters
    base_frame  = rospy.get_param("~base_frame", "base_link")
    ee_frame    = rospy.get_param("~ee_frame", "ee_link")
    marker_id   = rospy.get_param("~marker_id", 0)
    marker_len  = rospy.get_param("~marker_length", 0.10)  # meters
    dict_name   = rospy.get_param("~aruco_dict", "DICT_4X4_100")

    # Convert image to OpenCV
    try:
        cv_img = bridge.imgmsg_to_cv2(latest_img_msg, desired_encoding="bgr8")
    except Exception as e:
        return TriggerResponse(success=False, message="cv_bridge error: %s" % e)

    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

    # Prepare ArUco
    if not hasattr(cv2.aruco, dict_name):
        return TriggerResponse(success=False, message="Invalid ArUco dict: %s" % dict_name)
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    params = cv2.aruco.DetectorParameters_create()

    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

    if ids is None or len(ids) == 0:
        return TriggerResponse(success=False, message="No markers detected.")

    # Find our marker_id
    rvec = None
    tvec = None
    for c, mid in zip(corners, ids.flatten()):
        if int(mid) == int(marker_id):
            rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(
                c, marker_len, camera_K, camera_D
            )
            rvec = rvecs[0]
            tvec = tvecs[0]
            break

    if rvec is None:
        return TriggerResponse(success=False, message="Marker ID %d not found." % marker_id)

    # Convert target->cam
    R_t2c, _ = cv2.Rodrigues(rvec)
    t_t2c = tvec.reshape(3, 1)

    # Get TF base->ee, then invert to gripper->base
    try:
        tf_buffer = capture_cb.tf_buffer  # attached below
    except AttributeError:
        return TriggerResponse(success=False, message="TF buffer not attached.")

    try:
        trans = tf_buffer.lookup_transform(
            base_frame, ee_frame, rospy.Time(0), rospy.Duration(1.0)
        )
    except Exception as e:
        return TriggerResponse(success=False, message="TF lookup error: %s" % e)

    T_b_e = transform_to_matrix(trans)
    T_e_b = np.linalg.inv(T_b_e)

    R_g2b = T_e_b[:3, :3]
    t_g2b = T_e_b[:3, 3].reshape(3, 1)

    # Store samples
    R_g2b_list.append(R_g2b)
    t_g2b_list.append(t_g2b)
    R_t2c_list.append(R_t2c)
    t_t2c_list.append(t_t2c)

    msg = "Captured sample #%d" % len(R_g2b_list)
    rospy.loginfo(msg)
    return TriggerResponse(success=True, message=msg)


def save_cb(req):
    if len(R_g2b_list) < 5:
        return TriggerResponse(
            success=False,
            message="Too few samples (%d). Capture more." % len(R_g2b_list)
        )

    output_path = rospy.get_param("~output_file", "~/handeye_samples.npz")
    output_path = os.path.expanduser(output_path)

    np.savez(
        output_path,
        R_g2b=np.array(R_g2b_list),
        t_g2b=np.array(t_g2b_list),
        R_t2c=np.array(R_t2c_list),
        t_t2c=np.array(t_t2c_list),
    )

    msg = "Saved %d samples to %s" % (len(R_g2b_list), output_path)
    rospy.loginfo(msg)
    return TriggerResponse(success=True, message=msg)


def main():
    rospy.init_node("handeye_collect")

    # Subscribers
    rospy.Subscriber("image", Image, image_cb, queue_size=1)
    rospy.Subscriber("camera_info", CameraInfo, cam_info_cb, queue_size=1)

    # TF
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Hack to access buffer inside service
    capture_cb.tf_buffer = tf_buffer

    # Services
    rospy.Service("capture", Trigger, capture_cb)
    rospy.Service("save", Trigger, save_cb)

    rospy.loginfo("handeye_collect node ready.")
    rospy.spin()


if __name__ == "__main__":
    main()
