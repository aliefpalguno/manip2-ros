#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
import cv2
import tf.transformations as tr

# 1) Load the samples
path = os.path.expanduser("~/handeye_samples.npz")
print("Loading:", path)
data = np.load(path, allow_pickle=True)

R_g2b = data["R_g2b"]   # shape (N,3,3)
t_g2b = data["t_g2b"]   # shape (N,3,1) or (N,3)
R_t2c = data["R_t2c"]   # shape (N,3,3)
t_t2c = data["t_t2c"]   # shape (N,3,1) or (N,3)

N = R_g2b.shape[0]
print("Loaded %d samples" % N)

# 2) Convert to lists as OpenCV expects
R_gripper2base = []
t_gripper2base = []
R_target2cam = []
t_target2cam = []

for i in range(N):
    R_gripper2base.append(R_g2b[i])
    t_gripper2base.append(t_g2b[i].reshape(3, 1))

    R_target2cam.append(R_t2c[i])
    t_target2cam.append(t_t2c[i].reshape(3, 1))

# 3) Solve hand-eye: camera -> gripper (cam->end_effector)
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base, t_gripper2base,
    R_target2cam, t_target2cam,
    method=cv2.CALIB_HAND_EYE_TSAI
)

print("\nResult: camera -> gripper (end_effector)")
print("R_cam2gripper =\n", R_cam2gripper)
print("t_cam2gripper (m) =", t_cam2gripper.reshape(3))

# 4) Build homogeneous transform
T_c_e = np.eye(4)
T_c_e[:3, :3] = R_cam2gripper
T_c_e[:3, 3] = t_cam2gripper.reshape(3)

# 5) Invert to get gripper -> camera (end_effector -> camera)
T_e_c = np.linalg.inv(T_c_e)
R_e_c = T_e_c[:3, :3]
t_e_c = T_e_c[:3, 3]

print("\nTransform end_effector -> camera:")
print("R_e_c =\n", R_e_c)
print("t_e_c (m) =", t_e_c)

# 6) Convert to RPY (for URDF or static transform)
roll, pitch, yaw = tr.euler_from_matrix(T_e_c)

print("\nURDF origin suggestion (end_effector -> camera):")
print('xyz="%.6f %.6f %.6f"' % (t_e_c[0], t_e_c[1], t_e_c[2]))
print('rpy="%.6f %.6f %.6f"' % (roll, pitch, yaw))
