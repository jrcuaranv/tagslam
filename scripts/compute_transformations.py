import numpy as np
from scipy.spatial.transform import Rotation

# for zed2i stereo camera

# Assuming zedbaselink is the body frame

t_body_imu = [-0.012, 0.037, 0.015] # from tf tree - zed wrapper
q_body_imu = [0.000, -0.001, -0.002, 1.000] # from tf tree
R_body_imu = Rotation.from_quat(q_body_imu).as_dcm()
T_body_imu = np.eye(4)
T_body_imu[:3,:3] = R_body_imu
T_body_imu[0,3] = t_body_imu[0]
T_body_imu[1,3] = t_body_imu[1]
T_body_imu[2,3] = t_body_imu[2]

# from calibration we have T_left_imu
T_left_imu = np.array([[0.009616013576831256, -0.9999348125715275, 0.0061565323384645265, 0.02158327714973142],
                [-0.0034811856354942106, -0.006190255730352645, -0.9999747807222761, 1.4064939925089906e-05],
                [0.9999477054473707, 0.009594339035973176, -0.003540484289101753, -0.024843176169213624],
                [0.0, 0.0, 0.0, 1.0]])
T_imu_left = np.linalg.inv(T_left_imu)
T_body_left = T_body_imu @ T_imu_left

T_right_left = np.array([[0.9999997330809757, -0.0004672487886207458, -0.0005617085898294015, -0.11962646471283875],
                [0.0004668943731696662, 0.9999996919714181, -0.0006309252778135881, -3.283071730608761e-05],
                [0.0005620032158788717, 0.0006306628508276783, 0.9999996432083098, -0.001315466957253254],
                [0.0, 0.0, 0.0, 1.0]])
T_left_right = np.linalg.inv(T_right_left)
T_body_right = T_body_left @ T_left_right

q_body_imu = Rotation.from_dcm(T_body_imu[:3,:3]).as_quat()
t_body_imu = T_body_imu[:3,3]

q_body_left = Rotation.from_dcm(T_body_left[:3,:3]).as_quat()
t_body_left = T_body_left[:3,3]

q_body_right = Rotation.from_dcm(T_body_right[:3,:3]).as_quat()
t_body_right = T_body_right[:3,3]

print("Translations (x,y,z), Quaternions  (qx, qy, qz, qw)")
print("t_body_imu",t_body_imu)
print("q_body_imu",q_body_imu)
print("t_body_left",t_body_left)
print("q_body_left",q_body_left)
print("t_body_right",t_body_right)
print("q_body_right",q_body_right)

print("T_body_left", T_body_left)


