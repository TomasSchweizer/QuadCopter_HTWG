import numpy as np
import transforms3d as tf3d

def rad2deg(rad):
    return rad / np.pi * 180

def deg2rad(deg):
    return deg / 180 * np.pi

def get_rot_mat(q):

    '''
    c00 = q[0] ** 2 + q[1] ** 2 - q[2] ** 2 - q[3] ** 2
    c01 = 2 * (q[1] * q[2] - q[0] * q[3])
    c02 = 2 * (q[1] * q[3] + q[0] * q[2])
    c10 = 2 * (q[1] * q[2] + q[0] * q[3])
    c11 = q[0] ** 2 - q[1] ** 2 + q[2] ** 2 - q[3] ** 2
    c12 = 2 * (q[2] * q[3] - q[0] * q[1])
    c20 = 2 * (q[1] * q[3] - q[0] * q[2])
    c21 = 2 * (q[2] * q[3] + q[0] * q[1])
    c22 = q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2

    rot_mat = np.array([[c00, c01, c02, 0], [c10, c11, c12, 0], [c20, c21, c22, 0], [0, 0, 0, 1]], 'f')
    return rot_mat
    '''

    rot_mat_3x3 = tf3d.quaternions.quat2mat(q)
    rot_mat_4x3 = np.append(rot_mat_3x3, [[0.0, 0.0, 0.0]], axis=0)
    rot_mat_4x4 = np.append(rot_mat_4x3, [[0.0], [0.0], [0.0], [1.0]], axis=1)

    return rot_mat_4x4


def get_euler_angles(q):

    '''

        change later back to rotation matrix or correct quaternion to euler angles
    '''

    '''
    m_rot_gl = get_rot_mat(q)
    m = m_rot_gl[:3, :3]
    print(m)
    test = -m[2, 0]
    if test > 0.99999:
        yaw = 0
        pitch = np.pi / 2
        roll = np.arctan2(m[0, 1], m[0, 2])
    elif test < -0.99999:
        yaw = 0
        pitch = -np.pi / 2
        roll = np.arctan2(-m[0, 1], -m[0, 2])
    else:
        yaw = np.arctan2(m[1, 0], m[0, 0])
        pitch = np.arcsin(-m[2, 0])
        roll = np.arctan2(m[2, 1], m[2, 2])

    yaw = rad2deg(yaw)
    pitch = rad2deg(pitch)
    roll = rad2deg(roll)

    return yaw, pitch, roll
    '''

    '''
    
    a12 = 2.0 * (q[1] * q[2] + q[0] * q[3])
    a22 = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]
    a31 = 2.0 * (q[0] * q[1] + q[2] * q[3])
    a32 = 2.0 * (q[1] * q[3] - q[0] * q[2])
    a33 = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
    pitch = -np.arcsin(a32)
    roll = np.arctan2(a31, a33)
    yaw = np.arctan2(a12, a22)
    pitch *= 180.0 / np.pi
    yaw *= 180.0 / np.pi
    yaw += 2.56 #Declination
    if yaw < 0.0:
        yaw += 360.0 #Ensure yaw stays between 0 and 360
    roll *= 180.0 / np.pi

    return yaw, pitch, roll
    '''


    rot_axis_and_angles = tf3d.quaternions.quat2axangle(q)

    rot_axis = rot_axis_and_angles[0]
    rot_angle = rot_axis_and_angles[1]

    roll = rad2deg(rot_axis[0]*rot_angle)
    pitch = rad2deg(rot_axis[1]*rot_angle)
    yaw = rad2deg(rot_axis[2]*rot_angle)

    return roll, pitch, yaw

class Quaternion:
    def __init__(self):
        self.q = np.array([1, 0, 0, 0])  # Initial state of the quaternion

