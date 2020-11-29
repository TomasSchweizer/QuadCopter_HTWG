import numpy as np

def rad2deg(rad):
    return rad / np.pi * 180

def deg2rad(deg):
    return deg / 180 * np.pi

def get_rot_mat(q):
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

    yaw = np.arctan2(2*(q[1]*q[2]-q[0]*q[3]), 1-2*q[1]**2 + 2*q[2]**2)
    pitch_1 = -np.arcsin(2*q[1]*q[3]+2*q[0]*q[2])
    roll_1 = np.arctan2(2*q[2]*q[3]-2*q[0]*q[1], 2*q[0]**2+2*q[3]**2-1)

    # swop roll and pitch and inverse of roll
    yaw = rad2deg(yaw)
    pitch = -rad2deg(roll_1)
    roll = rad2deg(pitch_1)

    return yaw, pitch, roll


class Quaternion:
    def __init__(self):
        self.q = np.array([1, 0, 0, 0])  # Initial state of the quaternion

