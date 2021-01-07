import numpy as np
import transforms3d as tf3d


def rad2deg(rad):
    return rad / np.pi * 180


if __name__ == '__main__':

    q = np.array([-0.9824, -0.03, -0.05, -0.176])

    rot_axis_and_angles = tf3d.quaternions.quat2axangle(q)

    rot_axis = rot_axis_and_angles[0]
    rot_angle = rot_axis_and_angles[1]

    roll = rad2deg(rot_axis[0]*rot_angle)
    pitch = rad2deg(rot_axis[1]*rot_angle)
    yaw = rad2deg(rot_axis[2]*rot_angle)

    print("Roll: %f" %roll)
    print("Pitch: %f" %pitch)
    print("Yaw: %f" %yaw)