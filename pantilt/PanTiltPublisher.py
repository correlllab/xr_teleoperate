#!/usr/bin/env python3

import math
import math
import time
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize

from dataclasses import dataclass
from cyclonedds.idl import IdlStruct


# This class defines user data consisting of a float data and a string data
@dataclass
class PanTiltData(IdlStruct, typename="PanTiltData"):
    yaw: float
    pitch: float

class PanTiltControllerPublisher:
    """
    Publisher for the 2d pan tilt vector
    runs on avp computer and communicates with the listener on the robot
    consumers head_rmat and publishes the yaw and pitch angles
    """
    #TODO maybe use self.pub.close() to close the channel
    def __init__(self, channel_init = True):
        if channel_init:
            ChannelFactoryInitialize(0)
        self.pub = ChannelPublisher("pantilt", PanTiltData)
        self.pub.Init()

    def publish(self, head_rmat):
        _, desired_pitch, desired_yaw = matrix_to_rpy(head_rmat)
        msg = PanTiltData(desired_yaw, desired_pitch)
        msg.yaw = desired_yaw
        msg.pitch = desired_pitch
        # Publish message
        if self.pub.Write(msg, 0.5):
            print("Publish success. msg:", msg)
        else:
            print("Waitting for subscriber.")


#TODO could be wrong
def rpy_to_matrix(roll, pitch, yaw):
    """
    Create a rotation matrix from roll, pitch, yaw (in radians).
    Returns a 3x3 nested list.
    """
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    # ZYX rotation: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    rmat = [
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,             cp*cr]
    ]
    return rmat

#TODO could be wrong
def matrix_to_rpy(rmat):
    """
    Convert a 3x3 rotation matrix to roll, pitch, yaw (in radians).
    Returns (roll, pitch, yaw).
    """
    # ZYX convention
    if abs(rmat[2][0]) < 1.0:
        pitch = -math.asin(rmat[2][0])
        roll = math.atan2(rmat[2][1], rmat[2][2])
        yaw = math.atan2(rmat[1][0], rmat[0][0])
    else:
        # Gimbal lock
        pitch = math.pi/2 if rmat[2][0] <= -1 else -math.pi/2
        roll = math.atan2(-rmat[0][1], rmat[1][1])
        yaw = 0.0
    return roll, pitch, yaw

if __name__ == "__main__":
    #Test the PanTiltControllerPublisher and PanTiltControllerListener
    publisher = PanTiltControllerPublisher(channel_init = True)


    test_values = [
        (0, 0),
        (-1.5, -1),
        (1.5, -1),
        (-1.5, 1.5),
        (1.5, 1.5)
    ]

    for _ in range(10):
        for yaw, pitch in test_values:
            head_rmat = rpy_to_matrix(0, pitch, yaw)
            publisher.publish(head_rmat)
            # listener.ctrl()
            time.sleep(3)
