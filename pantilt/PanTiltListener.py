#!/usr/bin/env python3
import math
import math
import dynamixel_sdk as dxl
import time
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize

from dataclasses import dataclass
from cyclonedds.idl import IdlStruct

YAW_NEUTRAL_OFFSET = 1.5
PITCH_NEUTRAL_OFFSET = 3.2

YAW_MIN = math.radians(-70)   # Left - 1.22
YAW_MAX = math.radians(70)    # Right + 1.22
PITCH_MIN = math.radians(-20)    # Down - 0.35
PITCH_MAX = math.radians(65)     # Up + 1.14

ADDR_PROFILE_VELOCITY = 112
ADDR_PROFILE_ACCELERATION = 108

PROFILE_VELOCITY = 50     # 0-1023 (experiment with values, lower = slower)
PROFILE_ACCELERATION = 10 # 0-32767 (experiment, lower = smoother)


# This class defines user data consisting of a float data and a string data
@dataclass
class PanTiltData(IdlStruct, typename="PanTiltData"):
    yaw: float
    pitch: float


class DynamixelHandler:
    def __init__(self, port, baud=57600, ids={'yaw': 0, 'pitch': 1}):
        self.ids = ids
        self.portHandler = dxl.PortHandler(port)
        self.packetHandler = dxl.PacketHandler(2.0)
        if not self.portHandler.openPort():
            raise RuntimeError("Failed to open port")

        if not self.portHandler.setBaudRate(baud):
            raise RuntimeError("Failed to set baudrate")

        # Set velocity and acceleration profile for each joint
        for dxl_id in self.ids.values():
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY)
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION)

        # Enable torque for each joint
        ADDR_TORQUE_ENABLE = 64
        for dxl_id in self.ids.values():
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
        self.joint_pos = {'yaw': 0.0, 'pitch': 0.0}

    def radians_to_dxl(self, rad):
        # XL330: 0–4095 maps to 0–360°
        return int((rad % (2 * 3.14159)) * 4095 / (2 * 3.14159))

    def dxl_to_radians(self, pos):
        return float(pos) * (2 * 3.14159) / 4095

    def set_joint(self, joint_name, rad):
        dxl_pos = self.radians_to_dxl(rad)
        dxl_id = self.ids[joint_name]
        ADDR_GOAL_POSITION = 116
        self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_POSITION, dxl_pos)
        self.joint_pos[joint_name] = rad

    def get_joint(self, joint_name):
        dxl_id = self.ids[joint_name]
        ADDR_PRESENT_POSITION = 132
        dxl_pos, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
        return self.dxl_to_radians(dxl_pos)

    def disable_torque(self):
        ADDR_TORQUE_ENABLE = 64
        for dxl_id in self.ids.values():
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)

class PanTiltControllerListener:
    """
    listens for the 2d pan tilt vector from the publisher
    listener runs on the robot and communicates with the dynamixel motors
    publisher runs on the avp computer and sends the yaw and pitch angles to the dynamixel motors
    """
    #TODO maybe use self.sub.close() to close the channel
    def __init__(self, dev_path, channel_init = True):
        if channel_init:
            ChannelFactoryInitialize(0)
        self.dxl = DynamixelHandler(dev_path, 57600, {'yaw': 0, 'pitch': 1})
        self.joint_names = ['yaw', 'pitch']
        self.sub = ChannelSubscriber("pantilt", PanTiltData)
        self.sub.Init()

        self.cur_yaw_cmd = 0
        self.cur_pitch_cmd = 0

    def read_cmd(self):
        # print("[INFO] Waiting for message...")
        # print("\n1")
        print("")
        msg = self.sub.Read(timeout = 0.1)
        # print("2")
        
        # print(f"[INFO] Received message: {msg}")
        if msg is None:
            # print("No message received")
            return
        self.cur_yaw_cmd = msg.yaw
        self.cur_pitch_cmd = msg.pitch


    def ctrl(self):
        # print(f"[INFO] yaw, pitch: {self.cur_yaw_cmd}, {self.cur_pitch_cmd}")

        clamped_yaw = max(min(self.cur_yaw_cmd, YAW_MAX), YAW_MIN)
        clamped_pitch = max(min(self.cur_pitch_cmd, PITCH_MAX), PITCH_MIN)
        # print(f"[INFO] Clamped logical yaw, pitch: {clamped_yaw}, {clamped_pitch}")

        yaw_cmd = clamped_yaw + YAW_NEUTRAL_OFFSET
        pitch_cmd = clamped_pitch + PITCH_NEUTRAL_OFFSET

        self.dxl.set_joint('yaw', yaw_cmd)
        self.dxl.set_joint('pitch', pitch_cmd)

    def home(self):
        self.cur_yaw_cmd = 0
        self.cur_pitch_cmd = 0




if __name__ == "__main__":
    #Test the PanTiltControllerPublisher and PanTiltControllerListener
    #ChannelFactoryInitialize()
    print("Starting PanTiltControllerListener")
    listener = PanTiltControllerListener(dev_path = '/dev/ttyUSB2', channel_init = True)
    print("PanTiltControllerListener started")
    start_time = time.time()
    print("Starting listener loop")
    listener.home()
    time.sleep(0.1)
    try:
        while True:
            listener.read_cmd()
            listener.ctrl()
            print(f"\r Running for {time.time() - start_time:.2f} seconds", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt as k:
        pass