#!/usr/bin/env python3

# ========================== Gripper Class ==========================
from pymodbus.client.sync import ModbusTcpClient as ModbusClient

class RG():
    def __init__(self, gripper, ip, port):
        self.client = ModbusClient(
            ip,
            port=port,
            stopbits=1,
            bytesize=8,
            parity='E',
            baudrate=115200,
            timeout=1)
        if gripper not in ['rg2', 'rg6']:
            print("Please specify either rg2 or rg6.")
            return
        self.gripper = gripper
        if self.gripper == 'rg2':
            self.max_width = 500
            self.max_force = 400
        elif self.gripper == 'rg6':
            self.max_width = 1600
            self.max_force = 1200
        self.open_connection()

    def open_connection(self):
        self.client.connect()

    def close_connection(self):
        self.client.close()

    def get_fingertip_offset(self):
        result = self.client.read_holding_registers(address=258, count=1, unit=65)
        offset_mm = result.registers[0] / 10.0
        return offset_mm

    def get_width(self):
        result = self.client.read_holding_registers(address=267, count=1, unit=65)
        width_mm = result.registers[0] / 10.0
        return width_mm

    def get_status(self):
        result = self.client.read_holding_registers(address=268, count=1, unit=65)
        status = format(result.registers[0], '016b')
        status_list = [0] * 7
        if int(status[-1]):
            print("A motion is ongoing so new commands are not accepted.")
            status_list[0] = 1
        if int(status[-2]):
            print("An internal- or external grip is detected.")
            status_list[1] = 1
        if int(status[-3]):
            print("Safety switch 1 is pushed.")
            status_list[2] = 1
        if int(status[-4]):
            print("Safety circuit 1 is activated so it will not move.")
            status_list[3] = 1
        if int(status[-5]):
            print("Safety switch 2 is pushed.")
            status_list[4] = 1
        if int(status[-6]):
            print("Safety circuit 2 is activated so it will not move.")
            status_list[5] = 1
        if int(status[-7]):
            print("Any of the safety switch is pushed.")
            status_list[6] = 1
        return status_list

    def get_width_with_offset(self):
        result = self.client.read_holding_registers(address=275, count=1, unit=65)
        width_mm = result.registers[0] / 10.0
        return width_mm

    def set_control_mode(self, command):
        result = self.client.write_register(address=2, value=command, unit=65)

    def set_target_force(self, force_val):
        result = self.client.write_register(address=0, value=force_val, unit=65)

    def set_target_width(self, width_val):
        result = self.client.write_register(address=1, value=width_val, unit=65)

    def close_gripper(self, force_val=400):
        params = [force_val, 0, 16]
        mwait(2)
        print("Start closing gripper.")
        result = self.client.write_registers(address=0, values=params, unit=65)

    def open_gripper(self, force_val=400):
        params = [force_val, self.max_width, 16]
        print("Start opening gripper.")
        result = self.client.write_registers(address=0, values=params, unit=65)

    def open_gripper2(self, force_val=400):
        params = [force_val, self.max_width//2, 16]
        print("Start opening gripper.")
        result = self.client.write_registers(address=0, values=params, unit=65)

    def move_gripper(self, width_val, force_val=400):
        params = [force_val, width_val, 16]
        print("Start moving gripper.")
        result = self.client.write_registers(address=0, values=params, unit=65)

# ========================== Robot Control Class ==========================

import sys
import os
import time
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init

from od_msg.srv import SrvDepthPosition
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

# Constants
package_path = get_package_share_directory("pick_and_place_voice")
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0

# Initialize Doosan
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
rclpy.init()
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans
    from DSR_ROBOT2 import posx, DR_BASE, DR_MV_MOD_REL, check_force_condition, DR_TOOL, DR_AXIS_Z, task_compliance_ctrl, get_current_posj
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

# Gripper Setup
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

# RobotController Node
class RobotController(Node):
    def __init__(self):
        super().__init__("greep_control")
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
  
        '''
        movel(posx(0.0, 0.0, -30.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        time.sleep(2)
        gripper.close_gripper()
        time.sleep(3)
        movel(posx(0.0, 0.0, 100.0, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
        mwait()
        '''

        self.get_position_client = self.create_client(SrvDepthPosition, "/get_3d_position")
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")
        self.get_position_request = SrvDepthPosition.Request()

        self.get_keyword_client = self.create_client(Trigger, "/get_keyword")
        while not self.get_keyword_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_keyword service...")
        self.get_keyword_request = Trigger.Request()
        # 이동 완료 후 현재 좌표 출력
        # curr_pos = get_current_posx()[0]
        # print(f"\n✅ Reached target position:")
        # print(f"X: {curr_pos[0]:.2f} mm")
        # print(f"Y: {curr_pos[1]:.2f} mm")
        # print(f"Z: {curr_pos[2]:.2f} mm")
        # print(f"RX: {curr_pos[3]:.2f} deg")
        # print(f"RY: {curr_pos[4]:.2f} deg")
        # print(f"RZ: {curr_pos[5]:.2f} deg\n")
    
def main(args=None):
    node = RobotController()
    while rclpy.ok():
        node.robot_control()
    rclpy.shutdown()
    node.destroy_node()

if __name__ == "__main__":
    main()


