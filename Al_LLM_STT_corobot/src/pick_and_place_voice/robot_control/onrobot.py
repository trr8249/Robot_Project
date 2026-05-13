#!/usr/bin/env python3

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
        # 그리퍼 넓이
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
        print("Start closing gripper.")
        result = self.client.write_registers(address=0, values=params, unit=65)

    def open_gripper(self, force_val=400):
        params = [force_val, self.max_width, 16]
        print("Start opening gripper.")
        result = self.client.write_registers(address=0, values=params, unit=65)

    # 추가한 함수
    def open_gripper2(self, force_val=400):
        params = [force_val, self.max_width//2, 16]
        print("Start opening gripper.")
        result = self.client.write_registers(address=0, values=params, unit=65)

    def move_gripper(self, width_val, force_val=400):
        params = [force_val, width_val, 16]
        print("Start moving gripper.")
        result = self.client.write_registers(address=0, values=params, unit=65)
