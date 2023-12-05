'''
This file is provided by TA Juan (2022).
It is used to control the gripper (end effector) on the robot.
'''

import serial
import binascii
import time

class Gripper():
    def __init__(self):
        packet = bytearray()
        packet.append(0x09)
        packet.append(0x10)
        packet.append(0x03)
        packet.append(0xE8)
        packet.append(0x00)
        packet.append(0x03)
        packet.append(0x06)
        packet.append(0x00)
        packet.append(0x00)
        packet.append(0x00)
        packet.append(0x00)
        packet.append(0x00)
        packet.append(0x00)
        packet.append(0x73)
        packet.append(0x30)

        packet2 = bytearray()
        packet2.append(0x09)
        packet2.append(0x03)
        packet2.append(0x07)
        packet2.append(0xD0)
        packet2.append(0x00)
        packet2.append(0x01)
        packet2.append(0x85)
        packet2.append(0xCF)

        ClsPacket = bytearray()
        ClsPacket.append(0x09)
        ClsPacket.append(0x10)
        ClsPacket.append(0x03)
        ClsPacket.append(0xE8)
        ClsPacket.append(0x00)
        ClsPacket.append(0x03)
        ClsPacket.append(0x06)
        ClsPacket.append(0x09)
        ClsPacket.append(0x00)
        ClsPacket.append(0x00)
        ClsPacket.append(0xFF)
        ClsPacket.append(0xFF)
        ClsPacket.append(0xFF)
        ClsPacket.append(0x42)
        ClsPacket.append(0x29)

        OpnPacket = bytearray()
        OpnPacket.append(0x09)
        OpnPacket.append(0x10)
        OpnPacket.append(0x03)
        OpnPacket.append(0xE8)
        OpnPacket.append(0x00)
        OpnPacket.append(0x03)
        OpnPacket.append(0x06)
        OpnPacket.append(0x09)
        OpnPacket.append(0x00)
        OpnPacket.append(0x00)
        OpnPacket.append(0x00)
        OpnPacket.append(0xFF)
        OpnPacket.append(0xFF)
        OpnPacket.append(0x72)
        OpnPacket.append(0x19)

        self.initCode1 = packet
        self.initCode2 = packet2

        self.closePkt = ClsPacket
        self.openPkt = OpnPacket

        self.serPort = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=1, parity=serial.PARITY_NONE, stopbits=1, bytesize=8)

    def gripper_init(self):
        self.serPort.write(self.initCode1)
        data_raw = self.serPort.readline()
        print(data_raw)
        data = binascii.hexlify(data_raw)
        print("Response 1", data)
        time.sleep(0.01)

        self.serPort.write(self.initCode2)
        data_raw = self.serPort.readline()
        print(data_raw)
        data = binascii.hexlify(data_raw)
        print("Response 2", data)
        time.sleep(1)

    def OpenGripper(self):
        print("Open gripper")
        self.serPort.write(self.openPkt)
        data_raw = self.serPort.readline()
        print(data_raw)
        data = binascii.hexlify(data_raw)
        print("Response 3", data)
        time.sleep(2)

    def CloseGripper(self):
        print("Close gripper")
        self.serPort.write(self.closePkt)
        data_raw = self.serPort.readline()
        print(data_raw)
        data = binascii.hexlify(data_raw)
        print("Response 3", data)
        time.sleep(2)