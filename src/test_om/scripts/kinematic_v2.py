#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys

sys.path.append("/home/codybui/om_ros/src/test_om/scripts")
import AZDKDDirectMessageManager
import AZDKDParameter
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
from geometry_msgs.msg import Twist
import math
import time

ROBOT_WHEEL_SEPARATION = 0.5
ROBOT_WHEEL_RADIUS = 0.1
ROBOT_MOTOR_SPR = 18000

global Vl, Vr

class RS485:
    def __init__(self, port = '/dev/ttyUSB0', is_rtu="rtu", ip=None):
        self.port = port
        self.ip = ip
        self.is_rtu=is_rtu
    def connect(self):
        self.client = ModbusClient(
            method=self.is_rtu,
            port=self.port,
            stopbits=1,
            bytesize=8,
            parity="E",
            baudrate=115200,
            timeout=0.5,
        )
        print(self.client)
        return self.client
    def directCommand(self, vel_l: float, vel_r: float, cclient):
        AZDKDDirectMessageManager.method = AZDKDParameter.ControlMethod[
            "ContinusOperationWithSpeed"
        ]
        AZDKDDirectMessageManager.speed = int(vel_l)
        result = cclient.write_registers(
            address=AZDKDDirectMessageManager.getAddress(),
            values=AZDKDDirectMessageManager.makeMotionParameter(),
            slave=0x02,
            skip_encode=True,
        )
        AZDKDDirectMessageManager.speed = int(vel_r)
        result1 = cclient.write_registers(
            address=AZDKDDirectMessageManager.getAddress(),
            values=AZDKDDirectMessageManager.makeMotionParameter(),
            slave=0x03,
            skip_encode=True,
        )
        print(int(vel_l))
        print(int(vel_r))
        return
    def read_register(self, cclient):
    # HM-60262-9E PG.373
        cmdSpeed = 0xCA  # command speed (Hz)
        registerSpeed = 0xD0  # feedback speed (rpm)
        registerPosition = 0xCC  # feedback position (step)
        cmd = cclient.read_holding_registers(cmdSpeed, 2, slave=0x02)
        response = cclient.read_holding_registers(registerSpeed, 2, slave=0x02)
        response2 = cclient.read_holding_registers(registerPosition, slave=0x02)
        cmd1 = cclient.read_holding_registers(cmdSpeed, 2, slave=0x03)
        response1 = cclient.read_holding_registers(registerSpeed, 2, slave=0x03)
        response3 = cclient.read_holding_registers(registerPosition, slave=0x03)
        rospy.loginfo(cmd.registers)
        rospy.loginfo(cmd1.registers)
        rospy.loginfo(response.registers)
        rospy.loginfo(response1.registers)
        rospy.loginfo(response2.registers)
        rospy.loginfo(response3.registers)

class robot_kinematic:
    def __init__(self):
        rospy.Subscriber(
            'cmd_vel', 
            Twist, 
            self.kinematic, 
            10
        )

    def kinematic(self, twist: Twist):
        twist = twist
        print(twist)
        if twist.angular.z == 0:
            vl = twist.linear.x
            vr = twist.linear.x
        else:
            vl = twist.linear.x - twist.angular.z * ROBOT_WHEEL_SEPARATION / 2
            vr = twist.linear.x + twist.angular.z * ROBOT_WHEEL_SEPARATION / 2
        vl_rps = vl / (2 * math.pi * ROBOT_WHEEL_RADIUS)
        vl_dps = vl_rps*360
        vl_hertz = vl_dps*100
        vr_rps = vr / (2 * math.pi * ROBOT_WHEEL_RADIUS)
        vr_dps = vr_rps*360
        vr_hertz = vr_dps*100
        return vl_hertz, vr_hertz
    
    def updateOdometry(self, dl_step: int, dr_step: int):
        delta_l = (2 * math.pi * ROBOT_WHEEL_RADIUS * dl_step) / ROBOT_MOTOR_SPR
        delta_r = (2 * math.pi * ROBOT_WHEEL_RADIUS * dr_step) / ROBOT_MOTOR_SPR 
        delta_center = (delta_l + delta_r) / 2
        x_pos += delta_center * math.cos(theta)
        y_pos += delta_center * math.sin(theta)
        theta += (delta_r - delta_l) / ROBOT_WHEEL_SEPARATION

if __name__ == '__main__':
    rospy.init_node("robot_node", anonymous=True)
    rs485 = RS485()
    connection = rs485.connect()
    robot = robot_kinematic()
    rospy.Timer(rospy.Duration(1.0/100.0), RS485.directCommand(vL, vR, connection))
    rospy.Timer(rospy.Duration(4.0/100.0), RS485.read_register(connection))
    rospy.spin()