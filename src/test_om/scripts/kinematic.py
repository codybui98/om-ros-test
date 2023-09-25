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
import threading

global ROBOT_WHEEL_SEPARATION
global ROBOT_ROBOT_WHEEL_RADIUS
global ROBOT_MOTOR_SPR

ROBOT_MOTOR_SPR = 18000 
ROBOT_WHEEL_RADIUS = 0.1
ROBOT_WHEEL_SEPARATION = 0.5


def connect():
    client = ModbusClient(
        method="rtu",
        port="/dev/ttyUSB0",
        stopbits=1,
        bytesize=8,
        parity="E",
        baudrate=115200,
        timeout=0.5,
    )
    print(client)
    return client

global cclient 
cclient = connect()

def kinematic(twist: Twist, cclient):
    twist = twist
    print(twist)
    rate = rospy.Rate(10)
    if twist.angular.z == 0:
        vl = twist.linear.x
        vr = twist.linear.x
    else:
        vl = twist.linear.x - twist.angular.z * ROBOT_WHEEL_SEPARATION / 2
        vr = twist.linear.x + twist.angular.z * ROBOT_WHEEL_SEPARATION / 2
    if (vl > 0) and (vr > 0):
        l_dir = 1
        r_dir = 1
    else:
        if (vl > 0) and (vr < 0):
            l_dir = 1
            r_dir = 0
        if (vl < 0) and (vr > 0):
            l_dir = 0
            r_dir = 1
        if (vl < 0) and (vr < 0):
            l_dir = 0
            r_dir = 0
    vl_rps = vl / (2 * math.pi * ROBOT_WHEEL_RADIUS)
    # vl_rpm = vl_rps / 10
    vl_dps = vl_rps*360
    vl_hertz = vl_dps*100
    # vl_pulse = vl_rps*(360/0.02) #step angle of the DGM130R
    vr_rps = vr / (2 * math.pi * ROBOT_WHEEL_RADIUS)
    # vr_rpm = vr_rps * 10
    vr_dps = vr_rps*360
    vr_hertz = vr_dps*100
    directCommand(vl_hertz, vr_hertz)
    time.sleep(0.5)
    read_register()
    # vr_pulse = vr_rps*(360/0.02) #step angle of the DGM130R

def directCommand(vel_l: float, vel_r: float):
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
    decoder = BinaryPayloadDecoder.fromRegisters(result.registers,byteorder=Endian.Little,wordorder=Endian.Big)
    print(decoder.decode_string(32))
    print(result)
    print(result1)
    return

def read_register():
    # HM-60262-9E PG.373
    cmdSpeed = 0xCA  # command speed (Hz)
    registerSpeedHz = 0xD0  # feedback speed (Hz)
    registerSpeedrpm = 0xCF # feedback speed (rpm)
    registerPosition = 0xCD  # feedback position (step)
    cmd = cclient.read_holding_registers(cmdSpeed, 2, slave=0x02)
    time.sleep(0.0001)
    response1 = cclient.read_holding_registers(registerSpeedHz, 2, slave=0x02)
    time.sleep(0.0001)
    response2 = cclient.read_holding_registers(registerSpeedrpm, slave=0x02)   #*
    time.sleep(0.0001)
    response3 = cclient.read_holding_registers(registerPosition, slave=0x02)   #*
    time.sleep(0.0001)
    cmd1 = cclient.read_holding_registers(cmdSpeed, 2, slave=0x03)
    response4 = cclient.read_holding_registers(registerSpeedHz, 2, slave=0x03)
    time.sleep(0.0001)
    response5 = cclient.read_holding_registers(registerSpeedrpm, slave=0x03)   #*
    time.sleep(0.0001)
    response6 = cclient.read_holding_registers(registerPosition, slave=0x03)   #*
    time.sleep(0.0001)
    rospy.loginfo(f'setpoint speed (Hz): {cmd.registers[1]}')
    rospy.loginfo(f'Hz: {response1.registers[1]}')
    rospy.loginfo(f'rpm: {response2.registers}')
    rospy.loginfo(f'step: {response3.registers}')
    rospy.loginfo(f'setpoint speed (Hz): {cmd1.registers[1]}')
    rospy.loginfo(f'Hz: {response4.registers[1]}')
    rospy.loginfo(f'rpm: {response5.registers}')
    rospy.loginfo(f'step: {response6.registers}')

def updateOdometry(dl_step: int, dr_step: int):
    delta_l = (2 * math.pi * ROBOT_WHEEL_RADIUS * dl_step) / ROBOT_MOTOR_SPR
    delta_r = (2 * math.pi * ROBOT_WHEEL_RADIUS * dr_step) / ROBOT_MOTOR_SPR 
    delta_center = (delta_l + delta_r) / 2

    x_pos += delta_center * math.cos(theta)
    y_pos += delta_center * math.sin(theta)
    theta += (delta_r - delta_l) / ROBOT_WHEEL_SEPARATION

def main():
    rospy.init_node("listen_node", anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, kinematic, 10)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass