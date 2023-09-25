#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import sys
sys.path.append('/home/codybui/om_ros/src/test_om/scripts')
import time
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
from geometry_msgs.msg import Twist
# from test_om.scripts import AZDKDDirectMessageManager
import AZDKDDirectMessageManager
# import AZDKDDirectParameter
# import AZDKDMessageManager
import AZDKDParameter

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
    # rospy.loginfo(client)
    print(client)
    return client

global cclient
cclient = connect()

def listener():
    rospy.Subscriber('/cmd_vel', Twist, twist_callback, 10)
def directCommando(linear: float, angular: float, client):
        # HM-60262EE P.276
        AZDKDDirectMessageManager.method = AZDKDParameter.ControlMethod[
            "ContinusOperationWithSpeed"
        ]
        # AZDKDDirectMessageManager.position = 10000
        n = int(linear*1000)
        # n = 0
        # print('set point = ' + str(n))
        AZDKDDirectMessageManager.speed = n
        # print(AZDKDDirectMessageManager.speed)
        # print(type(AZDKDDirectMessageManager.speed))
        # print(type(cclient))
        # ----------------------------------------Choose Register to Write----------------------------Write Value to Register------------------Slave Addr-----??------------#
        result = cclient.write_registers(
            address=AZDKDDirectMessageManager.getAddress(),
            values=AZDKDDirectMessageManager.makeMotionParameter(),
            slave=0x02,
            skip_encode=True,
        )
        result1 = cclient.write_registers(
            address=AZDKDDirectMessageManager.getAddress(),
            values=AZDKDDirectMessageManager.makeMotionParameter(),
            slave=0x03,
            skip_encode=True,
        )
        # rospy.loginfo(result)
        # time.sleep(1)
        # read_register()
        return
def read_register():
        #HM-60262-9E PG.373
        cmdSpeed = 0xCA
        registerSpeed = 0xD0
        cmd10 = cclient.read_holding_registers(0xCE, 2, slave = 0x03)
        # registerPosition = 0xCC   
        cmd = cclient.read_holding_registers(cmdSpeed, 2, slave = 0x02)
        response = cclient.read_holding_registers(registerSpeed, 2, slave = 0x02)
        # response1 = cclient.read_holding_registers(registerPosition, 2, slave = 0x02)
        # registerOdometer = 0x35F
        # response2 = cclient.read_holding_registers(registerOdometer, 2, slave = 0x02)
        # registerSpeed1 = 0xD0
        cmd1 = cclient.read_holding_registers(cmdSpeed, 2, slave = 0x03)
        response1 = cclient.read_holding_registers(registerSpeed, 2, slave = 0x03)
        # registerPosition1 = 0xCC
        # response1 = cclient.read_holding_registers(registerPosition1, 2, slave = 0x02)
        # registerOdometer1 = 0x35F
        # response2 = cclient.read_holding_registers(registerOdometer1, 2, slave = 0x02)
        # rospy.loginfo(response)
        rospy.loginfo(f'command speed (Hz): {cmd.registers[1]}')
        # rospy.loginfo(cmd.registers[1])
        rospy.loginfo(f'command speed (rpm): {cmd10.registers}')
        # rospy.loginfo(response.registers[1])
        # rospy.loginfo(response1.registers[1])
        # rospy.loginfo(response2.registers[1])
        # rospy.loginfo(response.registers)
        decoder =   BinaryPayloadDecoder.fromRegisters(response.registers, byteorder=Endian.Big, wordorder=Endian.Little)
        # value = decoder.decode
        # rospy.loginfo(decoder)
        # print(decoder)
        # rospy.loginfo(value)
        # rospy.loginfo(decoder.decode_32bit_float())
def twist_callback(twist: Twist, cclient):
        twist = twist
        # rospy.loginfo(f'Twist received: {twist}')
        # print(f'Twist received: {twist}')
        directCommando(twist.linear.x, twist.angular.z, cclient)
        # read_register()
def reset_trip_metter():
      cclient.write_registers(address=0x1A2, values=1, slave=0x02, skip_encode=True)
def main():
    #   print (type(cclient))
    rospy.init_node('motor_node', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # read_register()
        listener()  
        # rate.sleep()
        # time.sleep(0.3)
        # read_register()
        rospy.spin()
if __name__ == '__main__':
      main()