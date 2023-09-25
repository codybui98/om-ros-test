#!/usr/bin/env python3

import rospy
import time
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
from geometry_msgs.msg import Twist
import AZDKDDirectMessageManager
import AZDKDDirectParameter
import AZDKDMessageManager
import AZDKDParameter

class MotorControlNode():
    def __init__(self):
        super().__init__('moto_control_node')
        # self.driver_port = '/dev/ttyUSB0'
        self.twist_subscription = self.creat_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )
        self.twist_subscription
        time.sleep(0.2)
        self.client = ModbusClient(
            method="rtu",
            port="/dev/ttyUSB0",
            stopbits=1,
            bytesize=8,
            parity="E",
            baudrate=115200,
            timeout=0.5,
        )
        rospy.loginfo(f'Using serial port {self.client}')
        self.twist = Twist()
        self.pub_period = 0.04
        self.pub_timer = self.create_timer(self.pub_period, self.pub_callback)
    def pub_callback(self):
        robot_state = self.directCommando(self.twist.linear.x, self.twist.angular.z)
        if robot_state is None: 
            return 
    
    def directCommando(self, linear: float, angular: float):
        # HM-60262EE P.276
        AZDKDDirectMessageManager.method = AZDKDParameter.ControlMethod[
            "ContinusOperationWithSpeed"
        ]
        # AZDKDDirectMessageManager.position = 10000
        AZDKDDirectMessageManager.speed = linear*10000
        # ----------------------------------------Choose Register to Write----------------------------Write Value to Register------------------Slave Addr-----??------------#
        result = self.client.write_registers(
            address=AZDKDDirectMessageManager.getAddress(),
            values=AZDKDDirectMessageManager.makeMotionParameter(),
            slave=0x02,
            skip_encode=True,
        )
        rospy.loginfo(result)
        return
    def read_register(client):
        #HM-60262-9E PG.373
        registerToRead = 0xD0
        response = client.read_holding_registers(registerToRead, 2, slave = 0x02)
        rospy.loginfo(response)
        rospy.loginfo(response.registers[1])
        rospy.loginfo(response.registers)
        decoder =   BinaryPayloadDecoder.fromRegisters(response.registers, byteorder=Endian.Big, wordorder=Endian.Little)
        # value = decoder.decode
        rospy(decoder)
        # rospy.loginfo(value)
        rospy.loginfo(decoder.decode_32bit_float())

    def twist_callback(self, twist: Twist):
        self.twist = twist
        rospy.loginfo(f'Twist received: {twist}')
        
def main(args=None):
    # rospy.init_node("motor_node", anonymous=True)
    rospy.init_node("motor_control_node")
    motor_control_node = MotorControlNode()
    rospy.loginfo("Motor Node have started")
    rospy.spin(motor_control_node)

    # MotorControlNode.__init__('motor_node')
    # while not rospy.is_shutdown():
    #     MotorControlNode.directCommando()
    #     MotorControlNode.read_register()


if __name__ == '__main__':
    main()