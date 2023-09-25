import serial
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
import AZDKDMessageManager
import AZDKDParameter
import AZDKDDirectMessageManager
import AZDKDDirectParameter
import time
import logging

# modbus rtu commando
# slave address : 8bit
# fuction code : 8bit
# data : N * 8bit
# error check(CRC-32) : 32bit

# function code
# read register : 0x03
# - register count : 1 - 125
# write register : 0x06
# - register count : 1
# check status : 0x08
# - register count: -
# write registers : 0x10
# - register count : 1 - 123
# readwrite registers : 0x17
# - read register count : 1 - 125
# - write register count : 1 - 121

# FORMAT = (
#     "%(asctime)-15s %(threadName)-15s "
#     "%(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s"
# )

# delay_ms = 300

# logging.basicConfig(format=FORMAT)
# log = logging.getLogger()
# log.setLevel(logging.DEBUG)


def normalCommando(client):
    AZDKDMessageManager.position = 8500
    # ---------------------------------------Choose Register to Write-----------------Write Value to Register------------------Slave Addr-----??------------#
    result = client.write_registers(
        address=AZDKDMessageManager.getAddress(0),
        values=AZDKDMessageManager.makeMotionParameter(),
        unit=0x01,
        skip_encode=True,
    )
    print(result)

    # Start  Driving
    # REQUIRED#
    result = client.write_registers(
        address=0x007C, values=getPayload(8), unit=0x01, skip_encode=True
    )
    print(result)

    # Ending
    # REQUIRED#
    result = client.write_registers(
        address=0x007C, values=getPayload(0), unit=0x01, skip_encode=True
    )
    print(result)
    return

def directCommando(client):
    # HM-60262EE P.276
    # builder = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
    # builder.add_32bit_int(0) #operation No.
    # builder.add_32bit_int(2) # operation type
    # builder.add_32bit_int(8500)
    # builder.add_32bit_int(2000)
    # builder.add_32bit_int(1500)
    # builder.add_32bit_int(1500)
    # builder.add_32bit_int(1000)
    # builder.add_32bit_int(1)
    # print(builder.build())
    # result = client.write_registers(
    #     address=0x58, values=builder.build(), skip_encode=True, slave = 0x02
    # )
    AZDKDDirectMessageManager.method = AZDKDParameter.ControlMethod[
        "ContinusOperationWithSpeed"
    ]
    # AZDKDDirectMessageManager.position = 10000
    AZDKDDirectMessageManager.speed = 0
    # ----------------------------------------Choose Register to Write----------------------------Write Value to Register------------------Slave Addr-----??------------#
    result = client.write_registers(
        address=AZDKDDirectMessageManager.getAddress(),
        values=AZDKDDirectMessageManager.makeMotionParameter(),
        slave=0x02,
        skip_encode=True,
    )
    print(result)
    return

def getPayload(num):
    builder = BinaryPayloadBuilder(byteorder=Endian.Big)
    builder.add_32bit_int(num)
    return builder.build()

def read_register(client):
    #HM-60262-9E PG.373
    registerToRead = 0xD0
    response = client.read_holding_registers(registerToRead, 2, slave = 0x02)
    print(response)
    print(response.registers[1])
    print(response.registers)
    decoder =   BinaryPayloadDecoder.fromRegisters(response.registers, byteorder=Endian.Big, wordorder=Endian.Little)
    # value = decoder.decode
    print(decoder)
    # print(value)
    print(decoder.decode_32bit_float())

if __name__ == "__main__":
    client = ModbusClient(
        method="rtu",
        port="/dev/ttyUSB0",
        stopbits=1,
        bytesize=8,
        parity="E",
        baudrate=115200,
        timeout=0.5,
    )
    directCommando(client)

    # while True:
    connection =  client.connect()
    # print("connection result:{0}".format(connection))
    # log.debug("Write to a holding register")
    read_register(client)
    client.close()
    # print("connection close.")
