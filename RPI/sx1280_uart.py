"""
SX1280 UART Module

Constants for UART communication and SX1280 commands.
"""

import serial
from gpiozero import DigitalInputDevice, DigitalOutputDevice
from time import sleep

UART_ID = "/dev/ttyAMA0"
BAUD_RATE = 115200
DATA_BITS = serial.EIGHTBITS
STOP_BITS = serial.STOPBITS_ONE
PARITY = serial.PARITY_EVEN

# Pin assignments
UART_TX_PIN = 14
UART_RX_PIN = 15
UART_CTS_PIN = 16
UART_RTS_PIN = 17
BUSY_PIN = 5
RESET_PIN = 10

RX_BASE_ADDR = b'\x00'
TX_BASE_ADDR = b'\x80'

DF_PACKET_LEN = 126

DF_PREAMBLE_LENGTH = b'\x0C'
DF_HEADER_TYPE = b'\x00'
DF_CYCLICAL_REDUNDANCY_CHECK = b'\x20'
DF_CHIRP_INVERT = b'\x40'

# SX1280 commands
GET_STATUS = b'\xC0'
WRITE_REGISTER = b'\x18'
READ_REGISTER = b'\x19'
WRITE_BUFFER = b'\x1A'
READ_BUFFER = b'\x1B'
SET_SLEEP = b'\x84'
SET_STANDBY = b'\x80'
SET_FS = b'\xC1'
SET_TX = b'\x83'
SET_RX = b'\x82'
SET_RX_DUTY_CYCLE = b'\x94'
SET_CAD = b'\xC5'
SET_TX_CONTINUOUS_WAVE = b'\xD1'
SET_TX_CONTINUOUS_PREAMBLE = b'\xD2'
SET_PACKET_TYPE = b'\x8A'
GET_PACKET_TYPE = b'\x03'
SET_RF_FREQUENCY = b'\x86'
SET_TX_PARAMS = b'\x8E'
SET_CAD_PARAMS = b'\x88'
SET_BUFFER_BASE_ADDRESS = b'\x8F'
SET_MODULATION_PARAMS = b'\x8B'
SET_PACKET_PARAMS = b'\x8C'
GET_RX_BUFFER_STATUS = b'\x17'
GET_PACKET_STATUS = b'\x1D'
GET_RSSI_LNST = b'\x1F'
SET_DIO_IRQ_PARAMS = b'\x8D'
GET_IRQ_STATUS = b'\x15'
CLR_IRQ_STATUS = b'\x97'
SET_REGULATOR_MODE = b'\x96'
SET_SAVE_CONTEXT = b'\xD5'
SET_AUTO_FS = b'\x9E'
SET_AUTO_TX = b'\x98'
SET_LONG_PREAMBLE = b'\x9B'
SET_UART_SPEED = b'\x9D'
SET_RANGING_ROLE = b'\xA3'
SET_ADVANCED_RANGING = b'\x9A'
SET_PERF_COUNTER_MODE = b'\x9C'

ser = serial.Serial(port=UART_ID, baudrate=BAUD_RATE, bytesize=DATA_BITS, parity=PARITY, stopbits=STOP_BITS, rtscts=False)
reset = DigitalOutputDevice(RESET_PIN)
busy = DigitalInputDevice(BUSY_PIN)
rts = DigitalOutputDevice(UART_RTS_PIN)

def initLora():
    reset.off()
    rts.off()
    #ser.set_output_flow_control(True)
    sleep(2)
    reset.on()
    waitBusyPin()
    

def waitBusyPin():
    busy.wait_for_inactive()

def uartSend(msg: bytes):
    ser.reset_output_buffer()
    ser.write(msg)
    waitBusyPin()

def uartSendRecv(msg: bytes, retLen: int) -> bytes:
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(msg)
    return ser.read(retLen)

def GetStatus() -> bytes:
    return uartSendRecv(GET_STATUS, 1)

def WriteRegister(addr: bytes, data: bytes):
    uartSend(WRITE_REGISTER + addr + len(data).to_bytes() + data)

def ReadRegister(addr: bytes, len: int) -> bytes:
    return uartSendRecv(READ_REGISTER + addr + len.to_bytes(), len)

def WriteBuffer(data: bytes):
    uartSend(WRITE_BUFFER + TX_BASE_ADDR + len(data).to_bytes() + data)

def ReadBuffer(addr: bytes, len: int) -> bytes:
    return uartSendRecv(READ_BUFFER + addr + len.to_bytes(), len)

def SetSleep(sleepConfig: bytes):
    uartSend(SET_SLEEP + sleepConfig)

def SetStandby(standbyConfig: bytes):
    uartSend(SET_STANDBY + b'\x01' + standbyConfig)

def SetFs():
    uartSend(SET_FS)

def SetTx(periodBase: bytes, periodBaseCount: bytes):
    uartSend(SET_TX + b'\x03' + periodBase + periodBaseCount)

def SetRx(periodBase: bytes, periodBaseCount: bytes):
    uartSend(SET_RX + b'\x03' + periodBase + periodBaseCount)

def SetRxDutyCycle(periodBase: bytes, rxPeriodBaseCount: bytes, sleepPeriodBaseCount: bytes):
    uartSend(SET_RX_DUTY_CYCLE + b'\x05' + periodBase + rxPeriodBaseCount + sleepPeriodBaseCount)

def SetCad():
    uartSend(SET_CAD)

def SetTxContinuousWave():
    uartSend(SET_TX_CONTINUOUS_WAVE)

def SetTxContinuousPreamble():
    uartSend(SET_TX_CONTINUOUS_PREAMBLE)

def SetPacketType(packetType: bytes):
    uartSend(SET_PACKET_TYPE + b'\x01' + packetType)

def GetPacketType() -> bytes:
    return uartSendRecv(GET_PACKET_TYPE + b'\x01', 1)

def SetRfFrequency(rfFrequency: bytes):
    uartSend(SET_RF_FREQUENCY + b'\x03' + rfFrequency)

def SetTxParams(power: bytes, rampTime: bytes):
    uartSend(SET_TX_PARAMS + b'\x02' + power + rampTime)

def SetCadParams(cadSymbolNum: bytes):
    uartSend(SET_CAD_PARAMS + b'\x01' + cadSymbolNum)

def SetBufferBaseAddress(txBaseAddress: bytes, rxBaseAddress: bytes):
    uartSend(SET_BUFFER_BASE_ADDRESS + b'\x02' + txBaseAddress + rxBaseAddress)

def SetModulationParams(modParam: bytes):
    uartSend(SET_MODULATION_PARAMS + b'\x03' + modParam)
    if modParam[0] == b'\x50' or modParam[0] == b'\x60':
        WriteRegister(b'\x09\x25', b'\x1E')
    elif modParam[0] == b'\x70' or modParam[0] == b'\x80':
        WriteRegister(b'\x09\x25', b'\x37')
    elif modParam[0] == b'\x90' or modParam[0] == b'\xA0' or modParam[0] == b'\xB0' or modParam[0] == b'\xC0':
        WriteRegister(b'\x09\x25', b'\x32')

def SetPacketParams(packetParams: bytes):
    uartSend(SET_PACKET_PARAMS + b'\x07' + packetParams)

def GetRxBufferStatus() -> bytes:
    return uartSendRecv(GET_RX_BUFFER_STATUS + b'\x02', 2)

def GetPacketStatus() -> bytes:
    return uartSendRecv(GET_PACKET_STATUS + b'\x05', 5)

def GetRssiLnst() -> bytes:
    return uartSendRecv(GET_RSSI_LNST + b'\x01', 1)

def SetDioIrqParams(irqMask: bytes, dio1Mask: bytes, dio2Mask: bytes, dio3Mask: bytes):
    uartSend(SET_DIO_IRQ_PARAMS + b'\x08' + irqMask + dio1Mask + dio2Mask + dio3Mask)

def GetIrqStatus() -> bytes:
    return uartSendRecv(GET_IRQ_STATUS + b'\x02', 2)

def ClrIrqStatus(irqMask: bytes):
    uartSend(CLR_IRQ_STATUS + b'\x02' + irqMask)

def SetRegulatorMode(regulatorMode: bytes):
    uartSend(SET_REGULATOR_MODE + b'\x01' + regulatorMode)

def SetSaveContext():
    uartSend(SET_SAVE_CONTEXT)

def SetAutoFS(state: bytes):
    uartSend(SET_AUTO_FS + b'\x01' + state)

def SetAutoTx(time: bytes):
    uartSend(SET_AUTO_TX + b'\x02' + time)

def SetPerfCounterMode(perfCounterMode: bytes):
    uartSend(SET_PERF_COUNTER_MODE + b'\x01' + perfCounterMode)

def SetLongPreamble(enable: bool):
    uartSend(SET_LONG_PREAMBLE + b'\x01' + (b'\x01' if enable else b'\x00'))

def SetUartSpeed(uartSpeed: bytes):
    uartSend(SET_UART_SPEED + b'\x01' + uartSpeed)

def SetRangingRole(mode: bytes):
    uartSend(SET_RANGING_ROLE + b'\x01' + mode)

def SetAdvancedRanging(state: bytes):
    uartSend(SET_ADVANCED_RANGING + b'\x01' + state)

