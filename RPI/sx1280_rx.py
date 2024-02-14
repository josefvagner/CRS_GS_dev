from sx1280_uart import *

sleep(2)
initLora()
print("1")
SetStandby(b'\x00')
print("2")
SetPacketType(b'\x01');
print("3")
SetRfFrequency(b'\xB8\x9D\x89');
print("4")
SetBufferBaseAddress(TX_BASE_ADDR, RX_BASE_ADDR);
print("5")
SetModulationParams(b'\x50\x0A\x01');
print("6")
SetPacketParams(DF_PREAMBLE_LENGTH + DF_HEADER_TYPE + DF_PACKET_LEN.to_bytes() + DF_CYCLICAL_REDUNDANCY_CHECK + DF_CHIRP_INVERT + (2*b'\x00'));
print("7")
SetDioIrqParams((0b0100000001100010).to_bytes(2), 2*b'\x00', 2*b'\x00', 2*b'\x00');
print("8")
SetRx(b'\x02', b'\xFF\xFF');
print("9")

print("packet type: " + str(int.from_bytes(GetPacketType(), "big")))

while True:
    irq = int.from_bytes(GetIrqStatus())
    if irq & 0x02:
        if irq & 0b1000000:
            print("CRC ERROR")
        elif irq & 0b100000:
            print("Header error...")
        elif irq & 0b100000000000000:
            print("Timeout...")
        else:
            buffStatus = GetRxBufferStatus();
            print(buffStatus)
            msg = ReadBuffer(buffStatus[1].to_bytes(), buffStatus[0])
            print("New msg: " + str(msg))
        ClrIrqStatus(b'\xFF\xFF');
    sleep(0.01)
        
