#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "sx1280_uart.h"

int main()
{
    sleep_ms(2000);
    stdio_init_all();
    sleep_ms(1000);
    printf("starting....RX\n");
    sx1280UartInit();
    waitBusyPin();
    SetStandby(0x00);
    SetPacketType(0x01);
    uint8_t rfFreq[3] = {0xB8, 0x9D, 0x89};
    SetRfFrequency(rfFreq);
    SetBufferBaseAddress(TX_BASE_ADDR, RX_BASE_ADDR);
    uint8_t modParams[3] = {0x50, 0x0A, 0x01};
    SetModulationParams(modParams);
    uint8_t packetParams[7] = {
        DF_PREAMBLE_LENGTH,
        DF_HEADER_TYPE,
        DF_PACKET_LEN,
        DF_CYCLICAL_REDUNDANCY_CHECK,
        DF_CHIRP_INVERT,
        0x00,
        0x00};
    SetPacketParams(packetParams);

    SetDioIrqParams(0b0100000001100010, 0, 0, 0);
    SetRx(0x02, 0xFFFF);

    uint8_t rxBuffStatus[2];

    uint8_t pt = GetPacketType();
    printf("packet type %u\n", pt);

    while (true)
    {
        uint8_t irq = GetIrqStatus();

        if (irq & 0x02)
        {
            if (irq & 0b1000000)
            {
                printf("Crc error...\n");
            }
            else if (irq & 0b100000)
            {
                printf("Header error...\n");
            }
            else if (irq & 0b100000000000000)
            {
                printf("Timeout...\n");
            }
            else
            {
                GetRxBufferStatus(rxBuffStatus);
                uint8_t msg[rxBuffStatus[0]];
                ReadBuffer(msg, (size_t)rxBuffStatus[0], rxBuffStatus[1]);
                printf("New msg [%d]: ", (int)rxBuffStatus[0]);
                printBuffChar(msg, (size_t)rxBuffStatus[0]);
            }
            ClrIrqStatus(0xFFFF);
        }
        sleep_ms(100);
    }
}
