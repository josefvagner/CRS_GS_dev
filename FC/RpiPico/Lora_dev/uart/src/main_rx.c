#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "sx1280_uart.h"

int main()
{
    stdio_init_all();
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
                buff_t rxBuffStatus = GetRxBufferStatus();
                buff_t msg = ReadBuffer(rxBuffStatus.data[0]);
                printf("New msg [%d]: ", msg.len);
                for (int i = 0; i < msg.len; i++)
                {
                    printf("%c", (char)msg.data[i]);
                }
                printf("\n");
            }
            ClrIrqStatus(0xFFFF);
        }
        sleep_ms(100);
    }
}
