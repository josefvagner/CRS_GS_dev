#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "sx1280_uart.h"

int main()
{
    sleep_ms(2000);
    stdio_init_all();
    sleep_ms(1000);
    printf("starting....TX\n");
    sx1280UartInit();
    printf("1\n");
    waitBusyPin();
    printf("2\n");
    SetStandby(0x00);
    printf("3\n");
    SetPacketType(0x01);
    printf("4\n");
    uint8_t rfFreq[3] = {0xB8, 0x9D, 0x89};
    SetRfFrequency(rfFreq);
    printf("5\n");
    SetBufferBaseAddress(TX_BASE_ADDR, RX_BASE_ADDR);
    printf("6\n");
    uint8_t modParams[3] = {0x50, 0x0A, 0x01};
    SetModulationParams(modParams);
    printf("7\n");
    uint8_t packetParams[7] = {
        DF_PREAMBLE_LENGTH,
        DF_HEADER_TYPE,
        4,
        DF_CYCLICAL_REDUNDANCY_CHECK,
        DF_CHIRP_INVERT,
        0x00,
        0x00};
    SetPacketParams(packetParams);

    printf("8\n");

    SetDioIrqParams(0b0100000000000001, 0, 0, 0);
    printf("9\n");
    SetTxParams(0x1F, 0xE0);
    printf("10\n");

    uint8_t msgId = 0;
    uint8_t msgTxt[4] = {'R',
                         'P',
                         ' ',
                         msgId + '0'};
    uint16_t irq = 0;

    uint8_t pt = GetPacketType();
    printf("packet type %u\n", pt);

    while (true)
    {
        msgTxt[3] = msgId + '0';
        WriteBuffer(msgTxt, 4);
        SetTx(0x02, 0x01F4);
        while (true)
        {
            irq = GetIrqStatus();
            if (irq & 0x01)
            {
                printf("Tx done: ");
                printBuffChar(msgTxt, 4);
                break;
            }
            if (irq & 0b0100000000000000)
            {
                printf("Tx timeout...\n");
                break;
            }
            sleep_ms(100);
        }
        ClrIrqStatus(0xFFFF);
        SetStandby(0x00);
        sleep_ms(1000);
        msgId = msgId >= 9 ? 0 : msgId + 1;
    }
    printf("end\n");
}
