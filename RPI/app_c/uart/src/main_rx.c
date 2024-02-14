#include <stdio.h>
#include <stdlib.h>
#include "sx1280_uart.h"

/*
bool rxDone = false;
bool rxError = false;

void dio1Callback()
{
    printf("rxDone\n");
    rxDone = true;
}

void dio2Callback()
{
    printf("rxError\n");
    rxError = true;
}
*/

int main()
{
    printf("starting....RX\n");
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
        DF_PACKET_LEN,
        DF_CYCLICAL_REDUNDANCY_CHECK,
        DF_CHIRP_INVERT,
        0x00,
        0x00};
    SetPacketParams(packetParams);
    printf("8\n");

    SetDioIrqParams(0b0100000001100010, 0b0000000000000010, 0b0100000001100000, 0);
    SetRx(0x02, 0xFFFF);

    uint8_t rxBuffStatus[2];
    printf("9\n");

    uint8_t pt = GetPacketType();
    printf("packet type %u\n", pt);
    ClrIrqStatus(0xFFFF);

    /*
    set_mode(pi, DIO1_PIN, PI_INPUT);
    set_mode(pi, DIO2_PIN, PI_INPUT);
    set_pull_up_down(pi, DIO1_PIN, PI_PUD_DOWN);
    set_pull_up_down(pi, DIO2_PIN, PI_PUD_DOWN);

    int dio1 = callback(pi, DIO1_PIN, RISING_EDGE, dio1Callback);
    int dio2 = callback(pi, DIO2_PIN, RISING_EDGE, dio2Callback);
    */

    while (true)
    {
        uint16_t irq = GetIrqStatus();

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
                FILE *fp = fopen("/home/gs/Desktop/CRS_GS_dev/RPI/data.bin", "w");
                fwrite(msg, 1, (size_t)rxBuffStatus[0], fp);
                fclose(fp);
            }
            ClrIrqStatus(0xFFFF);
        }
    }
    return 0;
}
