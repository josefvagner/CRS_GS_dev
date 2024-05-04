#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "sx1280_spi.h"

#define PING_FREQ_HZ 10

int main()
{
    stdio_init_all();
    sleep_ms(1000);
    printf("starting....RX\n");

    sx1280_spi_t dev = {spi0, 19, 16, 18, 17, 7, 6, 1, 2, 3, STANDBY_RC};

    Sx1280SPIInit(&dev);
    WaitForSetup(&dev);

    uint8_t rxBuffStatus[2];
    uint8_t msgRaw[256];
    GsMsg_t msg;
    GsPingMsg_t ping = {0};
    uint16_t irq = 0;

    uint32_t tLastPing = 0;

    printf("Starting RX loop\n");

    while (true)
    {
        if (gpio_get(dev.dio1Pin)) // TxDone
        {
            if (gpio_get(dev.dio3Pin)) // Error
            {
                perror("TX error");
            }
            else
            {
                printf("Ping done: idx = %d\n", ping.idx);
                if (ClrIrqStatus(&dev, 0xFFFF) == -1)
                {
                    WaitForSetup(&dev);
                }
                if (SetRx(&dev, 0x02, 0xFFFF) == -1)
                {
                    WaitForSetup(&dev);
                }
                dev.state = RX;
            }
        }
        if (gpio_get(dev.dio2Pin)) // RxDone
        {
            if (gpio_get(dev.dio3Pin)) // Error
            {
                perror("RX error");
            }
            else
            {
                if (GetRxBufferStatus(&dev, rxBuffStatus) == -1)
                {
                    WaitForSetup(&dev);
                }
                if (ReadBuffer(&dev, (uint8_t *)&msg, (size_t)rxBuffStatus[0], rxBuffStatus[1]) == -1)
                {
                    WaitForSetup(&dev);
                }
                printf("New msg [%d] [%d]: fsw_state = %d\n", (int)rxBuffStatus[0], (int)rxBuffStatus[1], msg.fsw_state);
            }
        }

        if ((int)(millis() - tLastPing) >= (int)(1000 / PING_FREQ_HZ))
        {
            ping.idx = ping.idx > 100 ? 0 : ping.idx + 1;
            if (WriteBuffer(&dev, (uint8_t *)&ping, sizeof(ping)) == -1)
            {
                WaitForSetup(&dev);
            }

            if (ClrIrqStatus(&dev, 0xFFFF) == -1)
            {
                WaitForSetup(&dev);
            }

            if (SetTx(&dev, 0x02, 0) == -1)
            {
                WaitForSetup(&dev);
            }
            dev.state = TX;
            tLastPing = millis();
        }

        if (gpio_get(dev.dio1Pin) || gpio_get(dev.dio2Pin) || gpio_get(dev.dio3Pin))
        {
            if (ClrIrqStatus(&dev, 0xFFFF) == -1)
            {
                WaitForSetup(&dev);
            }
        }
    }
}
