#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "sx1280_spi.h"

#define FREQ_HZ 20
#define PING_TRESHOLD_MS 500

int main()
{
    stdio_init_all();
    sleep_ms(1000);
    printf("starting....TX\n");

    sx1280_spi_t dev = {spi0, 19, 16, 18, 17, 7, 6, 1, 2, 3, STANDBY_RC};

    Sx1280SPIInit(&dev);
    WaitForSetup(&dev);

    GsMsg_t msg = {
        200, 100.2, 100.3, 100.3, 100.5, 234.4, 1364.545044, 1364.564087, 98121.734375, 175.699997, 7.784000, -11.128282, 23.315508, 19.417095, -0.205971, -9.958491, 0.253872, 25, 24, 23, 0};
    GsPingMsg_t ping;
    uint16_t irq = 0;
    uint32_t tSend = 0;
    uint32_t tLastPing = millis();
    uint8_t rxBuffStatus[2];

    printf("Starting TX loop\n");

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
                printf("Tx done: fsw_state = %d\n", msg.fsw_state);
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
                if (ReadBuffer(&dev, (uint8_t *)&ping, (size_t)rxBuffStatus[0], rxBuffStatus[1]) == -1)
                {
                    WaitForSetup(&dev);
                }
                tLastPing = millis();
                printf("New ping: idx = %d\n", ping.idx);
            }
        }

        if ((int)(millis() - tSend) >= (int)(1000 / FREQ_HZ))
        {
            msg.fsw_state = msg.fsw_state >= 9 ? 0 : msg.fsw_state + 1;
            printf("Sending new msg: fsw_state = %d, time = %d\n", msg.fsw_state, (int)(millis() - tSend));
            if (WriteBuffer(&dev, (uint8_t *)&msg, sizeof(msg)) == -1)
            {
                WaitForSetup(&dev);
            }

            if (SetTx(&dev, 0x02, 0) == -1)
            {
                WaitForSetup(&dev);
            }
            dev.state = TX;
            tSend = millis();
        }

        if ((int)(millis() - tLastPing) >= PING_TRESHOLD_MS)
        {
            perror("Ping timeout");
            WaitForSetup(&dev);
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
