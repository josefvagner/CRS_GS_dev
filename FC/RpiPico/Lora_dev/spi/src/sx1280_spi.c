#include "sx1280_spi.h"

uint32_t millis()
{
    return to_ms_since_boot(get_absolute_time());
}

void Sx1280SPIInit(sx1280_spi_t *dev)
{
    gpio_init(dev->busyPin);
    gpio_set_dir(dev->busyPin, 0);

    gpio_init(dev->dio1Pin);
    gpio_set_dir(dev->dio1Pin, 0);

    gpio_init(dev->dio2Pin);
    gpio_set_dir(dev->dio2Pin, 0);

    gpio_init(dev->dio3Pin);
    gpio_set_dir(dev->dio3Pin, 0);

    gpio_init(dev->resetPin);
    gpio_set_dir(dev->resetPin, 1);
    gpio_put(dev->resetPin, 1);

    spi_init(dev->spi, (uint)5e6);
    gpio_set_function(dev->sckPin, GPIO_FUNC_SPI);
    gpio_set_function(dev->mosiPin, GPIO_FUNC_SPI);
    gpio_set_function(dev->misoPin, GPIO_FUNC_SPI);

    gpio_init(dev->csPin);
    gpio_set_dir(dev->csPin, 1);
    gpio_put(dev->csPin, 1);
}

int ResetSx1280(sx1280_spi_t *dev)
{
    printf("Setting/Resseting sx1280 spi: %d\n", dev->spi);

    gpio_put(dev->resetPin, 0);
    gpio_put(dev->csPin, 1);
    sleep_us(20000);
    gpio_put(dev->resetPin, 1);
    sleep_us(20000);

    if (SetStandby(dev, 0x00) == -1)
    {
        return -1;
    }

    if (SetPacketType(dev, 0x01) == -1)
    {
        return -1;
    }

    uint8_t rfFreq[3] = {RF_FREQ_1, RF_FREQ_2, RF_FREQ_3};
    if (SetRfFrequency(dev, rfFreq) == -1)
    {
        return -1;
    }

    if (SetBufferBaseAddress(dev, TX_BASE_ADDR, RX_BASE_ADDR) == -1)
    {
        return -1;
    }

    uint8_t modParams[3] = {MOD_PARAM_1, MOD_PARAM_2, MOD_PARAM_3};
    if (SetModulationParams(dev, modParams) == -1)
    {
        return -1;
    }

    uint8_t packetParams[7] = {
        DF_PREAMBLE_LENGTH,
        DF_HEADER_TYPE,
        (uint8_t)sizeof(GsMsg_t),
        DF_CYCLICAL_REDUNDANCY_CHECK,
        DF_CHIRP_INVERT,
        0x00,
        0x00};
    if (SetPacketParams(dev, packetParams) == -1)
    {
        return -1;
    }

    if (SetRxDutyCycle(dev, 0x03, 0, 0x00FA) == -1)
    {
        return -1;
    }

    if (SetDioIrqParams(dev, (uint16_t)IRQ, (uint16_t)DIO1, (uint16_t)DIO2, (uint16_t)DIO3) == -1)
    {
        return -1;
    }

    if (SetTxParams(dev, 0x1F, 0x00) == -1)
    {
        return -1;
    }

    uint8_t pt = 0;
    if (GetPacketType(dev, &pt) == -1 || pt != 1)
    {
        printf("Packet type: %d\n", pt);
        return -1;
    }
    printf("Packet type: %d\n", pt);

    if (ClrIrqStatus(dev, 0xFFFF) == -1)
    {
        return -1;
    }

    printf("Sx1280 reset/setup complete\n");

    return 0;
}

void WaitForSetup(sx1280_spi_t *dev)
{
    while (ResetSx1280(dev) == -1)
    {
        /* wait for setup */
    }
    if (ClrIrqStatus(dev, 0xFFFF) == -1)
    {
        WaitForSetup(dev);
    }
    switch (dev->state)
    {
    case RX:
        if (SetRx(dev, 0x02, 0xFFFF) == -1)
        {
            WaitForSetup(dev);
        }
        break;
    case TX:
        if (SetTx(dev, 0x02, 0) == -1)
        {
            WaitForSetup(dev);
        }
        break;

    default:
        if (SetStandby(dev, 0x00) == -1)
        {
            WaitForSetup(dev);
        }
        break;
    }
}

int WaitBusyPin(sx1280_spi_t *dev)
{
    uint32_t t0 = millis();
    while (gpio_get(dev->busyPin) == 1)
    {
        if ((millis() - t0) >= BUSY_TIMEOUT_MS)
        {
            perror("WaitBusyPin timeout");
            return -1;
        }
    }
    return 0;
}

void PrintBuffHex(uint8_t *buff, size_t len)
{
    for (int i = 0; i < len; i++)
    {
        printf("0x%X ", buff[i]);
    }
    printf("\n");
}

void PrintBuffDec(uint8_t *buff, size_t len)
{
    for (int i = 0; i < len; i++)
    {
        printf("%u ", buff[i]);
    }
    printf("\n");
}

void PrintBuffChar(uint8_t *buff, size_t len)
{
    char text[len + 1];
    MyMemcpy(text, buff, len);
    text[len] = '\0';
    printf("%s\n", text);
}

int SpiSend(sx1280_spi_t *dev, uint8_t *buff, size_t len)
{
    if (WaitBusyPin(dev) == -1)
    {
        return -1;
    }
    if (spi_is_writable(dev->spi) == false)
    {
        perror("[ERROR] spi is not writable");
        return -1;
    }
    gpio_put(dev->csPin, 0);
    int ret = spi_write_blocking(dev->spi, buff, len);
    gpio_put(dev->csPin, 1);

    if (ret != len)
    {
        perror("[ERROR] spi don't write all bytes");
        return -1;
    }
    return 0;
}

void MyMemcpy(void *dest, void *src, size_t len)
{
    char *csrc = (char *)src;
    char *cdest = (char *)dest;

    for (int i = 0; i < len; i++)
        cdest[i] = csrc[i];
}

int SpiSendRecv(sx1280_spi_t *dev, uint8_t *msgBuff, size_t msgLen, uint8_t *recvBuff, size_t recvLen)
{
    size_t len = msgLen + recvLen + 1;
    uint8_t *send = calloc(len, 1);
    uint8_t *recv = calloc(len, 1);
    MyMemcpy(send, msgBuff, msgLen);
    if (WaitBusyPin(dev) == -1)
    {
        return -1;
    }

    if (spi_is_writable(dev->spi) == false)
    {
        perror("[ERROR] spi is not writable");
        return -1;
    }

    gpio_put(dev->csPin, 0);
    int ret = spi_write_read_blocking(dev->spi, send, recv, len);
    gpio_put(dev->csPin, 1);

    /* printf("Send/Recv: ");
PrintBuffHex(send, len);
printf("    ");
PrintBuffHex(recv, len); */

    if (ret != len)
    {
        perror("[ERROR] spi don't write/read all bytes");
        return -1;
    }

    MyMemcpy(recvBuff, recv + (msgLen + 1), recvLen);
    free(send);
    free(recv);
    return 0;
}

int GetStatus(sx1280_spi_t *dev, uint8_t *status)
{
    uint8_t send[1] = {GET_STATUS};

    if (WaitBusyPin(dev) == -1)
    {
        return -1;
    }

    if (spi_is_writable(dev->spi) == false)
    {
        perror("[ERROR] spi is not writable");
        return -1;
    }

    if (spi_is_readable(dev->spi) == false)
    {
        perror("[ERROR] spi is not readable");
        return -1;
    }

    gpio_put(dev->csPin, 0);
    int ret = spi_write_read_blocking(dev->spi, send, status, 1);
    gpio_put(dev->csPin, 1);

    if (ret != 1)
    {
        perror("[ERROR] spi don't write/read all bytes");
        return -1;
    }

    return 0;
}

int WriteRegister(sx1280_spi_t *dev, uint16_t addr, uint8_t *data, size_t len)
{
    uint8_t msg[len + 3];
    msg[0] = WRITE_REGISTER;
    msg[1] = (uint8_t)(addr >> 8);
    msg[2] = (uint8_t)addr;
    MyMemcpy(msg + 3, data, len);
    return SpiSend(dev, msg, len + 3);
}

int ReadRegister(sx1280_spi_t *dev, uint8_t *recv, size_t len, uint16_t addr)
{
    uint8_t msg[3] = {READ_REGISTER, (uint8_t)(addr >> 8), (uint8_t)addr};
    return SpiSendRecv(dev, msg, 3, recv, len);
}

int WriteBuffer(sx1280_spi_t *dev, uint8_t *data, size_t len)
{
    uint8_t msg[len + 2];
    msg[0] = WRITE_BUFFER;
    msg[1] = TX_BASE_ADDR;
    MyMemcpy(msg + 2, data, len);
    return SpiSend(dev, msg, len + 2);
}

int ReadBuffer(sx1280_spi_t *dev, uint8_t *recv, size_t len, uint8_t addr)
{
    uint8_t msg[2] = {READ_BUFFER, addr};
    return SpiSendRecv(dev, msg, 2, recv, len);
}

int SetSleep(sx1280_spi_t *dev, uint8_t sleepConfig)
{
    uint8_t msg[2] = {SET_SLEEP, sleepConfig};
    return SpiSend(dev, msg, 2);
}

int SetStandby(sx1280_spi_t *dev, uint8_t standbyConfig)
{
    uint8_t msg[2] = {SET_STANDBY, standbyConfig};
    return SpiSend(dev, msg, 2);
}

int SetFs(sx1280_spi_t *dev)
{
    uint8_t msg[1] = {SET_FS};
    return SpiSend(dev, msg, 1);
}

int SetTx(sx1280_spi_t *dev, uint8_t periodBase, uint16_t periodBaseCount)
{
    uint8_t msg[4] = {
        SET_TX,
        periodBase,
        (uint8_t)(periodBaseCount >> 8),
        (uint8_t)periodBaseCount};
    return SpiSend(dev, msg, 4);
}

int SetRx(sx1280_spi_t *dev, uint8_t periodBase, uint16_t periodBaseCount)
{
    uint8_t msg[4] = {
        SET_RX,
        periodBase,
        (uint8_t)(periodBaseCount >> 8),
        (uint8_t)periodBaseCount};
    return SpiSend(dev, msg, 4);
}

int SetRxDutyCycle(sx1280_spi_t *dev, uint8_t periodBase, uint16_t rxPeriodBaseCount, uint16_t sleepPeriodBaseCount)
{
    uint8_t msg[6] = {
        SET_RX_DUTY_CYCLE,
        periodBase,
        (uint8_t)(rxPeriodBaseCount >> 8),
        (uint8_t)rxPeriodBaseCount,
        (uint8_t)(sleepPeriodBaseCount >> 8),
        (uint8_t)sleepPeriodBaseCount};
    return SpiSend(dev, msg, 6);
}

int SetCad(sx1280_spi_t *dev)
{
    uint8_t msg[1] = {SET_CAD};
    return SpiSend(dev, msg, 1);
}

int SetTxContinuousWave(sx1280_spi_t *dev)
{
    uint8_t msg[1] = {SET_TX_CONTINUOUS_WAVE};
    return SpiSend(dev, msg, 1);
}

int SetTxContinuousPreamble(sx1280_spi_t *dev)
{
    uint8_t msg[1] = {SET_TX_CONTINUOUS_PREAMBLE};
    return SpiSend(dev, msg, 1);
}

int SetPacketType(sx1280_spi_t *dev, uint8_t packetType)
{
    uint8_t msg[2] = {SET_PACKET_TYPE, packetType};
    return SpiSend(dev, msg, 2);
}

int GetPacketType(sx1280_spi_t *dev, uint8_t *packetType)
{
    uint8_t msg[1] = {GET_PACKET_TYPE};
    return SpiSendRecv(dev, msg, 1, packetType, 1);
}

int SetRfFrequency(sx1280_spi_t *dev, uint8_t rfFrequency[3])
{
    uint8_t msg[4];
    msg[0] = SET_RF_FREQUENCY;
    MyMemcpy(msg + 1, rfFrequency, 3);
    return SpiSend(dev, msg, 4);
}

int SetTxParams(sx1280_spi_t *dev, uint8_t power, uint8_t rampTime)
{
    uint8_t msg[3] = {SET_TX_PARAMS, power, rampTime};
    return SpiSend(dev, msg, 3);
}

int SetCadParams(sx1280_spi_t *dev, uint8_t cadSymbolNum)
{
    uint8_t msg[2] = {SET_CAD_PARAMS, cadSymbolNum};
    return SpiSend(dev, msg, 2);
}

int SetBufferBaseAddress(sx1280_spi_t *dev, uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
    uint8_t msg[3] = {SET_BUFFER_BASE_ADDRESS, txBaseAddress, rxBaseAddress};
    return SpiSend(dev, msg, 3);
}

int SetModulationParams(sx1280_spi_t *dev, uint8_t modParam[3])
{
    uint8_t msg[4];
    msg[0] = SET_MODULATION_PARAMS;
    MyMemcpy(msg + 1, modParam, 3);
    if (SpiSend(dev, msg, 4) == -1)
    {
        return -1;
    }

    /* 0x1E Must be written to register 0x0925 for SF5 or SF6 */
    if (modParam[0] == 0x50 || modParam[0] == 0x60)
    {
        uint8_t msg[1] = {0x1E};
        if (WriteRegister(dev, 0x0925, msg, 1) == -1)
        {
            return -1;
        }
    }
    /* 0x37 Must be written to register 0x0925 for SF7 or SF8 */
    else if (modParam[0] == 0x70 || modParam[0] == 0x80)
    {
        uint8_t msg[1] = {0x37};
        if (WriteRegister(dev, 0x0925, msg, 1) == -1)
        {
            return -1;
        }
    }
    /* 0x32 Must be written to register 0x0925 for SF9, SF10, SF11, or SF12 */
    else if (modParam[0] == 0x90 || modParam[0] == 0xA0 || modParam[0] == 0xB0 || modParam[0] == 0xC0)
    {
        uint8_t msg[1] = {0x32};
        if (WriteRegister(dev, 0x0925, msg, 1) == -1)
        {
            return -1;
        }
    }
    return 0;
}

int SetPacketParams(sx1280_spi_t *dev, uint8_t packetParams[7])
{
    uint8_t msg[8];
    msg[0] = SET_PACKET_PARAMS;
    MyMemcpy(msg + 1, packetParams, 7);
    return SpiSend(dev, msg, 8);
}

int GetRxBufferStatus(sx1280_spi_t *dev, uint8_t recv[2])
{
    uint8_t msg[1] = {GET_RX_BUFFER_STATUS};
    return SpiSendRecv(dev, msg, 1, recv, 2);
}

int GetPacketStatus(sx1280_spi_t *dev, uint8_t recv[5])
{
    uint8_t msg[1] = {GET_PACKET_STATUS};
    return SpiSendRecv(dev, msg, 1, recv, 5);
}

int GetRssiLnst(sx1280_spi_t *dev, uint8_t *rssiLnst)
{
    uint8_t msg[1] = {GET_RSSI_LNST};
    return SpiSendRecv(dev, msg, 1, rssiLnst, 1);
}

int SetDioIrqParams(sx1280_spi_t *dev, uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t msg[9] = {
        SET_DIO_IRQ_PARAMS,
        (uint8_t)(irqMask >> 8),
        (uint8_t)irqMask,
        (uint8_t)(dio1Mask >> 8),
        (uint8_t)dio1Mask,
        (uint8_t)(dio2Mask >> 8),
        (uint8_t)dio2Mask,
        (uint8_t)(dio3Mask >> 8),
        (uint8_t)dio3Mask};
    return SpiSend(dev, msg, 9);
}

int GetIrqStatus(sx1280_spi_t *dev, uint16_t *irq)
{
    uint8_t msg[1] = {GET_IRQ_STATUS};
    uint8_t recv[2] = {0};
    int ret = SpiSendRecv(dev, msg, 1, recv, 2);
    *irq = ((uint16_t)recv[0] << 8) | recv[1];
    return ret;
}

int ClrIrqStatus(sx1280_spi_t *dev, uint16_t irqMask)
{
    uint8_t msg[3] = {
        CLR_IRQ_STATUS,
        (uint8_t)(irqMask >> 8),
        (uint8_t)(irqMask)};
    return SpiSend(dev, msg, 3);
}

int SetRegulatorMode(sx1280_spi_t *dev, uint8_t regulatorMode)
{
    uint8_t msg[2] = {SET_REGULATOR_MODE, regulatorMode};
    return SpiSend(dev, msg, 2);
}

int SetSaveContext(sx1280_spi_t *dev)
{
    uint8_t msg[1] = {SET_SAVE_CONTEXT};
    return SpiSend(dev, msg, 1);
}

int SetAutoFS(sx1280_spi_t *dev, uint8_t state)
{
    uint8_t msg[2] = {SET_AUTO_FS, state};
    return SpiSend(dev, msg, 2);
}

int SetAutoTx(sx1280_spi_t *dev, uint8_t time)
{
    uint8_t msg[3] = {
        SET_AUTO_TX,
        (uint8_t)(time >> 8),
        (uint8_t)time};
    return SpiSend(dev, msg, 3);
}

int SetPerfCounterMode(sx1280_spi_t *dev, uint8_t perfCounterMode)
{
    uint8_t msg[2] = {SET_PERF_COUNTER_MODE, perfCounterMode};
    return SpiSend(dev, msg, 2);
}

int SetLongPreamble(sx1280_spi_t *dev, uint8_t enable)
{
    uint8_t msg[2] = {SET_LONG_PREAMBLE, enable};
    return SpiSend(dev, msg, 2);
}

int SetUartSpeed(sx1280_spi_t *dev, uint8_t uartSpeed)
{
    uint8_t msg[2] = {SET_UART_SPEED, uartSpeed};
    return SpiSend(dev, msg, 2);
}

int SetRangingRole(sx1280_spi_t *dev, uint8_t mode)
{
    uint8_t msg[2] = {SET_RANGING_ROLE, mode};
    return SpiSend(dev, msg, 2);
}

int SetAdvancedRanging(sx1280_spi_t *dev, uint8_t state)
{
    uint8_t msg[2] = {SET_ADVANCED_RANGING, state};
    return SpiSend(dev, msg, 2);
}