#include "sx1280_spi.h"

void sx1280SPIInit(sx1280_spi_t *dev)
{
    gpio_init(dev->busyPin);
    gpio_set_dir(dev->busyPin, 0);

    gpio_init(dev->resetPin);
    gpio_set_dir(dev->resetPin, 1);
    gpio_put(dev->resetPin, 1);

    spi_init(dev->spi, 1000000);
    gpio_set_function(dev->sckPin, GPIO_FUNC_SPI);
    gpio_set_function(dev->mosiPin, GPIO_FUNC_SPI);
    gpio_set_function(dev->misoPin, GPIO_FUNC_SPI);

    gpio_init(dev->csPin);
    gpio_set_dir(dev->csPin, 1);
    gpio_put(dev->csPin, 1);
}

void waitBusyPin(sx1280_spi_t *dev)
{
    while (gpio_get(dev->busyPin) == 1)
    {
    }
}

void printBuffHex(uint8_t *buff, size_t len)
{
    for (int i = 0; i < len; i++)
    {
        printf("0x%X ", buff[i]);
    }
    printf("\n");
}

void printBuffDec(uint8_t *buff, size_t len)
{
    for (int i = 0; i < len; i++)
    {
        printf("%u ", buff[i]);
    }
    printf("\n");
}

void printBuffChar(uint8_t *buff, size_t len)
{
    char text[len + 1];
    myMemcpy(text, buff, len);
    text[len] = '\0';
    printf("%s\n", text);
}

void spiSend(sx1280_spi_t *dev, uint8_t *buff, size_t len)
{
    waitBusyPin(dev);
    gpio_put(dev->csPin, 0);
    spi_write_blocking(dev->spi, buff, len);
    gpio_put(dev->csPin, 1);
}

void myMemcpy(void *dest, void *src, size_t len)
{
    char *csrc = (char *)src;
    char *cdest = (char *)dest;

    for (int i = 0; i < len; i++)
        cdest[i] = csrc[i];
}

void spiSendRecv(sx1280_spi_t *dev, uint8_t *msgBuff, size_t msgLen, uint8_t *recvBuff, size_t recvLen)
{
    size_t len = msgLen + recvLen + 1;
    uint8_t *send = calloc(len, 1);
    uint8_t *recv = calloc(len, 1);
    myMemcpy(send, msgBuff, msgLen);
    waitBusyPin(dev);
    gpio_put(dev->csPin, 0);
    spi_write_read_blocking(dev->spi, send, recv, len);
    gpio_put(dev->csPin, 1);
    myMemcpy(recvBuff, recv + (msgLen + 1), recvLen);
    free(send);
    free(recv);
}

uint8_t GetStatus(sx1280_spi_t *dev)
{
    uint8_t send[1] = {GET_STATUS};
    uint8_t recv[1];

    waitBusyPin(dev);
    gpio_put(dev->csPin, 0);
    spi_write_read_blocking(dev->spi, send, recv, 1);
    gpio_put(dev->csPin, 1);

    return recv[0];
}

void WriteRegister(sx1280_spi_t *dev, uint16_t addr, uint8_t *data, size_t len)
{
    uint8_t msg[len + 3];
    msg[0] = WRITE_REGISTER;
    msg[1] = (uint8_t)(addr >> 8);
    msg[2] = (uint8_t)addr;
    myMemcpy(msg + 3, data, len);
    spiSend(dev, msg, len + 3);
}

void ReadRegister(sx1280_spi_t *dev, uint8_t *recv, size_t len, uint16_t addr)
{
    uint8_t msg[3] = {READ_REGISTER, (uint8_t)(addr >> 8), (uint8_t)addr};
    spiSendRecv(dev, msg, 3, recv, len);
}

void WriteBuffer(sx1280_spi_t *dev, uint8_t *data, size_t len)
{
    uint8_t msg[len + 2];
    msg[0] = WRITE_BUFFER;
    msg[1] = TX_BASE_ADDR;
    myMemcpy(msg + 2, data, len);
    spiSend(dev, msg, len + 2);
}

void ReadBuffer(sx1280_spi_t *dev, uint8_t *recv, size_t len, uint8_t addr)
{
    uint8_t msg[2] = {READ_BUFFER, addr};
    spiSendRecv(dev, msg, 2, recv, len);
}

void SetSleep(sx1280_spi_t *dev, uint8_t sleepConfig)
{
    uint8_t msg[2] = {SET_SLEEP, sleepConfig};
    spiSend(dev, msg, 2);
}

void SetStandby(sx1280_spi_t *dev, uint8_t standbyConfig)
{
    uint8_t msg[2] = {SET_STANDBY, standbyConfig};
    spiSend(dev, msg, 2);
}

void SetFs(sx1280_spi_t *dev)
{
    uint8_t msg[1] = {SET_FS};
    spiSend(dev, msg, 1);
}

void SetTx(sx1280_spi_t *dev, uint8_t periodBase, uint16_t periodBaseCount)
{
    uint8_t msg[4] = {
        SET_TX,
        periodBase,
        (uint8_t)(periodBaseCount >> 8),
        (uint8_t)periodBaseCount};
    spiSend(dev, msg, 4);
}

void SetRx(sx1280_spi_t *dev, uint8_t periodBase, uint16_t periodBaseCount)
{
    uint8_t msg[4] = {
        SET_RX,
        periodBase,
        (uint8_t)(periodBaseCount >> 8),
        (uint8_t)periodBaseCount};
    spiSend(dev, msg, 4);
}

void SetRxDutyCycle(sx1280_spi_t *dev, uint8_t periodBase, uint16_t rxPeriodBaseCount, uint16_t sleepPeriodBaseCount)
{
    uint8_t msg[6] = {
        SET_RX_DUTY_CYCLE,
        periodBase,
        (uint8_t)(rxPeriodBaseCount >> 8),
        (uint8_t)rxPeriodBaseCount,
        (uint8_t)(sleepPeriodBaseCount >> 8),
        (uint8_t)sleepPeriodBaseCount};
    spiSend(dev, msg, 6);
}

void SetCad(sx1280_spi_t *dev)
{
    uint8_t msg[1] = {SET_CAD};
    spiSend(dev, msg, 1);
}

void SetTxContinuousWave(sx1280_spi_t *dev)
{
    uint8_t msg[1] = {SET_TX_CONTINUOUS_WAVE};
    spiSend(dev, msg, 1);
}

void SetTxContinuousPreamble(sx1280_spi_t *dev)
{
    uint8_t msg[1] = {SET_TX_CONTINUOUS_PREAMBLE};
    spiSend(dev, msg, 1);
}

void SetPacketType(sx1280_spi_t *dev, uint8_t packetType)
{
    uint8_t msg[2] = {SET_PACKET_TYPE, packetType};
    spiSend(dev, msg, 2);
}

uint8_t GetPacketType(sx1280_spi_t *dev)
{
    uint8_t msg[1] = {GET_PACKET_TYPE};
    uint8_t recv[1];
    spiSendRecv(dev, msg, 1, recv, 1);
    return recv[0];
}

void SetRfFrequency(sx1280_spi_t *dev, uint8_t rfFrequency[3])
{
    uint8_t msg[4];
    msg[0] = SET_RF_FREQUENCY;
    myMemcpy(msg + 1, rfFrequency, 3);
    spiSend(dev, msg, 4);
}

void SetTxParams(sx1280_spi_t *dev, uint8_t power, uint8_t rampTime)
{
    uint8_t msg[3] = {SET_TX_PARAMS, power, rampTime};
    spiSend(dev, msg, 3);
}

void SetCadParams(sx1280_spi_t *dev, uint8_t cadSymbolNum)
{
    uint8_t msg[2] = {SET_CAD_PARAMS, cadSymbolNum};
    spiSend(dev, msg, 2);
}

void SetBufferBaseAddress(sx1280_spi_t *dev, uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
    uint8_t msg[3] = {SET_BUFFER_BASE_ADDRESS, txBaseAddress, rxBaseAddress};
    spiSend(dev, msg, 3);
}

void SetModulationParams(sx1280_spi_t *dev, uint8_t modParam[3])
{
    uint8_t msg[4];
    msg[0] = SET_MODULATION_PARAMS;
    myMemcpy(msg + 1, modParam, 3);
    spiSend(dev, msg, 4);

    /* 0x1E Must be written to register 0x0925 for SF5 or SF6 */
    if (modParam[0] == 0x50 || modParam[0] == 0x60)
    {
        uint8_t msg[1] = {0x1E};
        WriteRegister(dev, 0x0925, msg, 1);
    }
    /* 0x37 Must be written to register 0x0925 for SF7 or SF8 */
    else if (modParam[0] == 0x70 || modParam[0] == 0x80)
    {
        uint8_t msg[1] = {0x37};
        WriteRegister(dev, 0x0925, msg, 1);
    }
    /* 0x32 Must be written to register 0x0925 for SF9, SF10, SF11, or SF12 */
    else if (modParam[0] == 0x90 || modParam[0] == 0xA0 || modParam[0] == 0xB0 || modParam[0] == 0xC0)
    {
        uint8_t msg[1] = {0x32};
        WriteRegister(dev, 0x0925, msg, 1);
    }
}

void SetPacketParams(sx1280_spi_t *dev, uint8_t packetParams[7])
{
    uint8_t msg[8];
    msg[0] = SET_PACKET_PARAMS;
    myMemcpy(msg + 1, packetParams, 7);
    spiSend(dev, msg, 8);
}

void GetRxBufferStatus(sx1280_spi_t *dev, uint8_t recv[2])
{
    uint8_t msg[1] = {GET_RX_BUFFER_STATUS};
    spiSendRecv(dev, msg, 1, recv, 2);
}

void GetPacketStatus(sx1280_spi_t *dev, uint8_t recv[5])
{
    uint8_t msg[1] = {GET_PACKET_STATUS};
    spiSendRecv(dev, msg, 1, recv, 5);
}

uint8_t GetRssiLnst(sx1280_spi_t *dev)
{
    uint8_t msg[1] = {GET_RSSI_LNST};
    uint8_t recv[1];
    spiSendRecv(dev, msg, 1, recv, 1);
    return recv[0];
}

void SetDioIrqParams(sx1280_spi_t *dev, uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
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
    spiSend(dev, msg, 9);
}

uint16_t GetIrqStatus(sx1280_spi_t *dev)
{
    uint8_t msg[1] = {GET_IRQ_STATUS};
    uint8_t recv[2];
    spiSendRecv(dev, msg, 1, recv, 2);
    uint16_t irq = ((uint16_t)recv[0] << 8) | recv[1];
    return irq;
}

void ClrIrqStatus(sx1280_spi_t *dev, uint16_t irqMask)
{
    uint8_t msg[3] = {
        CLR_IRQ_STATUS,
        (uint8_t)(irqMask >> 8),
        (uint8_t)(irqMask)};
    spiSend(dev, msg, 3);
}

void SetRegulatorMode(sx1280_spi_t *dev, uint8_t regulatorMode)
{
    uint8_t msg[2] = {SET_REGULATOR_MODE, regulatorMode};
    spiSend(dev, msg, 2);
}

void SetSaveContext(sx1280_spi_t *dev)
{
    uint8_t msg[1] = {SET_SAVE_CONTEXT};
    spiSend(dev, msg, 1);
}

void SetAutoFS(sx1280_spi_t *dev, uint8_t state)
{
    uint8_t msg[2] = {SET_AUTO_FS, state};
    spiSend(dev, msg, 2);
}

void SetAutoTx(sx1280_spi_t *dev, uint8_t time)
{
    uint8_t msg[3] = {
        SET_AUTO_TX,
        (uint8_t)(time >> 8),
        (uint8_t)time};
    spiSend(dev, msg, 3);
}

void SetPerfCounterMode(sx1280_spi_t *dev, uint8_t perfCounterMode)
{
    uint8_t msg[2] = {SET_PERF_COUNTER_MODE, perfCounterMode};
    spiSend(dev, msg, 2);
}

void SetLongPreamble(sx1280_spi_t *dev, uint8_t enable)
{
    uint8_t msg[2] = {SET_LONG_PREAMBLE, enable};
    spiSend(dev, msg, 2);
}

void SetUartSpeed(sx1280_spi_t *dev, uint8_t uartSpeed)
{
    uint8_t msg[2] = {SET_UART_SPEED, uartSpeed};
    spiSend(dev, msg, 2);
}

void SetRangingRole(sx1280_spi_t *dev, uint8_t mode)
{
    uint8_t msg[2] = {SET_RANGING_ROLE, mode};
    spiSend(dev, msg, 2);
}

void SetAdvancedRanging(sx1280_spi_t *dev, uint8_t state)
{
    uint8_t msg[2] = {SET_ADVANCED_RANGING, state};
    spiSend(dev, msg, 2);
}
