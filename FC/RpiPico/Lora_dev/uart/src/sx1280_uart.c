#include "sx1280_uart.h"

void sx1280UartInit()
{
    gpio_init(RESET_PIN);
    gpio_init(BUSY_PIN);
    gpio_init(UART_RTS_PIN);

    gpio_set_dir(RESET_PIN, GPIO_OUT);
    gpio_set_dir(BUSY_PIN, GPIO_IN);
    gpio_set_dir(UART_RTS_PIN, GPIO_OUT);

    gpio_put(RESET_PIN, false);
    gpio_put(UART_RTS_PIN, false);
    sleep_ms(50);
    gpio_put(RESET_PIN, true);

    gpio_deinit(UART_RTS_PIN);

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_CTS_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RTS_PIN, GPIO_FUNC_UART);

    uart_init(UART_ID, BAUD_RATE);
    uart_set_hw_flow(UART_ID, true, true);
    uart_set_fifo_enabled(UART_ID, true);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
}

void waitBusyPin()
{
    while (gpio_get(BUSY_PIN) == 1)
    {
        sleep_ms(100);
    }
}

void initBuffer(uint8_t *buff, size_t len)
{
    buff = (uint8_t *)malloc(len);
    if (buff == NULL)
    {
        // Handle memory allocation failure
        perror("Failed to allocate memory");
        exit(EXIT_FAILURE);
    }
}

void resizeBuffer(uint8_t *buff, size_t newLen)
{
    buff = (uint8_t *)realloc(buff, newLen);
    if (buff == NULL)
    {
        // Handle memory allocation failure
        perror("Failed to allocate memory");
        exit(EXIT_FAILURE);
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

void uartSend(uint8_t *buff, size_t len)
{
    // printf("Sending buffer: ");
    // printBuffHex(buff, len);
    uart_write_blocking(UART_ID, buff, len);
    waitBusyPin();
}

void myMemcpy(void *dest, void *src, size_t len)
{
    char *csrc = (char *)src;
    char *cdest = (char *)dest;

    for (int i = 0; i < len; i++)
        cdest[i] = csrc[i];
}

void uartSendRecv(uint8_t *msgBuff, size_t msgLen, uint8_t *recvBuff, size_t recvLen)
{
    while (uart_is_readable(UART_ID))
    {
        char _ = uart_getc(UART_ID);
    }

    uart_write_blocking(UART_ID, msgBuff, msgLen);
    uart_read_blocking(UART_ID, recvBuff, recvLen);
    waitBusyPin();
    /*
    printf("Sending buffer: ");
    printBuffHex(msgBuff, msgLen);
    printf("Recieving buffer: ");
    printBuffHex(recvBuff, recvLen);
    */
}

uint8_t GetStatus()
{
    uint8_t msg[1];
    uint8_t recv[1];
    msg[0] = GET_STATUS;
    uartSendRecv(msg, 1, recv, 1);
    return recv[0];
}

void WriteRegister(uint16_t addr, uint8_t *data, size_t len)
{
    uint8_t msg[len + 4];
    msg[0] = WRITE_REGISTER;
    msg[1] = (uint8_t)(addr >> 8);
    msg[2] = (uint8_t)addr;
    msg[3] = (uint8_t)len;
    myMemcpy(msg + 4, data, len);
    uartSend(msg, len + 4);
}

void ReadRegister(uint8_t *recv, size_t len, uint16_t addr)
{
    uint8_t msg[4] = {READ_REGISTER, (uint8_t)(addr >> 8), (uint8_t)addr, (uint8_t)len};
    uartSendRecv(msg, 4, recv, len);
}

void WriteBuffer(uint8_t *data, size_t len)
{
    uint8_t msg[len + 3];
    msg[0] = WRITE_BUFFER;
    msg[1] = TX_BASE_ADDR;
    msg[2] = (uint8_t)len;
    myMemcpy(msg + 3, data, len);
    uartSend(msg, len + 3);
}

void ReadBuffer(uint8_t *recv, size_t len, uint8_t addr)
{
    uint8_t msg[3] = {READ_BUFFER, addr, (uint8_t)len};
    uartSendRecv(msg, 3, recv, len);
}

void SetSleep(uint8_t sleepConfig)
{
    uint8_t msg[2] = {SET_SLEEP, sleepConfig};
    uartSend(msg, 2);
}

void SetStandby(uint8_t standbyConfig)
{
    uint8_t msg[3] = {SET_STANDBY, 0x01, standbyConfig};
    uartSend(msg, 3);
}

void SetFs()
{
    uint8_t msg[1] = {SET_FS};
    uartSend(msg, 1);
}

void SetTx(uint8_t periodBase, uint16_t periodBaseCount)
{
    uint8_t msg[5] = {
        SET_TX,
        0x03,
        periodBase,
        (uint8_t)(periodBaseCount >> 8),
        (uint8_t)periodBaseCount};
    uartSend(msg, 5);
}

void SetRx(uint8_t periodBase, uint16_t periodBaseCount)
{
    uint8_t msg[5] = {
        SET_RX,
        0x03,
        periodBase,
        (uint8_t)(periodBaseCount >> 8),
        (uint8_t)periodBaseCount};
    uartSend(msg, 5);
}

void SetRxDutyCycle(uint8_t periodBase, uint16_t rxPeriodBaseCount, uint16_t sleepPeriodBaseCount)
{
    uint8_t msg[7] = {
        SET_RX_DUTY_CYCLE,
        0x05,
        periodBase,
        (uint8_t)(rxPeriodBaseCount >> 8),
        (uint8_t)rxPeriodBaseCount,
        (uint8_t)(sleepPeriodBaseCount >> 8),
        (uint8_t)sleepPeriodBaseCount};
    uartSend(msg, 7);
}

void SetCad()
{
    uint8_t msg[1] = {SET_CAD};
    uartSend(msg, 1);
}

void SetTxContinuousWave()
{
    uint8_t msg[1] = {SET_TX_CONTINUOUS_WAVE};
    uartSend(msg, 1);
}

void SetTxContinuousPreamble()
{
    uint8_t msg[1] = {SET_TX_CONTINUOUS_PREAMBLE};
    uartSend(msg, 1);
}

void SetPacketType(uint8_t packetType)
{
    uint8_t msg[3] = {SET_PACKET_TYPE, 0x01, packetType};
    uartSend(msg, 3);
}

uint8_t GetPacketType()
{
    uint8_t msg[2] = {GET_PACKET_TYPE, 0x01};
    uint8_t recv[1];
    uartSendRecv(msg, 2, recv, 1);
    return recv[0];
}

void SetRfFrequency(uint8_t rfFrequency[3])
{
    uint8_t msg[5];
    msg[0] = SET_RF_FREQUENCY;
    msg[1] = 0x03;
    myMemcpy(msg + 2, rfFrequency, 3);
    uartSend(msg, 5);
}

void SetTxParams(uint8_t power, uint8_t rampTime)
{
    uint8_t msg[4] = {SET_TX_PARAMS, 0x02, power, rampTime};
    uartSend(msg, 4);
}

void SetCadParams(uint8_t cadSymbolNum)
{
    uint8_t msg[3] = {SET_CAD_PARAMS, 0x01, cadSymbolNum};
    uartSend(msg, 3);
}

void SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
    uint8_t msg[4] = {SET_BUFFER_BASE_ADDRESS, 0x02, txBaseAddress, rxBaseAddress};
    uartSend(msg, 4);
}

void SetModulationParams(uint8_t modParam[3])
{
    uint8_t msg[5];
    msg[0] = SET_MODULATION_PARAMS;
    msg[1] = 0x03;
    myMemcpy(msg + 2, modParam, 3);
    uartSend(msg, 5);

    /* 0x1E Must be written to register 0x0925 for SF5 or SF6 */
    if (modParam[0] == 0x50 || modParam[0] == 0x60)
    {
        uint8_t msg[1] = {0x1E};
        WriteRegister(0x0925, msg, 1);
    }
    /* 0x37 Must be written to register 0x0925 for SF7 or SF8 */
    else if (modParam[0] == 0x70 || modParam[0] == 0x80)
    {
        uint8_t msg[1] = {0x37};
        WriteRegister(0x0925, msg, 1);
    }
    /* 0x32 Must be written to register 0x0925 for SF9, SF10, SF11, or SF12 */
    else if (modParam[0] == 0x90 || modParam[0] == 0xA0 || modParam[0] == 0xB0 || modParam[0] == 0xC0)
    {
        uint8_t msg[1] = {0x32};
        WriteRegister(0x0925, msg, 1);
    }
}

void SetPacketParams(uint8_t packetParams[7])
{
    uint8_t msg[9];
    msg[0] = SET_PACKET_PARAMS;
    msg[1] = 0x07;
    myMemcpy(msg + 2, packetParams, 7);
    uartSend(msg, 9);
}

void GetRxBufferStatus(uint8_t recv[2])
{
    uint8_t msg[2] = {GET_RX_BUFFER_STATUS, 0x02};
    uartSendRecv(msg, 2, recv, 2);
}

void GetPacketStatus(uint8_t recv[5])
{
    uint8_t msg[2] = {GET_PACKET_STATUS, 0x05};
    uartSendRecv(msg, 2, recv, 5);
}

uint8_t GetRssiLnst()
{
    uint8_t msg[2] = {GET_RSSI_LNST, 0x01};
    uint8_t recv[1];
    uartSendRecv(msg, 2, recv, 1);
    return recv[0];
}

void SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t msg[10] = {
        SET_DIO_IRQ_PARAMS,
        0x08,
        (uint8_t)(irqMask >> 8),
        (uint8_t)irqMask,
        (uint8_t)(dio1Mask >> 8),
        (uint8_t)dio1Mask,
        (uint8_t)(dio2Mask >> 8),
        (uint8_t)dio2Mask,
        (uint8_t)(dio3Mask >> 8),
        (uint8_t)dio3Mask};
    uartSend(msg, 10);
}

uint16_t GetIrqStatus()
{
    uint8_t msg[2] = {GET_IRQ_STATUS, 0x02};
    uint8_t recv[2];
    uartSendRecv(msg, 2, recv, 2);
    uint16_t irq = ((uint16_t)recv[0] << 8) | recv[1];
    return irq;
}

void ClrIrqStatus(uint16_t irqMask)
{
    uint8_t msg[4] = {
        CLR_IRQ_STATUS,
        0x02,
        (uint8_t)(irqMask >> 8),
        (uint8_t)(irqMask)};
    uartSend(msg, 4);
}

void SetRegulatorMode(uint8_t regulatorMode)
{
    uint8_t msg[3] = {SET_REGULATOR_MODE, 0x01, regulatorMode};
    uartSend(msg, 3);
}

void SetSaveContext()
{
    uint8_t msg[1] = {SET_SAVE_CONTEXT};
    uartSend(msg, 1);
}

void SetAutoFS(uint8_t state)
{
    uint8_t msg[3] = {SET_AUTO_FS, 0x01, state};
    uartSend(msg, 3);
}

void SetAutoTx(uint8_t time)
{
    uint8_t msg[4] = {
        SET_AUTO_TX,
        0x02,
        (uint8_t)(time >> 8),
        (uint8_t)time};
    uartSend(msg, 4);
}

void SetPerfCounterMode(uint8_t perfCounterMode)
{
    uint8_t msg[3] = {SET_PERF_COUNTER_MODE, 0x01, perfCounterMode};
    uartSend(msg, 3);
}

void SetLongPreamble(uint8_t enable)
{
    uint8_t msg[3] = {SET_LONG_PREAMBLE, 0x01, enable};
    uartSend(msg, 3);
}

void SetUartSpeed(uint8_t uartSpeed)
{
    uint8_t msg[3] = {SET_UART_SPEED, 0x01, uartSpeed};
    uartSend(msg, 3);
}

void SetRangingRole(uint8_t mode)
{
    uint8_t msg[3] = {SET_RANGING_ROLE, 0x01, mode};
    uartSend(msg, 3);
}

void SetAdvancedRanging(uint8_t state)
{
    uint8_t msg[3] = {SET_ADVANCED_RANGING, 0x01, state};
    uartSend(msg, 3);
}
