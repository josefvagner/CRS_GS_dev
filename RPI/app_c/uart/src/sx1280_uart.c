#include "sx1280_uart.h"

long long millis()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return t.tv_sec * 1e3 + (t.tv_nsec + 5e5) / 1e6;
}

void sx1280UartInit()
{
    pi = pigpio_start(NULL, NULL);
    if (pi < 0)
    {
        perror("Can't connect to pigpiod (run sudo pigpiod)");
        exit(100);
    }

    if (get_mode(pi, RESET_PIN) != PI_OUTPUT)
    {
        set_mode(pi, RESET_PIN, PI_OUTPUT);
    }

    if (get_mode(pi, BUSY_PIN) != PI_INPUT)
    {
        set_mode(pi, BUSY_PIN, PI_INPUT);
    }

    if (get_mode(pi, UART_RTS_PIN) != PI_OUTPUT)
    {
        set_mode(pi, UART_RTS_PIN, PI_OUTPUT);
    }

    if (get_mode(pi, UART_RTS_PIN) != PI_ALT3)
    {
        set_mode(pi, UART_RTS_PIN, PI_ALT3);
    }

    if (get_mode(pi, UART_CTS_PIN) != PI_ALT3)
    {
        set_mode(pi, UART_CTS_PIN, PI_ALT3);
    }

    if (get_mode(pi, UART_RX_PIN) != PI_ALT0)
    {
        set_mode(pi, UART_RX_PIN, PI_ALT0);
    }

    if (get_mode(pi, UART_TX_PIN) != PI_ALT0)
    {
        set_mode(pi, UART_TX_PIN, PI_ALT0);
    }

    uart = open("/dev/serial0", O_RDWR | O_NOCTTY);
    usleep(1000);
    if (uart == -1)
    {
        printf("uart setup error");
        exit(101);
    }

    struct termios options;
    tcgetattr(uart, &options);
    // cfsetispeed(&options, B115200);
    // cfsetospeed(&options, B115200);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD | PARENB;
    // options.c_cflag &= ~ICANON;
    options.c_iflag = INPCK;
    options.c_lflag = 0;
    options.c_oflag = 0;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;
    tcflush(uart, TCIOFLUSH);
    tcsetattr(uart, TCSANOW, &options);
}

int resetSx1280()
{
    printf("Setting/Resseting sx1280\n");
    if (get_mode(pi, RESET_PIN) != PI_OUTPUT)
    {
        set_mode(pi, RESET_PIN, PI_OUTPUT);
    }

    if (get_mode(pi, BUSY_PIN) != PI_INPUT)
    {
        set_mode(pi, BUSY_PIN, PI_INPUT);
    }

    if (get_mode(pi, UART_RTS_PIN) != PI_OUTPUT)
    {
        set_mode(pi, UART_RTS_PIN, PI_OUTPUT);
    }

    gpio_write(pi, RESET_PIN, 0);
    gpio_write(pi, UART_RTS_PIN, 0);
    usleep(2000);
    gpio_write(pi, RESET_PIN, 1);

    if (get_mode(pi, UART_RTS_PIN) != PI_ALT3)
    {
        set_mode(pi, UART_RTS_PIN, PI_ALT3);
    }

    if (SetStandby(0x00) == -1)
    {
        return -1;
    }

    if (SetPacketType(0x01) == -1)
    {
        return -1;
    }

    uint8_t rfFreq[3] = {0xB8, 0x9D, 0x89};
    if (SetRfFrequency(rfFreq) == -1)
    {
        return -1;
    }

    if (SetBufferBaseAddress(TX_BASE_ADDR, RX_BASE_ADDR) == -1)
    {
        return -1;
    }

    uint8_t modParams[3] = {MOD_PARAM_1, MOD_PARAM_2, MOD_PARAM_3};
    if (SetModulationParams(modParams) == -1)
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
    if (SetPacketParams(packetParams) == -1)
    {
        return -1;
    }

    if (SetRxDutyCycle(0x03, 0, 0x00FA) == -1)
    {
        return -1;
    }

    if (SetDioIrqParams(0b0100000001100010, 0, 0, 0) == -1)
    {
        return -1;
    }

    uint8_t pt = 0;
    if (GetPacketType(&pt) == -1 || pt != 1)
    {
        return -1;
    }

    if (ClrIrqStatus(0xFFFF) == -1)
    {
        return -1;
    }
    printf("Sx1280 reset/setup complete\n");

    return 0;
}

void waitForSetup()
{
    while (resetSx1280() == -1 || SetRx(0x02, 0xFFFF) == -1)
    {
        perror("setup error");
    }
}

int waitBusyPin()
{
    long long t0 = millis();
    while (gpio_read(pi, BUSY_PIN) == 1)
    {
        if ((millis() - t0) >= 1000)
        {
            perror("waitBusyPin timeout");
            return -1;
        }
    }
    return 0;
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

int uartSend(uint8_t *buff, uint8_t len)
{
    tcflush(uart, TCIOFLUSH);
    if (waitBusyPin() == -1)
    {
        return -1;
    }

    // printBuffHex(buff, len);

    if (uart != -1)
    {
        int count = 0;
        while (count < len)
        {
            int ret = write(uart, buff + count, len - count); // Filestream, bytes to write, number of bytes to write
            // printf("write bytes = %d\n", ret);
            if (ret <= 0)
            {
                perror("uartSend write error");
                return -1;
            }
            count += ret;
        }
    }
    return 0;
}

void myMemcpy(void *dest, void *src, size_t len)
{
    char *csrc = (char *)src;
    char *cdest = (char *)dest;

    for (int i = 0; i < len; i++)
        cdest[i] = csrc[i];
}

int uartSendRecv(uint8_t *msgBuff, uint8_t msgLen, uint8_t *recvBuff, uint8_t recvLen)
{
    tcflush(uart, TCIOFLUSH);
    if (waitBusyPin() == -1)
    {
        return -1;
    }

    // printBuffHex(msgBuff, msgLen);

    if (uart != -1)
    {
        int count = 0;
        while (count < msgLen)
        {
            int ret = write(uart, msgBuff + count, msgLen - count); // Filestream, bytes to write, number of bytes to write
            if (ret <= 0)
            {
                perror("uartSendRecv write error");
                return -1;
            }
            count += ret;
        }
    }
    if (uart != -1)
    {
        int count = 0;
        while (count < recvLen)
        {
            // printf("read\n");
            int ret = read(uart, recvBuff + count, recvLen - count); // Filestream, bytes to write, number of bytes to write
            // printf("read ret = %d\n", ret);
            if (ret <= 0)
            {
                perror("uartSendRecv read error");
                return -1;
            }
            count += ret;
        }
    }
    return 0;
}

int GetStatus(uint8_t *status)
{
    uint8_t msg[1];
    msg[0] = GET_STATUS;
    return uartSendRecv(msg, 1, status, 1);
}

int WriteRegister(uint16_t addr, uint8_t *data, size_t len)
{
    uint8_t msg[len + 4];
    msg[0] = WRITE_REGISTER;
    msg[1] = (uint8_t)(addr >> 8);
    msg[2] = (uint8_t)addr;
    msg[3] = (uint8_t)len;
    myMemcpy(msg + 4, data, len);
    return uartSend(msg, len + 4);
}

int ReadRegister(uint8_t *recv, size_t len, uint16_t addr)
{
    uint8_t msg[4] = {READ_REGISTER, (uint8_t)(addr >> 8), (uint8_t)addr, (uint8_t)len};
    return uartSendRecv(msg, 4, recv, len);
}

int WriteBuffer(uint8_t *data, size_t len)
{
    uint8_t msg[len + 3];
    msg[0] = WRITE_BUFFER;
    msg[1] = TX_BASE_ADDR;
    msg[2] = (uint8_t)len;
    myMemcpy(msg + 3, data, len);
    return uartSend(msg, len + 3);
}

int ReadBuffer(uint8_t *recv, uint8_t len, uint8_t addr)
{
    uint8_t msg[3] = {READ_BUFFER, addr, len};
    return uartSendRecv(msg, 3, recv, len);
}

int SetSleep(uint8_t sleepConfig)
{
    uint8_t msg[2] = {SET_SLEEP, sleepConfig};
    return uartSend(msg, 2);
}

int SetStandby(uint8_t standbyConfig)
{
    uint8_t msg[3] = {SET_STANDBY, 0x01, standbyConfig};
    return uartSend(msg, 3);
}

int SetFs()
{
    uint8_t msg[1] = {SET_FS};
    return uartSend(msg, 1);
}

int SetTx(uint8_t periodBase, uint16_t periodBaseCount)
{
    uint8_t msg[5] = {
        SET_TX,
        0x03,
        periodBase,
        (uint8_t)(periodBaseCount >> 8),
        (uint8_t)periodBaseCount};
    return uartSend(msg, 5);
}

int SetRx(uint8_t periodBase, uint16_t periodBaseCount)
{
    uint8_t msg[5] = {
        SET_RX,
        0x03,
        periodBase,
        (uint8_t)(periodBaseCount >> 8),
        (uint8_t)periodBaseCount};
    return uartSend(msg, 5);
}

int SetRxDutyCycle(uint8_t periodBase, uint16_t rxPeriodBaseCount, uint16_t sleepPeriodBaseCount)
{
    uint8_t msg[7] = {
        SET_RX_DUTY_CYCLE,
        0x05,
        periodBase,
        (uint8_t)(rxPeriodBaseCount >> 8),
        (uint8_t)rxPeriodBaseCount,
        (uint8_t)(sleepPeriodBaseCount >> 8),
        (uint8_t)sleepPeriodBaseCount};
    return uartSend(msg, 7);
}

int SetCad()
{
    uint8_t msg[1] = {SET_CAD};
    return uartSend(msg, 1);
}

int SetTxContinuousWave()
{
    uint8_t msg[1] = {SET_TX_CONTINUOUS_WAVE};
    return uartSend(msg, 1);
}

int SetTxContinuousPreamble()
{
    uint8_t msg[1] = {SET_TX_CONTINUOUS_PREAMBLE};
    return uartSend(msg, 1);
}

int SetPacketType(uint8_t packetType)
{
    uint8_t msg[3] = {SET_PACKET_TYPE, 0x01, packetType};
    return uartSend(msg, 3);
}

int GetPacketType(uint8_t *packetType)
{
    uint8_t msg[2] = {GET_PACKET_TYPE, 0x01};
    return uartSendRecv(msg, 2, packetType, 1);
}

int SetRfFrequency(uint8_t rfFrequency[3])
{
    uint8_t msg[5];
    msg[0] = SET_RF_FREQUENCY;
    msg[1] = 0x03;
    myMemcpy(msg + 2, rfFrequency, 3);
    return uartSend(msg, 5);
}

int SetTxParams(uint8_t power, uint8_t rampTime)
{
    uint8_t msg[4] = {SET_TX_PARAMS, 0x02, power, rampTime};
    return uartSend(msg, 4);
}

int SetCadParams(uint8_t cadSymbolNum)
{
    uint8_t msg[3] = {SET_CAD_PARAMS, 0x01, cadSymbolNum};
    return uartSend(msg, 3);
}

int SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
    uint8_t msg[4] = {SET_BUFFER_BASE_ADDRESS, 0x02, txBaseAddress, rxBaseAddress};
    return uartSend(msg, 4);
}

int SetModulationParams(uint8_t modParam[3])
{
    uint8_t msg[5];
    msg[0] = SET_MODULATION_PARAMS;
    msg[1] = 0x03;
    myMemcpy(msg + 2, modParam, 3);
    if (uartSend(msg, 5) == -1)
    {
        return -1;
    }

    /* 0x1E Must be written to register 0x0925 for SF5 or SF6 */
    if (modParam[0] == 0x50 || modParam[0] == 0x60)
    {
        uint8_t msg[1] = {0x1E};
        if (WriteRegister(0x0925, msg, 1) == -1)
        {
            return -1;
        }
    }
    /* 0x37 Must be written to register 0x0925 for SF7 or SF8 */
    else if (modParam[0] == 0x70 || modParam[0] == 0x80)
    {
        uint8_t msg[1] = {0x37};
        if (WriteRegister(0x0925, msg, 1) == -1)
        {
            return -1;
        }
    }
    /* 0x32 Must be written to register 0x0925 for SF9, SF10, SF11, or SF12 */
    else if (modParam[0] == 0x90 || modParam[0] == 0xA0 || modParam[0] == 0xB0 || modParam[0] == 0xC0)
    {
        uint8_t msg[1] = {0x32};
        if (WriteRegister(0x0925, msg, 1) == -1)
        {
            return -1;
        }
    }
}

int SetPacketParams(uint8_t packetParams[7])
{
    uint8_t msg[9];
    msg[0] = SET_PACKET_PARAMS;
    msg[1] = 0x07;
    myMemcpy(msg + 2, packetParams, 7);
    return uartSend(msg, 9);
}

int GetRxBufferStatus(uint8_t recv[2])
{
    uint8_t msg[2] = {GET_RX_BUFFER_STATUS, 0x02};
    return uartSendRecv(msg, 2, recv, 2);
}

int GetPacketStatus(uint8_t recv[5])
{
    uint8_t msg[2] = {GET_PACKET_STATUS, 0x05};
    return uartSendRecv(msg, 2, recv, 5);
}

int GetRssiLnst(uint8_t *rssiLnst)
{
    uint8_t msg[2] = {GET_RSSI_LNST, 0x01};
    uartSendRecv(msg, 2, rssiLnst, 1);
}

int SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
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
    return uartSend(msg, 10);
}

int GetIrqStatus(uint16_t *irq)
{
    uint8_t msg[2] = {GET_IRQ_STATUS, 0x02};
    uint8_t recv[2] = {0};
    int ret = uartSendRecv(msg, 2, recv, 2);
    *irq = ((uint16_t)recv[0] << 8) | recv[1];
    return ret;
}

int ClrIrqStatus(uint16_t irqMask)
{
    uint8_t msg[4] = {
        CLR_IRQ_STATUS,
        0x02,
        (uint8_t)(irqMask >> 8),
        (uint8_t)(irqMask)};
    return uartSend(msg, 4);
}

int SetRegulatorMode(uint8_t regulatorMode)
{
    uint8_t msg[3] = {SET_REGULATOR_MODE, 0x01, regulatorMode};
    return uartSend(msg, 3);
}

int SetSaveContext()
{
    uint8_t msg[1] = {SET_SAVE_CONTEXT};
    return uartSend(msg, 1);
}

int SetAutoFS(uint8_t state)
{
    uint8_t msg[3] = {SET_AUTO_FS, 0x01, state};
    return uartSend(msg, 3);
}

int SetAutoTx(uint8_t time)
{
    uint8_t msg[4] = {
        SET_AUTO_TX,
        0x02,
        (uint8_t)(time >> 8),
        (uint8_t)time};
    return uartSend(msg, 4);
}

int SetPerfCounterMode(uint8_t perfCounterMode)
{
    uint8_t msg[3] = {SET_PERF_COUNTER_MODE, 0x01, perfCounterMode};
    return uartSend(msg, 3);
}

int SetLongPreamble(uint8_t enable)
{
    uint8_t msg[3] = {SET_LONG_PREAMBLE, 0x01, enable};
    return uartSend(msg, 3);
}

int SetUartSpeed(uint8_t uartSpeed)
{
    uint8_t msg[3] = {SET_UART_SPEED, 0x01, uartSpeed};
    return uartSend(msg, 3);
}

int SetRangingRole(uint8_t mode)
{
    uint8_t msg[3] = {SET_RANGING_ROLE, 0x01, mode};
    return uartSend(msg, 3);
}

int SetAdvancedRanging(uint8_t state)
{
    uint8_t msg[3] = {SET_ADVANCED_RANGING, 0x01, state};
    return uartSend(msg, 3);
}
