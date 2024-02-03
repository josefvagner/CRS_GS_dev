#include "sx1280_uart.h"

void sx1280UartInit()
{
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_CTS_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RTS_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, true, true);

    gpio_init(RESET_PIN);
    gpio_set_dir(RESET_PIN, GPIO_OUT);
    gpio_put(RESET_PIN, true);

    gpio_init(BUSY_PIN);
    gpio_set_dir(BUSY_PIN, GPIO_IN);
}

void waitBusyPin()
{
    while (gpio_get(BUSY_PIN) == 1)
    {
        sleep_ms(100);
    }
}

void uartSend(buff_t buff)
{
    uart_write_blocking(UART_ID, buff.data, buff.len);

    while (gpio_get(BUSY_PIN) == 1)
    {
        sleep_ms(100);
    }
}

void myMemcpy(void *dest, void *src, size_t n)
{
    char *csrc = (char *)src;
    char *cdest = (char *)dest;

    for (int i = 0; i < n; i++)
        cdest[i] = csrc[i];
}

buff_t uartSendRecv(buff_t buff, size_t responseLen)
{
    uint8_t response[responseLen];
    buff_t ret;
    ret.data = response;
    ret.len = responseLen;
    printf("uart send \n");
    uart_write_blocking(UART_ID, buff.data, buff.len * sizeof(uint8_t));
    printf("uart recv \n");
    uart_read_blocking(UART_ID, ret.data, ret.len);

    while (gpio_get(BUSY_PIN) == 1)
    {
        printf("b ");
        sleep_ms(100);
    }
    printf("\n");

    return ret;
}

uint8_t GetStatus()
{
    uint8_t data[1] = {GET_STATUS};
    buff_t sendBuff = {data, 1};
    buff_t ret = uartSendRecv(sendBuff, 1);
    return *(ret.data);
}

void WriteRegister(uint16_t addr, buff_t buff)
{
    size_t len = buff.len + 4;
    uint8_t data[len];
    data[0] = WRITE_REGISTER;
    data[1] = addr & 0xff;
    data[2] = (addr >> 8) & 0xff;
    data[3] = (uint8_t)buff.len;
    myMemcpy(data + 4, buff.data, buff.len);
    buff_t sendBuff = {data, len};
    uartSend(sendBuff);
}

buff_t ReadRegister(uint16_t addr, size_t n)
{
    uint8_t data[4] = {
        READ_REGISTER,
        addr & 0xff,
        (addr >> 8) & 0xff,
        (uint8_t)n};
    buff_t sendBuff = {data, 4};
    buff_t ret = uartSendRecv(sendBuff, n);
    return ret;
}

void WriteBuffer(buff_t buff)
{
    size_t len = buff.len + 3;
    uint8_t data[len];
    data[0] = WRITE_BUFFER;
    data[1] = TX_BASE_ADDR;
    data[3] = (uint8_t)buff.len;
    myMemcpy(data + 3, buff.data, buff.len);
    buff_t sendBuff = {data, len};
    uartSend(sendBuff);
}

buff_t ReadBuffer(uint8_t n)
{
    uint8_t data[3] = {
        READ_BUFFER,
        RX_BASE_ADDR,
        n};
    buff_t sendBuff = {data, 3};
    buff_t ret = uartSendRecv(sendBuff, n);
    return ret;
}

void SetSleep(uint8_t sleepConfig)
{
    uint8_t data[2] = {SET_SLEEP, sleepConfig};
    buff_t sendBuff = {data, 2};
    uartSend(sendBuff);
}

void SetStandby(uint8_t standbyConfig)
{
    uint8_t data[3] = {SET_STANDBY, 0x01, standbyConfig};
    buff_t sendBuff = {data, 3};
    uartSend(sendBuff);
}

void SetFs()
{
    uint8_t data[1] = {SET_FS};
    buff_t sendBuff = {data, 1};
    uartSend(sendBuff);
}

void SetTx(uint8_t periodBase, uint16_t periodBaseCount)
{
    uint8_t data[5] = {
        SET_TX,
        0x03,
        periodBase,
        (uint8_t)(periodBaseCount & 0xff),
        (uint8_t)((periodBaseCount >> 8) & 0xff)};
    buff_t sendBuff = {data, 5};
    uartSend(sendBuff);
}

void SetRx(uint8_t periodBase, uint16_t periodBaseCount)
{
    uint8_t data[5] = {
        SET_RX,
        0x03,
        periodBase,
        (uint8_t)(periodBaseCount & 0xff),
        (uint8_t)((periodBaseCount >> 8) & 0xff)};
    buff_t sendBuff = {data, 5};
    uartSend(sendBuff);
}

void SetRxDutyCycle(uint8_t periodBase, uint16_t rxPeriodBaseCount, uint16_t sleepPeriodBaseCount)
{
    uint8_t data[8] = {
        SET_RX_DUTY_CYCLE,
        0x05,
        periodBase,
        (uint8_t)(rxPeriodBaseCount & 0xff),
        (uint8_t)((rxPeriodBaseCount >> 8) & 0xff),
        (uint8_t)(sleepPeriodBaseCount & 0xff),
        (uint8_t)((sleepPeriodBaseCount >> 8) & 0xff),
    };
    buff_t sendBuff = {data, 8};
    uartSend(sendBuff);
}

void SetCad()
{
    uint8_t data[1] = {SET_CAD};
    buff_t sendBuff = {data, 1};
    uartSend(sendBuff);
}

void SetTxContinuousWave()
{
    uint8_t data[1] = {SET_TX_CONTINUOUS_WAVE};
    buff_t sendBuff = {data, 1};
    uartSend(sendBuff);
}

void SetTxContinuousPreamble()
{
    uint8_t data[1] = {SET_TX_CONTINUOUS_PREAMBLE};
    buff_t sendBuff = {data, 1};
    uartSend(sendBuff);
}

void SetPacketType(uint8_t packetType)
{
    uint8_t data[3] = {SET_PACKET_TYPE, 0x01, packetType};
    buff_t sendBuff = {data, 3};
    uartSend(sendBuff);
}

uint8_t GetPacketType()
{
    uint8_t data[2] = {GET_PACKET_TYPE, 0x01};
    buff_t sendBuff = {data, 2};
    buff_t ret = uartSendRecv(sendBuff, 1);
    return *ret.data;
}

void SetRfFrequency(uint8_t rfFrequency[3])
{
    uint8_t data[5];
    data[0] = SET_RF_FREQUENCY;
    data[1] = 0x03;
    myMemcpy(data + 2, rfFrequency, 3);
    buff_t sendBuff = {data, 5};
    uartSend(sendBuff);
}

void SetTxParams(uint8_t power, uint8_t rampTime)
{
    uint8_t data[4] = {SET_TX_PARAMS, 0x02, power, rampTime};
    buff_t sendBuff = {data, 4};
    uartSend(sendBuff);
}

void SetCadParams(uint8_t cadSymbolNum)
{
    uint8_t data[3] = {SET_CAD_PARAMS, 0x01, cadSymbolNum};
    buff_t sendBuff = {data, 3};
    uartSend(sendBuff);
}

void SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
    uint8_t data[4] = {SET_BUFFER_BASE_ADDRESS, 0x02, txBaseAddress, rxBaseAddress};
    buff_t sendBuff = {data, 4};
    uartSend(sendBuff);
}

void SetModulationParams(uint8_t modParam[3])
{
    uint8_t data[5];
    data[0] = SET_MODULATION_PARAMS;
    data[1] = 0x03;
    myMemcpy(data + 2, modParam, 3);
    buff_t sendBuff = {data, 5};
    uartSend(sendBuff);

    /* 0x1E Must be written to register 0x0925 for SF5 or SF6 */
    if (modParam[0] == 0x50 || modParam[0] == 0x60)
    {
        uint8_t data[1] = {0x1E};
        buff_t sendBuff = {data, 1};
        WriteRegister(0x0925, sendBuff);
    }
    /* 0x37 Must be written to register 0x0925 for SF7 or SF8 */
    else if (modParam[0] == 0x70 || modParam[0] == 0x80)
    {
        uint8_t data[1] = {0x37};
        buff_t sendBuff = {data, 1};
        WriteRegister(0x0925, sendBuff);
    }
    /* 0x32 Must be written to register 0x0925 for SF9, SF10, SF11, or SF12 */
    else if (modParam[0] == 0x90 || modParam[0] == 0xA0 || modParam[0] == 0xB0 || modParam[0] == 0xC0)
    {
        uint8_t data[1] = {0x32};
        buff_t sendBuff = {data, 1};
        WriteRegister(0x0925, sendBuff);
    }
}

void SetPacketParams(uint8_t packetParams[7])
{
    uint8_t data[9];
    data[0] = SET_PACKET_PARAMS;
    data[1] = 0x07;
    myMemcpy(data + 2, packetParams, 7);
    buff_t sendBuff = {data, 9};
    uartSend(sendBuff);
}

buff_t GetRxBufferStatus()
{
    uint8_t data[2] = {GET_RX_BUFFER_STATUS, 0x02};
    buff_t sendBuff = {data, 2};
    buff_t ret = uartSendRecv(sendBuff, 2);
    return ret;
}

buff_t GetPacketStatus()
{
    uint8_t data[2] = {GET_PACKET_STATUS, 0x05};
    buff_t sendBuff = {data, 2};
    buff_t ret = uartSendRecv(sendBuff, 5);
    return ret;
}

uint8_t GetRssiLnst()
{
    uint8_t data[2] = {GET_RSSI_LNST, 0x01};
    buff_t sendBuff = {data, 2};
    buff_t ret = uartSendRecv(sendBuff, 1);
    return *ret.data;
}

void SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t data[10] = {
        SET_DIO_IRQ_PARAMS,
        0x08,
        (uint8_t)(irqMask & 0xff),
        (uint8_t)((irqMask >> 8) & 0xff),
        (uint8_t)(dio1Mask & 0xff),
        (uint8_t)((dio1Mask >> 8) & 0xff),
        (uint8_t)(dio2Mask & 0xff),
        (uint8_t)((dio2Mask >> 8) & 0xff),
        (uint8_t)(dio3Mask & 0xff),
        (uint8_t)((dio3Mask >> 8) & 0xff)};
    buff_t sendBuff = {data, 10};
    uartSend(sendBuff);
}

uint16_t GetIrqStatus()
{
    uint8_t data[2] = {GET_IRQ_STATUS, 0x02};
    buff_t sendBuff = {data, 2};
    buff_t ret = uartSendRecv(sendBuff, 2);
    return (uint16_t)((ret.data[0] << 8) | ret.data[1]);
}

void ClrIrqStatus(uint16_t irqMask)
{
    uint8_t data[4] = {
        CLR_IRQ_STATUS,
        0x02,
        (uint8_t)(irqMask & 0xff),
        (uint8_t)((irqMask >> 8) & 0xff)};
    buff_t sendBuff = {data, 4};
    uartSend(sendBuff);
}

void SetRegulatorMode(uint8_t regulatorMode)
{
    uint8_t data[3] = {SET_REGULATOR_MODE, 0x01, regulatorMode};
    buff_t sendBuff = {data, 3};
    uartSend(sendBuff);
}

void SetSaveContext()
{
    uint8_t data[1] = {SET_SAVE_CONTEXT};
    buff_t sendBuff = {data, 1};
    uartSend(sendBuff);
}

void SetAutoFS(uint8_t state)
{
    uint8_t data[3] = {SET_AUTO_FS, 0x01, state};
    buff_t sendBuff = {data, 3};
    uartSend(sendBuff);
}

void SetAutoTx(uint8_t time)
{
    uint8_t data[4] = {
        SET_AUTO_TX,
        0x02,
        (uint8_t)(time & 0xff),
        (uint8_t)((time >> 8) & 0xff)};
    buff_t sendBuff = {data, 4};
    uartSend(sendBuff);
}

void SetPerfCounterMode(uint8_t perfCounterMode)
{
    uint8_t data[3] = {SET_PERF_COUNTER_MODE, 0x01, perfCounterMode};
    buff_t sendBuff = {data, 3};
    uartSend(sendBuff);
}

void SetLongPreamble(uint8_t enable)
{
    uint8_t data[3] = {SET_LONG_PREAMBLE, 0x01, enable};
    buff_t sendBuff = {data, 3};
    uartSend(sendBuff);
}

void SetUartSpeed(uint8_t uartSpeed)
{
    uint8_t data[3] = {SET_UART_SPEED, 0x01, uartSpeed};
    buff_t sendBuff = {data, 3};
    uartSend(sendBuff);
}

void SetRangingRole(uint8_t mode)
{
    uint8_t data[3] = {SET_RANGING_ROLE, 0x01, mode};
    buff_t sendBuff = {data, 3};
    uartSend(sendBuff);
}

void SetAdvancedRanging(uint8_t state)
{
    uint8_t data[3] = {SET_ADVANCED_RANGING, 0x01, state};
    buff_t sendBuff = {data, 3};
    uartSend(sendBuff);
}
