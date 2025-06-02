#ifndef SX1280_SPI_H
#define SX1280_SPI_H

#include <stdio.h>
#include <stdlib.h>
#include <lgpio.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <string.h>

#define RF_FREQ_1 0xB8
#define RF_FREQ_2 0x9D
#define RF_FREQ_3 0x89

#define MOD_PARAM_1 0x70
#define MOD_PARAM_2 0x0A
#define MOD_PARAM_3 0x01

#define RX_BASE_ADDR 0x00
#define TX_BASE_ADDR 0x80

#define DF_PACKET_LEN 126

#define DF_PREAMBLE_LENGTH 0x0C
#define DF_HEADER_TYPE 0x00
#define DF_CYCLICAL_REDUNDANCY_CHECK 0x20
#define DF_CHIRP_INVERT 0x40

#define IRQ 0b0100000001100011
#define DIO1 0b0000000000000001
#define DIO2 0b0000000000000010
#define DIO3 0b0100000001100000

#define BUSY_TIMEOUT_MS 1000

/* ------------ Defining SD Card Command Index with Hexadecimel Commands ------------ */

/*  Retrieve the transceiver status
    Cannot be the first command sent over the interface
    Is not strictly necessary for SPI b/c device returns status info
        also on cammand bytes
    params( void ) return( status ) */
#define GET_STATUS 0xC0

/*  Writes a block of bytes in a data memory space starting
        at a specific address.
    params( address[15:8], address[7:0], data[0:n] ) return( void ) */
#define WRITE_REGISTER 0x18

/*  Reads a block of data starting at a given address
    The host must send a NOP after the address to receive data!!!!!!!!!!!!!!!
    params( address[15:8], address[7:0] ) return( data[0:n-1] )  */
#define READ_REGISTER 0x19

/*  Write the data payload to be transmitted
    Data sent in hex, most likely translated using ascii for text
    Audio data tbd
    params( offset, data[0:n] ) return( void ) */
#define WRITE_BUFFER 0x1A

/*  Function allows reading (n-3) bytes of payload received
        starting at offset.
    Data received in hex, most likely translated using ascii for text
    params( offset ) return( data[0:n-1] ) */
#define READ_BUFFER 0x1B

/*  Set transceiver to Sleep mode
    Lowest current consumption
    params( sleepConfig ) return( void )
    sleepConfig[7:4] unused
    sleepConfig[1] 1: Data buffer in retention mode
    sleepConfig[0] 0: Data Ram is flushed 1: Data Ram in retention  */
#define SET_SLEEP 0x84

/*  Set the device in either STDBY_RC or STDBY_XOSC mode
    Used to configure the transceiver
    Intermediate levels of power consumption
    params( standbyConfig )
    standbyConfig 0: STDBY_RC mode 1: STDBY_XOSC mode
    return( void ) */
#define SET_STANDBY 0x80

/*  Set the device in Frequency Synthesizer mode
    PLL(Phase Locked Loop) is locked to the carrier frequency
    For test purposes of PLL
    params( void ) return( void ) */
#define SET_FS 0xC1

/*  Set the device in Transmit mode
    Clear IRQ status before using this command
    Timeout = periodBase * periodBaseCount
    params( periodBase, periodBaseCount[15:8], periodBaseCount[7:0] )
    periodBase 0x00: 15.625us 0x01: 62.5us 0x02: 1ms 0x03: 4ms
    periodBaseCount[15:0] 0x0000: No Time Out Other: Time out active
    return( void ) */
#define SET_TX 0x83

/*  Set the device in Receiver mode
    Timeout = periodBase * periodBaseCount
    params( periodBase, periodBaseCount[15:8], periodBaseCount[7:0] )
    periodBase 0x00: 15.625us 0x01: 62.5us 0x02: 1ms 0x03: 4ms
    periodBaseCount[15:0] 0x0000: No Time Out
                          0xFFFF: Rx Continuous mode, multi-packet Rx
                          Other: Time out active
    return( void ) */
#define SET_RX 0x82

/*  Set transceiver in sniff mode
    setLongPreamble must be issued prior to setRxDutyCycle
    RxPeriod = periodBase * rxPeriodBaseCount
    SleepPeriod = periodBase * sleepPeriodBaseCount
    params( rxPeriodBase, rxPeriodBaseCount[15:8],
        rxPeriodBaseCount[7:0], sleepPeriodBase,
        sleepPeriodBaseCount[15:8], sleepPeriodBaseCount[7:0] )
    periodBase 0x00: 15.625us 0x01: 62.5us 0x02: 1ms 0x03: 4ms
    periodBaseCount[15:0] 0x0000: No Time Out
                          Other: Device will stay in Rx Mode for
                                 RxPeriod and return
                                 to Sleep Mode for SleepPeriod
    return( void ) */
#define SET_RX_DUTY_CYCLE 0x94

/*  Set transceiver to Channel Activity Detection mode
    Device searches for a Lora signal
    Returns to STDBY_RC mode when finished
    Always sends CadDone IRQ, sends CadDetected IRQ if signal found
    Useful in Listen before Talk Applications
    params( void ) return( void ) */
#define SET_CAD 0xC5

/*  Test command to generate a Continuous Wave (RF tone)
    Frequency and power settings from setRfFrequency, and setTxParams
    params( void ) return( void ) */
#define SET_TX_CONTINUOUS_WAVE 0xD1

/*  Test command to generate infinite sequence pf symbol 0 in Lora
    params( void ) return( void ) */
#define SET_TX_CONTINUOUS_PREAMBLE 0xD2

/*  Sets the transceiver radio frame
    MUST BE THE FIRST IN A RADIO CONFIGURATION SEQUENCE!!!!!!!
    params( packetType )
    packetType[8:0] 0x00: GFSK
                    0x01: Lora
                    0x02: Ranging
                    0x03: FLRC
                    0x04: BLE
    return( void ) */
#define SET_PACKET_TYPE 0x8A

/*  Returns the current operation packet type of the radio
    packetType probly comes in same format as setPacketType
    params( void ) return( packetType ) */
#define GET_PACKET_TYPE 0x03

/*  Set the frequency of the RF frequency mode
    rfFrequency sets the number of PLL steps
    Frf = ( Fxosc/2^18 ) * rfFrequency
        Gives frequency in kilohertz
    params( rfFrequency[23:16], rfFrequency[15:8], rfFrequency[7:0] )
    return( void ) */
#define SET_RF_FREQUENCY 0x86

/*  Sets the Tx output power and the Tx ramp time
    params( power, rampTime )
    power  Pout[dB] = -18 + power i.e. -18 + 0x1F(31) = 13dbm
    rampTime 0x00: 2um 0x20: 4us 0x40: 5us 0x60: 8us
            0x80: 10us 0xA0: 12us 0xC0: 16us 0xE0: 20us
    return( void ) */
#define SET_TX_PARAMS 0x8E

/*  Sets number of symbols which Channel Activity Detected operates
    For symbols 1 & 2, there are higher risks of false detection.
    params( cadSymbolNum )
    cadSymbolNum 0x00: 1 symbol
                 0x20: 2 symbols
                 0x40: 4 symbols
                 0x60: 8 symbols
                 0x80: 16 symbols
    return( void ) */
#define SET_CAD_PARAMS 0x88

/*  Fixes the base address for the packet handing operation
        in Tx and Rx mode for all packet types
    params( txBaseAddress, rxBaseAddress ) return( void ) */
#define SET_BUFFER_BASE_ADDRESS 0x8F

/*  Configure the modulation parameters of the radio
    Params passed will be interpreted depending on the frame type
    Frame Type
    params( modParam1, modParam2, modParam3 )
    modParam1 BLE: BitrateBandwidth   Lora/Ranging: Spreading Factor
    modParam2 BLE: ModulationIndex    Lora/Ranging: Bandwith
    modParam3 BLE: ModulationShaping  Lora & Ranging: Coding Rate
    return( void ) */
#define SET_MODULATION_PARAMS 0x8B

/*  Set the parameters of the packet handling block
    params( packetParam1, packetParam2, packetParam3, packetParam4,
        packetParam5, packetParam6, packetParam7 )
    packetParam1 BLE: ConnectionState Lora/Ranging: Preambl Length
    packetParam2 BLE: CrcLength       Lora/Ranging: Header Type
    packetParam3 BLE: BleTestPayload  Lora/Ranging: PayloadLength
    packetParam4 BLE: Whitening       Lora/Ranging: CRC
    packetParam5 BLE: Not Used     Lora/Ranging: InvertIQ/chirp invert
    packetParam6 BLE: Not Used        Lora/Ranging: Not Used
    packetParam7 BLE: Not Used        Lora/Ranging: not Used
    return( void ) */
#define SET_PACKET_PARAMS 0x8C

/*  Returns the length of the last received packet
        and the address of the first byte received
    In Lora packet type, 0x00 always returned for rxPayloadLength.
        Instead read register 0x901, for Lora payload length
    params( void ) return( payloadLength, rxBufferOffset ) */
#define GET_RX_BUFFER_STATUS 0x17

/*  Retrieve information about the last received packet
    rssiSync: RSSI value latched upon  detection of sync address.
        Actual signal power is –(rssiSync)/2dBm
    snr: Estimation of Signal to Noise Ratio on last packet received.
        In two’s compliment format multiplied by 4.
        Actual Signal to Noise Ratio(SNR) is (snr)/4dB. If SNR ≤ 0,
        RSSI_{packet, real} = RSSI_{packet,measured} – SNR_{measured}
    params( void )
    return( packetStatus[39:32], packetStatus[31:24],
        packetStatus[23:16], packetStatus[15:8], packetStatus[7:0] )
    packetStatus[7:0]   BLE: RFU        Lora/Ranging: rssiSync
    packetStatus[15:8]  BLE: rssiSync   Lora/Ranging: snr
    packetStatus[16:23] BLE: errors     Lora/Ranging: -
    packetStatus[24:31] BLE: status     Lora/Ranging: -
    packetStatus[32:39] BLE: sync       Lora/Ranging: - */
#define GET_PACKET_STATUS 0x1D

/*  Returns instantaneous RSSI value during reception of a  packet
    rssilnst: Signal power is (–rssiInst)/2dBm
    params( void ) return( rssilnst ) */
#define GET_RSSI_LNST 0x1F

/*  Enable IRQs and to route IRQs to DIO pins
    An interrupt is flagged in IRQ register if the corresponding
        bit in flag register is set
    irqMask[15:0] set which IRQ's are active,
        pg 95 in sx1280 manual has IRQ table
    dioMasks active bits correspond to the active bits irqMasks
        If coresponding bits are both on, IRQ is sent through that DIO
    params( irqMask[15:8], irqMask[7:0], dio1Mask[15:8],dio1Mask[7:0],
    dio2Mask[15:8], dio2Mask[7:0], dio3Mask[15:8], dio3Mask[7:0] )
    return( void ) */
#define SET_DIO_IRQ_PARAMS 0x8D

/*  Returns the value of the IRQ register
    IRQ register is only interacatable through these commands
    params( void ) return( irqStatus[15:8], irqStatus[7:0] ) */
#define GET_IRQ_STATUS 0x15

/*  Clears an IRQ flag in IRQ register
    Corresponding bits in irqMask will clear flag of that IRQ
    params( irqMask[15:8], irqMask[7:0] ) return( void ) */
#define CLR_IRQ_STATUS 0x97

/*  Why Kansas but no arkansas
    Havent found in book
    params( regulatorMode ) return( void ) */
#define SET_REGULATOR_MODE 0x96

/*  Havent found in book
    params( void ) return( void ) */
#define SET_SAVE_CONTEXT 0xD5

/*  Set the state following a Rx or Tx operation is FS, not STDBY
    Reduces switching time between consecutive Rx and/or Tx operations
    params( 0x00=disable or 0x01=enable ) return( void ) */
#define SET_AUTO_FS 0x9E

/*  Allows transceiver to send a packet at a user programmable time
        after the end of a packet reception
    Must be issued in STDBY_RC mode
    TxDelay = time + 33us(time needed for transceiver to switch modes)
    params( time[15:8], time[7:0] ) return( void ) */
#define SET_AUTO_TX 0x98

/*  Set the transceiver into Long Preamble mode
    RxDutyCycle is modified so that if a preamble is detected,
         the Rx window is extended by SleepPeriod + 2 * RxPeriod
    params( enable )
    enable 0x00: disable 0x01: enable
    return( void ) */
#define SET_LONG_PREAMBLE 0x9B

#define SET_UART_SPEED 0x9D

/*  params( 0x00=slave or 0x01=master ) return( void ) */
#define SET_RANGING_ROLE 0xA3

/* params( 0x00=slave or 0x01=master ) return( void ) */
#define SET_ADVANCED_RANGING 0x9A

#define SET_PERF_COUNTER_MODE 0x9C

enum LORA_STATE
{
    SLEEP,
    STANDBY_RC,
    STANDBY_XOSC,
    FS,
    TX,
    RX,
    CAD
};

typedef struct sx1280_spi_t
{
    int pi;
    int spi;
    int spiDev;
    int spiChen;
    int misoPin;
    int mosiPin;
    int sckPin;
    int csPin;
    int busyPin;
    int resetPin;
    int dio1Pin;
    int dio2Pin;
    int dio3Pin;
    int state;
} sx1280_spi_t;

typedef struct sd_csv_data_state_t
{
    uint8_t state_cnb;
    uint8_t state_pwb;
    uint8_t state_cmb;
} sd_csv_data_state_t;

typedef struct sd_csv_data_status_t
{
    uint8_t pwb_status;
    uint8_t dob_status;
    uint8_t cnb_status;
    uint8_t snb_status;
    uint8_t cmb_status;
} sd_csv_data_status_t;

typedef struct sd_csv_data_eject_outputs_t
{
    uint8_t doors;
    uint8_t doors_dob;
    uint8_t payload;
    uint8_t drogue;
    uint8_t parachute;
} sd_csv_data_eject_outputs_t;

typedef struct sd_csv_data_bmp_t
{
    float pressure;
    float temperature;
} sd_csv_data_bmp_t;

typedef struct sd_csv_data_pos_t
{
    float abs_altitude;
    float rel_altitude;
    float est_velocity;
    float est_acceleration;
} sd_csv_data_pos_t;

typedef struct sd_csv_data_bno_t
{
    float acceleration;
    float rotation;
    float euler_h;
    float euler_p;
    float euler_r;
} sd_csv_data_bno_t;

typedef struct sd_csv_data_ina_t
{
    float voltage;
    float current;
} sd_csv_data_ina_t;

typedef struct sd_csv_gps_t
{
    bool is_connected;
    uint8_t num_fixed_satellites;
    float latitude;
    float longitude;
} sd_csv_gps_t;

typedef struct sd_csv_data_t
{
    unsigned int timestamp;
    sd_csv_data_state_t fsw_state;
    sd_csv_data_status_t board_status;
    sd_csv_data_eject_outputs_t eject_output;
    sd_csv_data_bmp_t bmp_data;
    sd_csv_data_pos_t pos_data;
    sd_csv_data_bno_t bno_data;
    sd_csv_data_ina_t ina_data_pwb;
    sd_csv_data_ina_t ina_data_dob;
    sd_csv_gps_t gps_data;
} sd_csv_data_t;

typedef struct emergency_timer_data_t
{
    float main;
    float drogue;
    float cansat;
    float door;
} emergency_timer_data_t;

typedef struct
{
    uint8_t idx;
    // uint8_t rbf;
    uint8_t fsw_state;
    // uint8_t main;
    // uint8_t drogue;
    // uint8_t cansat;
    // uint8_t door;
    // emergency_timer_data_t emergency_timer;
} GsLoraMsg_t;

typedef struct
{
    uint8_t idx;
} GsTestMsg_t;

long long millis();

void Sx1280SPIInit(sx1280_spi_t *dev);
int ResetSx1280(sx1280_spi_t *dev);
void WaitForSetup(sx1280_spi_t *dev);

int SpiSend(sx1280_spi_t *dev, uint8_t *buff, size_t len);
int SpiSendRecv(sx1280_spi_t *dev, uint8_t *msgBuff, size_t msgLen, uint8_t *recvBuff, size_t recvLen);
int WaitBusyPin(sx1280_spi_t *dev);

void PrintBuffHex(uint8_t *buff, size_t len);
void PrintBuffDec(uint8_t *buff, size_t len);
void PrintBuffChar(uint8_t *buff, size_t len);

int GetStatus(sx1280_spi_t *dev, uint8_t *status);
int WriteRegister(sx1280_spi_t *dev, uint16_t addr, uint8_t *data, size_t len);
int ReadRegister(sx1280_spi_t *dev, uint8_t *recv, size_t len, uint16_t addr);
int WriteBuffer(sx1280_spi_t *dev, uint8_t *data, size_t len);
int ReadBuffer(sx1280_spi_t *dev, uint8_t *recv, size_t len, uint8_t addr);
int SetSleep(sx1280_spi_t *dev, uint8_t sleepConfig);
int SetStandby(sx1280_spi_t *dev, uint8_t standbyConfig);
int SetFs(sx1280_spi_t *dev);
int SetTx(sx1280_spi_t *dev, uint8_t periodBase, uint16_t periodBaseCount);
int SetRx(sx1280_spi_t *dev, uint8_t periodBase, uint16_t periodBaseCount);
int SetRxDutyCycle(sx1280_spi_t *dev, uint8_t periodBase, uint16_t rxPeriodBaseCount, uint16_t sleepPeriodBaseCount);
int SetCad(sx1280_spi_t *dev);
int SetTxContinuousWave(sx1280_spi_t *dev);
int SetTxContinuousPreamble(sx1280_spi_t *dev);
int SetPacketType(sx1280_spi_t *dev, uint8_t packetType);
int GetPacketType(sx1280_spi_t *dev, uint8_t *packetType);
int SetRfFrequency(sx1280_spi_t *dev, uint8_t rfFrequency[3]);
int SetTxParams(sx1280_spi_t *dev, uint8_t power, uint8_t rampTime);
int SetCadParams(sx1280_spi_t *dev, uint8_t cadSymbolNum);
int SetBufferBaseAddress(sx1280_spi_t *dev, uint8_t txBaseAddress, uint8_t rxBaseAddress);
int SetModulationParams(sx1280_spi_t *dev, uint8_t modParam[3]);
int SetPacketParams(sx1280_spi_t *dev, uint8_t packetParams[7]);
int GetRxBufferStatus(sx1280_spi_t *dev, uint8_t recv[2]);
int GetPacketStatus(sx1280_spi_t *dev, uint8_t recv[5]);
int GetRssiLnst(sx1280_spi_t *dev, uint8_t *rssiLnst);
int SetDioIrqParams(sx1280_spi_t *dev, uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);
int GetIrqStatus(sx1280_spi_t *dev, uint16_t *irq);
int ClrIrqStatus(sx1280_spi_t *dev, uint16_t irqMask);
int SetRegulatorMode(sx1280_spi_t *dev, uint8_t regulatorMode);
int SetSaveContext(sx1280_spi_t *dev);
int SetAutoFS(sx1280_spi_t *dev, uint8_t state);
int SetAutoTx(sx1280_spi_t *dev, uint8_t time);
int SetPerfCounterMode(sx1280_spi_t *dev, uint8_t perfCounterMode);
int SetLongPreamble(sx1280_spi_t *dev, uint8_t enable);
int SetUartSpeed(sx1280_spi_t *dev, uint8_t uartSpeed);
int SetRangingRole(sx1280_spi_t *dev, uint8_t mode);
int SetAdvancedRanging(sx1280_spi_t *dev, uint8_t state);

#endif /* SX1280_SPI_H */
