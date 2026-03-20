/**
 * @file LR2021.h
 * @brief Arduino driver for Semtech LR2021 LoRa transceiver
 * @version 1.0.0
 *
 * This driver provides LoRa transmit/receive functionality for the LR2021 chip.
 * Supports all frequency bands (Sub-GHz LF, Sub-GHz HF, 2.4GHz).
 */

#ifndef LR2021_H
#define LR2021_H

#include <Arduino.h>
#include <SPI.h>

// Enable debug output (comment out to disable)
// #define LR2021_DEBUG

#ifdef LR2021_DEBUG
#define LR2021_DEBUG_PRINT(x)    Serial.print(x)
#define LR2021_DEBUG_PRINTLN(x)  Serial.println(x)
#define LR2021_DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define LR2021_DEBUG_PRINT(x)
#define LR2021_DEBUG_PRINTLN(x)
#define LR2021_DEBUG_PRINTF(...)
#endif

// ============================================================================
// Command Opcodes (from Datasheet)
// ============================================================================

// Direct Radio FIFO Read/Write Commands
#define LR2021_CMD_READ_RX_FIFO             0x0001
#define LR2021_CMD_WRITE_TX_FIFO            0x0002

// Register/Memory Access Commands
#define LR2021_CMD_WRITE_REG_MEM32          0x0104
#define LR2021_CMD_WRITE_REG_MEM_MASK32     0x0105
#define LR2021_CMD_READ_REG_MEM32           0x0106

// System Configuration/Status Commands
#define LR2021_CMD_GET_STATUS               0x0100
#define LR2021_CMD_GET_VERSION              0x0101
#define LR2021_CMD_GET_ERRORS               0x0110
#define LR2021_CMD_CLEAR_ERRORS             0x0111
#define LR2021_CMD_SET_DIO_FUNCTION         0x0112
#define LR2021_CMD_SET_DIO_RF_SWITCH_CONFIG 0x0113
#define LR2021_CMD_CLEAR_FIFO_IRQ_FLAGS     0x0114
#define LR2021_CMD_SET_DIO_IRQ_CONFIG       0x0115
#define LR2021_CMD_CLEAR_IRQ                0x0116
#define LR2021_CMD_GET_AND_CLEAR_IRQ        0x0117
#define LR2021_CMD_CONFIG_LF_CLOCK          0x0118
#define LR2021_CMD_CONFIG_CLK_OUTPUTS       0x0119
#define LR2021_CMD_CONFIG_FIFO_IRQ          0x011A
#define LR2021_CMD_GET_FIFO_IRQ_FLAGS       0x011B
#define LR2021_CMD_GET_RX_FIFO_LEVEL        0x011C
#define LR2021_CMD_GET_TX_FIFO_LEVEL        0x011D
#define LR2021_CMD_CLEAR_RX_FIFO            0x011E
#define LR2021_CMD_CLEAR_TX_FIFO            0x011F
#define LR2021_CMD_SET_TCXO_MODE            0x0120
#define LR2021_CMD_SET_REG_MODE             0x0121
#define LR2021_CMD_CALIBRATE                0x0122
#define LR2021_CMD_CALIB_FE                 0x0123
#define LR2021_CMD_GET_VBAT                 0x0124
#define LR2021_CMD_GET_TEMP                 0x0125
#define LR2021_CMD_GET_RANDOM_NUMBER        0x0126
#define LR2021_CMD_SET_SLEEP                0x0127
#define LR2021_CMD_SET_STANDBY              0x0128
#define LR2021_CMD_SET_FS                   0x0129
#define LR2021_CMD_SET_ADDITIONAL_REG_RETAIN 0x012A
#define LR2021_CMD_GET_AND_CLEAR_FIFO_IRQ   0x012E

// Common Radio Commands
#define LR2021_CMD_SET_RF_FREQUENCY         0x0200
#define LR2021_CMD_SET_RX_PATH              0x0201
#define LR2021_CMD_SET_PA_CONFIG            0x0202
#define LR2021_CMD_SET_TX_PARAMS            0x0203
#define LR2021_CMD_SET_RSSI_CALIBRATION     0x0205
#define LR2021_CMD_SET_RX_TX_FALLBACK_MODE  0x0206
#define LR2021_CMD_SET_PACKET_TYPE          0x0207
#define LR2021_CMD_GET_PACKET_TYPE          0x0208
#define LR2021_CMD_STOP_TIMEOUT_ON_PREAMBLE 0x0209
#define LR2021_CMD_RESET_RX_STATS           0x020A
#define LR2021_CMD_GET_RSSI_INST            0x020B
#define LR2021_CMD_SET_RX                   0x020C
#define LR2021_CMD_SET_TX                   0x020D
#define LR2021_CMD_SET_TX_TEST_MODE         0x020E
#define LR2021_CMD_SEL_PA                   0x020F
#define LR2021_CMD_SET_RX_DUTY_CYCLE        0x0210
#define LR2021_CMD_SET_AUTO_RX_TX           0x0211
#define LR2021_CMD_GET_RX_PKT_LENGTH        0x0212
#define LR2021_CMD_SET_DEFAULT_RX_TX_TIMEOUT 0x0215
#define LR2021_CMD_SET_TIMESTAMP_SOURCE     0x0216
#define LR2021_CMD_GET_TIMESTAMP_VALUE      0x0217
#define LR2021_CMD_SET_CCA                  0x0218
#define LR2021_CMD_GET_CCA_RESULT           0x0219
#define LR2021_CMD_SET_AGC_GAIN_MANUAL      0x021A
#define LR2021_CMD_SET_CAD_PARAMS           0x021B
#define LR2021_CMD_SET_CAD                  0x021C

// LoRa Packet Radio Commands
#define LR2021_CMD_SET_LORA_MOD_PARAMS      0x0220
#define LR2021_CMD_SET_LORA_PACKET_PARAMS   0x0221
#define LR2021_CMD_SET_LORA_SYNCH_TIMEOUT   0x0222
#define LR2021_CMD_SET_LORA_SYNCWORD        0x0223
#define LR2021_CMD_SET_LORA_SIDE_DET_CONFIG 0x0224
#define LR2021_CMD_SET_LORA_SIDE_DET_SYNCWORD 0x0225
#define LR2021_CMD_SET_LORA_CAD_PARAMS      0x0227
#define LR2021_CMD_SET_LORA_CAD             0x0228
#define LR2021_CMD_GET_LORA_RX_STATS        0x0229
#define LR2021_CMD_GET_LORA_PACKET_STATUS   0x022A
#define LR2021_CMD_SET_LORA_ADDRESS         0x022B
#define LR2021_CMD_SET_LORA_HOPPING         0x022C
#define LR2021_CMD_SET_LORA_SIDE_DET_CAD    0x021E

// ============================================================================
// Parameter Constants
// ============================================================================

// Packet Types
#define LR2021_PACKET_TYPE_LORA             0x00
#define LR2021_PACKET_TYPE_FSK              0x02
#define LR2021_PACKET_TYPE_BLE              0x03
#define LR2021_PACKET_TYPE_RTTOF            0x04
#define LR2021_PACKET_TYPE_FLRC            0x05
#define LR2021_PACKET_TYPE_BPSK             0x06
#define LR2021_PACKET_TYPE_LR_FHSS          0x07

// Standby Modes
#define LR2021_STDBY_RC                     0x00
#define LR2021_STDBY_XOSC                   0x01

// Fallback Modes
#define LR2021_FALLBACK_STDBY_RC            0x00
#define LR2021_FALLBACK_STDBY_XOSC          0x01
#define LR2021_FALLBACK_FS                  0x02

// Calibration Blocks (bits for Calibrate command)
#define LR2021_CALIB_LF_RC                  (1 << 0)
#define LR2021_CALIB_HF_RC                  (1 << 1)
#define LR2021_CALIB_PLL                    (1 << 2)
#define LR2021_CALIB_AAF                    (1 << 3)
#define LR2021_CALIB_MU                     (1 << 5)
#define LR2021_CALIB_PA_OFF                 (1 << 6)
#define LR2021_CALIB_ALL                    0x6F

// LoRa Spreading Factors
#define LR2021_LORA_SF5                     0x05
#define LR2021_LORA_SF6                     0x06
#define LR2021_LORA_SF7                     0x07
#define LR2021_LORA_SF8                     0x08
#define LR2021_LORA_SF9                     0x09
#define LR2021_LORA_SF10                    0x0A
#define LR2021_LORA_SF11                    0x0B
#define LR2021_LORA_SF12                    0x0C

// LoRa Bandwidths
#define LR2021_LORA_BW_31                   0x02  // 31.25 kHz
#define LR2021_LORA_BW_62                   0x03  // 62.50 kHz
#define LR2021_LORA_BW_125                  0x04  // 125 kHz
#define LR2021_LORA_BW_250                  0x05  // 250 kHz
#define LR2021_LORA_BW_500                  0x06  // 500 kHz
#define LR2021_LORA_BW_1000                 0x07  // 1000 kHz (for 2.4GHz)
#define LR2021_LORA_BW_41                   0x0A  // 41.67 kHz
#define LR2021_LORA_BW_83                   0x0B  // 83.34 kHz
#define LR2021_LORA_BW_101                  0x0C  // 101.5625 kHz
#define LR2021_LORA_BW_203                  0x0D  // 203.125 kHz
#define LR2021_LORA_BW_406                  0x0E  // 406.25 kHz
#define LR2021_LORA_BW_812                  0x0F  // 812.5 kHz

// LoRa Coding Rates (FEC)
// Standard Hamming codes (compatible with SX127x/SX128x/LR11xx)
#define LR2021_LORA_CR_4_5                  0x01  // Hamming CR 4/5, overhead 1.25x
#define LR2021_LORA_CR_4_6                  0x02  // Hamming CR 4/6, overhead 1.5x
#define LR2021_LORA_CR_4_7                  0x03  // Hamming CR 4/7, overhead 1.75x
#define LR2021_LORA_CR_4_8                  0x04  // Hamming CR 4/8, overhead 2x

// Long Interleaver Hamming codes (better burst error correction)
#define LR2021_LORA_CR_LI_4_5               0x05  // Long interleaver Hamming CR 4/5
#define LR2021_LORA_CR_LI_4_6               0x06  // Long interleaver Hamming CR 4/6
#define LR2021_LORA_CR_LI_4_8               0x07  // Long interleaver Hamming CR 4/8

// Convolutional codes (LR2021 only - NOT compatible with SX127x/SX128x/LR11xx!)
#define LR2021_LORA_CR_CONV_4_6             0x08  // Long interleaver Convolutional CR 4/6
#define LR2021_LORA_CR_CONV_4_8             0x09  // Long interleaver Convolutional CR 4/8

// LoRa LDRO (Low Data Rate Optimization)
#define LR2021_LORA_LDRO_OFF                0x00
#define LR2021_LORA_LDRO_ON                 0x01

// LoRa Header Type
#define LR2021_LORA_HEADER_EXPLICIT         0x00  // Variable packet length
#define LR2021_LORA_HEADER_IMPLICIT         0x01  // Fixed packet length

// LoRa CRC
#define LR2021_LORA_CRC_OFF                 0x00
#define LR2021_LORA_CRC_ON                  0x01

// LoRa IQ
#define LR2021_LORA_IQ_STANDARD             0x00
#define LR2021_LORA_IQ_INVERTED             0x01

// LoRa Syncword
#define LR2021_LORA_SYNCWORD_PUBLIC         0x34  // Public network
#define LR2021_LORA_SYNCWORD_PRIVATE        0x12  // Private network (default)

// PA Selection
#define LR2021_PA_SEL_LF                    0x00  // Low frequency PA (Sub-GHz)
#define LR2021_PA_SEL_HF                    0x01  // High frequency PA (2.4GHz)

// PA LF Mode
#define LR2021_PA_LF_MODE_FSM               0x00  // Full single-ended mode

// Rx Path Selection
#define LR2021_RX_PATH_LF                   0x00  // Low frequency path
#define LR2021_RX_PATH_HF                   0x01  // High frequency path

// TCXO Voltage (for SetTcxoMode command)
// Use LR2021_TCXO_NONE (0xFF) for passive crystal (no TCXO)
// Use one of the following voltage values for TCXO:
#define LR2021_TCXO_NONE                    0xFF  // No TCXO (passive crystal)
#define LR2021_TCXO_VOLTAGE_1_6V            0x00  // 1.6V
#define LR2021_TCXO_VOLTAGE_1_7V            0x01  // 1.7V
#define LR2021_TCXO_VOLTAGE_1_8V            0x02  // 1.8V
#define LR2021_TCXO_VOLTAGE_2_2V            0x03  // 2.2V
#define LR2021_TCXO_VOLTAGE_2_4V            0x04  // 2.4V
#define LR2021_TCXO_VOLTAGE_2_7V            0x05  // 2.7V
#define LR2021_TCXO_VOLTAGE_3_0V            0x06  // 3.0V
#define LR2021_TCXO_VOLTAGE_3_3V            0x07  // 3.3V

// Tx Ramp Time
#define LR2021_RAMP_2US                     0x00
#define LR2021_RAMP_4US                     0x01
#define LR2021_RAMP_8US                     0x02
#define LR2021_RAMP_16US                    0x03
#define LR2021_RAMP_32US                    0x04
#define LR2021_RAMP_48US                    0x05
#define LR2021_RAMP_64US                    0x06
#define LR2021_RAMP_80US                    0x07
#define LR2021_RAMP_96US                    0x08
#define LR2021_RAMP_112US                   0x09
#define LR2021_RAMP_128US                   0x0A
#define LR2021_RAMP_144US                   0x0B
#define LR2021_RAMP_160US                   0x0C
#define LR2021_RAMP_176US                   0x0D
#define LR2021_RAMP_192US                   0x0E
#define LR2021_RAMP_208US                   0x0F
#define LR2021_RAMP_240US                   0x10
#define LR2021_RAMP_272US                   0x11
#define LR2021_RAMP_304US                   0x12

// CAD Exit Mode
#define LR2021_CAD_EXIT_STDBY               0x00  // CAD_ONLY: Exit to fallback mode after CAD done
#define LR2021_CAD_EXIT_RX                  0x01  // CAD_RX: If detected, enter Rx mode; else fallback
#define LR2021_CAD_EXIT_TX                  0x10  // CAD_LBT: If NOT detected (clear), enter Tx mode

// LoRa CAD Number of Symbols
#define LR2021_CAD_SYMBOLS_1                0x01
#define LR2021_CAD_SYMBOLS_2                0x02
#define LR2021_CAD_SYMBOLS_4                0x04
#define LR2021_CAD_SYMBOLS_8                0x08
#define LR2021_CAD_SYMBOLS_16               0x10

// LoRa CAD Detection Mode
#define LR2021_CAD_DETECT_ANY               0x00  // Detect any LoRa symbol
#define LR2021_CAD_DETECT_PREAMBLE          0x01  // Detect preamble only

// LoRa CAD Detection Peak (recommended values per SF for 1 symbol)
// Format: LR2021_CAD_DET_PEAK_SFx
#define LR2021_CAD_DET_PEAK_SF5             52
#define LR2021_CAD_DET_PEAK_SF6             52
#define LR2021_CAD_DET_PEAK_SF7             60
#define LR2021_CAD_DET_PEAK_SF8             55
#define LR2021_CAD_DET_PEAK_SF9             52
#define LR2021_CAD_DET_PEAK_SF10            50
#define LR2021_CAD_DET_PEAK_SF11            48
#define LR2021_CAD_DET_PEAK_SF12            46
#define LR2021_CAD_DET_PEAK_DEFAULT         55

// DIO Function (for SetDioFunction command)
#define LR2021_DIO_FUNCTION_NONE            0x00  // Hi-Z
#define LR2021_DIO_FUNCTION_IRQ             0x01  // IRQ output
#define LR2021_DIO_FUNCTION_RF_SWITCH       0x02  // RF switch control
#define LR2021_DIO_FUNCTION_GPIO_LOW        0x05  // GPIO output low
#define LR2021_DIO_FUNCTION_GPIO_HIGH       0x06  // GPIO output high
#define LR2021_DIO_FUNCTION_HF_CLK_OUT      0x07  // HF clock output
#define LR2021_DIO_FUNCTION_LF_CLK_OUT      0x08  // LF clock output (DIO7-11 only)

// DIO Sleep Pull configuration
#define LR2021_DIO_SLEEP_PULL_NONE          0x00
#define LR2021_DIO_SLEEP_PULL_DOWN          0x01
#define LR2021_DIO_SLEEP_PULL_UP            0x02
#define LR2021_DIO_SLEEP_PULL_AUTO          0x03

// Multi-SF Detection (Side Detectors)
#define LR2021_DETECTOR_MAIN                0x01  // Main detector flag
#define LR2021_DETECTOR_SIDE1               0x02  // Side detector 1 flag
#define LR2021_DETECTOR_SIDE2               0x04  // Side detector 2 flag
#define LR2021_DETECTOR_SIDE3               0x08  // Side detector 3 flag

// Helper macro to create side detector config byte
// Format: bits[7:4]=SF, bits[3:2]=LDRO, bit[1]=RFU, bit[0]=InvertIQ
#define LR2021_SIDE_DET_CONFIG(sf, ldro, invertIQ) \
    (((sf) << 4) | (((ldro) & 0x03) << 2) | ((invertIQ) & 0x01))

// ============================================================================
// IRQ Flags (32-bit)
// ============================================================================
#define LR2021_IRQ_RX_FIFO                  (1UL << 0)
#define LR2021_IRQ_TX_FIFO                  (1UL << 1)
#define LR2021_IRQ_RNG_REQ_VLD              (1UL << 2)
#define LR2021_IRQ_TX_TIMESTAMP             (1UL << 3)
#define LR2021_IRQ_RX_TIMESTAMP             (1UL << 4)
#define LR2021_IRQ_PREAMBLE_DETECTED        (1UL << 5)
#define LR2021_IRQ_HEADER_VALID             (1UL << 6)   // LoRa header valid
#define LR2021_IRQ_SYNCWORD_VALID           (1UL << 6)   // Syncword valid (FSK)
#define LR2021_IRQ_CAD_DETECTED             (1UL << 7)
#define LR2021_IRQ_LORA_HDR_TIMESTAMP       (1UL << 8)
#define LR2021_IRQ_LORA_HEADER_ERR          (1UL << 9)
#define LR2021_IRQ_LOW_BATTERY              (1UL << 10)
#define LR2021_IRQ_PA_OCP_OVP               (1UL << 11)
#define LR2021_IRQ_LORA_TX_RX_HOP           (1UL << 12)
#define LR2021_IRQ_SYNC_FAIL                (1UL << 13)
#define LR2021_IRQ_LORA_SYMBOL_END          (1UL << 14)
#define LR2021_IRQ_LORA_TIMESTAMP_STAT      (1UL << 15)
#define LR2021_IRQ_ERROR                    (1UL << 16)
#define LR2021_IRQ_CMD_ERROR                (1UL << 17)
#define LR2021_IRQ_RX_DONE                  (1UL << 18)
#define LR2021_IRQ_TX_DONE                  (1UL << 19)
#define LR2021_IRQ_CAD_DONE                 (1UL << 20)
#define LR2021_IRQ_TIMEOUT                  (1UL << 21)
#define LR2021_IRQ_CRC_ERROR                (1UL << 22)
#define LR2021_IRQ_LEN_ERROR                (1UL << 23)
#define LR2021_IRQ_ADDR_ERROR               (1UL << 24)
#define LR2021_IRQ_FHSS                     (1UL << 25)
#define LR2021_IRQ_INTER_PACKET1            (1UL << 26)
#define LR2021_IRQ_INTER_PACKET2            (1UL << 27)
#define LR2021_IRQ_RNG_RESP_DONE            (1UL << 28)
#define LR2021_IRQ_RNG_REQ_DIS              (1UL << 29)
#define LR2021_IRQ_RNG_EXCH_VLD             (1UL << 30)
#define LR2021_IRQ_RNG_TIMEOUT              (1UL << 31)

// Common IRQ masks
#define LR2021_IRQ_ALL                      0xFFFFFFFF
#define LR2021_IRQ_NONE                     0x00000000

// ============================================================================
// Frequency Band Helpers
// ============================================================================

// Frequency band detection (official supported bands)
#define LR2021_FREQ_BAND_LF_MIN             150000000UL   // 150 MHz
#define LR2021_FREQ_BAND_LF_MAX             1000000000UL   // 1000 MHz
#define LR2021_FREQ_BAND_HF_MIN             1500000000UL   // 1500 MHz
#define LR2021_FREQ_BAND_HF_MAX             2500000000UL  // 2500 MHz

// Extended frequency range (unsupported/experimental - PLL may lock but RF performance degraded)
#define LR2021_FREQ_BAND_EXT_MIN            1000000000UL  // 1 GHz
#define LR2021_FREQ_BAND_EXT_MAX            1500000000UL  // 2.4 GHz

// ============================================================================
// Default Values
// ============================================================================
#define LR2021_DEFAULT_SPI_FREQUENCY        8000000  // 8 MHz

// ============================================================================
// Structures
// ============================================================================

/**
 * @brief LoRa packet status structure
 */
typedef struct {
    int16_t rssiPkt;       // Packet RSSI in dBm
    int8_t  snrPkt;        // Packet SNR in dB
    int16_t signalRssi;    // Signal RSSI in dBm
    uint8_t detector;      // Detector flags (0x01=Main, 0x02=Side1, 0x04=Side2, 0x08=Side3)
} lr2021_lora_packet_status_t;

/**
 * @brief LoRa Rx statistics structure
 */
typedef struct {
    uint16_t pktReceived;
    uint16_t pktCrcError;
    uint16_t pktHeaderCrcError;
    uint16_t falseSynch;
} lr2021_lora_rx_stats_t;

// ============================================================================
// Callback function types
// ============================================================================
typedef void (*lr2021_tx_done_cb_t)(void);
typedef void (*lr2021_rx_done_cb_t)(uint8_t* data, uint16_t len, int16_t rssi, int8_t snr);
typedef void (*lr2021_timeout_cb_t)(void);
typedef void (*lr2021_cad_done_cb_t)(bool detected);
typedef void (*lr2021_crc_error_cb_t)(void);

// ============================================================================
// LR2021 Class Definition
// ============================================================================

class LR2021 {
public:
    /**
     * @brief Constructor
     * @param nss SPI chip select pin
     * @param busy Busy signal pin
     * @param reset Reset pin
     * @param dio1 DIO1 interrupt pin
     * @param spi Pointer to SPI instance (default: &SPI)
     */
    LR2021(int8_t nss, int8_t busy, int8_t reset, int8_t dio1, SPIClass* spi = &SPI);

    /**
     * @brief Initialize the LR2021
     * @param vtcxo TCXO voltage setting.
     *              Use LR2021_TCXO_NONE (0xFF) for passive crystal (no TCXO).
     *              Use LR2021_TCXO_VOLTAGE_x_xV (0x00-0x07) for TCXO with specific voltage.
     * @return true if initialization successful
     */
    bool begin(uint8_t vtcxo = LR2021_TCXO_NONE);

    /**
     * @brief Hardware reset the chip
     */
    void reset();

    /**
     * @brief Get chip version
     * @return Version information (16-bit)
     */
    uint16_t getVersion();

    // ========================================================================
    // Operating Mode Control
    // ========================================================================

    /**
     * @brief Set standby mode
     * @param mode STDBY_RC (0) or STDBY_XOSC (1)
     */
    void setStandby(uint8_t mode = LR2021_STDBY_RC);

    /**
     * @brief Set frequency synthesizer mode
     */
    void setFs();

    /**
     * @brief Set sleep mode
     * @param config Sleep configuration
     * @param sleepTime Sleep time in 32kHz RTC periods
     */
    void setSleep(uint8_t config, uint32_t sleepTime = 0);

    /**
     * @brief Set Rx/Tx fallback mode
     * @param mode Fallback mode (STDBY_RC, STDBY_XOSC, or FS)
     */
    void setRxTxFallbackMode(uint8_t mode);

    // ========================================================================
    // Calibration
    // ========================================================================

    /**
     * @brief Configure TCXO mode
     * @param voltage TCXO supply voltage (LR2021_TCXO_VOLTAGE_x_xV)
     * @param startTimeMs TCXO startup time in milliseconds (default 5ms)
     *
     * Note: This command only works in Standby RC mode.
     * After using TCXO mode, a complete reset is required to return to normal XOSC operation.
     */
    void setTcxoMode(uint8_t voltage, uint8_t startTimeMs = 5);

    /**
     * @brief Run calibration
     * @param blocks Bitmask of blocks to calibrate
     */
    void calibrate(uint8_t blocks = LR2021_CALIB_ALL);

    /**
     * @brief Run front-end calibration for specific frequencies
     * @param freq1 First frequency in MHz
     * @param freq2 Second frequency in MHz
     * @param freq3 Third frequency in MHz
     */
    void calibrateFE(uint16_t freq1, uint16_t freq2, uint16_t freq3);

    // ========================================================================
    // RF Configuration
    // ========================================================================

    /**
     * @brief Set RF frequency
     * @param frequency Frequency in Hz
     */
    void setRfFrequency(uint32_t frequency);

    /**
     * @brief Set packet type
     * @param type Packet type (LORA, FSK, etc.)
     */
    void setPacketType(uint8_t type);

    /**
     * @brief Get current packet type
     * @return Current packet type
     */
    uint8_t getPacketType();

    /**
     * @brief Set Rx path (LF or HF)
     * @param path RX_PATH_LF or RX_PATH_HF
     * @param boost Rx boost level (0-7)
     */
    void setRxPath(uint8_t path, uint8_t boost = 0);

    // ========================================================================
    // PA/Tx Configuration
    // ========================================================================

    /**
     * @brief Configure PA settings
     * @param paSel PA_SEL_LF or PA_SEL_HF
     * @param paLfMode PA LF mode (default: FSM)
     * @param paLfDutyCycle PA LF duty cycle
     * @param paLfSlices PA LF slices
     * @param paHfDutyCycle PA HF duty cycle
     */
    void setPaConfig(uint8_t paSel, uint8_t paLfMode = LR2021_PA_LF_MODE_FSM,
                     uint8_t paLfDutyCycle = 6, uint8_t paLfSlices = 7,
                     uint8_t paHfDutyCycle = 16);

    /**
     * @brief Select which PA to use (quick switch after SetPaConfig)
     * @param paSel PA_SEL_LF or PA_SEL_HF
     */
    void selPa(uint8_t paSel);

    /**
     * @brief Set Tx parameters
     * @param power Output power in 0.5dB steps
     * @param rampTime PA ramp time
     */
    void setTxParams(int8_t power, uint8_t rampTime = LR2021_RAMP_80US);

    /**
     * @brief Configure PA for a specific frequency and power (helper function)
     * @param frequency Frequency in Hz
     * @param powerDbm Target output power in dBm
     */
    void configurePaForFrequency(uint32_t frequency, int8_t powerDbm);

    // ========================================================================
    // LoRa Modulation Configuration
    // ========================================================================

    /**
     * @brief Set LoRa modulation parameters
     * @param sf Spreading factor (SF5-SF12)
     * @param bw Bandwidth
     * @param cr Coding rate
     * @param ldro Low data rate optimization
     */
    void setLoraModulationParams(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro = LR2021_LORA_LDRO_OFF);

    /**
     * @brief Set LoRa packet parameters
     * @param preambleLen Preamble length in symbols
     * @param payloadLen Payload length (0 for any in explicit mode)
     * @param headerType Explicit or Implicit header
     * @param crc CRC on/off
     * @param invertIQ IQ standard/inverted
     */
    void setLoraPacketParams(uint16_t preambleLen, uint8_t payloadLen,
                             uint8_t headerType = LR2021_LORA_HEADER_EXPLICIT,
                             uint8_t crc = LR2021_LORA_CRC_ON,
                             uint8_t invertIQ = LR2021_LORA_IQ_STANDARD);

    /**
     * @brief Set LoRa syncword
     * @param syncword Syncword value (0x12=private, 0x34=public)
     */
    void setLoraSyncword(uint8_t syncword);

    /**
     * @brief Set LoRa synchronization timeout
     * @param nbSymbols Number of symbols
     * @param format Format (0=number, 1=mantissa/exponent)
     */
    void setLoraSynchTimeout(uint8_t nbSymbols, uint8_t format = 0);

    // ========================================================================
    // Multi-SF Parallel Reception (Side Detectors)
    // ========================================================================

    /**
     * @brief Configure a single side detector for multi-SF reception
     * @param detector Side detector number (1, 2, or 3)
     * @param sf Spreading factor for this detector
     * @param ldro Low data rate optimization (0=off, 1=on)
     * @param invertIQ IQ inversion (0=standard, 1=inverted)
     *
     * Note: Main SF is set via setLoraModulationParams() and must be the smallest SF.
     *       Side detector SFs must be larger than main SF.
     *       Max difference between largest and smallest SF is 4.
     */
    void setLoraSideDetector(uint8_t detector, uint8_t sf, uint8_t ldro = LR2021_LORA_LDRO_OFF,
                              uint8_t invertIQ = LR2021_LORA_IQ_STANDARD);

    /**
     * @brief Configure all side detectors at once for multi-SF reception
     * @param config1 Config for side detector 1 (0 to disable)
     * @param config2 Config for side detector 2 (0 to disable)
     * @param config3 Config for side detector 3 (0 to disable)
     *
     * Config format: bits[7:4]=SF, bits[3:2]=LDRO, bit[1]=RFU, bit[0]=InvertIQ
     * Use LR2021_SIDE_DET_CONFIG() macro to create config value.
     */
    void setLoraSideDetConfig(uint8_t config1 = 0, uint8_t config2 = 0, uint8_t config3 = 0);

    /**
     * @brief Disable all side detectors
     */
    void disableLoraSideDetectors();

    /**
     * @brief Set syncwords for side detectors
     * @param syncword1 Syncword for side detector 1
     * @param syncword2 Syncword for side detector 2
     * @param syncword3 Syncword for side detector 3
     */
    void setLoraSideDetSyncword(uint8_t syncword1, uint8_t syncword2, uint8_t syncword3);

    /**
     * @brief Configure multi-SF CAD parameters for side detectors
     * @param pnrDelta1 Peak-to-noise ratio delta for detector 1
     * @param detPeak1 Detection peak for detector 1
     * @param pnrDelta2 Peak-to-noise ratio delta for detector 2
     * @param detPeak2 Detection peak for detector 2
     * @param pnrDelta3 Peak-to-noise ratio delta for detector 3
     * @param detPeak3 Detection peak for detector 3
     */
    void setLoraSideDetCad(uint8_t pnrDelta1, uint8_t detPeak1,
                           uint8_t pnrDelta2, uint8_t detPeak2,
                           uint8_t pnrDelta3, uint8_t detPeak3);

    /**
     * @brief Helper function to enable multi-SF reception with common settings
     * @param mainSf Main spreading factor (smallest, e.g., SF7)
     * @param sideSf1 Side detector 1 SF (0 to disable)
     * @param sideSf2 Side detector 2 SF (0 to disable)
     * @param sideSf3 Side detector 3 SF (0 to disable)
     * @param bw Bandwidth (same for all detectors)
     * @param cr Coding rate
     *
     * Multi-SF Detection Rules (auto-enforced):
     *   - Up to 4 SFs can be detected in parallel (main + 3 side detectors)
     *   - Main SF must be the smallest SF (for normal Rx operation)
     *   - If BW > 500kHz, only 2 side detectors allowed (max 3 SFs)
     *   - If main SF is 10/11/12, only 1 side detector allowed (max 2 SFs)
     *   - All SFs must be different (no duplicates)
     *   - Max SF - Min SF must be <= 4
     *
     * Example: enableMultiSfRx(LR2021_LORA_SF7, LR2021_LORA_SF8, LR2021_LORA_SF9, LR2021_LORA_SF10, LR2021_LORA_BW_125, LR2021_LORA_CR_4_5)
     *          This enables parallel reception of SF7, SF8, SF9, SF10
     */
    void enableMultiSfRx(uint8_t mainSf, uint8_t sideSf1, uint8_t sideSf2, uint8_t sideSf3,
                         uint8_t bw, uint8_t cr);

    /**
     * @brief Get which detector received the last packet
     * @return Detector flags: bit0=Main, bit1=Side1, bit2=Side2, bit3=Side3
     */
    uint8_t getLastRxDetector();

    /**
     * @brief Get the SF used for the last received packet
     * @return Spreading factor value (5-12)
     */
    uint8_t getLastRxSf();

    // ========================================================================
    // Transmit Operations
    // ========================================================================

    /**
     * @brief Transmit data
     * @param data Pointer to data buffer
     * @param len Data length
     * @param timeout Tx timeout in 32kHz RTC periods (0 = no timeout)
     * @return true if transmission started successfully
     */
    bool transmit(uint8_t* data, uint16_t len, uint32_t timeout = 0);

    /**
     * @brief Blocking transmit - waits for TxDone or Timeout
     * @param data Pointer to data buffer
     * @param len Data length (up to 65535)
     * @param timeout Tx timeout in ms
     * @return true if transmission completed successfully
     */
    bool transmitBlocking(uint8_t* data, uint16_t len, uint32_t timeoutMs = 5000);

    /**
     * @brief Set device to Tx mode
     * @param timeout Timeout in 32kHz RTC periods
     */
    void setTx(uint32_t timeout = 0);

    /**
     * @brief Write data to Tx FIFO
     * @param data Pointer to data buffer
     * @param len Data length (up to 65535)
     */
    void writeFifo(uint8_t* data, uint16_t len);

    // ========================================================================
    // Receive Operations
    // ========================================================================

    /**
     * @brief Start receive mode
     * @param timeout Rx timeout in 32kHz RTC periods (0xFFFFFF = continuous)
     * @return true if Rx mode started successfully
     */
    bool receive(uint32_t timeout = 0xFFFFFF);

    /**
     * @brief Blocking receive - waits for RxDone, Timeout, or CRC error
     * @param buffer Pointer to receive buffer
     * @param maxLen Maximum bytes to receive (up to 65535)
     * @param timeoutMs Timeout in ms
     * @return Number of bytes received (0 if timeout or error)
     */
    uint16_t receiveBlocking(uint8_t* buffer, uint16_t maxLen, uint32_t timeoutMs = 10000);

    /**
     * @brief Set device to Rx mode
     * @param timeout Timeout in 32kHz RTC periods
     */
    void setRx(uint32_t timeout = 0xFFFFFF);

    /**
     * @brief Read data from Rx FIFO
     * @param buffer Pointer to receive buffer
     * @param len Number of bytes to read (up to 65535)
     */
    void readFifo(uint8_t* buffer, uint16_t len);

    /**
     * @brief Get Rx FIFO level
     * @return Number of bytes in Rx FIFO
     */
    uint16_t getRxFifoLevel();

    /**
     * @brief Get Tx FIFO level
     * @return Number of bytes in Tx FIFO
     */
    uint16_t getTxFifoLevel();

    /**
     * @brief Clear Rx FIFO
     */
    void clearRxFifo();

    /**
     * @brief Clear Tx FIFO
     */
    void clearTxFifo();

    /**
     * @brief Get received packet length
     * @return Packet length in bytes (up to 65535)
     */
    uint16_t getRxPacketLength();

    // ========================================================================
    // CAD (Channel Activity Detection)
    // ========================================================================

    /**
     * @brief Set LoRa CAD parameters (low-level)
     * @param nbSymbols Number of symbols for CAD (1, 2, 4, 8, 16)
     * @param pblAny 0=detect any LoRa symbol, 1=detect preamble only
     * @param pnrDelta 0=standard CAD, 8=fast CAD (early exit to save power)
     * @param exitMode Exit mode after CAD:
     *                 - LR2021_CAD_EXIT_STDBY (0x00): Return to fallback mode after CAD done
     *                 - LR2021_CAD_EXIT_RX (0x01): If detected, enter Rx; else fallback
     *                 - LR2021_CAD_EXIT_TX (0x10): If NOT detected (clear), enter Tx (LBT)
     * @param timeout Timeout in 32MHz clock periods (only for CAD_RX/CAD_LBT modes)
     * @param detPeak Detection peak threshold (use LR2021_CAD_DET_PEAK_SFx or 55 default)
     */
    void setLoraCadParams(uint8_t nbSymbols, uint8_t pblAny, uint8_t pnrDelta,
                          uint8_t exitMode, uint32_t timeout, uint8_t detPeak);

    /**
     * @brief Start LoRa CAD (non-blocking)
     * @return true if CAD started successfully
     *
     * After calling this, use processIrq() or check IRQ status for CAD_DONE/CAD_DETECTED.
     */
    bool startLoraCad();

    /**
     * @brief Configure and start LoRa CAD with recommended settings (blocking)
     * @param sf Spreading factor being used (for optimal det_peak selection)
     * @param nbSymbols Number of symbols to listen (1, 2, 4, 8, 16)
     * @param timeoutMs Software timeout in milliseconds
     * @return true if activity detected, false if channel is clear or timeout
     *
     * This is a high-level helper that:
     * 1. Configures CAD params with recommended det_peak for the given SF
     * 2. Sets up CAD_ONLY mode (returns to fallback after done)
     * 3. Starts CAD and waits for completion
     * 4. Returns detection result
     *
     * Prerequisites:
     * - LoRa mode must be set via setPacketType(LR2021_PACKET_TYPE_LORA)
     * - Modulation params should be configured via setLoraModulationParams()
     * - Packet params should be configured via setLoraPacketParams()
     */
    bool loraCadBlocking(uint8_t sf, uint8_t nbSymbols = LR2021_CAD_SYMBOLS_4,
                         uint32_t timeoutMs = 1000);

    /**
     * @brief Perform Listen-Before-Talk using CAD
     * @param sf Spreading factor being used
     * @param nbSymbols Number of symbols to listen
     * @param maxRetries Maximum CAD attempts if channel is busy
     * @param retryDelayMs Delay between retries in milliseconds
     * @return true if channel became clear and ready to transmit, false if still busy
     *
     * This function performs CAD repeatedly until the channel is clear or max retries reached.
     * Useful for implementing CSMA/CA or duty cycle compliance.
     */
    bool listenBeforeTalk(uint8_t sf, uint8_t nbSymbols = LR2021_CAD_SYMBOLS_4,
                          uint8_t maxRetries = 5, uint32_t retryDelayMs = 100);

    /**
     * @brief Get recommended detection peak threshold for a given SF
     * @param sf Spreading factor (5-12)
     * @return Recommended det_peak value
     */
    static uint8_t getRecommendedCadDetPeak(uint8_t sf, uint8_t num_symbols);

    // ========================================================================
    // FSK CAD (RSSI-based)
    // ========================================================================

    /**
     * @brief Set FSK CAD parameters (RSSI-based detection)
     * @param timeout CAD timeout in 32MHz clock periods
     * @param rssiThreshold RSSI threshold for detection (negative dBm value)
     *
     * Note: FSK CAD uses RSSI threshold instead of LoRa symbol detection.
     */
    void setFskCadParams(uint32_t timeout, int8_t rssiThreshold);

    /**
     * @brief Start FSK CAD (non-blocking)
     * @return true if CAD started successfully
     */
    bool startFskCad();

    // ========================================================================
    // Status and Statistics
    // ========================================================================

    /**
     * @brief Get device status
     * @return Status word
     */
    uint16_t getStatus();

    /**
     * @brief Get instantaneous RSSI
     * @return RSSI in dBm (x2 for 0.5dB resolution)
     */
    int16_t getRssiInst();

    /**
     * @brief Get LoRa packet status
     * @param status Pointer to status structure
     */
    void getLoraPacketStatus(lr2021_lora_packet_status_t* status);

    /**
     * @brief Get packet RSSI (last received)
     * @return RSSI in dBm
     */
    int16_t getPacketRssi();

    /**
     * @brief Get packet SNR (last received)
     * @return SNR in dB
     */
    int8_t getPacketSnr();

    /**
     * @brief Get LoRa Rx statistics
     * @param stats Pointer to stats structure
     */
    void getLoraRxStats(lr2021_lora_rx_stats_t* stats);

    /**
     * @brief Reset Rx statistics
     */
    void resetRxStats();

    /**
     * @brief Get errors
     * @return Error flags
     */
    uint32_t getErrors();

    /**
     * @brief Clear errors
     */
    void clearErrors();

    // ========================================================================
    // IRQ Management
    // ========================================================================

    /**
     * @brief Set DIO pin function
     * @param dio DIO pin number (5-11 for LR2021)
     * @param func Function (NONE, IRQ, RF_SWITCH, GPIO_LOW, GPIO_HIGH, etc.)
     * @param pullDrive Pull configuration for sleep mode
     */
    void setDioFunction(uint8_t dio, uint8_t func, uint8_t pullDrive = 0);

    /**
     * @brief Configure DIO IRQ
     * @param dio DIO pin number (5-11 for LR2021)
     * @param irqMask IRQ flags to enable on this DIO
     */
    void setDioIrqConfig(uint8_t dio, uint32_t irqMask);

    /**
     * @brief Get current IRQ status
     * @return IRQ flags
     */
    uint32_t getIrqStatus();

    /**
     * @brief Clear specific IRQ flags
     * @param irqMask IRQ flags to clear
     */
    void clearIrq(uint32_t irqMask);

    /**
     * @brief Get and clear IRQ status (atomic operation)
     * @return IRQ flags before clearing
     */
    uint32_t getAndClearIrq();

    // ========================================================================
    // Callback Registration
    // ========================================================================

    /**
     * @brief Register Tx done callback
     * @param callback Function pointer
     */
    void onTxDone(lr2021_tx_done_cb_t callback);

    /**
     * @brief Register Rx done callback
     * @param callback Function pointer
     */
    void onRxDone(lr2021_rx_done_cb_t callback);

    /**
     * @brief Register timeout callback
     * @param callback Function pointer
     */
    void onTimeout(lr2021_timeout_cb_t callback);

    /**
     * @brief Register CAD done callback
     * @param callback Function pointer
     */
    void onCadDone(lr2021_cad_done_cb_t callback);

    /**
     * @brief Register CRC error callback
     * @param callback Function pointer
     */
    void onCrcError(lr2021_crc_error_cb_t callback);

    /**
     * @brief Handle DIO1 interrupt (call this from ISR)
     */
    void handleDio1Irq();

    /**
     * @brief Process pending IRQ (call from main loop for non-blocking operation)
     */
    void processIrq();

    // ========================================================================
    // Utility Functions
    // ========================================================================

    /**
     * @brief Check if chip is busy
     * @return true if busy
     */
    bool isBusy();

    /**
     * @brief Get a random number from the chip
     * @param source Source (0-3)
     * @return 32-bit random number
     */
    uint32_t getRandomNumber(uint8_t source = 0);

    /**
     * @brief Get temperature reading
     * @return Temperature in degrees Celsius
     */
    int8_t getTemperature();

    /**
     * @brief Get battery voltage
     * @return Voltage in mV
     */
    uint16_t getBatteryVoltage();

private:
    // Pin configuration
    int8_t _nss;
    int8_t _busy;
    int8_t _reset;
    int8_t _dio1;
    SPIClass* _spi;
    SPISettings _spiSettings;

    // State
    volatile bool _irqPending;
    uint8_t _packetType;
    uint32_t _frequency;

    // LoRa packet params (cached for transmit)
    uint16_t _preambleLen;
    uint8_t _headerType;
    uint8_t _crcOn;
    uint8_t _invertIQ;

    // Callbacks
    lr2021_tx_done_cb_t _cbTxDone;
    lr2021_rx_done_cb_t _cbRxDone;
    lr2021_timeout_cb_t _cbTimeout;
    lr2021_cad_done_cb_t _cbCadDone;
    lr2021_crc_error_cb_t _cbCrcError;

    // SPI low-level operations
    void writeCommand(uint16_t opcode, uint8_t* params, uint8_t len);
    void readCommand(uint16_t opcode, uint8_t* params, uint8_t paramLen,
                     uint8_t* result, uint8_t resultLen);
    void writeDirectFifo(uint8_t* data, uint16_t len);
    void readDirectFifo(uint8_t* data, uint16_t len);

    bool waitBusy(uint32_t timeoutMs = 1000);
    void beginTransaction();
    void endTransaction();
};

#endif // LR2021_H