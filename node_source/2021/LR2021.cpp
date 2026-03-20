/**
 * @file LR2021.cpp
 * @brief Arduino driver implementation for Semtech LR2021 LoRa transceiver
 * @version 1.0.0
 */

#include "LR2021.h"

// ============================================================================
// Constructor
// ============================================================================

LR2021::LR2021(int8_t nss, int8_t busy, int8_t reset, int8_t dio1, SPIClass* spi)
    : _nss(nss), _busy(busy), _reset(reset), _dio1(dio1), _spi(spi),
      _spiSettings(LR2021_DEFAULT_SPI_FREQUENCY, MSBFIRST, SPI_MODE0),
      _irqPending(false), _packetType(LR2021_PACKET_TYPE_LORA), _frequency(0),
      _preambleLen(8), _headerType(LR2021_LORA_HEADER_EXPLICIT),
      _crcOn(LR2021_LORA_CRC_ON), _invertIQ(LR2021_LORA_IQ_STANDARD),
      _cbTxDone(nullptr), _cbRxDone(nullptr), _cbTimeout(nullptr),
      _cbCadDone(nullptr), _cbCrcError(nullptr) {
}

// ============================================================================
// Initialization
// ============================================================================

bool LR2021::begin(uint8_t vtcxo) {
    // Initialize pins
    pinMode(_nss, OUTPUT);
    digitalWrite(_nss, HIGH);

    pinMode(_busy, INPUT);

    if (_reset >= 0) {
        pinMode(_reset, OUTPUT);
        digitalWrite(_reset, HIGH);
    }

    if (_dio1 >= 0) {
        pinMode(_dio1, INPUT);
    }

    // Initialize SPI
    _spi->begin();

    // Hardware reset
    reset();

    // Wait for chip to be ready
    if (!waitBusy(100)) {
        LR2021_DEBUG_PRINTLN("LR2021: Timeout waiting for chip ready after reset");
        return false;
    }

    // Get and verify version
    uint16_t version = getVersion();
    LR2021_DEBUG_PRINTF("LR2021: Version = 0x%04X\n", version);

    // Set standby mode (required before SetTcxoMode)
    setStandby(LR2021_STDBY_RC);

    // Configure TCXO if specified (vtcxo != 0xFF means TCXO is used)
    if (vtcxo != LR2021_TCXO_NONE) {
        // vtcxo contains the voltage setting (0x00-0x07)
        // Use 5ms startup time by default
        setTcxoMode(vtcxo, 5);

        // Wait for TCXO to stabilize
        if (!waitBusy(100)) {
            LR2021_DEBUG_PRINTLN("LR2021: Timeout waiting for TCXO");
            return false;
        }
        LR2021_DEBUG_PRINTF("LR2021: TCXO configured with voltage setting 0x%02X\n", vtcxo);
    } else {
        LR2021_DEBUG_PRINTLN("LR2021: Using passive crystal (no TCXO)");
    }

    // Run calibrations
    calibrate(LR2021_CALIB_ALL);

    // Wait for calibration to complete
    if (!waitBusy(500)) {
        LR2021_DEBUG_PRINTLN("LR2021: Timeout waiting for calibration");
        return false;
    }

    LR2021_DEBUG_PRINTLN("LR2021: Initialization complete");
    return true;
}

void LR2021::reset() {
    if (_reset >= 0) {
        digitalWrite(_reset, LOW);
        delay(1);
        digitalWrite(_reset, HIGH);
        delay(10);  // Wait for chip to boot
    }
}

uint16_t LR2021::getVersion() {
    uint8_t result[2];
    readCommand(LR2021_CMD_GET_VERSION, nullptr, 0, result, 2);
    return ((uint16_t)result[0] << 8) | result[1];
}

// ============================================================================
// SPI Low-Level Operations
// ============================================================================

void LR2021::beginTransaction() {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_nss, LOW);
}

void LR2021::endTransaction() {
    digitalWrite(_nss, HIGH);
    _spi->endTransaction();
}

bool LR2021::waitBusy(uint32_t timeoutMs) {
    uint32_t start = millis();
    while (digitalRead(_busy) == HIGH) {
        if ((millis() - start) > timeoutMs) {
            return false;
        }
        yield();  // Allow ESP32 background tasks
    }
    return true;
}

bool LR2021::isBusy() {
    return digitalRead(_busy) == HIGH;
}

void LR2021::writeCommand(uint16_t opcode, uint8_t* params, uint8_t len) {
    // Wait for chip to be ready
    if (!waitBusy()) {
        LR2021_DEBUG_PRINTF("LR2021: Busy timeout before write cmd 0x%04X\n", opcode);
        return;
    }

    beginTransaction();

    // Send opcode (MSB first)
    _spi->transfer((opcode >> 8) & 0xFF);
    _spi->transfer(opcode & 0xFF);

    // Send parameters
    for (uint8_t i = 0; i < len; i++) {
        _spi->transfer(params[i]);
    }

    endTransaction();

    LR2021_DEBUG_PRINTF("LR2021: Write cmd 0x%04X, %d params\n", opcode, len);
}

void LR2021::readCommand(uint16_t opcode, uint8_t* params, uint8_t paramLen,
                          uint8_t* result, uint8_t resultLen) {
    // Phase 1: Send command with parameters
    if (!waitBusy()) {
        LR2021_DEBUG_PRINTF("LR2021: Busy timeout before read cmd 0x%04X phase 1\n", opcode);
        return;
    }

    beginTransaction();

    // Send opcode
    _spi->transfer((opcode >> 8) & 0xFF);
    _spi->transfer(opcode & 0xFF);

    // Send parameters
    for (uint8_t i = 0; i < paramLen; i++) {
        _spi->transfer(params[i]);
    }

    endTransaction();

    // Phase 2: Wait for data ready and read
    if (!waitBusy()) {
        LR2021_DEBUG_PRINTF("LR2021: Busy timeout before read cmd 0x%04X phase 2\n", opcode);
        return;
    }

    beginTransaction();

    // Skip 2 status bytes
    _spi->transfer(0x00);
    _spi->transfer(0x00);

    // Read result bytes
    for (uint8_t i = 0; i < resultLen; i++) {
        result[i] = _spi->transfer(0x00);
    }

    endTransaction();

    LR2021_DEBUG_PRINTF("LR2021: Read cmd 0x%04X, %d result bytes\n", opcode, resultLen);
}

void LR2021::writeDirectFifo(uint8_t* data, uint16_t len) {
    // Direct write: data goes directly to FIFO during opcode transfer
    if (!waitBusy()) {
        LR2021_DEBUG_PRINTLN("LR2021: Busy timeout before FIFO write");
        return;
    }

    beginTransaction();

    // Send opcode 0x0002
    _spi->transfer(0x00);
    _spi->transfer(0x02);

    // Send data directly
    for (uint16_t i = 0; i < len; i++) {
        _spi->transfer(data[i]);
    }

    endTransaction();

    LR2021_DEBUG_PRINTF("LR2021: FIFO write %u bytes\n", len);
}

void LR2021::readDirectFifo(uint8_t* data, uint16_t len) {
    // Direct read: data comes immediately after opcode (single SPI frame)
    // According to datasheet 5.4.1.3 "Direct Read":
    // "data is read byte-by-byte from the radio FIFO immediately after the opcode"
    // This is different from normal read commands - NO status bytes to skip!
    if (!waitBusy()) {
        LR2021_DEBUG_PRINTLN("LR2021: Busy timeout before FIFO read");
        return;
    }

    beginTransaction();

    // Send opcode 0x0001 (ReadRadioRxFifo)
    _spi->transfer(0x00);
    _spi->transfer(0x01);

    // Direct read: data follows immediately after opcode, no status bytes
    // Read data directly
    for (uint16_t i = 0; i < len; i++) {
        data[i] = _spi->transfer(0x00);
    }

    endTransaction();

    LR2021_DEBUG_PRINTF("LR2021: FIFO read %u bytes\n", len);
}

// ============================================================================
// Operating Mode Control
// ============================================================================

void LR2021::setStandby(uint8_t mode) {
    uint8_t params[] = {mode};
    writeCommand(LR2021_CMD_SET_STANDBY, params, 1);
}

void LR2021::setFs() {
    writeCommand(LR2021_CMD_SET_FS, nullptr, 0);
}

void LR2021::setSleep(uint8_t config, uint32_t sleepTime) {
    uint8_t params[] = {
        config,
        (uint8_t)(sleepTime >> 24),
        (uint8_t)(sleepTime >> 16),
        (uint8_t)(sleepTime >> 8),
        (uint8_t)(sleepTime)
    };
    writeCommand(LR2021_CMD_SET_SLEEP, params, 5);
}

void LR2021::setRxTxFallbackMode(uint8_t mode) {
    uint8_t params[] = {mode};
    writeCommand(LR2021_CMD_SET_RX_TX_FALLBACK_MODE, params, 1);
}

// ============================================================================
// TCXO Configuration
// ============================================================================

void LR2021::setTcxoMode(uint8_t voltage, uint8_t startTimeMs) {
    // Convert startTimeMs to 32MHz clock periods
    // 32MHz = 32000 cycles per ms
    uint32_t startTime = (uint32_t)startTimeMs * 32000;

    uint8_t params[] = {
        voltage,                          // tune: TCXO supply voltage
        (uint8_t)(startTime >> 24),       // start_time(31:24)
        (uint8_t)(startTime >> 16),       // start_time(23:16)
        (uint8_t)(startTime >> 8),        // start_time(15:8)
        (uint8_t)(startTime)              // start_time(7:0)
    };
    writeCommand(LR2021_CMD_SET_TCXO_MODE, params, 5);

    LR2021_DEBUG_PRINTF("LR2021: TCXO mode set, voltage=0x%02X, startTime=%lu cycles\n",
                         voltage, startTime);
}

// ============================================================================
// Calibration
// ============================================================================

void LR2021::calibrate(uint8_t blocks) {
    uint8_t params[] = {blocks};
    writeCommand(LR2021_CMD_CALIBRATE, params, 1);
}

void LR2021::calibrateFE(uint16_t freq1, uint16_t freq2, uint16_t freq3) {
    uint8_t params[] = {
        (uint8_t)(freq1 >> 8), (uint8_t)(freq1),
        (uint8_t)(freq2 >> 8), (uint8_t)(freq2),
        (uint8_t)(freq3 >> 8), (uint8_t)(freq3)
    };
    writeCommand(LR2021_CMD_CALIB_FE, params, 6);
}

// ============================================================================
// RF Configuration
// ============================================================================

void LR2021::setRfFrequency(uint32_t frequency) {
    _frequency = frequency;
    uint8_t params[] = {
        (uint8_t)(frequency >> 24),
        (uint8_t)(frequency >> 16),
        (uint8_t)(frequency >> 8),
        (uint8_t)(frequency)
    };
    writeCommand(LR2021_CMD_SET_RF_FREQUENCY, params, 4);

    LR2021_DEBUG_PRINTF("LR2021: Set frequency %lu Hz\n", frequency);
}

void LR2021::setPacketType(uint8_t type) {
    _packetType = type;
    uint8_t params[] = {type};
    writeCommand(LR2021_CMD_SET_PACKET_TYPE, params, 1);
}

uint8_t LR2021::getPacketType() {
    uint8_t result[1];
    readCommand(LR2021_CMD_GET_PACKET_TYPE, nullptr, 0, result, 1);
    return result[0];
}

void LR2021::setRxPath(uint8_t path, uint8_t boost) {
    uint8_t params[] = {
        (uint8_t)(path & 0x01),         // Byte 2: rfu(6:0), rx_path
        (uint8_t)(boost & 0x07)         // Byte 3: rfu(4:0), rx_boost(2:0)
    };
    writeCommand(LR2021_CMD_SET_RX_PATH, params, 2);
}

// ============================================================================
// PA/Tx Configuration
// ============================================================================

void LR2021::setPaConfig(uint8_t paSel, uint8_t paLfMode,
                          uint8_t paLfDutyCycle, uint8_t paLfSlices,
                          uint8_t paHfDutyCycle) {
    // Byte 2: pa_sel (bit 7), rfu (bits 6:2), pa_lf_mode (bits 1:0)
    // Byte 3: pa_lf_duty_cycle (bits 7:4), pa_lf_slices (bits 3:0)
    // Byte 4: rfu (bits 7:5), pa_hf_duty_cycle (bits 4:0)
    uint8_t params[] = {
        (uint8_t)(((paSel & 0x01) << 7) | (paLfMode & 0x03)),  // pa_sel in bit 7, pa_lf_mode in bits 1:0
        (uint8_t)(((paLfDutyCycle & 0x0F) << 4) | (paLfSlices & 0x0F)),  // pa_lf_duty_cycle(3:0), pa_lf_slices(3:0)
        (uint8_t)(paHfDutyCycle & 0x1F)  // rfu(2:0), pa_hf_duty_cycle(4:0)
    };
    writeCommand(LR2021_CMD_SET_PA_CONFIG, params, 3);
}

void LR2021::selPa(uint8_t paSel) {
    uint8_t params[] = {(uint8_t)(paSel & 0x01)};
    writeCommand(LR2021_CMD_SEL_PA, params, 1);
}

void LR2021::setTxParams(int8_t power, uint8_t rampTime) {
    uint8_t params[] = {(uint8_t)power, rampTime};
    writeCommand(LR2021_CMD_SET_TX_PARAMS, params, 2);
}

void LR2021::configurePaForFrequency(uint32_t frequency, int8_t powerDbm) {
    if (frequency >= LR2021_FREQ_BAND_EXT_MIN && frequency < LR2021_FREQ_BAND_EXT_MAX) {
        // EXPERIMENTAL: Extended frequency range 1-2.4GHz (e.g., 1400MHz)
        // This is outside official spec - PLL may lock but RF performance will be degraded
        // Use LF PA path as it's closer to Sub-GHz band
        LR2021_DEBUG_PRINTF("LR2021: WARNING - Experimental frequency %lu Hz (outside spec)\n", frequency);

        int8_t powerParam = powerDbm * 2;
        if (powerParam < -19) powerParam = -19;
        if (powerParam > 44) powerParam = 44;

        // Use LF PA with maximum duty cycle and slices for best chance of output
        setPaConfig(LR2021_PA_SEL_LF, LR2021_PA_LF_MODE_FSM, 7, 7, 16);
        setTxParams(powerParam, LR2021_RAMP_80US);
        setRxPath(LR2021_RX_PATH_LF, 0);  // Use LF path

    } else if (frequency >= LR2021_FREQ_BAND_HF_MIN && frequency <= LR2021_FREQ_BAND_HF_MAX) {
        // Sub-GHz 800-1000MHz - use LF PA (optimized for 915MHz)
        // PA_LF: power range [-9.5:22]dBm, steps of 0.5dB
        int8_t powerParam = powerDbm * 2;
        if (powerParam < -19) powerParam = -19;
        if (powerParam > 44) powerParam = 44;

        // Use optimal values from datasheet Table 7-16 (915MHz reference design)
        uint8_t dutyCycle = 7, slices = 6;
        if (powerDbm >= 20) {
            dutyCycle = 7; slices = 6;
        } else if (powerDbm >= 18) {
            dutyCycle = 6; slices = 6;
        } else if (powerDbm >= 16) {
            dutyCycle = 5; slices = 6;
        } else if (powerDbm >= 14) {
            dutyCycle = 5; slices = 4;
        } else {
            dutyCycle = 4; slices = 3;
        }

        setPaConfig(LR2021_PA_SEL_HF, LR2021_PA_LF_MODE_FSM, dutyCycle, slices, 16);
        setTxParams(powerParam, LR2021_RAMP_80US);
        setRxPath(LR2021_RX_PATH_LF, 0);  // LF path, boost=0

    } else {
        int8_t powerParam = powerDbm * 2;
        if (powerParam < -19) powerParam = -19;
        if (powerParam > 44) powerParam = 44;

        // Use optimal values from datasheet Table 7-17 (490MHz reference design)
        uint8_t dutyCycle = 7, slices = 7;
        if (powerDbm >= 18) {
            dutyCycle = 7; slices = 6;
        } else if (powerDbm >= 15) {
            dutyCycle = 7; slices = 4;
        } else if (powerDbm >= 12) {
            dutyCycle = 6; slices = 2;
        } else {
            dutyCycle = 5; slices = 2;
        }

        setPaConfig(LR2021_PA_SEL_LF, LR2021_PA_LF_MODE_FSM, dutyCycle, slices, 16);
        setTxParams(powerParam, LR2021_RAMP_80US);
        setRxPath(LR2021_RX_PATH_LF, 0);  // LF path, boost=0
    }

    LR2021_DEBUG_PRINTF("LR2021: Configured PA for %lu Hz, %d dBm\n", frequency, powerDbm);
}

// ============================================================================
// LoRa Modulation Configuration
// ============================================================================

void LR2021::setLoraModulationParams(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro) {
    // Byte 2: sf(3:0), bw(3:0)
    // Byte 3: cr(3:0), rfu(1:0), ldro(1:0)
    uint8_t params[] = {
        (uint8_t)(((sf & 0x0F) << 4) | (bw & 0x0F)),
        (uint8_t)(((cr & 0x0F) << 4) | (ldro & 0x03))
    };
    writeCommand(LR2021_CMD_SET_LORA_MOD_PARAMS, params, 2);

    LR2021_DEBUG_PRINTF("LR2021: LoRa mod params SF%d, BW=0x%02X, CR=0x%02X, LDRO=%d\n",
                         sf, bw, cr, ldro);
}

void LR2021::setLoraPacketParams(uint16_t preambleLen, uint8_t payloadLen,
                                  uint8_t headerType, uint8_t crc, uint8_t invertIQ) {
    // Cache parameters for later use in transmit()
    _preambleLen = preambleLen;
    _headerType = headerType;
    _crcOn = crc;
    _invertIQ = invertIQ;

    // Byte 2: pbl_len(15:8)
    // Byte 3: pbl_len(7:0)
    // Byte 4: payload_len(7:0)
    // Byte 5: rfu(4:0), header_type, crc, invert_iq
    uint8_t byte5 = ((headerType & 0x01) << 2) | ((crc & 0x01) << 1) | (invertIQ & 0x01);

    uint8_t params[] = {
        (uint8_t)(preambleLen >> 8),
        (uint8_t)(preambleLen & 0xFF),
        payloadLen,
        byte5
    };
    writeCommand(LR2021_CMD_SET_LORA_PACKET_PARAMS, params, 4);

    LR2021_DEBUG_PRINTF("LR2021: LoRa pkt params preamble=%d, payload=%d, hdr=%d, crc=%d, iq=%d\n",
                         preambleLen, payloadLen, headerType, crc, invertIQ);
}

void LR2021::setLoraSyncword(uint8_t syncword) {
    uint8_t params[] = {syncword};
    writeCommand(LR2021_CMD_SET_LORA_SYNCWORD, params, 1);
}

void LR2021::setLoraSynchTimeout(uint8_t nbSymbols, uint8_t format) {
    uint8_t params[] = {nbSymbols, format};
    writeCommand(LR2021_CMD_SET_LORA_SYNCH_TIMEOUT, params, 2);
}

// ============================================================================
// Multi-SF Parallel Reception (Side Detectors)
// ============================================================================

void LR2021::setLoraSideDetector(uint8_t detector, uint8_t sf, uint8_t ldro, uint8_t invertIQ) {
    if (detector < 1 || detector > 3) {
        LR2021_DEBUG_PRINTLN("LR2021: Invalid side detector number (must be 1-3)");
        return;
    }

    // Build config byte: bits[7:4]=SF, bits[3:2]=LDRO, bit[1]=RFU, bit[0]=InvertIQ
    uint8_t config = LR2021_SIDE_DET_CONFIG(sf, ldro, invertIQ);

    // We need to set all 3 configs, so read current state if needed
    // For simplicity, just set the single detector (others will be 0/disabled)
    uint8_t params[3] = {0, 0, 0};
    params[detector - 1] = config;

    writeCommand(LR2021_CMD_SET_LORA_SIDE_DET_CONFIG, params, 3);

    LR2021_DEBUG_PRINTF("LR2021: Side detector %d configured: SF%d, LDRO=%d, IQ=%d\n",
                         detector, sf, ldro, invertIQ);
}

void LR2021::setLoraSideDetConfig(uint8_t config1, uint8_t config2, uint8_t config3) {
    if (config1 == 0 && config2 == 0 && config3 == 0) {
        // Disable all side detectors - send command without params
        writeCommand(LR2021_CMD_SET_LORA_SIDE_DET_CONFIG, nullptr, 0);
        LR2021_DEBUG_PRINTLN("LR2021: All side detectors disabled");
    } else {
        uint8_t params[] = {config1, config2, config3};
        writeCommand(LR2021_CMD_SET_LORA_SIDE_DET_CONFIG, params, 3);
        LR2021_DEBUG_PRINTF("LR2021: Side detectors configured: 0x%02X, 0x%02X, 0x%02X\n",
                             config1, config2, config3);
    }
}

void LR2021::disableLoraSideDetectors() {
    // Send command without parameters to disable all side detectors
    writeCommand(LR2021_CMD_SET_LORA_SIDE_DET_CONFIG, nullptr, 0);
    LR2021_DEBUG_PRINTLN("LR2021: All side detectors disabled");
}

void LR2021::setLoraSideDetSyncword(uint8_t syncword1, uint8_t syncword2, uint8_t syncword3) {
    uint8_t params[] = {syncword1, syncword2, syncword3};
    writeCommand(LR2021_CMD_SET_LORA_SIDE_DET_SYNCWORD, params, 3);
}

void LR2021::setLoraSideDetCad(uint8_t pnrDelta1, uint8_t detPeak1,
                                uint8_t pnrDelta2, uint8_t detPeak2,
                                uint8_t pnrDelta3, uint8_t detPeak3) {
    uint8_t params[] = {
        (uint8_t)(pnrDelta1 & 0x0F),  // rfu(2:0), pnr_delta1(3:0)
        detPeak1,
        (uint8_t)(pnrDelta2 & 0x0F),
        detPeak2,
        (uint8_t)(pnrDelta3 & 0x0F),
        detPeak3
    };
    writeCommand(LR2021_CMD_SET_LORA_SIDE_DET_CAD, params, 6);
}

void LR2021::enableMultiSfRx(uint8_t mainSf, uint8_t sideSf1, uint8_t sideSf2, uint8_t sideSf3,
                              uint8_t bw, uint8_t cr) {
    // Validate: main SF must be smallest
    if (sideSf1 != 0 && sideSf1 <= mainSf) {
        LR2021_DEBUG_PRINTLN("LR2021: Warning - Side SF1 should be > main SF");
    }
    if (sideSf2 != 0 && sideSf2 <= mainSf) {
        LR2021_DEBUG_PRINTLN("LR2021: Warning - Side SF2 should be > main SF");
    }
    if (sideSf3 != 0 && sideSf3 <= mainSf) {
        LR2021_DEBUG_PRINTLN("LR2021: Warning - Side SF3 should be > main SF");
    }

    // Determine LDRO settings based on SF and BW
    // LDRO = OFF: SF5-10 any BW, SF11 + BW>=250, SF12 + BW>=500
    // LDRO = ON: SF11 + BW<250, SF12 + BW<500
    // Note: bw is enum value, BW_125=0x04, BW_250=0x05, BW_500=0x06
    auto calcLdro = [](uint8_t sf, uint8_t bwCode) -> uint8_t {
        if (sf <= LR2021_LORA_SF10) return LR2021_LORA_LDRO_OFF;
        if (sf == LR2021_LORA_SF11 && bwCode == LR2021_LORA_BW_250) return LR2021_LORA_LDRO_OFF;
        if (sf == LR2021_LORA_SF11 && bwCode == LR2021_LORA_BW_406) return LR2021_LORA_LDRO_OFF;
        if (sf == LR2021_LORA_SF11 && bwCode == LR2021_LORA_BW_500) return LR2021_LORA_LDRO_OFF;
        if (sf == LR2021_LORA_SF11 && bwCode == LR2021_LORA_BW_812) return LR2021_LORA_LDRO_OFF;
        if (sf == LR2021_LORA_SF11 && bwCode == LR2021_LORA_BW_1000) return LR2021_LORA_LDRO_OFF;

        if (sf == LR2021_LORA_SF12 && bwCode == LR2021_LORA_BW_406) return LR2021_LORA_LDRO_OFF;
        if (sf == LR2021_LORA_SF12 && bwCode == LR2021_LORA_BW_500) return LR2021_LORA_LDRO_OFF;
        if (sf == LR2021_LORA_SF12 && bwCode == LR2021_LORA_BW_812) return LR2021_LORA_LDRO_OFF;
        if (sf == LR2021_LORA_SF12 && bwCode == LR2021_LORA_BW_1000) return LR2021_LORA_LDRO_OFF;
        return LR2021_LORA_LDRO_ON;
    };
    uint8_t mainLdro = calcLdro(mainSf, bw);
    uint8_t ldro1 = (sideSf1 != 0) ? calcLdro(sideSf1, bw) : LR2021_LORA_LDRO_OFF;
    uint8_t ldro2 = (sideSf2 != 0) ? calcLdro(sideSf2, bw) : LR2021_LORA_LDRO_OFF;
    uint8_t ldro3 = (sideSf3 != 0) ? calcLdro(sideSf3, bw) : LR2021_LORA_LDRO_OFF;

    // Set main detector (this also disables all side detectors)
    setLoraModulationParams(mainSf, bw, cr, mainLdro);

    // Configure side detectors
    uint8_t config1 = (sideSf1 != 0) ? LR2021_SIDE_DET_CONFIG(sideSf1, ldro1, 0) : 0;
    uint8_t config2 = (sideSf2 != 0) ? LR2021_SIDE_DET_CONFIG(sideSf2, ldro2, 0) : 0;
    uint8_t config3 = (sideSf3 != 0) ? LR2021_SIDE_DET_CONFIG(sideSf3, ldro3, 0) : 0;

    setLoraSideDetConfig(config1, config2, config3);

    // Set same syncword for all side detectors
    uint8_t syncword = LR2021_LORA_SYNCWORD_PRIVATE;
    setLoraSideDetSyncword(syncword, syncword, syncword);

    LR2021_DEBUG_PRINTF("LR2021: Multi-SF enabled: Main=SF%d", mainSf);
    if (sideSf1 != 0) LR2021_DEBUG_PRINTF(", Side1=SF%d", sideSf1);
    if (sideSf2 != 0) LR2021_DEBUG_PRINTF(", Side2=SF%d", sideSf2);
    if (sideSf3 != 0) LR2021_DEBUG_PRINTF(", Side3=SF%d", sideSf3);
    LR2021_DEBUG_PRINTLN("");
}

uint8_t LR2021::getLastRxDetector() {
    uint8_t result[6];
    readCommand(LR2021_CMD_GET_LORA_PACKET_STATUS, nullptr, 0, result, 6);

    // Byte 5 (result[5]): rfu(1:0), detector(3:0), rssi_pkt_bit(0), rssi_signal_pkt(0)
    // detector is bits [5:2] of byte 5... actually bits [5:2] based on table
    // Re-reading: "detector(3:0)" so it's 4 bits
    // From table: byte 7 has "rfu (1:0) detector(3:0) rssi_pkt_bit(0) ssi_signal_pkt(0)"
    // So detector is bits [5:2]
    return (result[5] >> 2) & 0x0F;
}

uint8_t LR2021::getLastRxSf() {
    uint8_t detector = getLastRxDetector();

    // Determine which SF was used based on detector
    // This requires knowing the configured SFs for each detector
    // For now, return the detector flags - user should track SF mapping

    // Alternative: could store configured SFs in class members
    // For a more complete implementation, we'd need to track this

    LR2021_DEBUG_PRINTF("LR2021: Last Rx detector flags: 0x%02X\n", detector);

    // Return detector flags (user interprets based on their config)
    return detector;
}

// ============================================================================
// Transmit Operations
// ============================================================================

void LR2021::writeFifo(uint8_t* data, uint16_t len) {
    writeDirectFifo(data, len);
}

void LR2021::setTx(uint32_t timeout) {
    uint8_t params[] = {
        (uint8_t)(timeout >> 16),
        (uint8_t)(timeout >> 8),
        (uint8_t)(timeout)
    };
    writeCommand(LR2021_CMD_SET_TX, params, 3);
}

bool LR2021::transmit(uint8_t* data, uint16_t len, uint32_t timeout) {
    // Clear Tx FIFO first
    clearTxFifo();

    // Update packet params with actual payload length
    // For LoRa explicit header mode, length is encoded in the header
    // Note: For lengths > 255, this may need special handling depending on modem type
    setLoraPacketParams(_preambleLen, (len > 255) ? 0 : len, _headerType, _crcOn, _invertIQ);

    // Write data to FIFO
    writeFifo(data, len);

    // Start Tx
    setTx(timeout);

    return true;
}

bool LR2021::transmitBlocking(uint8_t* data, uint16_t len, uint32_t timeoutMs) {
    // Start transmission
    if (!transmit(data, len, 0)) {
        return false;
    }

    // Wait for TxDone or Timeout
    uint32_t start = millis();
    while ((millis() - start) < timeoutMs) {
        uint32_t irq = getIrqStatus();

        if (irq & LR2021_IRQ_TX_DONE) {
            clearIrq(LR2021_IRQ_TX_DONE);
            LR2021_DEBUG_PRINTLN("LR2021: Tx done");
            return true;
        }

        if (irq & LR2021_IRQ_TIMEOUT) {
            clearIrq(LR2021_IRQ_TIMEOUT);
            LR2021_DEBUG_PRINTLN("LR2021: Tx timeout");
            return false;
        }

        yield();
    }

    LR2021_DEBUG_PRINTLN("LR2021: Tx blocking timeout");
    return false;
}

// ============================================================================
// Receive Operations
// ============================================================================

void LR2021::readFifo(uint8_t* buffer, uint16_t len) {
    readDirectFifo(buffer, len);
}

void LR2021::setRx(uint32_t timeout) {
    uint8_t params[] = {
        (uint8_t)(timeout >> 16),
        (uint8_t)(timeout >> 8),
        (uint8_t)(timeout)
    };
    writeCommand(LR2021_CMD_SET_RX, params, 3);
}

bool LR2021::receive(uint32_t timeout) {
    // Clear Rx FIFO
    clearRxFifo();

    // Start Rx
    setRx(timeout);

    return true;
}

uint16_t LR2021::receiveBlocking(uint8_t* buffer, uint16_t maxLen, uint32_t timeoutMs) {
    LR2021_DEBUG_PRINTLN("LR2021: receiveBlocking start");

    // Start reception
    if (!receive(0xFFFFFF)) {  // Continuous mode with software timeout
        LR2021_DEBUG_PRINTLN("LR2021: receive() failed");
        return 0;
    }

    LR2021_DEBUG_PRINTLN("LR2021: Waiting for IRQ...");

    // Wait for RxDone, Timeout, or CRC error
    uint32_t start = millis();
    uint32_t lastPrint = 0;
    while ((millis() - start) < timeoutMs) {
        // getAndClearIrq() reads AND clears IRQ flags in one command
        uint32_t irq = getAndClearIrq();

        // Debug: print IRQ status periodically
        if (irq != 0) {
            LR2021_DEBUG_PRINTF("LR2021: IRQ = 0x%08X\n", irq);
        }

        if (irq & LR2021_IRQ_PREAMBLE_DETECTED) {
            LR2021_DEBUG_PRINTLN("LR2021: Preamble detected!");
        }

        if (irq & LR2021_IRQ_HEADER_VALID) {
            LR2021_DEBUG_PRINTLN("LR2021: Header valid!");
        }

        if (irq & LR2021_IRQ_RX_DONE) {
            LR2021_DEBUG_PRINTLN("LR2021: RX_DONE!");

            // Check for CRC error
            if (irq & LR2021_IRQ_CRC_ERROR) {
                LR2021_DEBUG_PRINTLN("LR2021: Rx CRC error");
                clearRxFifo();
                return 0;
            }

            // Get packet length and read data
            uint16_t pktLen = getRxPacketLength();
            LR2021_DEBUG_PRINTF("LR2021: Packet length = %u\n", pktLen);

            if (pktLen > maxLen) {
                pktLen = maxLen;
            }
            if (pktLen > 0) {
                readFifo(buffer, pktLen);
            }

            LR2021_DEBUG_PRINTF("LR2021: Rx done, %u bytes\n", pktLen);
            clearRxFifo();
            return pktLen;
        }

        if (irq & LR2021_IRQ_TIMEOUT) {
            LR2021_DEBUG_PRINTLN("LR2021: Rx timeout (chip)");
            return 0;
        }

        if (irq & LR2021_IRQ_LORA_HEADER_ERR) {
            LR2021_DEBUG_PRINTLN("LR2021: Header error!");
        }

        delay(1);  // Small delay to reduce CPU usage
    }

    // Stop Rx mode
    setStandby();
    clearRxFifo();

    LR2021_DEBUG_PRINTLN("LR2021: Rx blocking timeout (software)");
    return 0;
}

uint16_t LR2021::getRxFifoLevel() {
    uint8_t result[2];
    readCommand(LR2021_CMD_GET_RX_FIFO_LEVEL, nullptr, 0, result, 2);
    return ((uint16_t)result[0] << 8) | result[1];
}

uint16_t LR2021::getTxFifoLevel() {
    uint8_t result[2];
    readCommand(LR2021_CMD_GET_TX_FIFO_LEVEL, nullptr, 0, result, 2);
    return ((uint16_t)result[0] << 8) | result[1];
}

void LR2021::clearRxFifo() {
    writeCommand(LR2021_CMD_CLEAR_RX_FIFO, nullptr, 0);
}

void LR2021::clearTxFifo() {
    writeCommand(LR2021_CMD_CLEAR_TX_FIFO, nullptr, 0);
}

uint16_t LR2021::getRxPacketLength() {
    // Response: pkt_len(15:8), pkt_len(7:0) - 16-bit length
    uint8_t result[2];
    readCommand(LR2021_CMD_GET_RX_PKT_LENGTH, nullptr, 0, result, 2);
    return ((uint16_t)result[0] << 8) | result[1];
}

// ============================================================================
// CAD (Channel Activity Detection)
// ============================================================================

void LR2021::setLoraCadParams(uint8_t nbSymbols, uint8_t pblAny, uint8_t pnrDelta,
                               uint8_t exitMode, uint32_t timeout, uint8_t detPeak) {
    // SetLoraCadParams (Opcode: 0x0227)
    // Byte 2: nb_symbols(7:0)
    // Byte 3: rfu(2:0), pbl_any(0), pnr_delta(3:0)
    // Byte 4: exit_mode(7:0)
    // Byte 5-7: cad_timeout(23:0)
    // Byte 8: det_peak(7:0)
    uint8_t params[] = {
        nbSymbols,
        (uint8_t)(((pblAny & 0x01) << 4) | (pnrDelta & 0x0F)),
        exitMode,
        (uint8_t)(timeout >> 16),
        (uint8_t)(timeout >> 8),
        (uint8_t)(timeout),
        detPeak
    };
    writeCommand(LR2021_CMD_SET_LORA_CAD_PARAMS, params, 7);

    LR2021_DEBUG_PRINTF("LR2021: CAD params set - symbols=%d, pblAny=%d, pnrDelta=%d, exitMode=0x%02X, detPeak=%d\n",
                         nbSymbols, pblAny, pnrDelta, exitMode, detPeak);
}

bool LR2021::startLoraCad() {
    // SetLoraCAD (Opcode: 0x0228) - no parameters
    writeCommand(LR2021_CMD_SET_LORA_CAD, nullptr, 0);
    LR2021_DEBUG_PRINTLN("LR2021: LoRa CAD started");
    return true;
}

uint8_t LR2021::getRecommendedCadDetPeak(uint8_t sf, uint8_t num_symbols) {
    // Select the base value based on Spreading Factor (SF)
    // Values derived from LR2021 Datasheet Table 6-19
    switch (sf) {
        case LR2021_LORA_SF5:
        case LR2021_LORA_SF6:
            if (num_symbols == 1) return 60;
            if (num_symbols == 2) return 56;
            return 51; // Symbols 3 and 4

        case LR2021_LORA_SF7:
            if (num_symbols == 1) return 60;
            if (num_symbols == 2) return 56;
            if (num_symbols == 3) return 52;
            return 51; // Symbol 4

        case LR2021_LORA_SF8:
            if (num_symbols == 1) return 64;
            if (num_symbols == 2) return 58;
            return 54; // Symbols 3 and 4

        case LR2021_LORA_SF9:
            if (num_symbols == 1) return 64;
            if (num_symbols == 2) return 58;
            return 56; // Symbols 3 and 4

        case LR2021_LORA_SF10:
            if (num_symbols == 1) return 66;
            return 60; // Symbols 2, 3, and 4

        case LR2021_LORA_SF11:
            if (num_symbols == 1) return 70;
            if (num_symbols == 2) return 64;
            return 60; // Symbols 3 and 4

        case LR2021_LORA_SF12:
            if (num_symbols == 1) return 74;
            if (num_symbols == 2) return 68;
            if (num_symbols == 3) return 65;
            return 64; // Symbol 4

        default:
            return LR2021_CAD_DET_PEAK_DEFAULT;
    }
}
bool LR2021::loraCadBlocking(uint8_t sf, uint8_t nbSymbols, uint32_t timeoutMs) {
    // Get recommended detection peak for this SF
    uint8_t detPeak = getRecommendedCadDetPeak(sf, nbSymbols);

    // Configure CAD parameters:
    // - nbSymbols: user specified (default 4)
    // - pblAny: 0 = detect any LoRa symbol (more sensitive)
    // - pnrDelta: 0 = standard CAD (not fast CAD)
    // - exitMode: CAD_ONLY (return to fallback after done)
    // - timeout: 0 (not used in CAD_ONLY mode)
    // - detPeak: based on SF
    setLoraCadParams(nbSymbols, LR2021_CAD_DETECT_ANY, 0,
                     LR2021_CAD_EXIT_STDBY, 0, detPeak);

    // Clear any pending IRQ flags
    clearIrq(LR2021_IRQ_CAD_DONE | LR2021_IRQ_CAD_DETECTED);

    // Start CAD
    startLoraCad();

    // Wait for CAD completion
    uint32_t start = millis();
    while ((millis() - start) < timeoutMs) {
        uint32_t irq = getAndClearIrq();

        if (irq & LR2021_IRQ_CAD_DONE) {
            // CAD completed - check if activity was detected
            bool detected = (irq & LR2021_IRQ_CAD_DETECTED) != 0;

            LR2021_DEBUG_PRINTF("LR2021: CAD done, detected=%d\n", detected);

            return detected;
        }

        delay(1);  // Small delay to reduce CPU usage
    }

    // Timeout - return to standby and report no detection
    setStandby();
    LR2021_DEBUG_PRINTLN("LR2021: CAD blocking timeout");
    return false;
}

bool LR2021::listenBeforeTalk(uint8_t sf, uint8_t nbSymbols,
                               uint8_t maxRetries, uint32_t retryDelayMs) {
    for (uint8_t attempt = 0; attempt <= maxRetries; attempt++) {
        // Perform CAD
        bool activityDetected = loraCadBlocking(sf, nbSymbols, 1000);

        if (!activityDetected) {
            // Channel is clear - ready to transmit
            LR2021_DEBUG_PRINTF("LR2021: LBT - Channel clear after %d attempts\n", attempt + 1);
            return true;
        }

        // Channel is busy
        LR2021_DEBUG_PRINTF("LR2021: LBT - Channel busy, attempt %d/%d\n", attempt + 1, maxRetries + 1);

        if (attempt < maxRetries) {
            // Wait before retry
            delay(retryDelayMs);
        }
    }

    // Max retries reached, channel still busy
    LR2021_DEBUG_PRINTLN("LR2021: LBT - Max retries reached, channel still busy");
    return false;
}

// ============================================================================
// FSK CAD (RSSI-based)
// ============================================================================

void LR2021::setFskCadParams(uint32_t timeout, int8_t rssiThreshold) {
    // SetCadParams (Opcode: 0x021B) for FSK mode
    // Byte 2-4: cad_timeout(23:0) in 32MHz clock periods
    // Byte 5: rssi_threshold (signed, in dBm)
    uint8_t params[] = {
        (uint8_t)(timeout >> 16),
        (uint8_t)(timeout >> 8),
        (uint8_t)(timeout),
        (uint8_t)rssiThreshold
    };
    writeCommand(LR2021_CMD_SET_CAD_PARAMS, params, 4);

    LR2021_DEBUG_PRINTF("LR2021: FSK CAD params set - timeout=%lu, rssiThreshold=%d dBm\n",
                         timeout, rssiThreshold);
}

bool LR2021::startFskCad() {
    // SetCad (Opcode: 0x021C) for FSK mode - no parameters
    writeCommand(LR2021_CMD_SET_CAD, nullptr, 0);
    LR2021_DEBUG_PRINTLN("LR2021: FSK CAD started");
    return true;
}

// ============================================================================
// Status and Statistics
// ============================================================================

uint16_t LR2021::getStatus() {
    // GetStatus returns status during the command phase
    if (!waitBusy()) {
        return 0;
    }

    beginTransaction();
    _spi->transfer(0x01);
    _spi->transfer(0x00);
    uint16_t status = (_spi->transfer(0x00) << 8) | _spi->transfer(0x00);
    endTransaction();

    return status;
}

int16_t LR2021::getRssiInst() {
    uint8_t result[2];
    readCommand(LR2021_CMD_GET_RSSI_INST, nullptr, 0, result, 2);
    // RSSI = -(result[0] + result[1]/256) / 2 in dBm
    // Simplified: return raw 9-bit value, user divides by 2
    // RSSI 9 bit value, result[0] 8 bit and result[1] bit 7 is the 9th bit
    int16_t rssiRaw = ((uint16_t)result[0] << 8) | (result[1] & 0x80);

    return -(rssiRaw) / 2;  // Return as -dBm for convenience

}

void LR2021::getLoraPacketStatus(lr2021_lora_packet_status_t* status) {
    // Response is 8 bytes according to datasheet Table 9-13:
    // Byte 0-1: Status
    // Byte 2: rfu(2:0), crc, coding_rate(3:0)
    // Byte 3: pkt_length(7:0)
    // Byte 4: snr_pkt(7:0)
    // Byte 5: rssi_pkt(8:1)
    // Byte 6: rssi_signal_pkt(8:1)
    // Byte 7: rfu(1:0), detector(3:0), rssi_pkt_bit(0), rssi_signal_pkt(0)
    uint8_t result[6];
    readCommand(LR2021_CMD_GET_LORA_PACKET_STATUS, nullptr, 0, result, 6);

    // SNR: snr_pkt/4 (two's complement, 0.25dB resolution)
    status->snrPkt = (int8_t)result[2] / 4;

    // RSSI: -rssi_pkt/2 [dBm]
    // rssi_pkt is in result[3] (bits 8:1) and result[5] bit 1 (bit 0)
    uint16_t rssiRaw = ((uint16_t)result[3] << 1) | ((result[5] >> 1) & 0x01);
    status->rssiPkt = -((int16_t)rssiRaw) / 2;

    // Signal RSSI: -rssi_signal_pkt/2 [dBm]
    uint16_t signalRssiRaw = ((uint16_t)result[4] << 1) | (result[5] & 0x01);
    status->signalRssi = -((int16_t)signalRssiRaw) / 2;

    // Detector flags (for multi-SF)
    status->detector = (result[5] >> 2) & 0x0F;
}

int16_t LR2021::getPacketRssi() {
    lr2021_lora_packet_status_t status;
    getLoraPacketStatus(&status);
    return status.rssiPkt;
}

int8_t LR2021::getPacketSnr() {
    lr2021_lora_packet_status_t status;
    getLoraPacketStatus(&status);
    return status.snrPkt;
}

void LR2021::getLoraRxStats(lr2021_lora_rx_stats_t* stats) {
    uint8_t result[8];
    readCommand(LR2021_CMD_GET_LORA_RX_STATS, nullptr, 0, result, 8);

    stats->pktReceived = ((uint16_t)result[0] << 8) | result[1];
    stats->pktCrcError = ((uint16_t)result[2] << 8) | result[3];
    stats->pktHeaderCrcError = ((uint16_t)result[4] << 8) | result[5];
    stats->falseSynch = ((uint16_t)result[6] << 8) | result[7];
}

void LR2021::resetRxStats() {
    writeCommand(LR2021_CMD_RESET_RX_STATS, nullptr, 0);
}

uint32_t LR2021::getErrors() {
    uint8_t result[4];
    readCommand(LR2021_CMD_GET_ERRORS, nullptr, 0, result, 4);
    return ((uint32_t)result[0] << 24) | ((uint32_t)result[1] << 16) |
           ((uint32_t)result[2] << 8) | result[3];
}

void LR2021::clearErrors() {
    writeCommand(LR2021_CMD_CLEAR_ERRORS, nullptr, 0);
}

// ============================================================================
// IRQ Management
// ============================================================================

void LR2021::setDioFunction(uint8_t dio, uint8_t func, uint8_t pullDrive) {
    // SetDioFunction: Dio(7:0), [Func(3:0), pull_drive(3:0)]
    // Func is in upper nibble, pull_drive is in lower nibble
    uint8_t params[] = {
        dio,
        (uint8_t)((func << 4) | (pullDrive & 0x0F))
    };
    writeCommand(LR2021_CMD_SET_DIO_FUNCTION, params, 2);

    LR2021_DEBUG_PRINTF("LR2021: Set DIO%d function=0x%02X, pull=0x%02X\n", dio, func, pullDrive);
}

void LR2021::setDioIrqConfig(uint8_t dio, uint32_t irqMask) {
    // First, set DIO function to IRQ output
    setDioFunction(dio, LR2021_DIO_FUNCTION_IRQ, LR2021_DIO_SLEEP_PULL_NONE);

    // Then configure which IRQs to map to this DIO
    uint8_t params[] = {
        dio,
        (uint8_t)(irqMask >> 24),
        (uint8_t)(irqMask >> 16),
        (uint8_t)(irqMask >> 8),
        (uint8_t)(irqMask)
    };
    writeCommand(LR2021_CMD_SET_DIO_IRQ_CONFIG, params, 5);

    LR2021_DEBUG_PRINTF("LR2021: Set DIO%d IRQ mask to 0x%08X\n", dio, irqMask);
}

uint32_t LR2021::getIrqStatus() {
    uint8_t result[4];
    readCommand(LR2021_CMD_GET_AND_CLEAR_IRQ, nullptr, 0, result, 4);
    uint32_t irq = ((uint32_t)result[0] << 24) | ((uint32_t)result[1] << 16) |
                   ((uint32_t)result[2] << 8) | result[3];
    return irq;
}

void LR2021::clearIrq(uint32_t irqMask) {
    uint8_t params[] = {
        (uint8_t)(irqMask >> 24),
        (uint8_t)(irqMask >> 16),
        (uint8_t)(irqMask >> 8),
        (uint8_t)(irqMask)
    };
    writeCommand(LR2021_CMD_CLEAR_IRQ, params, 4);
}

uint32_t LR2021::getAndClearIrq() {
    uint8_t result[4];
    readCommand(LR2021_CMD_GET_AND_CLEAR_IRQ, nullptr, 0, result, 4);
    return ((uint32_t)result[0] << 24) | ((uint32_t)result[1] << 16) |
           ((uint32_t)result[2] << 8) | result[3];
}

// ============================================================================
// Callback Registration
// ============================================================================

void LR2021::onTxDone(lr2021_tx_done_cb_t callback) {
    _cbTxDone = callback;
}

void LR2021::onRxDone(lr2021_rx_done_cb_t callback) {
    _cbRxDone = callback;
}

void LR2021::onTimeout(lr2021_timeout_cb_t callback) {
    _cbTimeout = callback;
}

void LR2021::onCadDone(lr2021_cad_done_cb_t callback) {
    _cbCadDone = callback;
}

void LR2021::onCrcError(lr2021_crc_error_cb_t callback) {
    _cbCrcError = callback;
}

void LR2021::handleDio1Irq() {
    _irqPending = true;
}

void LR2021::processIrq() {
    if (!_irqPending) {
        return;
    }
    _irqPending = false;

    uint32_t irq = getAndClearIrq();

    // Debug: print all IRQ flags
    LR2021_DEBUG_PRINTF("LR2021: IRQ flags = 0x%08X\n", irq);

    // Debug events (not passed to callbacks, just for debugging)
    if (irq & LR2021_IRQ_PREAMBLE_DETECTED) {
        LR2021_DEBUG_PRINTLN("LR2021: >>> Preamble detected!");
    }
    if (irq & LR2021_IRQ_HEADER_VALID) {
        LR2021_DEBUG_PRINTLN("LR2021: >>> Header valid!");
    }
    if (irq & LR2021_IRQ_LORA_HEADER_ERR) {
        LR2021_DEBUG_PRINTLN("LR2021: >>> Header error!");
    }

    if (irq & LR2021_IRQ_TX_DONE) {
        if (_cbTxDone) {
            _cbTxDone();
        }
    }

    if (irq & LR2021_IRQ_RX_DONE) {
        if (_cbRxDone) {
            // Get packet info
            uint16_t len = getRxPacketLength();
            int16_t rssi = getPacketRssi();
            int8_t snr = getPacketSnr();

            // Read data - for callback we use a stack buffer
            // For large packets (>256), user should use receiveBlocking with their own buffer
            uint8_t buffer[256];
            uint16_t readLen = (len > 256) ? 256 : len;
            readFifo(buffer, readLen);

            _cbRxDone(buffer, readLen, rssi, snr);
        }
    }

    if (irq & LR2021_IRQ_CRC_ERROR) {
        if (_cbCrcError) {
            _cbCrcError();
        }
    }

    if (irq & LR2021_IRQ_TIMEOUT) {
        if (_cbTimeout) {
            _cbTimeout();
        }
    }

    if (irq & LR2021_IRQ_CAD_DONE) {
        if (_cbCadDone) {
            bool detected = (irq & LR2021_IRQ_CAD_DETECTED) != 0;
            _cbCadDone(detected);
        }
    }
}

// ============================================================================
// Utility Functions
// ============================================================================

uint32_t LR2021::getRandomNumber(uint8_t source) {
    uint8_t params[] = {source};
    uint8_t result[4];
    readCommand(LR2021_CMD_GET_RANDOM_NUMBER, params, 1, result, 4);
    return ((uint32_t)result[0] << 24) | ((uint32_t)result[1] << 16) |
           ((uint32_t)result[2] << 8) | result[3];
}

int8_t LR2021::getTemperature() {
    uint8_t params[] = {0x00};  // Default format
    uint8_t result[2];
    readCommand(LR2021_CMD_GET_TEMP, params, 1, result, 2);
    return (int8_t)result[0];  // Simplified - returns raw value
}

uint16_t LR2021::getBatteryVoltage() {
    uint8_t params[] = {0x00};  // Default format
    uint8_t result[2];
    readCommand(LR2021_CMD_GET_VBAT, params, 1, result, 2);
    return ((uint16_t)result[0] << 8) | result[1];
}