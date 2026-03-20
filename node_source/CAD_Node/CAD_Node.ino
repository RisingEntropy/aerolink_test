/**
 * @file CAD_Node.ino
 * @brief LR2021 CAD Test Node - Standalone firmware with custom binary protocol
 *
 * This node accepts CAD commands from the host Python script, performs
 * loraCadBlocking(), and reports the result back.
 *
 * Custom Protocol (independent of existing node_comm protocol):
 *   Frame: [0xCA][type:1][len:1][payload:N][xor_checksum:1][0xCA]
 *
 *   Host → Node:
 *     CMD_INIT (0x01): freq_hz(4) + sf(1) + bw_khz(2) + cr(1)
 *                      + num_symbols(1) + timeout_ms(2)  [11 bytes]
 *     CMD_CAD  (0x02): no payload
 *
 *   Node → Host:
 *     RES_READY      (0x80): no payload
 *     RES_CAD_RESULT (0x81): detected(1) + timed_out(1) + duration_us(4)  [6 bytes]
 *
 * Wiring (STICK2021):
 *   LR2021    ESP32
 *   ------    -----
 *   NSS       GPIO8
 *   SCK       GPIO9
 *   MOSI      GPIO10
 *   MISO      GPIO11
 *   BUSY      GPIO13
 *   RESET     GPIO12
 *   DIO9      GPIO14
 */

#include <SPI.h>
#include "LR2021.h"

// ============================================================================
// Pin Definitions
// ============================================================================
#define PIN_NSS     8
#define PIN_BUSY    13
#define PIN_RST     12
#define PIN_DIO9    14
#define PIN_MISO    11
#define PIN_MOSI    10
#define PIN_SCK     9

// ============================================================================
// Protocol Constants
// ============================================================================
#define MAGIC           0xCA
#define CMD_PING        0x00  // Host sends this after connecting; node replies RES_READY
#define CMD_INIT        0x01
#define CMD_CAD         0x02
#define RES_READY       0x80
#define RES_CAD_RESULT  0x81

// ============================================================================
// Globals
// ============================================================================
SPIClass spiLR(HSPI);
LR2021 radio(PIN_NSS, PIN_BUSY, PIN_RST, PIN_DIO9, &spiLR);

uint32_t g_freq_hz     = 900000000UL;
uint8_t  g_sf          = 7;
uint16_t g_bw_khz      = 125;
uint8_t  g_cr          = 5;
uint8_t  g_num_symbols = 4;     // actual count: 1/2/4/8/16
uint16_t g_timeout_ms  = 500;
bool     g_configured  = false;

// Frame parser state
#define RX_BUF_SIZE 32
uint8_t  rxBuf[RX_BUF_SIZE];
uint8_t  rxIdx   = 0;
bool     inFrame = false;

// ============================================================================
// Helpers: LR2021 code translation
// ============================================================================

uint8_t getBwCode(uint16_t bw_khz) {
    if (bw_khz <= 31)  return LR2021_LORA_BW_31;
    if (bw_khz <= 41)  return LR2021_LORA_BW_41;
    if (bw_khz <= 62)  return LR2021_LORA_BW_62;
    if (bw_khz <= 83)  return LR2021_LORA_BW_83;
    if (bw_khz <= 101) return LR2021_LORA_BW_101;
    if (bw_khz <= 125) return LR2021_LORA_BW_125;
    if (bw_khz <= 203) return LR2021_LORA_BW_203;
    if (bw_khz <= 250) return LR2021_LORA_BW_250;
    if (bw_khz <= 406) return LR2021_LORA_BW_406;
    if (bw_khz <= 500) return LR2021_LORA_BW_500;
    if (bw_khz <= 812) return LR2021_LORA_BW_812;
    return LR2021_LORA_BW_1000;
}

uint8_t getCrCode(uint8_t cr) {
    switch (cr) {
        case 6: return LR2021_LORA_CR_4_6;
        case 7: return LR2021_LORA_CR_4_7;
        case 8: return LR2021_LORA_CR_4_8;
        default: return LR2021_LORA_CR_4_5;
    }
}

uint8_t getLdro(uint8_t sf, uint16_t bw_khz) {
    if (sf >= 11 && bw_khz <= 125) return LR2021_LORA_LDRO_ON;
    if (sf >= 12 && bw_khz <= 250) return LR2021_LORA_LDRO_ON;
    return LR2021_LORA_LDRO_OFF;
}

uint8_t getSymbolsCode(uint8_t n) {
    switch (n) {
        case 1:  return LR2021_CAD_SYMBOLS_1;
        case 2:  return LR2021_CAD_SYMBOLS_2;
        case 8:  return LR2021_CAD_SYMBOLS_8;
        case 16: return LR2021_CAD_SYMBOLS_16;
        default: return LR2021_CAD_SYMBOLS_4;  // 4 is the safe default
    }
}

// ============================================================================
// Protocol: Send
// ============================================================================

void sendFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++) checksum ^= payload[i];

    Serial.write(MAGIC);
    Serial.write(type);
    Serial.write(len);
    if (len > 0) Serial.write(payload, len);
    Serial.write(checksum);
    Serial.write(MAGIC);
    Serial.flush();
}

void sendReady() {
    sendFrame(RES_READY, nullptr, 0);
}

void sendCadResult(bool detected, bool timed_out, uint32_t duration_us) {
    uint8_t payload[6];
    payload[0] = detected   ? 1 : 0;
    payload[1] = timed_out  ? 1 : 0;
    payload[2] = (uint8_t)(duration_us & 0xFF);
    payload[3] = (uint8_t)((duration_us >> 8) & 0xFF);
    payload[4] = (uint8_t)((duration_us >> 16) & 0xFF);
    payload[5] = (uint8_t)((duration_us >> 24) & 0xFF);
    sendFrame(RES_CAD_RESULT, payload, 6);
}

// ============================================================================
// Command Handlers
// ============================================================================

void configureRadio() {
    radio.setStandby();
    radio.setPacketType(LR2021_PACKET_TYPE_LORA);
    radio.setRfFrequency(g_freq_hz);

    // Select RX path by frequency band
    if (g_freq_hz >= 2400000000UL) {
        radio.setRxPath(LR2021_RX_PATH_HF, 4);
    } else {
        radio.setRxPath(LR2021_RX_PATH_LF, 0);
    }

    radio.setLoraModulationParams(
        g_sf,
        getBwCode(g_bw_khz),
        getCrCode(g_cr),
        getLdro(g_sf, g_bw_khz)
    );
    radio.setLoraPacketParams(8, 255, LR2021_LORA_HEADER_EXPLICIT,
                              LR2021_LORA_CRC_ON, LR2021_LORA_IQ_STANDARD);
    radio.setLoraSyncword(LR2021_LORA_SYNCWORD_PRIVATE);
    radio.setRxTxFallbackMode(LR2021_FALLBACK_STDBY_RC);
    radio.setDioIrqConfig(9, LR2021_IRQ_CAD_DONE | LR2021_IRQ_CAD_DETECTED);
}

// CMD_INIT payload: freq_hz(4) sf(1) bw_khz(2) cr(1) num_symbols(1) timeout_ms(2)
void handleInit(const uint8_t* payload, uint8_t len) {
    if (len < 11) {
        // Malformed: send ready anyway so host doesn't hang
        sendReady();
        return;
    }

    g_freq_hz = (uint32_t)payload[0]
              | ((uint32_t)payload[1] << 8)
              | ((uint32_t)payload[2] << 16)
              | ((uint32_t)payload[3] << 24);
    g_sf          = payload[4];
    g_bw_khz      = (uint16_t)payload[5] | ((uint16_t)payload[6] << 8);
    g_cr          = payload[7];
    g_num_symbols = payload[8];
    g_timeout_ms  = (uint16_t)payload[9] | ((uint16_t)payload[10] << 8);

    configureRadio();
    g_configured = true;
    sendReady();
}

void handleCad() {
    if (!g_configured) {
        // Return a result indicating failure so host doesn't hang
        sendCadResult(false, true, 0);
        return;
    }

    uint32_t t_start = micros();

    bool detected = radio.loraCadBlocking(
        g_sf,
        getSymbolsCode(g_num_symbols),
        g_timeout_ms
    );

    uint32_t duration_us = micros() - t_start;

    // Heuristic: if duration >= 90% of timeout, treat as timed out
    bool timed_out = (duration_us >= (uint32_t)g_timeout_ms * 900);

    sendCadResult(detected, timed_out, duration_us);
}

// ============================================================================
// Protocol: Frame Parser
// ============================================================================

void processFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
    switch (type) {
        case CMD_PING: sendReady();              break;
        case CMD_INIT: handleInit(payload, len); break;
        case CMD_CAD:  handleCad();              break;
        default: break;
    }
}

void feedByte(uint8_t b) {
    if (!inFrame) {
        if (b == MAGIC) {
            inFrame = true;
            rxIdx   = 0;
        }
        return;
    }

    if (rxIdx >= RX_BUF_SIZE) {
        // Overflow — reset
        inFrame = false;
        rxIdx   = 0;
        return;
    }

    rxBuf[rxIdx++] = b;

    // rxBuf layout: [type:1][len:1][payload:len][checksum:1][end_magic:1]
    if (rxIdx < 2) return;  // need at least type + len

    uint8_t frame_len    = rxBuf[1];
    uint8_t total_needed = 2 + frame_len + 1 + 1;  // type+len+payload+checksum+end

    if (total_needed > RX_BUF_SIZE) {
        // Too large — reset
        inFrame = false;
        rxIdx   = 0;
        return;
    }

    if (rxIdx < total_needed) return;  // not yet complete

    // Validate checksum and end magic
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < frame_len; i++) checksum ^= rxBuf[2 + i];

    uint8_t rx_checksum = rxBuf[2 + frame_len];
    uint8_t end_magic   = rxBuf[2 + frame_len + 1];

    if (checksum == rx_checksum && end_magic == MAGIC) {
        processFrame(rxBuf[0], &rxBuf[2], frame_len);
    }

    inFrame = false;
    rxIdx   = 0;
}

// ============================================================================
// Arduino Entry Points
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(100);

    spiLR.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);

    if (!radio.begin(LR2021_TCXO_VOLTAGE_1_8V)) {
        // If init fails, stall — host will time out on READY
        while (1) delay(1000);
    }

    sendReady();
}

void loop() {
    while (Serial.available()) {
        feedByte((uint8_t)Serial.read());
    }
}
