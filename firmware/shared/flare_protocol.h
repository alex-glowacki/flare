// =============================================================================
// FLARE - flare_protocol.h
// Shared RC packet definition for ESP-NOW link (remote → quad)
//
// Include this header in both esp32_quad and esp32_remote firmware.
// Shared location: firmware/shared/flare_protocol.h
//
// Packet layout (14 bytes total):
//   [0]      magic      0xAF — sync/identity byte
//   [1–2]    throttle   uint16_t, 1000–2000 (1000 = min)
//   [3–4]    roll       uint16_t, 1000–2000 (1500 = center)
//   [5–6]    pitch      uint16_t, 1000–2000 (1500 = center)
//   [7–8]    yaw        uint16_t, 1000–2000 (1500 = center)
//   [9]      armed      uint8_t,  0 = disarmed, 1 = armed
//   [10]     mode       uint8_t,  FLARE_MODE_* constants
//   [11–12]  reserved   uint8_t[2], zero-padded, reserved for future use
//   [13]     checksum   CRC-8/MAXIM over bytes [0–12]
// =============================================================================

#pragma once

#include <stddef.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------

#define FLARE_PACKET_MAGIC 0xAFU /* sync byte - identifies FLARE packets */
#define FLARE_PACKET_SIZE 14U    /* total packet size in bytes */
#define FLARE_PACKET_DATA_LEN                                                  \
  13U /* bytes covered by CRC (everything but checksum) */

/* Channel range */
#define FLARE_CH_MIN 1000U
#define FLARE_CH_MID 1500U
#define FLARE_CH_MAX 2000U

/* Armed flag */
#define FLARE_ARMED 1U
#define FLARE_DISARMED 0U

/* Flight mode
 * FLARE_MODE_SAFE: switch in center/off position — treated as disarmed
 * regardless of arm switch state. Requires deliberate mode selection
 * (ANGLE or ACRO) before the FC will arm. */
#define FLARE_MODE_ANGLE 0U /* self-levelling (angle mode) */
#define FLARE_MODE_ACRO 1U  /* rate mode (acro) */
#define FLARE_MODE_SAFE 2U  /* center/off position — forces disarm on FC */

// ---------------------------------------------------------------------------
// Packet struct
//
// __attribute__((packed)) ensures no padding bytes are inserted by the
// compiler. Both ESP32 targets are little-endian — multi-byte fields
// transmit LSB first, which is consistent across both ends.
// ---------------------------------------------------------------------------

typedef struct __attribute__((packed)) {
  uint8_t magic;     /* 0xAF */
  uint16_t throttle; /* 1000-2000, 1000 = motors off */
  uint16_t roll;     /* 1000-2000, 1500 = center */
  uint16_t pitch;    /* 1000-2000, 1500 = center */
  uint16_t yaw;      /* 1000-2000, 1500 = center */
  uint8_t armed;     /* FLARE_ARMED / FLARE_DISARMED */
  uint8_t mode;      /* FLARE_MODE_ANGLE / FLARE_MODE_ACRO / FLARE_MODE_SAFE */
  uint8_t reserved[2]; /* zero-pad - reserved for future channels */
  uint8_t checksum;    /* CRC-8/MAXIM over bytes [0-12] */
} FLARE_RC_Packet_t;

// Compile-time size guard - catches struct padding bugs immediately
_Static_assert(sizeof(FLARE_RC_Packet_t) == FLARE_PACKET_SIZE,
               "FLARE_RC_Packet_t size mismatch - check for padding");

// ---------------------------------------------------------------------------
// CRC-8/MAXIM (Dallas/Maxim 1-Wire CRC)
//   Polynomial : 0x31 (x^8 + x^5 + x^4 + 1), reflected
//   Init       : 0x00
//   RefIn/Out  : true
//   XorOut     : 0x00
//
// Computed byte-by-byte with a lookup table for speed. The table is
// defined as a static const array inside the inline function to keep
// this header self-contained (no separate .c file required).
// ---------------------------------------------------------------------------

static inline uint8_t flare_crc8(const uint8_t *data, size_t len) {
  /* CRC-8/MAXIM lookup table - generated for polynomial 0x31 reflected */
  static const uint8_t kCrcTable[256] = {
      0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20,
      0xA3, 0xFD, 0x1F, 0x41, 0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
      0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC, 0x23, 0x7D, 0x9F, 0xC1,
      0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
      0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E,
      0x1D, 0x43, 0xA1, 0xFF, 0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
      0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07, 0xDB, 0x85, 0x67, 0x39,
      0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
      0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45,
      0xC6, 0x98, 0x7A, 0x24, 0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
      0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9, 0x8C, 0xD2, 0x30, 0x6E,
      0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
      0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31,
      0xB2, 0xEC, 0x0E, 0x50, 0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
      0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE, 0x32, 0x6C, 0x8E, 0xD0,
      0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
      0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA,
      0x69, 0x37, 0xD5, 0x8B, 0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
      0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16, 0xE9, 0xB7, 0x55, 0x0B,
      0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
      0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54,
      0xD7, 0x89, 0x6B, 0x35,
  };

  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc = kCrcTable[crc ^ data[i]];
  }
  return crc;
}

// ---------------------------------------------------------------------------
// flare_checksum() — compute the CRC-8 checksum for a packet
//
// Covers all bytes except the checksum field itself (bytes [0–12]).
// Call this to both populate and verify the checksum field.
//
// Usage (populate):
//   pkt.checksum = flare_checksum(&pkt);
//
// Usage (verify):
//   if (pkt.checksum != flare_checksum(&pkt)) { /* reject */ }
// ---------------------------------------------------------------------------

static inline uint8_t flare_checksum(const FLARE_RC_Packet_t *pkt) {
  return flare_crc8((const uint8_t *)pkt, FLARE_PACKET_DATA_LEN);
}

#ifdef __cplusplus
}
#endif