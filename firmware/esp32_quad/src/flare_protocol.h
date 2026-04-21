#pragma once

#include <stdint.h>

// ---------------------------------------------------------------------------
// FLARE ESP-NOW packet — shared between esp32_quad and esp32_remote
//
// Wire format (little-endian, packed):
//   [0]     magic       0xFA
//   [1]     ch_roll     uint8  0–255 (stick left-right)
//   [2]     ch_pitch    uint8  0–255 (stick fore-aft)
//   [3]     ch_throttle uint8  0–255 (stick up-down)
//   [4]     ch_yaw      uint8  0–255 (stick left-right)
//   [5]     switches    uint8  bit0=SW1, bit1=SW2
//   [6]     checksum    uint8  XOR of bytes [0]..[5]
// ---------------------------------------------------------------------------

#define FLARE_PACKET_MAGIC 0xFA
#define FLARE_PACKET_SIZE 7         // total bytes including magic + checksum

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t ch_roll;
    uint8_t ch_pitch;
    uint8_t ch_throttle;
    uint8_t ch_yaw;
    uint8_t switches;
    uint8_t checksum;
} FLARE_RC_Packet_t;

// Compute XOR checksum over bytes [0]..[5] (everything except checksum itself)
static inline uint8_t flare_checksum(const FLARE_RC_Packet_t *pkt) {
    const uint8_t *b = (const uint8_t *)pkt;
    uint8_t cs = 0;
    for (int i = 0; i < FLARE_PACKET_SIZE - 1; i++) {
        cs ^= b[i];
    }
    return cs;
}