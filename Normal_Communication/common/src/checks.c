#include "kuse/checks.h"

void kuse_order_tracker_reset(kuse_order_tracker_t *tracker, uint32_t modulus) {
    tracker->previous = 0;
    tracker->modulus = modulus;
    tracker->initialized = false;
}

kuse_order_status_t kuse_order_tracker_accept(kuse_order_tracker_t *tracker,
                                               uint16_t current,
                                               uint16_t *missing_count) {
    uint16_t delta;
    *missing_count = 0;
    if (!tracker->initialized) {
        tracker->previous = current;
        tracker->initialized = true;
        return KUSE_ORDER_FIRST;
    }
    delta = (uint16_t)(((uint32_t)current + tracker->modulus - tracker->previous) % tracker->modulus);
    if (delta == 0) return KUSE_ORDER_DUPLICATE;
    if (delta >= tracker->modulus / 2u) return KUSE_ORDER_OLD;
    tracker->previous = current;
    if (delta == 1) return KUSE_ORDER_NEXT;
    *missing_count = (uint16_t)(delta - 1u);
    return KUSE_ORDER_GAP;
}

static uint8_t popcount8(uint8_t value) {
    uint8_t count = 0;
    while (value != 0) {
        count = (uint8_t)(count + (value & 1u));
        value >>= 1;
    }
    return count;
}

uint8_t kuse_whl_spd11_checksum(const uint8_t data[8]) {
    uint8_t count = 0;
    uint8_t i;
    for (i = 0; i < 8; ++i) {
        const uint8_t value = (i & 1u) != 0 ? (uint8_t)(data[i] & 0x3Fu) : data[i];
        count = (uint8_t)(count + popcount8(value));
    }
    return (uint8_t)((count ^ 9u) & 0x0Fu);
}

void kuse_whl_spd11_set_counter_checksum(uint8_t data[8], uint8_t counter) {
    uint8_t checksum;
    data[1] = (uint8_t)((data[1] & 0x3Fu) | ((counter & 0x03u) << 6));
    data[3] = (uint8_t)((data[3] & 0x3Fu) | (((counter >> 2) & 0x03u) << 6));
    data[5] &= 0x3Fu;
    data[7] &= 0x3Fu;
    checksum = kuse_whl_spd11_checksum(data);
    data[5] = (uint8_t)(data[5] | ((checksum & 0x03u) << 6));
    data[7] = (uint8_t)(data[7] | (((checksum >> 2) & 0x03u) << 6));
}

bool kuse_whl_spd11_checksum_valid(const uint8_t data[8]) {
    const uint8_t stored = (uint8_t)(((data[5] >> 6) & 0x03u) | (((data[7] >> 6) & 0x03u) << 2));
    return stored == kuse_whl_spd11_checksum(data);
}

uint8_t kuse_crc8_j1850_opendbc(const uint8_t *data, size_t size) {
    uint8_t crc = 0x22;
    size_t n;
    for (n = 0; n < size; ++n) {
        uint8_t i;
        crc ^= data[n];
        for (i = 0; i < 8; ++i) {
            crc = (crc & 0x80u) != 0 ? (uint8_t)((crc << 1) ^ 0x1Du) : (uint8_t)(crc << 1);
        }
    }
    return (uint8_t)(crc ^ 0xDFu);
}

uint8_t kuse_lkas11_crc(const uint8_t data[8]) {
    const uint8_t input[7] = {data[0], data[1], data[2], data[3], data[4], data[5], data[7]};
    return kuse_crc8_j1850_opendbc(input, 7);
}

void kuse_lkas11_set_crc(uint8_t data[8]) {
    data[6] = kuse_lkas11_crc(data);
}

bool kuse_lkas11_crc_valid(const uint8_t data[8]) {
    return data[6] == kuse_lkas11_crc(data);
}

