#ifndef KUSE_CHECKS_H
#define KUSE_CHECKS_H

#include <cstddef>
#include <cstdint>

namespace kuse {

enum kuse_order_status_t {
    KUSE_ORDER_FIRST = 0,
    KUSE_ORDER_NEXT,
    KUSE_ORDER_GAP,
    KUSE_ORDER_DUPLICATE,
    KUSE_ORDER_OLD
};

struct kuse_order_tracker_t {
    uint16_t previous;
    uint32_t modulus;
    bool initialized;
};

void kuse_order_tracker_reset(kuse_order_tracker_t *tracker, uint32_t modulus);
kuse_order_status_t kuse_order_tracker_accept(kuse_order_tracker_t *tracker,
                                               uint16_t current,
                                               uint16_t *missing_count);
uint8_t kuse_whl_spd11_checksum(const uint8_t data[8]);
void kuse_whl_spd11_set_counter_checksum(uint8_t data[8], uint8_t counter);
bool kuse_whl_spd11_checksum_valid(const uint8_t data[8]);
uint8_t kuse_lkas11_crc(const uint8_t data[8]);
uint8_t kuse_crc8_j1850_opendbc(const uint8_t *data, size_t size);
void kuse_lkas11_set_crc(uint8_t data[8]);
bool kuse_lkas11_crc_valid(const uint8_t data[8]);

} // namespace kuse

#endif

