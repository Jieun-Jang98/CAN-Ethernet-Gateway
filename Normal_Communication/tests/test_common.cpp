#include "kuse/common.hpp"
#include <cstdio>
#include <cstring>

using namespace kuse;

static int failures = 0;

#define CHECK(expr) do { if (!(expr)) { \
    std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); ++failures; \
} } while (0)

static void check_bytes(const uint8_t *actual, const uint8_t *expected, size_t size) {
    CHECK(std::memcmp(actual, expected, size) == 0);
}

static void test_udp_vectors(void) {
    const uint8_t vehicle_expected[12] = {0x10,0x00,0x00,0x08,0x13,0x88,0x13,0x88,0x00,0x14,0x00,0x00};
    const uint8_t command_expected[8] = {0x20,0x00,0x00,0x04,0xFF,0xFD,0x01,0x00};
    uint8_t bytes[12];
    kuse_vehicle_state_t state = {0, 5000, 5000, 20, 0};
    kuse_vehicle_state_t decoded_state;
    kuse_steering_command_t command = {0, -3, 1};
    kuse_steering_command_t decoded_command;
    kuse_encode_vehicle_state(&state, bytes);
    check_bytes(bytes, vehicle_expected, 12);
    CHECK(kuse_decode_vehicle_state(bytes, 12, &decoded_state) == KUSE_DECODE_OK);
    CHECK(decoded_state.wheel_speed_fl == 5000 && decoded_state.steering_angle == 20);
    kuse_encode_steering_command(&command, bytes);
    check_bytes(bytes, command_expected, 8);
    CHECK(kuse_decode_steering_command(bytes, 8, &decoded_command) == KUSE_DECODE_OK);
    CHECK(decoded_command.torque_request == -3 && decoded_command.enable == 1);
    bytes[7] = 1;
    CHECK(kuse_decode_steering_command(bytes, 8, &decoded_command) == KUSE_DECODE_BAD_RESERVED);
    bytes[7] = 0; bytes[6] = 2;
    CHECK(kuse_decode_steering_command(bytes, 8, &decoded_command) == KUSE_DECODE_BAD_ENABLE);
    bytes[6] = 1; bytes[4] = 0x04; bytes[5] = 0x00;
    CHECK(kuse_decode_steering_command(bytes, 8, &decoded_command) == KUSE_DECODE_BAD_RANGE);
    CHECK(kuse_decode_steering_command(bytes, 7, &decoded_command) == KUSE_DECODE_BAD_LENGTH);
}

static void test_can_vectors(void) {
    const uint8_t minus3[8] = {0x00,0x00,0xFD,0x0B,0x00,0x00,0x17,0x00};
    const uint8_t minus20[8] = {0x00,0x00,0xEC,0x0B,0x00,0x00,0x93,0x00};
    const uint8_t release[8] = {0x00,0x00,0x00,0x04,0x00,0x00,0x44,0x00};
    uint8_t data[8];
    kuse_encode_lkas_fields(data, -3, true, 0); kuse_lkas11_set_crc(data);
    check_bytes(data, minus3, 8);
    CHECK(kuse_decode_lkas_torque(data) == -3 && kuse_decode_lkas_enable(data));
    kuse_encode_lkas_fields(data, -20, true, 0); kuse_lkas11_set_crc(data);
    check_bytes(data, minus20, 8);
    kuse_encode_lkas_fields(data, 0, false, 0); kuse_lkas11_set_crc(data);
    check_bytes(data, release, 8);
}

static void test_all_crc_vectors(void) {
    static const uint8_t vectors[][8] = {
        {0x00,0x00,0xFD,0x0B,0x00,0x00,0x17,0x00},
        {0x00,0x00,0xEC,0x0B,0x00,0x00,0x93,0x00},
        {0x00,0x00,0xEC,0x0B,0x50,0x00,0xEB,0x00},
        {0x00,0x00,0x14,0x0C,0xF0,0x00,0x0B,0x00},
        {0x00,0x00,0x00,0x04,0x00,0x00,0x44,0x00},
        {0x00,0x00,0x00,0x04,0x90,0x00,0x9C,0x00}
    };
    size_t i;
    uint8_t zero[8] = {0};
    const uint8_t ascii[9] = {'1','2','3','4','5','6','7','8','9'};
    for (i = 0; i < sizeof(vectors)/sizeof(vectors[0]); ++i) {
        CHECK(kuse_lkas11_crc(vectors[i]) == vectors[i][6]);
        CHECK(kuse_lkas11_crc_valid(vectors[i]));
    }
    CHECK(kuse_lkas11_crc(zero) == 0x0A);
    CHECK(kuse_crc8_j1850_opendbc(ascii, 9) == 0x1E);
    zero[6] = 1;
    CHECK(!kuse_lkas11_crc_valid(zero));
}

static void test_dbc_and_checks(void) {
    uint8_t data[8] = {0};
    kuse_encode_wheel_speeds(data, 50.0, 50.0);
    CHECK(kuse_decode_wheel_speed_fl(data) == 50.0);
    CHECK(kuse_decode_wheel_speed_fr(data) == 50.0);
    kuse_whl_spd11_set_counter_checksum(data, 15);
    CHECK(kuse_whl_spd11_checksum_valid(data));
    data[0] ^= 1;
    CHECK(!kuse_whl_spd11_checksum_valid(data));
    std::memset(data, 0, sizeof(data));
    kuse_encode_driver_torque(data, -200);
    CHECK(kuse_decode_driver_torque(data) == -200);
    std::memset(data, 0, sizeof(data));
    kuse_encode_sas_angle_deci_deg(data, -123);
    CHECK(kuse_decode_sas_angle_deci_deg(data) == -123);
    CHECK(kuse_round_half_away_from_zero(2.5) == 3);
    CHECK(kuse_round_half_away_from_zero(-2.5) == -3);
}

static void test_order_tracking(void) {
    kuse_order_tracker_t tracker;
    uint16_t missing;
    kuse_order_tracker_reset(&tracker, 16);
    CHECK(kuse_order_tracker_accept(&tracker, 15, &missing) == KUSE_ORDER_FIRST);
    CHECK(kuse_order_tracker_accept(&tracker, 0, &missing) == KUSE_ORDER_NEXT);
    CHECK(kuse_order_tracker_accept(&tracker, 2, &missing) == KUSE_ORDER_GAP && missing == 1);
    CHECK(kuse_order_tracker_accept(&tracker, 2, &missing) == KUSE_ORDER_DUPLICATE);
    CHECK(kuse_order_tracker_accept(&tracker, 1, &missing) == KUSE_ORDER_OLD);
    kuse_order_tracker_reset(&tracker, 65536u);
    CHECK(kuse_order_tracker_accept(&tracker, 65535, &missing) == KUSE_ORDER_FIRST);
    CHECK(kuse_order_tracker_accept(&tracker, 0, &missing) == KUSE_ORDER_NEXT);
}

static void test_torque(void) {
    kuse_rate_result_t r;
    CHECK(kuse_torque_limit_for_speed(1900) == 0);
    CHECK(kuse_torque_limit_for_speed(2000) == 100);
    CHECK(kuse_torque_limit_for_speed(2999) == 100);
    CHECK(kuse_torque_limit_for_speed(3000) == 60);
    CHECK(kuse_torque_limit_for_speed(7999) == 60);
    CHECK(kuse_torque_limit_for_speed(8000) == 30);
    CHECK(kuse_torque_limit_for_speed(12000) == 30);
    CHECK(kuse_torque_limit_for_speed(12001) == 0);
    CHECK(!kuse_torque_within_limit(61, 60));
    CHECK(kuse_torque_clamp_to_limit(-101, 100) == -100);
    r = kuse_torque_rate_limit(-20, 0, 60); CHECK(r.torque == -3 && r.rate_clamped);
    r = kuse_torque_rate_limit(-20, -3, 60); CHECK(r.torque == -6 && r.rate_clamped);
    r = kuse_torque_rate_limit(0, 60, 60); CHECK(r.torque == 53 && r.rate_clamped);
    r = kuse_torque_rate_limit(-60, 4, 60); CHECK(r.torque == -3 && r.rate_clamped);
    r = kuse_torque_rate_limit(60, 100, 60); CHECK(r.torque == 60 && !r.rate_clamped);
}

static void test_required_rate_sequences(void) {
    kuse_torque_t actual = 0;
    int i;
    const kuse_torque_t toward_zero[] = {53,46,39,32,25,18,11,4,0};
    const kuse_torque_t crossing[] = {53,46,39,32,25,18,11,4,-3,-6};
    for (i = 1; i <= 20; ++i) {
        actual = kuse_torque_rate_limit(60, actual, 60).torque;
        CHECK(actual == static_cast<kuse_torque_t>((i * 3)));
    }
    actual = 60;
    for (i = 0; i < static_cast<int>((sizeof(toward_zero) / sizeof(toward_zero[0]))); ++i) {
        actual = kuse_torque_rate_limit(0, actual, 60).torque;
        CHECK(actual == toward_zero[i]);
    }
    actual = 60;
    for (i = 0; i < static_cast<int>((sizeof(crossing) / sizeof(crossing[0]))); ++i) {
        actual = kuse_torque_rate_limit(-60, actual, 60).torque;
        CHECK(actual == crossing[i]);
    }
}

int main(void) {
    test_udp_vectors();
    test_can_vectors();
    test_all_crc_vectors();
    test_dbc_and_checks();
    test_order_tracking();
    test_torque();
    test_required_rate_sequences();
    if (failures != 0) return 1;
    std::puts("All common tests passed.");
    return 0;
}

