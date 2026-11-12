#include "straight_node.hpp"
#include "kuse/can_codec.hpp"
#include "kuse/checks.hpp"
#include <cassert>
#include <cstdint>

static void make_lkas(uint8_t data[8], int16_t torque, bool enabled, uint8_t counter) {
    kuse::kuse_encode_lkas_fields(data, torque, enabled, counter);
    kuse::kuse_lkas11_set_crc(data);
}

int main() {
    kuse::straight_node node;
    auto wheel = node.make_status(kuse::straight_tx_kind::wheel, 0);
    assert(wheel.id == 0x386 && wheel.dlc == 8);
    assert(kuse::kuse_decode_wheel_speed_fl(wheel.data) == 50.0);
    assert(kuse::kuse_decode_wheel_speed_fr(wheel.data) == 50.0);
    assert(kuse::kuse_can_get_unsigned_le(wheel.data, 32, 14) == 1600);
    assert(kuse::kuse_can_get_unsigned_le(wheel.data, 48, 14) == 1600);
    assert(kuse::kuse_whl_spd11_checksum_valid(wheel.data));
    assert(kuse::kuse_can_get_unsigned_le(wheel.data, 14, 2) == 0);
    node.tx_queued(kuse::straight_tx_kind::wheel);
    wheel = node.make_status(kuse::straight_tx_kind::wheel, 20);
    assert(kuse::kuse_can_get_unsigned_le(wheel.data, 14, 2) == 1);
    assert(kuse::kuse_whl_spd11_checksum_valid(wheel.data));

    const auto steering = node.make_status(kuse::straight_tx_kind::steering, 0);
    assert(steering.id == 0x2B0 && steering.dlc == 5);
    assert(kuse::kuse_decode_sas_angle_deci_deg(steering.data) == 0);
    const auto driver = node.make_status(kuse::straight_tx_kind::driver, 0);
    assert(driver.id == 0x251 && driver.dlc == 8);
    assert(kuse::kuse_decode_driver_torque(driver.data) == 0);

    uint8_t lkas[8] = {};
    make_lkas(lkas, -3, true, 0);
    assert(lkas[6] == 0x17);
    assert(node.receive_lkas(0x340, false, false, 8, lkas, 100) ==
           kuse::straight_rx_result::accepted);
    assert(node.output_enabled() && node.output_torque() == -3);
    assert(node.scenario_started() && node.cycle(100) == 0);
    assert(node.angle_deci_deg(2099) == 0);
    assert(node.angle_deci_deg(2100) == 20);
    assert(node.angle_deci_deg(4100) == 10);
    assert(node.angle_deci_deg(6100) == 0);
    assert(node.angle_deci_deg(8100) == -20);
    assert(node.angle_deci_deg(10100) == 0);
    assert(node.cycle(12100) == 1 && node.angle_deci_deg(12100) == 0);
    assert(kuse::kuse_decode_sas_angle_deci_deg(
               node.make_status(kuse::straight_tx_kind::steering, 8100).data) == -20);

    // Bad CRC, duplicate counter and wrong DLC cannot refresh the 100 ms deadline.
    lkas[6] ^= 1;
    assert(node.receive_lkas(0x340, false, false, 8, lkas, 150) ==
           kuse::straight_rx_result::bad_crc);
    lkas[6] ^= 1;
    assert(node.receive_lkas(0x340, false, false, 8, lkas, 160) ==
           kuse::straight_rx_result::bad_counter);
    assert(node.receive_lkas(0x340, false, false, 7, lkas, 170) ==
           kuse::straight_rx_result::wrong_frame);
    node.poll_timeout(199);
    assert(!node.timed_out());
    node.poll_timeout(200);
    assert(node.timed_out() && node.recovering());
    assert(!node.output_enabled() && node.output_torque() == 0);

    // After timeout, the counter baseline is reset. Three consecutive zero
    // frames recover; the third still releases and the following frame applies.
    for (uint8_t n = 0; n != 3; ++n) {
        make_lkas(lkas, 0, false, n);
        assert(node.receive_lkas(0x340, false, false, 8, lkas, 220 + 20 * n) ==
               kuse::straight_rx_result::accepted);
        assert(!node.output_enabled() && node.output_torque() == 0);
    }
    assert(!node.recovering());
    make_lkas(lkas, -3, true, 3);
    assert(node.receive_lkas(0x340, false, false, 8, lkas, 280) ==
           kuse::straight_rx_result::accepted);
    assert(node.output_enabled() && node.output_torque() == -3);

    // A gap is valid for sequencing but restarts the consecutive recovery count.
    node.poll_timeout(380);
    make_lkas(lkas, 0, false, 0);
    node.receive_lkas(0x340, false, false, 8, lkas, 400);
    make_lkas(lkas, 0, false, 2);
    node.receive_lkas(0x340, false, false, 8, lkas, 420);
    assert(node.recovering());
    make_lkas(lkas, 0, false, 3);
    node.receive_lkas(0x340, false, false, 8, lkas, 440);
    make_lkas(lkas, 0, false, 4);
    node.receive_lkas(0x340, false, false, 8, lkas, 460);
    assert(node.recovering());
    make_lkas(lkas, 0, false, 5);
    node.receive_lkas(0x340, false, false, 8, lkas, 480);
    assert(!node.recovering());

    // A rejected LKAS11 frame also interrupts the three-frame recovery run.
    node.poll_timeout(580);
    make_lkas(lkas, 0, false, 0);
    node.receive_lkas(0x340, false, false, 8, lkas, 600);
    make_lkas(lkas, 0, false, 1);
    lkas[6] ^= 1;
    assert(node.receive_lkas(0x340, false, false, 8, lkas, 620) ==
           kuse::straight_rx_result::bad_crc);
    lkas[6] ^= 1;
    node.receive_lkas(0x340, false, false, 8, lkas, 640);
    make_lkas(lkas, 0, false, 2);
    node.receive_lkas(0x340, false, false, 8, lkas, 660);
    assert(node.recovering());
    make_lkas(lkas, 0, false, 3);
    node.receive_lkas(0x340, false, false, 8, lkas, 680);
    assert(!node.recovering());
}
