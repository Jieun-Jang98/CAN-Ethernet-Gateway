#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace adas {
constexpr std::size_t STATE_SIZE = 12, COMMAND_SIZE = 10;
constexpr double STATE_TIMEOUT_SECONDS = 0.100;
inline double limit(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }
inline std::uint16_t u16(const std::uint8_t* p) {
    return std::uint16_t(std::uint16_t(p[0]) | (std::uint16_t(p[1]) << 8));
}
inline int i16(const std::uint8_t* p) { const int v = u16(p); return v >= 32768 ? v - 65536 : v; }
inline void put16(std::uint8_t* p, int value) {
    const auto v = static_cast<std::uint16_t>(value);
    p[0] = static_cast<std::uint8_t>(v & 255);
    p[1] = static_cast<std::uint8_t>(v >> 8);
}
enum class Reason : std::uint8_t {
    Active = 0, Timeout = 1, InvalidValue = 2, WheelMismatch = 3,
    AngleLimit = 4, DriverOverride = 5, InvalidTarget = 6,
    InvalidDt = 7, NoState = 8, Shutdown = 9
};
inline const char* reasonName(Reason r) {
    switch (r) {
    case Reason::Active: return "ACTIVE";
    case Reason::Timeout: return "STATE_TIMEOUT";
    case Reason::InvalidValue: return "INVALID_VALUE";
    case Reason::WheelMismatch: return "WHEEL_SPEED_MISMATCH";
    case Reason::AngleLimit: return "ANGLE_OUT_OF_RANGE";
    case Reason::DriverOverride: return "DRIVER_OVERRIDE";
    case Reason::InvalidTarget: return "INVALID_TARGET";
    case Reason::InvalidDt: return "INVALID_DT";
    case Reason::NoState: return "NO_STATE";
    case Reason::Shutdown: return "SHUTDOWN";
    }
    return "UNKNOWN";
}
struct VehicleState {
    std::uint16_t counter = 0;
    double speedFL = 0, speedFR = 0, angle = 0, driverTorque = 0;
};
inline bool decodeState(const std::uint8_t* p, std::size_t n, VehicleState& s) {
    if (n != STATE_SIZE || p[0] != 0x10 || p[3] != 8) return false;
    s = {u16(p + 1), u16(p + 4) / 100.0, u16(p + 6) / 100.0,
         i16(p + 8) / 10.0, double(i16(p + 10))};
    return true;
}
inline Reason validate(const VehicleState& s) {
    if (!std::isfinite(s.speedFL) || !std::isfinite(s.speedFR) ||
        !std::isfinite(s.angle) || !std::isfinite(s.driverTorque) ||
        s.speedFL < 0 || s.speedFL > 250 || s.speedFR < 0 || s.speedFR > 250)
        return Reason::InvalidValue;
    if (std::abs(s.speedFL - s.speedFR) > 15) return Reason::WheelMismatch;
    const double speed = (s.speedFL + s.speedFR) / 2;
    const double torqueLimit = speed < 30 ? 10 : speed < 80 ? 5 : 2;
    if (std::abs(s.driverTorque) > torqueLimit) return Reason::DriverOverride;
    return Reason::Active;
}
// Receive-time freshness only: wire protocol contains no sensor timestamp.
class StateTracker {
    bool haveState_ = false, haveCounter_ = false;
    std::uint16_t lastCounter_ = 0;
    double lastValid_ = 0;
    Reason reason_ = Reason::NoState;
    VehicleState latest_{};
public:
    bool accept(const std::uint8_t* p, std::size_t n, double now) {
        VehicleState state;
        if (!decodeState(p, n, state)) return false;
        const auto delta = static_cast<std::uint16_t>(state.counter - lastCounter_);
        if (haveCounter_ && (delta == 0 || delta > 32767)) return false;
        const Reason reason = validate(state);
        if (reason != Reason::Active) {
            haveState_ = false; reason_ = reason; return false;
        }
        latest_ = state; lastCounter_ = state.counter; lastValid_ = now;
        haveState_ = haveCounter_ = true; reason_ = Reason::Active;
        return true;
    }
    Reason status(double now) const {
        if (!haveState_) return reason_;
        return now - lastValid_ >= STATE_TIMEOUT_SECONDS ? Reason::Timeout : Reason::Active;
    }
    const VehicleState& state() const { return latest_; }
};
struct Command {
    bool enable = false;
    double targetAngle = 0;
    std::uint16_t torqueLimit = 0;
    double targetSpeed = 0;
    std::uint8_t decelerationPercent = 0;
    Reason reason = Reason::NoState;
};
inline Command disabled(Reason r) { Command c; c.reason = r; return c; }
class Controller {
    bool initialized_ = false;
    double previousAngle_ = 0, previousDecel_ = 0;
public:
    void reset() { initialized_ = false; previousAngle_ = previousDecel_ = 0; }
    Command step(const VehicleState& s, double targetKmh, double dt) {
        Reason reason = validate(s);
        if (reason == Reason::Active && (!std::isfinite(targetKmh) || targetKmh < 0 || targetKmh > 120))
            reason = Reason::InvalidTarget;
        if (reason == Reason::Active && (!std::isfinite(dt) || dt <= 0)) reason = Reason::InvalidDt;
        if (reason != Reason::Active) { reset(); return disabled(reason); }
        dt = std::min(dt, 0.1);
        const double speed = (s.speedFL + s.speedFR) / 2;
        const auto torqueLimit = static_cast<std::uint16_t>(speed < 30 ? 10 : speed < 80 ? 5 : 2);
        const double angleLimit = speed < 30 ? 30 : speed < 80 ? 10 : 3;
        const bool angleCorrection = std::abs(s.angle) > angleLimit;
        // Gateway wants a TARGET ANGLE, not a signed steering torque.
        // Straight driving reference = 0 degrees. Start from measured angle on re-enable.
        if (angleCorrection) {
            // Out-of-range steering is recoverable: actively request straight ahead.
            previousAngle_ = 0;
            initialized_ = true;
        } else {
            if (!initialized_) { previousAngle_ = s.angle; initialized_ = true; }
            previousAngle_ = limit(0, previousAngle_ - 10 * dt, previousAngle_ + 10 * dt);
            previousAngle_ = limit(previousAngle_, -angleLimit, angleLimit);
        }
        // Synthetic bench braking law: percent is NOT a calibrated m/s^2 conversion.
        // Target speed implies acceleration by the downstream target-speed controller.
        const double requested = limit(5.0 * (speed - targetKmh - 0.5), 0, 30);
        if (requested == 0) previousDecel_ = 0; // release braking once overspeed clears
        else previousDecel_ = limit(requested, previousDecel_ - 20 * dt, previousDecel_ + 20 * dt);
        return {true, previousAngle_, torqueLimit, targetKmh,
                static_cast<std::uint8_t>(std::lround(previousDecel_)),
                angleCorrection ? Reason::AngleLimit : Reason::Active};
    }
};
inline std::array<std::uint8_t, COMMAND_SIZE> steeringPacket(std::uint16_t sequence, const Command& c) {
    std::array<std::uint8_t, COMMAND_SIZE> p{};
    p[0] = 0x20; put16(p.data() + 1, sequence); p[3] = 6;
    if (c.enable) {
        put16(p.data() + 4, int(std::lround(limit(c.targetAngle, -3276.8, 3276.7) * 10)));
        put16(p.data() + 6, std::min<std::uint16_t>(c.torqueLimit, 200));
        p[8] = 1;
    }
    p[9] = static_cast<std::uint8_t>(c.reason);
    return p;
}
inline std::array<std::uint8_t, COMMAND_SIZE> speedPacket(std::uint16_t sequence, const Command& c) {
    std::array<std::uint8_t, COMMAND_SIZE> p{};
    p[0] = 0x21; put16(p.data() + 1, sequence); p[3] = 6;
    if (c.enable) {
        put16(p.data() + 4, int(std::lround(limit(c.targetSpeed, 0, 120) * 100)));
        p[6] = std::min<std::uint8_t>(c.decelerationPercent, 100); p[7] = 1;
    }
    p[8] = static_cast<std::uint8_t>(c.reason);
    // byte 9 reserved, always zero
    return p;
}
} // namespace adas

