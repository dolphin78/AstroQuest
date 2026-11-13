// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

// The gamepad's own motion sensors, together with the hands around it. The gyroscope says how
// it turns, quickly and smoothly but drifting; gravity, in the accelerometer, takes the drift
// out of pitch and roll; the hands, which know which way it is turned, take it out of the
// heading. This is the emulator's filter (Runtime::UpdatePadGyro in
// shadps4-arm64-main/src/core/vr/vr_runtime.cpp), gains and all, so that what the viewer shows
// is what the emulator would make of the same sensors. No Windows, OpenXR or SDL in here
// (tools/tests/held_pad_fusion_test.cpp).
//
// Readings are in the gamepad's frame as SDL reports them: +X right, +Y out of the face
// buttons, +Z towards the player; rad/s, and m/s² with gravity included. The attitude takes
// that frame to the headset's space (+Y up, -Z ahead).

#include <algorithm>
#include <cmath>

#include "held_pad.h"

namespace HeldPadFusion {

using HeldPad::Quat;
using HeldPad::Vec;

inline constexpr float Pi = 3.14159265f;

using HeldPad::CentreFrom;
using HeldPad::Multiply;
using HeldPad::Normalize;

/// About an axis of length one.
inline Quat AxisAngle(const Vec& axis, float angle) {
    const float s = std::sin(angle * 0.5f);
    return {axis.x * s, axis.y * s, axis.z * s, std::cos(angle * 0.5f)};
}

inline float Wrap(float angle) {
    return angle - 2.0f * Pi * std::round(angle / (2.0f * Pi));
}

/// Heading (positive to the left, 0 straight ahead), how far the front points up, and how far
/// the right side is up, in radians: the same as HeldPad::Locate's.
struct Angles {
    float yaw{}, pitch{}, roll{};
};

inline Angles AnglesOf(const Quat& attitude) {
    const Vec front = HeldPad::Rotate(attitude, {0.0f, 0.0f, -1.0f});
    const Vec right = HeldPad::Rotate(attitude, {1.0f, 0.0f, 0.0f});
    return {std::atan2(-front.x, -front.z), std::asin(std::clamp(front.y, -1.0f, 1.0f)),
            std::asin(std::clamp(right.y, -1.0f, 1.0f))};
}

class Imu {
public:
    static constexpr float Gravity = 9.80665f;
    /// How fast the estimate leans towards what the accelerometer calls up, per second.
    static constexpr float TiltGain = 2.0f;
    /// How fast it leans towards the heading the hands give, per second, and how long one
    /// counts for.
    static constexpr float YawGain = 1.5f;
    static constexpr float YawFresh = 0.5f;

    void Accel(const Vec& acceleration) {
        acceleration_ = acceleration;
        acceleration_valid_ = true;
    }

    /// The heading the hands give, at `now` (seconds, the same clock as Gyro's).
    void HandYaw(float yaw, double now) {
        yaw_reference_ = yaw;
        yaw_reference_time_ = now;
        yaw_reference_valid_ = true;
    }

    /// One gyroscope reading, `elapsed` seconds after the one before.
    void Gyro(const Vec& rate, float elapsed, double now) {
        static constexpr Vec Up{0.0f, 1.0f, 0.0f};
        elapsed = std::clamp(elapsed, 0.0f, 0.05f);

        const float gravity = acceleration_valid_ ? HeldPad::Length(acceleration_) : 0.0f;
        // The accelerometer tells which way is down only while the gamepad is held fairly still.
        steady_ = gravity > 0.8f * Gravity && gravity < 1.2f * Gravity;
        const Vec measured_up = steady_ ? HeldPad::Scale(acceleration_, 1.0f / gravity) : Up;
        const bool yaw_fresh = yaw_reference_valid_ && now - yaw_reference_time_ < YawFresh;

        if (!valid_) {
            if (!steady_) {
                return;
            }
            // Level with the horizon, as gravity says, and turned the way the hands say if
            // they do, straight ahead if not.
            const Vec axis = HeldPad::Cross(measured_up, Up);
            const float sine = HeldPad::Length(axis);
            attitude_ = sine > 1e-6f ? AxisAngle(HeldPad::Scale(axis, 1.0f / sine),
                                                 std::atan2(sine, measured_up.y))
                                     : Quat{};
            if (yaw_fresh) {
                attitude_ = Normalize(Multiply(
                    AxisAngle(Up, Wrap(yaw_reference_ - AnglesOf(attitude_).yaw)), attitude_));
            }
            valid_ = true;
            return;
        }

        const float speed = HeldPad::Length(rate);
        if (speed > 1e-6f) {
            attitude_ = Multiply(attitude_,
                                 AxisAngle(HeldPad::Scale(rate, 1.0f / speed), speed * elapsed));
        }
        if (steady_) {
            const Vec correction = HeldPad::Cross(HeldPad::Rotate(attitude_, measured_up), Up);
            const float error = HeldPad::Length(correction);
            if (error > 1e-6f) {
                attitude_ = Multiply(AxisAngle(HeldPad::Scale(correction, 1.0f / error),
                                               std::asin(std::min(error, 1.0f)) * TiltGain *
                                                   elapsed),
                                     attitude_);
            }
        }
        // Not while it points nearly straight up or down, where heading means little.
        if (yaw_fresh) {
            const Vec front = HeldPad::Rotate(attitude_, {0.0f, 0.0f, -1.0f});
            if (std::abs(front.y) < 0.9f) {
                const float error = Wrap(yaw_reference_ - std::atan2(-front.x, -front.z));
                attitude_ =
                    Multiply(AxisAngle(Up, error * std::min(YawGain * elapsed, 1.0f)), attitude_);
            }
        }
        attitude_ = Normalize(attitude_);
    }

    /// Start afresh from gravity (and the hands) with the next reading.
    void Reset() {
        valid_ = false;
    }

    bool Valid() const {
        return valid_;
    }
    /// The accelerometer's last reading was about one g: it was held still enough to tell
    /// which way is up.
    bool Steady() const {
        return steady_;
    }
    /// The hands have given a heading recently enough for it to count.
    bool HasYawReference(double now) const {
        return yaw_reference_valid_ && now - yaw_reference_time_ < YawFresh;
    }
    const Quat& Attitude() const {
        return attitude_;
    }

private:
    Quat attitude_;
    bool valid_{};
    bool steady_{};
    Vec acceleration_;
    bool acceleration_valid_{};
    float yaw_reference_{};
    double yaw_reference_time_{};
    bool yaw_reference_valid_{};
};

/// How far what the hands say is from what the sensors say, averaged over the last couple of
/// seconds: sensors minus hands. The pitch is the hands' own, without tilt, so the average
/// pitch difference is the tilt that would make them agree.
class Agreement {
public:
    static constexpr float AverageTime = 2.0f;

    void Add(const Angles& hands, const Angles& sensors, float elapsed) {
        const Angles difference{Wrap(sensors.yaw - hands.yaw), sensors.pitch - hands.pitch,
                                sensors.roll - hands.roll};
        last_ = difference;
        const float blend = any_ ? 1.0f - std::exp(-std::max(elapsed, 0.0f) / AverageTime) : 1.0f;
        mean_.yaw += (difference.yaw - mean_.yaw) * blend;
        mean_.pitch += (difference.pitch - mean_.pitch) * blend;
        mean_.roll += (difference.roll - mean_.roll) * blend;
        any_ = true;
    }

    void Reset() {
        *this = {};
    }

    bool Any() const {
        return any_;
    }
    const Angles& Last() const {
        return last_;
    }
    const Angles& Mean() const {
        return mean_;
    }

private:
    Angles last_;
    Angles mean_;
    bool any_{};
};

} // namespace HeldPadFusion
