// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

// What the held-pad viewer makes of the hands, frame by frame: palms and the way they point,
// HeldPad::Locate on them, and the same smoothing the Quest app and the emulator use. No
// Windows or OpenXR in here, so that it can be checked on its own
// (tools/tests/held_pad_follow_test.cpp).

#include <cmath>
#include <optional>

#include "held_pad.h"

namespace HeldPadFollow {

using HeldPad::Quat;
using HeldPad::Vec;

/// How long the shown attitude takes to follow the hands (xr_host.cpp's HeldFollowTime).
inline constexpr float FollowTime = 0.06f;
/// After a gap this long the attitude (and the position, with smoothing) starts afresh.
inline constexpr float FollowGap = 0.25f;
/// The velocity estimate is blended this much towards each new measurement, and starts afresh
/// after a gap this long (openxr_host.cpp's UpdatePad).
inline constexpr float VelocityBlend = 0.4f;
inline constexpr float VelocityGap = 0.1f;

struct Palm {
    Vec position;
    bool pointing_known{};
    Vec pointing;
};

/// One hand's palm joint, as the headset reports it: nothing if the hand or the palm's
/// position is not tracked. The way the hand points is the palm's -Z, from the wrist to the
/// fingers (as quest-host/cpp/xr_host.cpp's LocatePalm).
inline std::optional<Palm> PalmFrom(bool active, bool position_valid, bool orientation_valid,
                                    const Vec& position, const Quat& orientation) {
    if (!active || !position_valid) {
        return std::nullopt;
    }
    Palm palm{position, false, {}};
    if (orientation_valid) {
        palm.pointing_known = true;
        palm.pointing = HeldPad::Rotate(orientation, {0.0f, 0.0f, -1.0f});
    }
    return palm;
}

inline std::optional<HeldPad::Hands> HandsFrom(const std::optional<Palm>& left,
                                               const std::optional<Palm>& right) {
    if (!left || !right) {
        return std::nullopt;
    }
    HeldPad::Hands hands;
    hands.left_palm = left->position;
    hands.right_palm = right->position;
    hands.pointing_known = left->pointing_known && right->pointing_known;
    hands.left_pointing = left->pointing;
    hands.right_pointing = right->pointing;
    return hands;
}

/// What is shown of the gamepad.
struct Shown {
    /// There has been a gamepad in the hands at some point: there is something to draw.
    bool ever{};
    /// The hands hold it now; otherwise what is shown is the last pose, held.
    bool live{};
    /// This frame's pose told the heading and attitude, and whether the pitch was measured.
    bool turned_known{};
    bool pitch_measured{};
    Vec position;
    /// Halfway between the palms, smoothed as the position is.
    Vec middle;
    Vec velocity;
    Quat orientation;
    /// The last heading and pitch (without tilt) that were told, in radians.
    float yaw{};
    float pitch{};
    /// Distance between the palms this frame, or -1 when either is missing.
    float palm_distance{-1.0f};
};

class Follower {
public:
    /// Added to how far the hands point up (radians), as HeldPad::Locate's `tilt`.
    float tilt{0.0f};
    bool smoothing{true};

    /// Forget everything followed so far (tilt and smoothing stay).
    void Reset() {
        shown_ = {};
        raw_position_ = {};
        position_time_ = -1.0;
        orientation_time_ = -1.0;
        held_pitch_ = 0.0f;
    }

    /// The pitch taken while the headset cannot tell where the hands point: the last one
    /// measured.
    float AssumedPitch() const {
        return held_pitch_;
    }

    /// `time` in seconds, from any fixed start.
    const Shown& Update(const std::optional<HeldPad::Hands>& hands, double time) {
        shown_.live = false;
        shown_.turned_known = false;
        shown_.pitch_measured = false;
        shown_.palm_distance =
            hands ? HeldPad::Length(HeldPad::Sub(hands->right_palm, hands->left_palm)) : -1.0f;
        if (!hands) {
            return shown_;
        }
        const auto held = HeldPad::Locate(*hands, tilt, held_pitch_);
        if (!held) {
            return shown_;
        }

        const float position_gap = Gap(position_time_, time);
        if (position_gap < VelocityGap) {
            const Vec step = HeldPad::Scale(HeldPad::Sub(held->position, raw_position_),
                                            1.0f / position_gap);
            shown_.velocity = Add(shown_.velocity,
                                  HeldPad::Scale(HeldPad::Sub(step, shown_.velocity),
                                                 VelocityBlend));
        } else {
            shown_.velocity = {};
        }
        raw_position_ = held->position;
        shown_.position = smoothing && position_gap < FollowGap
                              ? Add(shown_.position,
                                    HeldPad::Scale(HeldPad::Sub(held->position, shown_.position),
                                                   Blend(position_gap)))
                              : held->position;
        const Vec middle = HeldPad::Scale(Add(hands->left_palm, hands->right_palm), 0.5f);
        shown_.middle = smoothing && position_gap < FollowGap
                            ? Add(shown_.middle,
                                  HeldPad::Scale(HeldPad::Sub(middle, shown_.middle),
                                                 Blend(position_gap)))
                            : middle;
        position_time_ = time;

        if (held->turned_known) {
            if (held->pitch_measured) {
                held_pitch_ = held->pitch;
            }
            const float orientation_gap = Gap(orientation_time_, time);
            shown_.orientation =
                smoothing && orientation_gap < FollowGap
                    ? HeldPad::Towards(shown_.orientation, held->orientation,
                                       Blend(orientation_gap))
                    : held->orientation;
            orientation_time_ = time;
            shown_.yaw = held->yaw;
            shown_.pitch = held->pitch;
        }
        shown_.ever = true;
        shown_.live = true;
        shown_.turned_known = held->turned_known;
        shown_.pitch_measured = held->pitch_measured;
        return shown_;
    }

    const Shown& Current() const {
        return shown_;
    }

private:
    static Vec Add(const Vec& a, const Vec& b) {
        return {a.x + b.x, a.y + b.y, a.z + b.z};
    }

    /// Seconds since `since`, or something larger than any gap when there was nothing then.
    static float Gap(double since, double now) {
        return since >= 0.0 && now > since ? static_cast<float>(now - since) : 1e9f;
    }

    static float Blend(float elapsed) {
        return 1.0f - std::exp(-elapsed / FollowTime);
    }

    Shown shown_;
    Vec raw_position_;
    double position_time_{-1.0};
    double orientation_time_{-1.0};
    float held_pitch_{0.0f};
};

} // namespace HeldPadFollow
