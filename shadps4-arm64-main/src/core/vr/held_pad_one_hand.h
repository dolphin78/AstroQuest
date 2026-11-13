// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

// A gamepad held in one hand. Two palms a gamepad's width apart say where it is; one palm
// alone could be anywhere near it, or holding nothing at all. What gives a holding hand away
// is its fingers, wrapped around the grip; where the gamepad is from there, and how it is
// turned, is what that hand's grip was seen to be while both hands held it (or, for a gamepad
// with motion sensors, how they have it turned and a standard grip). Shared by the emulator
// (core/vr/openxr_host.cpp) and the held-pad viewer (tools/held-pad-viewer); checked by
// tools/tests/held_pad_one_hand_test.cpp.
//
// Everything is in the headset's space: metres, +X right, +Y up, -Z forward. Joints are
// numbered as XR_EXT_hand_tracking numbers them.

#include <algorithm>
#include <cmath>
#include <optional>

#include "held_pad.h"

namespace HeldPadOneHand {

using HeldPad::Quat;
using HeldPad::Vec;

inline constexpr float Pi = 3.14159265f;

inline constexpr int JointCount = 26;

struct Joint {
    bool valid{};
    Vec position;
};

/// How far a finger is bent, 0 (straight) to 1 (bent right back on itself): the angle between
/// its bone in the palm and its last bone. `first` is its metacarpal joint, the four after it
/// the proximal, intermediate, distal and tip; -1 when not all of them are known.
inline float FingerCurl(const Joint (&joints)[JointCount], int first) {
    for (int i = first; i < first + 5; ++i) {
        if (!joints[i].valid) {
            return -1.0f;
        }
    }
    const Vec palm_bone = HeldPad::Sub(joints[first + 1].position, joints[first].position);
    const Vec last_bone = HeldPad::Sub(joints[first + 4].position, joints[first + 3].position);
    const float lengths = HeldPad::Length(palm_bone) * HeldPad::Length(last_bone);
    if (!(lengths > 1e-10f)) {
        return -1.0f;
    }
    const float cosine = std::clamp(HeldPad::Dot(palm_bone, last_bone) / lengths, -1.0f, 1.0f);
    return std::acos(cosine) / Pi;
}

/// How far the middle, ring and little fingers are curled, on average (the index finger is on
/// a trigger, the thumb on a stick: they say nothing of holding). -1 when none is known.
inline float GripCurl(const Joint (&joints)[JointCount]) {
    float sum = 0.0f;
    int count = 0;
    for (const int first : {11, 16, 21}) {
        const float curl = FingerCurl(joints, first);
        if (curl >= 0.0f) {
            sum += curl;
            ++count;
        }
    }
    return count > 0 ? sum / count : -1.0f;
}

/// Whether a hand is closed around something, with a margin either way so that it does not
/// flicker on the edge.
class GripDetector {
public:
    float on{0.5f};
    float off{0.35f};

    bool Update(float curl) {
        if (curl < 0.0f) {
            gripping_ = false;
        } else if (gripping_) {
            gripping_ = curl > off;
        } else {
            gripping_ = curl > on;
        }
        return gripping_;
    }

    bool Gripping() const {
        return gripping_;
    }

private:
    bool gripping_{};
};

struct Palm {
    Vec position;
    bool orientation_valid{};
    Quat orientation;
};

using HeldPad::AngleBetween;
using HeldPad::Conjugate;

/// Where one hand holds the gamepad: its palm in the gamepad's own frame, and the gamepad's
/// attitude from the palm's.
class Grip {
public:
    /// How long the grip is averaged over while it is learned.
    static constexpr float LearnTime = 0.5f;
    /// Seen for this long, it is known.
    static constexpr float KnownAfter = 0.5f;

    /// Where a palm closes around a DualSense's grip: half a hand's width to the side of the
    /// middle, behind and below it (HeldPad::Locate's 3.5 cm and 1.5 cm). `hand` 0 is the left.
    static Vec StandardOffset(int hand) {
        return {hand == 0 ? -0.09f : 0.09f, -0.015f, 0.035f};
    }

    explicit Grip(int hand = 0) : offset_{StandardOffset(hand)} {}

    /// While the gamepad's middle and attitude are known otherwise (both hands hold it): what
    /// this hand's grip is, `elapsed` seconds after the last time.
    void Learn(const Palm& palm, const Vec& centre, const Quat& pad, float elapsed) {
        const Vec offset = HeldPad::Rotate(Conjugate(pad), HeldPad::Sub(palm.position, centre));
        const float blend =
            seen_ > 0.0f ? 1.0f - std::exp(-std::max(elapsed, 0.0f) / LearnTime) : 1.0f;
        offset_ = {offset_.x + (offset.x - offset_.x) * blend,
                   offset_.y + (offset.y - offset_.y) * blend,
                   offset_.z + (offset.z - offset_.z) * blend};
        if (palm.orientation_valid) {
            const Quat rotation =
                HeldPad::Normalize(HeldPad::Multiply(Conjugate(palm.orientation), pad));
            rotation_ = rotation_seen_ ? HeldPad::Towards(rotation_, rotation, blend) : rotation;
            rotation_seen_ = true;
        }
        seen_ += std::max(elapsed, 0.0f);
    }

    /// The offset has been seen, not just assumed.
    bool Known() const {
        return seen_ >= KnownAfter;
    }
    /// The gamepad's attitude can be told from the palm's.
    bool TurnKnown() const {
        return Known() && rotation_seen_;
    }

    /// The gamepad's middle, from the palm and the gamepad's attitude.
    Vec Centre(const Palm& palm, const Quat& pad) const {
        return HeldPad::Sub(palm.position, HeldPad::Rotate(pad, offset_));
    }

    /// The gamepad's attitude, from the palm's.
    Quat PadFrom(const Palm& palm) const {
        return HeldPad::Normalize(HeldPad::Multiply(palm.orientation, rotation_));
    }

    const Vec& Offset() const {
        return offset_;
    }

    void Forget(int hand) {
        *this = Grip{hand};
    }

private:
    Vec offset_;
    Quat rotation_;
    bool rotation_seen_{};
    float seen_{};
};

struct Held {
    /// 0 the left hand, 1 the right.
    int hand{};
    Vec position;
    Quat orientation;
    /// The attitude is the sensors', not the hand's.
    bool turned_by_sensors{};
    /// What the hand says of the gamepad's attitude, where it can: for the heading.
    bool hand_turn_known{};
    Quat hand_orientation;
};

/// A palm and the hand's grip disagreeing by more than this with the motion sensors is not
/// holding this gamepad.
inline constexpr float MostDisagreement = 35.0f * Pi / 180.0f;

/// The gamepad held in one hand, if one is: a hand that grips, whose grip is known or, with
/// the sensors saying how the gamepad is turned, assumed; and, where both the hand and the
/// sensors can tell how it is turned, agree on that. With both hands like that, the one that
/// held it last, or the one that agrees best.
inline std::optional<Held> Locate(const std::optional<Palm> (&palms)[2], const bool (&gripping)[2],
                                  const Grip (&grips)[2], const std::optional<Quat>& sensors,
                                  int last_hand) {
    std::optional<Held> best;
    float best_disagreement = 10.0f;
    for (int hand = 0; hand < 2; ++hand) {
        if (!palms[hand] || !gripping[hand]) {
            continue;
        }
        const Palm& palm = *palms[hand];
        const Grip& grip = grips[hand];
        const bool hand_turn = palm.orientation_valid && grip.TurnKnown();
        if (!sensors && !hand_turn) {
            continue;
        }
        Held held;
        held.hand = hand;
        held.hand_turn_known = hand_turn;
        if (hand_turn) {
            held.hand_orientation = grip.PadFrom(palm);
        }
        float disagreement = 0.0f;
        if (sensors) {
            held.orientation = *sensors;
            held.turned_by_sensors = true;
            if (hand_turn) {
                disagreement = AngleBetween(held.hand_orientation, *sensors);
                if (disagreement > MostDisagreement) {
                    continue;
                }
            }
        } else {
            held.orientation = held.hand_orientation;
        }
        held.position = grip.Centre(palm, held.orientation);
        if (hand == last_hand) {
            disagreement -= 1.0f;
        }
        if (!best || disagreement < best_disagreement) {
            best = held;
            best_disagreement = disagreement;
        }
    }
    return best;
}

/// Hands tremble: the one-handed pose follows them a little behind, as the two-handed one
/// does (HeldPadFollow), starting afresh after a gap.
class Smoother {
public:
    void Update(const Vec& position, const Quat& orientation, double time, bool smoothing) {
        const float gap = time_ >= 0.0 && time > time_ ? static_cast<float>(time - time_) : 1e9f;
        if (smoothing && gap < 0.25f) {
            const float blend = 1.0f - std::exp(-gap / 0.06f);
            position_ = {position_.x + (position.x - position_.x) * blend,
                         position_.y + (position.y - position_.y) * blend,
                         position_.z + (position.z - position_.z) * blend};
            orientation_ = HeldPad::Towards(orientation_, orientation, blend);
        } else {
            position_ = position;
            orientation_ = orientation;
        }
        time_ = time;
    }

    void Reset() {
        time_ = -1.0;
    }

    const Vec& Position() const {
        return position_;
    }
    const Quat& Orientation() const {
        return orientation_;
    }

private:
    Vec position_;
    Quat orientation_;
    double time_{-1.0};
};

} // namespace HeldPadOneHand
