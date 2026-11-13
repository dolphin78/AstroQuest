// Checks what the held-pad viewer (tools/held-pad-viewer) makes of the hands frame by frame:
// palms from joints, the shown pose following the hands, holding when they are lost, and
// starting afresh after a gap.
//
//   clang-cl /std:c++latest /EHsc -fuse-ld=lld /I shadps4-arm64-main/src/core/vr
//       /I tools/held-pad-viewer
//       tools/tests/held_pad_follow_test.cpp /Fe:build/tests/held_pad_follow_test.exe
#include <cmath>
#include <cstdio>
#include <string>

#include "held_pad_follow.h"

using namespace HeldPad;
using namespace HeldPadFollow;

static int failures = 0;

static void Check(bool ok, const std::string& what) {
    std::printf("%s  %s\n", ok ? "ok  " : "FAIL", what.c_str());
    failures += ok ? 0 : 1;
}

static bool Near(float a, float b, float within = 0.002f) {
    return std::abs(a - b) <= within;
}

static bool Near(const Vec& a, const Vec& b, float within = 0.002f) {
    return Near(a.x, b.x, within) && Near(a.y, b.y, within) && Near(a.z, b.z, within);
}

static constexpr float Pi = 3.14159265f;

/// Level hands 20 cm apart around `middle`, pointing straight ahead with the palms facing in
/// (a palm joint's -Z runs towards the fingers), or pointing `pitch` above level.
static Hands Level(const Vec& middle, float pitch = 0.0f) {
    const Quat up{std::sin(pitch * 0.5f), 0.0f, 0.0f, std::cos(pitch * 0.5f)};
    const auto left = PalmFrom(true, true, true, {middle.x - 0.1f, middle.y, middle.z}, up);
    const auto right = PalmFrom(true, true, true, {middle.x + 0.1f, middle.y, middle.z}, up);
    return *HandsFrom(left, right);
}

static float PitchOf(const Quat& q) {
    const Vec front = Rotate(q, {0.0f, 0.0f, -1.0f});
    return std::asin(front.y);
}

int main() {
    {
        const auto palm = PalmFrom(true, true, true, {1, 2, 3}, {});
        Check(palm && palm->pointing_known && Near(palm->pointing, {0, 0, -1}),
              "a palm with no rotation points straight ahead");
        Check(!PalmFrom(false, true, true, {}, {}), "an inactive hand has no palm");
        Check(!PalmFrom(true, false, true, {}, {}), "a palm without a position is no palm");
        const auto blind = PalmFrom(true, true, false, {}, {});
        Check(blind && !blind->pointing_known, "a palm without orientation has no pointing");
        Check(!HandsFrom(palm, std::nullopt), "one palm is not two hands");
        Check(!HandsFrom(palm, blind)->pointing_known,
              "pointing is known only when both hands say");
    }
    {
        Follower follower;
        const auto& first = follower.Update(Level({0, 0, -0.4f}), 0.0);
        Check(first.live && first.ever && first.turned_known && first.pitch_measured,
              "level hands hold a gamepad, turned and pitched");
        Check(Near(first.position, {0, 0.015f, -0.435f}),
              "the first pose is where the hands put it, with no smoothing yet");
        Check(Near(first.velocity, {}), "and nothing is known yet of how it moves");

        // Moving 1 cm a frame to the right at 90 Hz: the velocity heads for 0.9 m/s, the
        // position trails the hands a little.
        double time = 0.0;
        for (int frame = 1; frame <= 30; ++frame) {
            time = frame / 90.0;
            follower.Update(Level({frame * 0.01f, 0, -0.4f}), time);
        }
        const auto& moving = follower.Current();
        Check(Near(moving.velocity.x, 0.9f, 0.01f), "the velocity follows the movement");
        Check(moving.position.x < 0.30f && moving.position.x > 0.24f,
              "the smoothed position trails the hands");

        follower.smoothing = false;
        follower.Update(Level({0.31f, 0, -0.4f}), time + 1 / 90.0);
        Check(Near(follower.Current().position.x, 0.31f), "without smoothing it is exact");
        follower.smoothing = true;

        // Hands gone: the last pose holds.
        const Vec before = follower.Current().position;
        const auto& lost = follower.Update(std::nullopt, time + 2 / 90.0);
        Check(!lost.live && lost.ever && Near(lost.position, before) && lost.palm_distance < 0,
              "when the hands are lost the last pose holds");
        // Hands too far apart: no gamepad either.
        Hands apart = Level({0, 0, -0.4f});
        apart.left_palm.x -= 0.3f;
        const auto& wide = follower.Update(apart, time + 3 / 90.0);
        Check(!wide.live && Near(wide.palm_distance, 0.5f), "hands half a metre apart hold none");

        // Back after a second: no trailing from where it was, no velocity from the jump.
        const auto& back = follower.Update(Level({-0.5f, 0, -0.4f}), time + 1.0);
        Check(back.live && Near(back.position.x, -0.5f) && Near(back.velocity, {}),
              "after a gap it starts afresh");
    }
    {
        Follower follower;
        const float up = 20.0f * Pi / 180.0f;
        follower.Update(Level({0, 0, -0.4f}, up), 0.0);
        Check(Near(PitchOf(follower.Current().orientation), up), "pitch follows the hands");
        Check(Near(follower.AssumedPitch(), up), "and is kept for when it cannot be told");

        // Pointing unknown: the last measured pitch is assumed.
        Hands blind = Level({0, 0, -0.4f});
        blind.pointing_known = false;
        const auto& assumed = follower.Update(blind, 1.0);
        Check(assumed.live && !assumed.pitch_measured &&
                  Near(PitchOf(assumed.orientation), up),
              "without pointing the last pitch is assumed");

        // Tipped down, smoothed: it follows part of the way in one frame, nearly all of it
        // in a few tenths of a second.
        follower.Update(Level({0, 0, -0.4f}, 0.0f), 1.0 + 1 / 90.0);
        const float partway = PitchOf(follower.Current().orientation);
        Check(partway < up && partway > 0.0f, "smoothing follows part of the way at once");
        for (int frame = 2; frame <= 30; ++frame) {
            follower.Update(Level({0, 0, -0.4f}, 0.0f), 1.0 + frame / 90.0);
        }
        Check(Near(PitchOf(follower.Current().orientation), 0.0f, 0.01f),
              "and all of it soon after");

        follower.tilt = 10.0f * Pi / 180.0f;
        follower.smoothing = false;
        follower.Update(Level({0, 0, -0.4f}, 0.0f), 2.0);
        Check(Near(PitchOf(follower.Current().orientation), follower.tilt),
              "tilt is added to where the hands point");
        Check(Near(follower.Current().pitch, 0.0f), "but not to the pitch measured");

        follower.Reset();
        Check(!follower.Current().ever && follower.AssumedPitch() == 0.0f &&
                  follower.tilt > 0.0f && !follower.smoothing,
              "reset forgets the pose, keeps the settings");
    }
    std::printf("%s\n", failures == 0 ? "all passed" : "FAILED");
    return failures == 0 ? 0 : 1;
}
