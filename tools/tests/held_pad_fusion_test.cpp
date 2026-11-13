// Checks the held-pad viewer's fusion of a gamepad's motion sensors with the hands around it
// (tools/held-pad-viewer/held_pad_fusion.h): starting level from gravity and turned as the
// hands say, following the gyroscope, gravity taking out drift in pitch and roll, the hands
// taking it out of the heading, and the averaged difference between hands and sensors.
//
//   clang-cl /std:c++latest /EHsc -fuse-ld=lld /I shadps4-arm64-main/src/core/vr
//       /I tools/held-pad-viewer tools/tests/held_pad_fusion_test.cpp
//       /Fe:build/tests/held_pad_fusion_test.exe
#include <cmath>
#include <cstdio>
#include <string>

#include "held_pad_fusion.h"

using namespace HeldPad;
using namespace HeldPadFusion;

static int failures = 0;

static void Check(bool ok, const std::string& what) {
    std::printf("%s  %s\n", ok ? "ok  " : "FAIL", what.c_str());
    failures += ok ? 0 : 1;
}

static bool Near(float a, float b, float within) {
    return std::abs(a - b) <= within;
}

static float Radians(float degrees) {
    return degrees * Pi / 180.0f;
}

/// What the accelerometer of a gamepad held still with this attitude reads: up, in its frame.
static Vec GravityIn(const Quat& attitude) {
    const Quat back{-attitude.x, -attitude.y, -attitude.z, attitude.w};
    return Scale(Rotate(back, {0.0f, 1.0f, 0.0f}), Imu::Gravity);
}

/// The attitude of a gamepad turned `yaw` to the left with its front `pitch` up.
static Quat Held(float yaw, float pitch) {
    return Normalize(Multiply(AxisAngle({0, 1, 0}, yaw), AxisAngle({1, 0, 0}, pitch)));
}

int main() {
    constexpr float Step = 1.0f / 250.0f;
    {
        Imu imu;
        imu.Gyro({}, Step, 0.0);
        Check(!imu.Valid(), "nothing without the accelerometer");
        imu.Accel({0, 3.0f * Imu::Gravity, 0});
        imu.Gyro({}, Step, 0.0);
        Check(!imu.Valid(), "nothing while it is shaken");

        const Quat truth = Held(Radians(30), Radians(20));
        imu.Accel(GravityIn(truth));
        imu.HandYaw(Radians(30), 0.0);
        imu.Gyro({}, Step, 0.0);
        const Angles start = AnglesOf(imu.Attitude());
        Check(imu.Valid() && Near(start.pitch, Radians(20), 0.002f) &&
                  Near(start.roll, 0.0f, 0.002f),
              "it starts as gravity says");
        Check(Near(start.yaw, Radians(30), 0.002f), "turned the way the hands say");
    }
    {
        Imu imu;
        imu.Accel(GravityIn(Quat{}));
        imu.Gyro({}, Step, 0.0);
        Check(Near(AnglesOf(imu.Attitude()).yaw, 0.0f, 1e-4f),
              "without the hands it starts straight ahead");

        // Turning left at 90 degrees a second for a second, no hands to say so: the gyroscope
        // alone. (The accelerometer still reads straight up throughout: a turn about up.)
        double time = 0.0;
        for (int i = 1; i <= 250; ++i) {
            time = i * Step;
            imu.Gyro({0, Radians(90), 0}, Step, time);
        }
        Check(Near(AnglesOf(imu.Attitude()).yaw, Radians(90), 0.01f),
              "the gyroscope says how far it turned");

        // A gyroscope that reads a little pitch up when held still: gravity keeps it level.
        for (int i = 1; i <= 2500; ++i) {
            imu.Gyro({Radians(1), 0, 0}, Step, time + i * Step);
        }
        Check(std::abs(AnglesOf(imu.Attitude()).pitch) < Radians(1.5f),
              "gravity takes the drift out of pitch");
        time += 2500 * Step;

        // The hands say it points 30 degrees left of where the gyroscope has it: it leans that
        // way, a little at once and all of it within a few seconds.
        const float hands = Radians(120);
        imu.HandYaw(hands, time);
        imu.Gyro({}, Step, time + Step);
        const float after_one = AnglesOf(imu.Attitude()).yaw;
        Check(after_one > Radians(90) && after_one < Radians(91), "the hands pull a little at once");
        for (int i = 2; i <= 1000; ++i) {
            const double now = time + i * Step;
            if (i % 3 == 0) {
                imu.HandYaw(hands, now);
            }
            imu.Gyro({}, Step, now);
        }
        Check(Near(AnglesOf(imu.Attitude()).yaw, hands, Radians(1)),
              "and the heading follows them");

        // A heading from long ago counts for nothing.
        imu.HandYaw(0.0f, time);
        const float before = AnglesOf(imu.Attitude()).yaw;
        imu.Gyro({}, Step, time + 10.0);
        Check(Near(AnglesOf(imu.Attitude()).yaw, before, 1e-4f), "a stale heading is ignored");

        imu.Reset();
        Check(!imu.Valid(), "reset starts afresh");
    }
    {
        Agreement agreement;
        agreement.Add({0, Radians(10), 0}, {Radians(5), Radians(25), Radians(-2)}, 0.0f);
        Check(Near(agreement.Mean().pitch, Radians(15), 1e-4f) &&
                  Near(agreement.Mean().yaw, Radians(5), 1e-4f) &&
                  Near(agreement.Mean().roll, Radians(-2), 1e-4f),
              "the first difference is the average");
        for (int i = 0; i < 900; ++i) {
            agreement.Add({0, 0, 0}, {0, Radians(5), 0}, 1.0f / 90.0f);
        }
        Check(Near(agreement.Mean().pitch, Radians(5), Radians(0.1f)),
              "the average settles on the new difference");
        agreement.Add({Pi - 0.01f, 0, 0}, {-Pi + 0.01f, 0, 0}, 0.0f);
        Check(Near(agreement.Last().yaw, 0.02f, 1e-4f), "headings either side of behind are close");
    }
    {
        const Vec level = CentreFrom({0, 0, 0}, Quat{});
        Check(Near(level.y, 0.015f, 1e-5f) && Near(level.z, -0.035f, 1e-5f),
              "level, the middle is ahead of and above the palms");
        const Vec up = CentreFrom({0, 0, 0}, Held(0, Radians(90)));
        Check(Near(up.y, 0.035f, 1e-4f) && Near(up.z, 0.015f, 1e-4f),
              "pointing up, ahead is up and above is back");
    }
    std::printf("%s\n", failures == 0 ? "all passed" : "FAILED");
    return failures == 0 ? 0 : 1;
}
