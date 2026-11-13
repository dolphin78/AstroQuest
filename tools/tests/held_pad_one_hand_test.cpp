// Checks the held-pad viewer's gamepad in one hand (shadps4-arm64-main/src/core/vr/held_pad_one_hand.h):
// curled fingers against straight ones, a grip learned while both hands hold the gamepad, and
// which hand, if any, holds it when only one does.
//
//   clang-cl /std:c++latest /EHsc -fuse-ld=lld /I shadps4-arm64-main/src/core/vr
//       tools/tests/held_pad_one_hand_test.cpp /Fe:build/tests/held_pad_one_hand_test.exe
#include <cmath>
#include <cstdio>
#include <string>

#include "held_pad_one_hand.h"

using namespace HeldPad;
using namespace HeldPadOneHand;

static int failures = 0;

static void Check(bool ok, const std::string& what) {
    std::printf("%s  %s\n", ok ? "ok  " : "FAIL", what.c_str());
    failures += ok ? 0 : 1;
}

static bool Near(float a, float b, float within) {
    return std::abs(a - b) <= within;
}

static bool Near(const Vec& a, const Vec& b, float within) {
    return Near(a.x, b.x, within) && Near(a.y, b.y, within) && Near(a.z, b.z, within);
}

/// About an axis of length one.
static Quat AxisAngle(const Vec& axis, float angle) {
    const float s = std::sin(angle * 0.5f);
    return {axis.x * s, axis.y * s, axis.z * s, std::cos(angle * 0.5f)};
}

static float Radians(float degrees) {
    return degrees * Pi / 180.0f;
}

/// A hand whose fingers run ahead (-Z) from the palm, each joint of the middle, ring and
/// little fingers bent down by `bend` (so the last bone is bent by three times that).
static void Hand(Joint (&joints)[JointCount], float bend) {
    for (auto& joint : joints) {
        joint = {true, {}};
    }
    for (const int first : {6, 11, 16, 21}) {
        Vec at{static_cast<float>(first) * 0.005f, 0.0f, 0.0f};
        joints[first].position = at;
        at.z -= 0.07f; // the bone in the palm, straight ahead
        joints[first + 1].position = at;
        float angle = 0.0f;
        for (int bone = 2; bone <= 4; ++bone) {
            angle += bend;
            at = {at.x, at.y - std::sin(angle) * 0.03f, at.z - std::cos(angle) * 0.03f};
            joints[first + bone].position = at;
        }
    }
}

int main() {
    {
        Joint joints[JointCount];
        Hand(joints, 0.0f);
        Check(Near(GripCurl(joints), 0.0f, 0.01f), "straight fingers are not curled");
        Hand(joints, Radians(60));
        Check(Near(GripCurl(joints), 1.0f, 0.01f), "fingers wrapped round are fully curled");
        Hand(joints, Radians(30));
        Check(Near(GripCurl(joints), 0.5f, 0.01f), "half way is half");
        joints[13].valid = false;
        joints[18].valid = false;
        joints[23].valid = false;
        Check(GripCurl(joints) < 0.0f, "fingers not seen say nothing");

        GripDetector detector;
        Check(!detector.Update(0.45f), "a little curl is no grip");
        Check(detector.Update(0.55f), "curled past the mark is a grip");
        Check(detector.Update(0.4f), "and stays one a little below it");
        Check(!detector.Update(0.3f), "until the hand opens");
        detector.Update(0.6f);
        Check(!detector.Update(-1.0f), "a hand not seen grips nothing");
    }

    // A gamepad turned 30 degrees left and tipped up 20, held 18 cm wide.
    const Quat pad = Normalize(
        Multiply(AxisAngle({0, 1, 0}, Radians(30)), AxisAngle({1, 0, 0}, Radians(20))));
    const Vec centre{0.1f, -0.3f, -0.4f};
    const auto palm_at = [&](const Vec& in_pad, const Quat& turn) {
        Palm palm;
        palm.position = Add(centre, Rotate(pad, in_pad));
        palm.orientation_valid = true;
        palm.orientation = Normalize(Multiply(pad, turn));
        return palm;
    };
    // (A left palm faces right, into the grip: turned a quarter about the fingers' line.)
    const Quat left_turn = AxisAngle({0, 0, 1}, Radians(-90));
    const Quat right_turn = AxisAngle({0, 0, 1}, Radians(90));
    const Vec left_in_pad{-0.085f, -0.02f, 0.04f};
    const Vec right_in_pad{0.085f, -0.02f, 0.04f};

    Grip grips[2]{Grip{0}, Grip{1}};
    {
        Check(!grips[0].Known() && !grips[0].TurnKnown(), "a grip not seen is not known");
        const Palm left = palm_at(left_in_pad, left_turn);
        Check(Near(grips[0].Centre(left, pad), centre, 0.02f),
              "the standard grip puts the middle near where it is");
        for (int i = 0; i < 60; ++i) {
            grips[0].Learn(left, centre, pad, 1.0f / 90.0f);
            grips[1].Learn(palm_at(right_in_pad, right_turn), centre, pad, 1.0f / 90.0f);
        }
        Check(grips[0].Known() && grips[0].TurnKnown(), "two thirds of a second learns it");
        Check(Near(grips[0].Offset(), left_in_pad, 0.001f), "where the palm is on the gamepad");
        Check(Near(grips[0].Centre(left, pad), centre, 0.001f), "and so where the middle is");
        Check(AngleBetween(grips[0].PadFrom(left), pad) < Radians(0.5f),
              "and how the gamepad is turned, from the palm");

        // The same grip, the gamepad turned elsewhere: the hand says where and how.
        const Quat turned = AxisAngle({0, 1, 0}, Radians(-50));
        Palm moved;
        moved.position = Add(Vec{-0.2f, 0.0f, -0.3f}, Rotate(turned, left_in_pad));
        moved.orientation_valid = true;
        moved.orientation = Multiply(turned, left_turn);
        Check(AngleBetween(grips[0].PadFrom(moved), turned) < Radians(0.5f),
              "the grip carries over to another attitude");
        Check(Near(grips[0].Centre(moved, turned), {-0.2f, 0.0f, -0.3f}, 0.001f),
              "and to another place");
    }
    {
        const std::optional<Palm> palms[2] = {palm_at(left_in_pad, left_turn), std::nullopt};
        const bool open[2] = {false, false};
        const bool left_grips[2] = {true, false};
        Check(!Locate(palms, open, grips, pad, -1), "an open hand holds nothing");

        const auto by_hand = Locate(palms, left_grips, grips, std::nullopt, -1);
        Check(by_hand && by_hand->hand == 0 && !by_hand->turned_by_sensors &&
                  Near(by_hand->position, centre, 0.002f) &&
                  AngleBetween(by_hand->orientation, pad) < Radians(1),
              "a gripping hand with a known grip holds it, by itself");

        const auto with_sensors = Locate(palms, left_grips, grips, pad, -1);
        Check(with_sensors && with_sensors->turned_by_sensors && with_sensors->hand_turn_known,
              "with the sensors, they say how it is turned");

        const Quat elsewhere = AxisAngle({0, 1, 0}, Radians(90));
        Check(!Locate(palms, left_grips, grips, elsewhere, -1),
              "a hand that disagrees with the sensors does not hold this gamepad");

        Grip fresh[2]{Grip{0}, Grip{1}};
        Check(!Locate(palms, left_grips, fresh, std::nullopt, -1),
              "without sensors, a grip never seen tells nothing");
        const auto standard = Locate(palms, left_grips, fresh, pad, -1);
        Check(standard && Near(standard->position, centre, 0.02f),
              "with sensors, the standard grip will do");

        const std::optional<Palm> both[2] = {palm_at(left_in_pad, left_turn),
                                             palm_at(right_in_pad, right_turn)};
        const bool both_grip[2] = {true, true};
        Check(Locate(both, both_grip, grips, pad, 1)->hand == 1,
              "of two gripping hands, the one that held it last");
    }
    {
        Smoother smoother;
        smoother.Update({1, 0, 0}, {}, 0.0, true);
        Check(Near(smoother.Position(), {1, 0, 0}, 1e-6f), "the first pose is taken as it is");
        smoother.Update({2, 0, 0}, {}, 1.0 / 90.0, true);
        Check(smoother.Position().x > 1.0f && smoother.Position().x < 1.5f,
              "then it follows part of the way");
        smoother.Update({3, 0, 0}, {}, 1.0, true);
        Check(Near(smoother.Position(), {3, 0, 0}, 1e-6f), "and after a gap starts afresh");
    }
    std::printf("%s\n", failures == 0 ? "all passed" : "FAILED");
    return failures == 0 ? 0 : 1;
}
