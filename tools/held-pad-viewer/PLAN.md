# Prototype: a gamepad placed by the hands (PC OpenXR, standalone)

## Context
On the PC path, the emulator only takes the gamepad's **position and heading** from hand
tracking (`shadps4-arm64-main/src/core/vr/openxr_host.cpp:2044-2150`, `UpdatePad`). Pitch and
roll come from the DualSense gyro. The Quest app already has a fuller, unit-tested estimator,
`quest-host/cpp/held_pad.h` (`HeldPad::Locate`). It is header-only with no Android
dependencies, and it gives position, yaw, roll (from the line between the palms) and pitch
(from where the hands point). Before putting more of this into the emulator, we want to *see*
how well hands alone place a gamepad. The prototype is a small Windows x64 OpenXR app: a plain
virtual room, your tracked hand joints, and a DualSense-sized controller model posed purely
from the hands. It reads no controller input.

## Approach
New folder `tools/held-pad-viewer/`, built like the existing `tools/probes/xr-probe-win`. It
uses the same CMake pattern, with the OpenXR loader linked statically from
`shadps4-arm64-main/externals/openxr-sdk`, and the clang-cl/xwin toolchain from `tools/env-win.sh`.

**Graphics: D3D11** (`XR_KHR_D3D11_enable`). Virtual Desktop's VDXR, SteamVR and the Meta XR
Simulator all support it. HLSL compiles at runtime with `D3DCompile`, so the build needs no
SPIR-V/glslang step. This is a throwaway viewer, so simplicity beats matching the emulator's
Vulkan code.

### Files
- `tools/held-pad-viewer/CMakeLists.txt`: copy of the xr-probe-win one. It links
  `d3d11 dxgi d3dcompiler` and adds `quest-host/cpp` to the include path for `held_pad.h`.
- `tools/held-pad-viewer/main.cpp` (~700 lines), in sections:
  1. **OpenXR setup**: instance with `XR_KHR_D3D11_enable` + `XR_EXT_hand_tracking` (exits with
     a clear message if hand tracking isn't offered, e.g. VD's hand-tracking forwarding is off),
     system, D3D11 device on the runtime's adapter LUID, session, `LOCAL` space (and `STAGE` if
     offered, for the floor), one swapchain per eye, two `XrHandTrackerEXT`s.
  2. **Frame loop**: the standard `xrWaitFrame/BeginFrame/LocateViews/EndFrame`. Session-state
     events, Esc/closing the console quits.
  3. **Hands → pose**: per frame, locate 26 joints per hand. Fill `HeldPad::Hands`: palms =
     `XR_HAND_JOINT_PALM_EXT`; pointing = normalized(middle metacarpal → middle proximal), the
     same as `quest-host/cpp/xr_host.cpp:~1727`. Call `HeldPad::Locate(hands, tilt, assumed_pitch)`.
     Smooth the orientation with `HeldPad::Towards` and the position with the same velocity
     blend as `openxr_host.cpp:2108-2125`. When the hands are lost, hold the last pose and draw
     it greyed out.
  4. **Renderer**: one instanced-cube pipeline (per-instance world matrix + colour, simple
     directional lighting) draws everything:
     - floor grid of thin boxes, plus a few reference cubes for depth;
     - hand joints as small cubes (left blue, right red), palms bigger;
     - pointing vectors as thin elongated boxes;
     - **the controller**: a DualSense-sized compound of boxes (body ~16×6.6×2 cm, two
       angled grips, touchpad, two stick nubs, light bar strip), in the pose from step 3, with
       local axes drawn. Coloured green when `turned_known && pitch_measured`, yellow when
       pitch is assumed, grey when hands are lost.
  5. **Console readout** (~5 Hz): palm distance, yaw/pitch/roll in degrees, whether pitch was
     measured, tracking-confidence flags.
  6. **Live tuning keys** (console window focus, `GetAsyncKeyState`): `T`/`G` change `tilt` ±2°,
     `S` toggles smoothing, `R` resets the smoothing state, `H` hides the hand joints. Values
     print on change, so good ones can be carried into the emulator later.

### Reused, not rewritten
- `quest-host/cpp/held_pad.h`: `Locate`, `Towards`, `Rotate`, `FromAxes`, `Vec/Quat`.
- Joint-to-pointing extraction and smoothing constants from `quest-host/cpp/xr_host.cpp`
  (~1720-1800) and `openxr_host.cpp:UpdatePad`.
- Build skeleton: `tools/probes/xr-probe-win/CMakeLists.txt` and the helpers at the top of its `main.cpp`
  (`ResultName`, `GetFunction`).

## Verification
1. Build from Git Bash on Windows: `source tools/env-win.sh && cmake -S tools/held-pad-viewer
   -B build/held-pad-viewer -G Ninja -DCMAKE_C_COMPILER=clang-cl -DCMAKE_CXX_COMPILER=clang-cl
   && ninja -C build/held-pad-viewer`.
2. **Without a headset**: run with `XR_RUNTIME_JSON` pointing to the Meta XR Simulator (the same path
   as `tools/pc-xrsim-test.sh`). Its simulated hands should show joints, and the controller
   should appear when the palms are brought 5–32 cm apart.
3. **On the Quest 3 via Virtual Desktop**: VDXR as runtime, hand-tracking forwarding on, Touch
   controllers put down. Hold the DualSense (or any similar object) and check that:
   - yaw follows turning;
   - roll follows tilting one hand down;
   - pitch follows tipping the front up (green state);
   - the model sits between the hands at the right depth;
   - it greys out and holds when the hands drop away.
4. Note jitter, and tracking loss when fingers wrap the gamepad. Those results decide whether
   to port `HeldPad`'s roll and pitch into `openxr_host.cpp` and fuse them with the gyro
   (`Runtime::UpdatePadHeldOrientation` / `UpdatePadYawReference` already exist for that).

## Notes for the implementing session (written 2026-11-12, from WSL)
- Build and run on **Windows**. This WSL checkout has neither `tools/llvm` (clang-cl, lld-link)
  nor `tools/winsdk` (xwin's MSVC CRT and Windows SDK), which `tools/env-win.sh` expects. Use the
  Windows-side toolchain, or Visual Studio's clang-cl/MSVC.
- Pointing per hand: the quest-host code takes the **palm joint's orientation, its -Z axis**
  (`Rotate(palm.pose.orientation, {0,0,-1})`, valid when `XR_SPACE_LOCATION_ORIENTATION_VALID_BIT`
  is set; `quest-host/cpp/xr_host.cpp:1600-1622`), not metacarpal→proximal. Use the same.
- Orientation smoothing constant: `HeldFollowTime = 0.06f` s, blend `1 - exp(-elapsed / 0.06)`,
  reset after a 0.25 s gap (`xr_host.cpp:1780-1795`, `:1999`). Position velocity blend 0.4,
  reset after a 0.1 s gap.
- Keep the last measured pitch as `assumed_pitch` for frames where pointing isn't known
  (`held_pitch` in xr_host.cpp).
- Optional: put the hands→pose and smoothing code in a header with no Windows dependencies, so
  it can be unit-tested like `tools/tests/held_pad_test.cpp`.
