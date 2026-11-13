// A gamepad placed by the hands: a plain room, the tracked hand joints, and a DualSense-sized
// model posed by HeldPad::Locate (shadps4-arm64-main/src/core/vr/held_pad.h) from the palms.
// For seeing how well hands place a gamepad, and trying out more of it before it goes into
// the emulator. See PLAN.md.
// With a gamepad that has motion sensors (a DualSense, by USB or Bluetooth), its gyroscope and
// accelerometer are fused with the hands as the emulator fuses them (held_pad_fusion.h): the
// solid model is then posed by that, the dim axes by the hands alone (F swaps them).
//   held_pad_viewer.exe
// Keys (in the console window): T/G tilt +-2 degrees, A take the tilt that matches the
// sensors, S smoothing on/off, R reset smoothing, sensors and learned grips, F sensors or
// hands for the solid model, H hide/show hand joints, Esc quit.
// In one hand (held_pad_one_hand.h): a hand whose fingers are curled around the grip holds
// it; where that hand holds it is learned while both hands do. The model is purple then.

#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <d3d11.h>
#include <d3dcompiler.h>
#include <dxgi1_2.h>

#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_D3D11
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include <SDL3/SDL.h>

#include "held_pad_follow.h"
#include "held_pad_fusion.h"
#include "held_pad_one_hand.h"

namespace {

using HeldPad::Quat;
using HeldPad::Vec;

constexpr float Pi = 3.14159265f;
constexpr float Degrees = Pi / 180.0f;

// ---------------------------------------------------------------------------------------------
// Helpers (as tools/probes/xr-probe-win)

const char* ResultName(XrInstance instance, XrResult result) {
    static char buffer[XR_MAX_RESULT_STRING_SIZE];
    if (instance == XR_NULL_HANDLE || XR_FAILED(xrResultToString(instance, result, buffer))) {
        std::snprintf(buffer, sizeof(buffer), "%d", static_cast<int>(result));
    }
    return buffer;
}

template <typename Function>
Function GetFunction(XrInstance instance, const char* name) {
    PFN_xrVoidFunction function = nullptr;
    xrGetInstanceProcAddr(instance, name, &function);
    return reinterpret_cast<Function>(function);
}

template <typename T>
void Release(T*& object) {
    if (object) {
        object->Release();
        object = nullptr;
    }
}

// ---------------------------------------------------------------------------------------------
// Maths: column vectors, matrices stored by rows.

using HeldPad::Add;
using HeldPad::Multiply;

Vec Normalize(const Vec& a) {
    const float length = HeldPad::Length(a);
    return length > 1e-9f ? HeldPad::Scale(a, 1.0f / length) : Vec{0.0f, 0.0f, 1.0f};
}

Quat AxisAngle(const Vec& axis, float angle) {
    const Vec a = Normalize(axis);
    const float s = std::sin(angle * 0.5f);
    return {a.x * s, a.y * s, a.z * s, std::cos(angle * 0.5f)};
}

struct Mat3 {
    float m[3][3];
};

Mat3 FromQuat(const Quat& q) {
    const float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    const float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    const float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    return {{{1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)},
             {2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)},
             {2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)}}};
}

Mat3 FromColumns(const Vec& x, const Vec& y, const Vec& z) {
    return {{{x.x, y.x, z.x}, {x.y, y.y, z.y}, {x.z, y.z, z.z}}};
}

Mat3 Multiply(const Mat3& a, const Mat3& b) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
        }
    }
    return r;
}

Vec Multiply(const Mat3& a, const Vec& v) {
    return {a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
            a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
            a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z};
}

/// A rotation then a move.
struct Affine {
    Mat3 rotation{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    Vec translation;
};

Affine Pose(const Vec& position, const Quat& orientation) {
    return {FromQuat(orientation), position};
}

/// `inner` placed within `outer`.
Affine Within(const Affine& outer, const Affine& inner) {
    return {Multiply(outer.rotation, inner.rotation),
            Add(Multiply(outer.rotation, inner.translation), outer.translation)};
}

struct Mat4 {
    float m[4][4];
};

Mat4 Multiply(const Mat4& a, const Mat4& b) {
    Mat4 r{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                r.m[i][j] += a.m[i][k] * b.m[k][j];
            }
        }
    }
    return r;
}

/// Asymmetric perspective from an OpenXR field of view, depth 0 (near) to 1 (far), for a view
/// space looking down -Z.
Mat4 Projection(const XrFovf& fov, float near_z, float far_z) {
    const float left = std::tan(fov.angleLeft), right = std::tan(fov.angleRight);
    const float up = std::tan(fov.angleUp), down = std::tan(fov.angleDown);
    Mat4 p{};
    p.m[0][0] = 2.0f / (right - left);
    p.m[0][2] = (right + left) / (right - left);
    p.m[1][1] = 2.0f / (up - down);
    p.m[1][2] = (up + down) / (up - down);
    p.m[2][2] = -far_z / (far_z - near_z);
    p.m[2][3] = -(far_z * near_z) / (far_z - near_z);
    p.m[3][2] = -1.0f;
    return p;
}

/// From the space the eye's pose is in to the eye's own.
Mat4 ViewFrom(const XrPosef& eye) {
    const Mat3 r = FromQuat({eye.orientation.x, eye.orientation.y, eye.orientation.z,
                             eye.orientation.w});
    const Vec p{eye.position.x, eye.position.y, eye.position.z};
    Mat4 v{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            v.m[i][j] = r.m[j][i];
        }
        v.m[i][3] = -(r.m[0][i] * p.x + r.m[1][i] * p.y + r.m[2][i] * p.z);
    }
    v.m[3][3] = 1.0f;
    return v;
}

// ---------------------------------------------------------------------------------------------
// Renderer: every thing drawn is a box, one instanced draw per eye.

struct Color {
    float r, g, b, a{1.0f};
};

struct Instance {
    float rows[3][4];
    float color[4];
};

struct Scene {
    std::vector<Instance> boxes;

    /// A box of `size` centred at `place`.
    void Box(const Affine& place, const Vec& size, const Color& color) {
        Instance instance{};
        const float s[3] = {size.x, size.y, size.z};
        const float t[3] = {place.translation.x, place.translation.y, place.translation.z};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                instance.rows[i][j] = place.rotation.m[i][j] * s[j];
            }
            instance.rows[i][3] = t[i];
        }
        instance.color[0] = color.r;
        instance.color[1] = color.g;
        instance.color[2] = color.b;
        instance.color[3] = color.a;
        boxes.push_back(instance);
    }

    void Box(const Vec& centre, const Vec& size, const Color& color) {
        Box(Affine{{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}, centre}, size, color);
    }

    /// A thin box from one point to another.
    void Line(const Vec& from, const Vec& to, float thickness, const Color& color) {
        const Vec along = HeldPad::Sub(to, from);
        const float length = HeldPad::Length(along);
        if (length < 1e-5f) {
            return;
        }
        const Vec z = HeldPad::Scale(along, 1.0f / length);
        const Vec helper = std::abs(z.y) < 0.9f ? Vec{0, 1, 0} : Vec{1, 0, 0};
        const Vec x = Normalize(HeldPad::Cross(helper, z));
        const Vec y = HeldPad::Cross(z, x);
        Box(Affine{FromColumns(x, y, z), HeldPad::Scale(Add(from, to), 0.5f)},
            {thickness, thickness, length}, color);
    }
};

const char* const ShaderSource = R"(
cbuffer Frame : register(b0) {
    row_major float4x4 view_projection;
    float4 light;
};
struct Input {
    float3 position : POSITION;
    float3 normal : NORMAL;
    float4 row0 : WORLD0;
    float4 row1 : WORLD1;
    float4 row2 : WORLD2;
    float4 color : COLOR;
};
struct Varying {
    float4 position : SV_Position;
    float3 normal : NORMAL;
    float4 color : COLOR;
};
Varying VS(Input input) {
    const float4 p = float4(input.position, 1.0);
    const float3 world = float3(dot(input.row0, p), dot(input.row1, p), dot(input.row2, p));
    Varying output;
    output.position = mul(view_projection, float4(world, 1.0));
    output.normal = float3(dot(input.row0.xyz, input.normal), dot(input.row1.xyz, input.normal),
                           dot(input.row2.xyz, input.normal));
    output.color = input.color;
    return output;
}
float4 PS(Varying input) : SV_Target {
    const float3 n = normalize(input.normal);
    const float lit = 0.35 + 0.65 * saturate(dot(n, -light.xyz));
    return float4(input.color.rgb * lit, input.color.a);
}
)";

struct FrameConstants {
    float view_projection[4][4];
    float light[4];
};

class Renderer {
public:
    static constexpr UINT MaxBoxes = 8192;

    bool Init(ID3D11Device* device) {
        ID3DBlob* vs = Compile("VS", "vs_5_0");
        ID3DBlob* ps = Compile("PS", "ps_5_0");
        if (!vs || !ps) {
            Release(vs);
            Release(ps);
            return false;
        }
        bool ok = SUCCEEDED(device->CreateVertexShader(vs->GetBufferPointer(),
                                                       vs->GetBufferSize(), nullptr, &vs_)) &&
                  SUCCEEDED(device->CreatePixelShader(ps->GetBufferPointer(),
                                                      ps->GetBufferSize(), nullptr, &ps_));
        const D3D11_INPUT_ELEMENT_DESC layout[] = {
            {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
            {"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0},
            {"WORLD", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 1, 0, D3D11_INPUT_PER_INSTANCE_DATA, 1},
            {"WORLD", 1, DXGI_FORMAT_R32G32B32A32_FLOAT, 1, 16, D3D11_INPUT_PER_INSTANCE_DATA, 1},
            {"WORLD", 2, DXGI_FORMAT_R32G32B32A32_FLOAT, 1, 32, D3D11_INPUT_PER_INSTANCE_DATA, 1},
            {"COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 1, 48, D3D11_INPUT_PER_INSTANCE_DATA, 1},
        };
        ok = ok && SUCCEEDED(device->CreateInputLayout(layout, 6, vs->GetBufferPointer(),
                                                       vs->GetBufferSize(), &layout_));
        Release(vs);
        Release(ps);

        // A unit cube: four corners per face, so that each face has its own normal.
        struct Vertex {
            float position[3];
            float normal[3];
        };
        std::vector<Vertex> vertices;
        std::vector<uint16_t> indices;
        const Vec normals[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
        for (const Vec& n : normals) {
            const Vec u = std::abs(n.y) > 0.5f ? Vec{1, 0, 0} : Vec{0, 1, 0};
            const Vec v = HeldPad::Cross(n, u);
            const auto base = static_cast<uint16_t>(vertices.size());
            const float corners[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
            for (const auto& c : corners) {
                const Vec p = HeldPad::Scale(
                    Add(n, Add(HeldPad::Scale(u, c[0]), HeldPad::Scale(v, c[1]))), 0.5f);
                vertices.push_back({{p.x, p.y, p.z}, {n.x, n.y, n.z}});
            }
            for (const uint16_t i : {0, 1, 2, 0, 2, 3}) {
                indices.push_back(static_cast<uint16_t>(base + i));
            }
        }
        index_count_ = static_cast<UINT>(indices.size());

        D3D11_BUFFER_DESC desc{};
        D3D11_SUBRESOURCE_DATA data{};
        desc.Usage = D3D11_USAGE_IMMUTABLE;
        desc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
        desc.ByteWidth = static_cast<UINT>(vertices.size() * sizeof(Vertex));
        data.pSysMem = vertices.data();
        ok = ok && SUCCEEDED(device->CreateBuffer(&desc, &data, &vertices_));
        desc.BindFlags = D3D11_BIND_INDEX_BUFFER;
        desc.ByteWidth = static_cast<UINT>(indices.size() * sizeof(uint16_t));
        data.pSysMem = indices.data();
        ok = ok && SUCCEEDED(device->CreateBuffer(&desc, &data, &indices_));

        desc.Usage = D3D11_USAGE_DYNAMIC;
        desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        desc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
        desc.ByteWidth = MaxBoxes * sizeof(Instance);
        ok = ok && SUCCEEDED(device->CreateBuffer(&desc, nullptr, &instances_));
        desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        desc.ByteWidth = sizeof(FrameConstants);
        ok = ok && SUCCEEDED(device->CreateBuffer(&desc, nullptr, &constants_));

        D3D11_RASTERIZER_DESC raster{};
        raster.FillMode = D3D11_FILL_SOLID;
        raster.CullMode = D3D11_CULL_NONE;
        raster.DepthClipEnable = TRUE;
        ok = ok && SUCCEEDED(device->CreateRasterizerState(&raster, &raster_));
        D3D11_DEPTH_STENCIL_DESC depth{};
        depth.DepthEnable = TRUE;
        depth.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
        depth.DepthFunc = D3D11_COMPARISON_LESS;
        ok = ok && SUCCEEDED(device->CreateDepthStencilState(&depth, &depth_));
        return ok;
    }

    /// Once a frame, for both eyes.
    void Upload(ID3D11DeviceContext* context, const Scene& scene) {
        box_count_ = static_cast<UINT>(std::min<size_t>(scene.boxes.size(), MaxBoxes));
        D3D11_MAPPED_SUBRESOURCE mapped{};
        if (box_count_ > 0 && SUCCEEDED(context->Map(instances_, 0, D3D11_MAP_WRITE_DISCARD, 0,
                                                     &mapped))) {
            std::memcpy(mapped.pData, scene.boxes.data(), box_count_ * sizeof(Instance));
            context->Unmap(instances_, 0);
        }
    }

    void Draw(ID3D11DeviceContext* context, ID3D11RenderTargetView* target,
              ID3D11DepthStencilView* depth, int width, int height, const Mat4& view_projection) {
        const float sky[4] = {0.10f, 0.12f, 0.16f, 1.0f};
        context->ClearRenderTargetView(target, sky);
        context->ClearDepthStencilView(depth, D3D11_CLEAR_DEPTH, 1.0f, 0);
        context->OMSetRenderTargets(1, &target, depth);
        const D3D11_VIEWPORT viewport{0, 0, static_cast<float>(width), static_cast<float>(height),
                                      0, 1};
        context->RSSetViewports(1, &viewport);

        FrameConstants constants{};
        std::memcpy(constants.view_projection, view_projection.m, sizeof(constants.view_projection));
        const Vec light = Normalize({0.3f, -1.0f, -0.5f});
        constants.light[0] = light.x;
        constants.light[1] = light.y;
        constants.light[2] = light.z;
        D3D11_MAPPED_SUBRESOURCE mapped{};
        if (SUCCEEDED(context->Map(constants_, 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped))) {
            std::memcpy(mapped.pData, &constants, sizeof(constants));
            context->Unmap(constants_, 0);
        }
        if (box_count_ == 0) {
            return;
        }

        ID3D11Buffer* buffers[2] = {vertices_, instances_};
        const UINT strides[2] = {24, sizeof(Instance)};
        const UINT offsets[2] = {0, 0};
        context->IASetVertexBuffers(0, 2, buffers, strides, offsets);
        context->IASetIndexBuffer(indices_, DXGI_FORMAT_R16_UINT, 0);
        context->IASetInputLayout(layout_);
        context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        context->VSSetShader(vs_, nullptr, 0);
        context->VSSetConstantBuffers(0, 1, &constants_);
        context->PSSetShader(ps_, nullptr, 0);
        context->RSSetState(raster_);
        context->OMSetDepthStencilState(depth_, 0);
        context->DrawIndexedInstanced(index_count_, box_count_, 0, 0, 0);
    }

    ~Renderer() {
        Release(vs_);
        Release(ps_);
        Release(layout_);
        Release(vertices_);
        Release(indices_);
        Release(instances_);
        Release(constants_);
        Release(raster_);
        Release(depth_);
    }

private:
    static ID3DBlob* Compile(const char* entry, const char* target) {
        ID3DBlob* code = nullptr;
        ID3DBlob* errors = nullptr;
        const HRESULT result =
            D3DCompile(ShaderSource, std::strlen(ShaderSource), "held_pad_viewer.hlsl", nullptr,
                       nullptr, entry, target, D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, &code, &errors);
        if (FAILED(result)) {
            std::printf("shader %s: %s\n", entry,
                        errors ? static_cast<const char*>(errors->GetBufferPointer()) : "failed");
        }
        Release(errors);
        return code;
    }

    ID3D11VertexShader* vs_{};
    ID3D11PixelShader* ps_{};
    ID3D11InputLayout* layout_{};
    ID3D11Buffer* vertices_{};
    ID3D11Buffer* indices_{};
    ID3D11Buffer* instances_{};
    ID3D11Buffer* constants_{};
    ID3D11RasterizerState* raster_{};
    ID3D11DepthStencilState* depth_{};
    UINT index_count_{};
    UINT box_count_{};
};

// ---------------------------------------------------------------------------------------------
// What is drawn

struct HandView {
    bool active{};
    XrHandJointLocationEXT joints[XR_HAND_JOINT_COUNT_EXT]{};
};

void AddRoom(Scene& scene, float floor) {
    const Color line{0.35f, 0.38f, 0.45f};
    for (int i = -6; i <= 6; ++i) {
        const float at = i * 0.5f;
        scene.Box(Vec{at, floor, 0.0f}, {0.006f, 0.002f, 6.0f}, line);
        scene.Box(Vec{0.0f, floor, at}, {6.0f, 0.002f, 0.006f}, line);
    }
    // A few things at known places, for depth.
    scene.Box(Vec{0.7f, floor + 0.15f, -1.2f}, {0.3f, 0.3f, 0.3f}, {0.55f, 0.35f, 0.25f});
    scene.Box(Vec{-0.9f, floor + 0.25f, -1.6f}, {0.5f, 0.5f, 0.5f}, {0.3f, 0.45f, 0.6f});
    scene.Box(Vec{0.0f, floor + 1.0f, -2.8f}, {0.25f, 2.0f, 0.25f}, {0.6f, 0.6f, 0.55f});
    scene.Box(Vec{1.6f, floor + 0.4f, 0.4f}, {0.4f, 0.8f, 0.4f}, {0.45f, 0.55f, 0.35f});
    scene.Box(Vec{-1.5f, floor + 0.05f, 0.8f}, {0.6f, 0.1f, 0.6f}, {0.5f, 0.4f, 0.5f});
    // A table in front, at the height a seated player's hands are.
    scene.Box(Vec{0.0f, floor + 0.72f, -0.75f}, {0.8f, 0.03f, 0.4f}, {0.42f, 0.33f, 0.24f});
}

void AddHands(Scene& scene, const HandView hands[2], const std::optional<HeldPadFollow::Palm> palms[2]) {
    const Color colors[2] = {{0.25f, 0.45f, 1.0f}, {1.0f, 0.3f, 0.25f}};
    for (int hand = 0; hand < 2; ++hand) {
        if (!hands[hand].active) {
            continue;
        }
        for (int joint = 0; joint < XR_HAND_JOINT_COUNT_EXT; ++joint) {
            const auto& location = hands[hand].joints[joint];
            if ((location.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) == 0) {
                continue;
            }
            const Vec position{location.pose.position.x, location.pose.position.y,
                               location.pose.position.z};
            const Quat orientation =
                (location.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0
                    ? Quat{location.pose.orientation.x, location.pose.orientation.y,
                           location.pose.orientation.z, location.pose.orientation.w}
                    : Quat{};
            const float size = joint == XR_HAND_JOINT_PALM_EXT    ? 0.02f
                               : joint == XR_HAND_JOINT_WRIST_EXT ? 0.012f
                                                                  : 0.008f;
            scene.Box(Pose(position, orientation), {size, size, size}, colors[hand]);
        }
    }
    for (int hand = 0; hand < 2; ++hand) {
        if (palms[hand] && palms[hand]->pointing_known) {
            const Color& c = colors[hand];
            scene.Line(palms[hand]->position,
                       Add(palms[hand]->position, HeldPad::Scale(palms[hand]->pointing, 0.12f)),
                       0.004f, {0.5f + c.r * 0.5f, 0.5f + c.g * 0.5f, 0.5f + c.b * 0.5f});
        }
    }
    if (palms[0] && palms[1]) {
        scene.Line(palms[0]->position, palms[1]->position, 0.002f, {0.85f, 0.85f, 0.85f});
    }
}

/// The colour of the model posed by the hands alone: green with the pitch measured, yellow
/// with it assumed, orange when the hands are not level enough to tell the heading, grey when
/// they are lost and the last pose holds.
Color HandsColor(const HeldPadFollow::Shown& shown) {
    if (!shown.live) {
        return {0.42f, 0.42f, 0.42f};
    }
    if (!shown.turned_known) {
        return {0.95f, 0.5f, 0.15f};
    }
    return shown.pitch_measured ? Color{0.25f, 0.85f, 0.35f} : Color{0.95f, 0.82f, 0.2f};
}

/// The colour of the model the sensors turn: cyan while the hands place it, slate when they
/// are lost (it is still turned, where they left it).
Color SensorsColor(const HeldPadFollow::Shown& shown) {
    return shown.live ? Color{0.2f, 0.75f, 0.95f} : Color{0.4f, 0.46f, 0.58f};
}

/// Axes of a pose, `length` long: +X red, +Y green, front (-Z) blue, `bright` 0 to 1.
void AddAxes(Scene& scene, const Vec& position, const Quat& orientation, float length,
             float thickness, float bright) {
    const Mat3 rotation = FromQuat(orientation);
    const auto axis = [&](const Vec& direction, const Color& color) {
        scene.Line(position,
                   Add(position, HeldPad::Scale(Multiply(rotation, direction), length)),
                   thickness, {color.r * bright, color.g * bright, color.b * bright});
    };
    axis({1, 0, 0}, {1.0f, 0.15f, 0.15f});
    axis({0, 1, 0}, {0.15f, 1.0f, 0.15f});
    axis({0, 0, -1}, {0.2f, 0.4f, 1.0f});
}

/// A DualSense-sized gamepad in its own space: +X right, +Y out of its face, -Z where its
/// front points; the middle of the body at the origin.
void AddGamepad(Scene& scene, const Vec& position, const Quat& orientation, const Color& body) {
    const Color dark{body.r * 0.35f, body.g * 0.35f, body.b * 0.35f};
    const Color black{0.06f, 0.06f, 0.07f};
    const Affine pad = Pose(position, orientation);
    const auto part = [&](const Vec& at, const Quat& turn, const Vec& size, const Color& color) {
        scene.Box(Within(pad, Pose(at, turn)), size, color);
    };

    part({0, 0, 0}, {}, {0.16f, 0.02f, 0.066f}, body);
    for (const float side : {-1.0f, 1.0f}) {
        // The grips run back and down from the body, splayed outwards.
        const Quat grip = Multiply(AxisAngle({0, 1, 0}, side * -14.0f * Degrees),
                                   AxisAngle({1, 0, 0}, -28.0f * Degrees));
        part({side * 0.058f, -0.014f, 0.05f}, grip, {0.04f, 0.03f, 0.075f}, body);
        // Sticks, shoulder buttons, d-pad and face buttons.
        part({side * 0.028f, 0.016f, 0.014f}, {}, {0.016f, 0.012f, 0.016f}, black);
        part({side * 0.052f, 0.004f, -0.037f}, {}, {0.032f, 0.008f, 0.008f}, dark);
        for (int button = 0; button < 4; ++button) {
            const float angle = button * Pi * 0.5f;
            part({side * 0.056f + std::cos(angle) * 0.011f, 0.0115f,
                  -0.008f + std::sin(angle) * 0.011f},
                 {}, {0.008f, 0.004f, 0.008f}, side > 0 ? black : dark);
        }
    }
    part({0, 0.0115f, -0.012f}, {}, {0.062f, 0.004f, 0.036f}, dark);
    part({0, 0.0125f, -0.0315f}, {}, {0.05f, 0.003f, 0.004f}, {0.3f, 0.6f, 1.0f});
    AddAxes(scene, position, orientation, 0.1f, 0.003f, 1.0f);
}

// ---------------------------------------------------------------------------------------------
// The gamepad's motion sensors, through SDL as the emulator reads them

struct Sensors {
    SDL_Gamepad* gamepad{};
    SDL_JoystickID id{};
    bool gyro{};
    Uint64 gyro_time{};
    int gyro_readings{};
    HeldPadFusion::Imu imu;

    void Open(SDL_JoystickID which) {
        SDL_Gamepad* opened = SDL_OpenGamepad(which);
        if (!opened) {
            std::printf("gamepad: could not open it: %s\n", SDL_GetError());
            return;
        }
        const bool has_gyro = SDL_GamepadHasSensor(opened, SDL_SENSOR_GYRO);
        const bool has_accel = SDL_GamepadHasSensor(opened, SDL_SENSOR_ACCEL);
        if (!has_gyro || !has_accel) {
            std::printf("gamepad: %s, without motion sensors: the hands alone\n",
                        SDL_GetGamepadName(opened));
            SDL_CloseGamepad(opened);
            return;
        }
        SDL_SetGamepadSensorEnabled(opened, SDL_SENSOR_GYRO, true);
        SDL_SetGamepadSensorEnabled(opened, SDL_SENSOR_ACCEL, true);
        gamepad = opened;
        id = which;
        gyro = true;
        gyro_time = 0;
        imu.Reset();
        std::printf("gamepad: %s, gyroscope at %.0f Hz, accelerometer at %.0f Hz\n",
                    SDL_GetGamepadName(opened),
                    SDL_GetGamepadSensorDataRate(opened, SDL_SENSOR_GYRO),
                    SDL_GetGamepadSensorDataRate(opened, SDL_SENSOR_ACCEL));
    }

    void Close() {
        if (gamepad) {
            SDL_CloseGamepad(gamepad);
        }
        gamepad = nullptr;
        gyro = false;
        imu.Reset();
    }

    /// Everything SDL has heard since the last call.
    void Pump() {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
            case SDL_EVENT_GAMEPAD_ADDED:
                if (!gamepad) {
                    Open(event.gdevice.which);
                }
                break;
            case SDL_EVENT_GAMEPAD_REMOVED:
                if (gamepad && event.gdevice.which == id) {
                    std::printf("gamepad: gone\n");
                    Close();
                }
                break;
            case SDL_EVENT_GAMEPAD_SENSOR_UPDATE: {
                if (!gamepad || event.gsensor.which != id) {
                    break;
                }
                const float* data = event.gsensor.data;
                if (event.gsensor.sensor == SDL_SENSOR_ACCEL) {
                    imu.Accel({data[0], data[1], data[2]});
                } else if (event.gsensor.sensor == SDL_SENSOR_GYRO) {
                    const Uint64 at = event.gsensor.sensor_timestamp;
                    const float elapsed =
                        gyro_time != 0 && at > gyro_time ? (at - gyro_time) * 1e-9f : 0.0f;
                    gyro_time = at;
                    imu.Gyro({data[0], data[1], data[2]}, elapsed, Now());
                    ++gyro_readings;
                }
                break;
            }
            default:
                break;
            }
        }
    }

    /// Seconds, on the clock hand headings are given by too.
    static double Now() {
        return SDL_GetTicksNS() * 1e-9;
    }
};

// ---------------------------------------------------------------------------------------------
// Quitting

std::atomic<bool> quit_requested{false};
std::atomic<bool> finished{false};

BOOL WINAPI OnConsoleEvent(DWORD event) {
    quit_requested = true;
    if (event == CTRL_CLOSE_EVENT || event == CTRL_LOGOFF_EVENT || event == CTRL_SHUTDOWN_EVENT) {
        // The process ends when this returns: give the session a moment to end properly.
        for (int i = 0; i < 300 && !finished; ++i) {
            Sleep(10);
        }
    }
    return TRUE;
}

/// Keys pressed in the console window since the last call.
std::vector<WORD> KeysPressed(HANDLE input) {
    std::vector<WORD> keys;
    DWORD count = 0;
    while (input != INVALID_HANDLE_VALUE && GetNumberOfConsoleInputEvents(input, &count) &&
           count > 0) {
        INPUT_RECORD record;
        DWORD read = 0;
        if (!ReadConsoleInputW(input, &record, 1, &read) || read == 0) {
            break;
        }
        if (record.EventType == KEY_EVENT && record.Event.KeyEvent.bKeyDown) {
            keys.push_back(record.Event.KeyEvent.wVirtualKeyCode);
        }
    }
    return keys;
}

struct Eye {
    XrSwapchain swapchain{XR_NULL_HANDLE};
    int width{}, height{};
    std::vector<ID3D11RenderTargetView*> targets;
    ID3D11Texture2D* depth_texture{};
    ID3D11DepthStencilView* depth{};
};

float RollOf(const Quat& orientation) {
    const Vec right = HeldPad::Rotate(orientation, {1, 0, 0});
    return std::asin(std::clamp(right.y, -1.0f, 1.0f));
}

} // namespace

int main(int argc, char** argv) {
    bool use_gamepad = true;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--no-gamepad") == 0) {
            use_gamepad = false;
        }
    }
    // Each line as it is printed, also when the output goes to a file.
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::printf("held-pad viewer\n");
    SetConsoleCtrlHandler(OnConsoleEvent, TRUE);
    const HANDLE console_input = GetStdHandle(STD_INPUT_HANDLE);
    {
        DWORD mode = 0;
        if (GetConsoleMode(console_input, &mode)) {
            // Keys as they are pressed, not lines; Ctrl+C still quits.
            SetConsoleMode(console_input, (mode & ~(ENABLE_LINE_INPUT | ENABLE_ECHO_INPUT |
                                                    ENABLE_QUICK_EDIT_MODE)) |
                                              ENABLE_EXTENDED_FLAGS | ENABLE_PROCESSED_INPUT);
        }
    }
    if (const char* json = std::getenv("XR_RUNTIME_JSON")) {
        std::printf("runtime named by XR_RUNTIME_JSON: %s\n", json);
    }

    // --- Instance ---------------------------------------------------------------------------
    uint32_t count = 0;
    if (XR_FAILED(xrEnumerateInstanceExtensionProperties(nullptr, 0, &count, nullptr))) {
        std::printf("no OpenXR runtime answers\n");
        return 1;
    }
    std::vector<XrExtensionProperties> available(count, {XR_TYPE_EXTENSION_PROPERTIES});
    xrEnumerateInstanceExtensionProperties(nullptr, count, &count, available.data());
    const auto has = [&](const char* name) {
        return std::any_of(available.begin(), available.end(), [&](const auto& property) {
            return std::strcmp(property.extensionName, name) == 0;
        });
    };
    if (!has(XR_KHR_D3D11_ENABLE_EXTENSION_NAME)) {
        std::printf("the runtime does not offer %s\n", XR_KHR_D3D11_ENABLE_EXTENSION_NAME);
        return 1;
    }
    if (!has(XR_EXT_HAND_TRACKING_EXTENSION_NAME)) {
        std::printf("the runtime does not offer hand tracking (%s). With Virtual Desktop, turn on "
                    "forwarding of hand tracking in its settings; with Quest Link, its "
                    "developer runtime features.\n",
                    XR_EXT_HAND_TRACKING_EXTENSION_NAME);
        return 1;
    }
    const char* extensions[] = {XR_KHR_D3D11_ENABLE_EXTENSION_NAME,
                                XR_EXT_HAND_TRACKING_EXTENSION_NAME};
    XrInstanceCreateInfo instance_info{XR_TYPE_INSTANCE_CREATE_INFO};
    std::strcpy(instance_info.applicationInfo.applicationName, "Astro held-pad viewer");
    instance_info.applicationInfo.applicationVersion = 1;
    instance_info.applicationInfo.apiVersion = XR_API_VERSION_1_0;
    instance_info.enabledExtensionCount = 2;
    instance_info.enabledExtensionNames = extensions;
    XrInstance instance = XR_NULL_HANDLE;
    XrResult result = xrCreateInstance(&instance_info, &instance);
    if (XR_FAILED(result)) {
        std::printf("xrCreateInstance: %s\n", ResultName(XR_NULL_HANDLE, result));
        return 1;
    }
    XrInstanceProperties instance_properties{XR_TYPE_INSTANCE_PROPERTIES};
    xrGetInstanceProperties(instance, &instance_properties);
    std::printf("runtime: %s %u.%u.%u\n", instance_properties.runtimeName,
                XR_VERSION_MAJOR(instance_properties.runtimeVersion),
                XR_VERSION_MINOR(instance_properties.runtimeVersion),
                XR_VERSION_PATCH(instance_properties.runtimeVersion));

    const auto get_d3d11_requirements = GetFunction<PFN_xrGetD3D11GraphicsRequirementsKHR>(
        instance, "xrGetD3D11GraphicsRequirementsKHR");
    const auto create_hand_tracker =
        GetFunction<PFN_xrCreateHandTrackerEXT>(instance, "xrCreateHandTrackerEXT");
    const auto destroy_hand_tracker =
        GetFunction<PFN_xrDestroyHandTrackerEXT>(instance, "xrDestroyHandTrackerEXT");
    const auto locate_hand_joints =
        GetFunction<PFN_xrLocateHandJointsEXT>(instance, "xrLocateHandJointsEXT");
    if (!get_d3d11_requirements || !create_hand_tracker || !destroy_hand_tracker ||
        !locate_hand_joints) {
        std::printf("the runtime lacks functions of the extensions it offers\n");
        xrDestroyInstance(instance);
        return 1;
    }

    // --- System -----------------------------------------------------------------------------
    XrSystemGetInfo system_info{XR_TYPE_SYSTEM_GET_INFO};
    system_info.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    XrSystemId system = XR_NULL_SYSTEM_ID;
    result = xrGetSystem(instance, &system_info, &system);
    if (XR_FAILED(result)) {
        std::printf("xrGetSystem: %s (is the headset connected?)\n", ResultName(instance, result));
        xrDestroyInstance(instance);
        return 1;
    }
    XrSystemHandTrackingPropertiesEXT hand_properties{XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT};
    XrSystemProperties system_properties{XR_TYPE_SYSTEM_PROPERTIES};
    system_properties.next = &hand_properties;
    xrGetSystemProperties(instance, system, &system_properties);
    std::printf("system: %s\n", system_properties.systemName);
    if (!hand_properties.supportsHandTracking) {
        std::printf("the system does not track hands. With Virtual Desktop, turn on forwarding "
                    "of hand tracking in its settings; with Quest Link, its developer runtime "
                    "features.\n");
        xrDestroyInstance(instance);
        return 1;
    }

    // --- D3D11 device on the runtime's adapter -----------------------------------------------
    XrGraphicsRequirementsD3D11KHR requirements{XR_TYPE_GRAPHICS_REQUIREMENTS_D3D11_KHR};
    get_d3d11_requirements(instance, system, &requirements);
    IDXGIFactory1* factory = nullptr;
    IDXGIAdapter1* adapter = nullptr;
    if (SUCCEEDED(CreateDXGIFactory1(__uuidof(IDXGIFactory1), reinterpret_cast<void**>(&factory)))) {
        for (UINT i = 0; factory->EnumAdapters1(i, &adapter) != DXGI_ERROR_NOT_FOUND; ++i) {
            DXGI_ADAPTER_DESC1 desc{};
            adapter->GetDesc1(&desc);
            if (std::memcmp(&desc.AdapterLuid, &requirements.adapterLuid, sizeof(LUID)) == 0) {
                std::printf("adapter: %ls\n", desc.Description);
                break;
            }
            Release(adapter);
        }
    }
    Release(factory);
    const D3D_FEATURE_LEVEL levels[] = {D3D_FEATURE_LEVEL_11_1, D3D_FEATURE_LEVEL_11_0};
    ID3D11Device* device = nullptr;
    ID3D11DeviceContext* context = nullptr;
    HRESULT hr = D3D11CreateDevice(adapter, adapter ? D3D_DRIVER_TYPE_UNKNOWN : D3D_DRIVER_TYPE_HARDWARE,
                                   nullptr, 0, levels, 2, D3D11_SDK_VERSION, &device, nullptr,
                                   &context);
    Release(adapter);
    if (FAILED(hr)) {
        std::printf("D3D11CreateDevice: 0x%08lx\n", static_cast<unsigned long>(hr));
        xrDestroyInstance(instance);
        return 1;
    }
    {
        Renderer renderer;
        if (!renderer.Init(device)) {
            std::printf("the renderer could not be set up\n");
            Release(context);
            Release(device);
            xrDestroyInstance(instance);
            return 1;
        }

        // --- Session, spaces, hand trackers ------------------------------------------------
        XrGraphicsBindingD3D11KHR binding{XR_TYPE_GRAPHICS_BINDING_D3D11_KHR};
        binding.device = device;
        XrSessionCreateInfo session_info{XR_TYPE_SESSION_CREATE_INFO};
        session_info.next = &binding;
        session_info.systemId = system;
        XrSession session = XR_NULL_HANDLE;
        result = xrCreateSession(instance, &session_info, &session);
        if (XR_FAILED(result)) {
            std::printf("xrCreateSession: %s\n", ResultName(instance, result));
            Release(context);
            Release(device);
            xrDestroyInstance(instance);
            return 1;
        }

        XrReferenceSpaceCreateInfo space_info{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
        space_info.poseInReferenceSpace.orientation.w = 1.0f;
        space_info.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
        XrSpace local_space = XR_NULL_HANDLE;
        xrCreateReferenceSpace(session, &space_info, &local_space);
        XrSpace stage_space = XR_NULL_HANDLE;
        {
            xrEnumerateReferenceSpaces(session, 0, &count, nullptr);
            std::vector<XrReferenceSpaceType> spaces(count);
            xrEnumerateReferenceSpaces(session, count, &count, spaces.data());
            if (std::find(spaces.begin(), spaces.end(), XR_REFERENCE_SPACE_TYPE_STAGE) !=
                spaces.end()) {
                space_info.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
                xrCreateReferenceSpace(session, &space_info, &stage_space);
            }
        }
        std::printf("floor: %s\n", stage_space != XR_NULL_HANDLE
                                       ? "from the STAGE space"
                                       : "no STAGE space, taken 1.6 m below the start");

        XrHandTrackerEXT trackers[2] = {XR_NULL_HANDLE, XR_NULL_HANDLE};
        for (int hand = 0; hand < 2; ++hand) {
            XrHandTrackerCreateInfoEXT tracker_info{XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT};
            tracker_info.hand = hand == 0 ? XR_HAND_LEFT_EXT : XR_HAND_RIGHT_EXT;
            tracker_info.handJointSet = XR_HAND_JOINT_SET_DEFAULT_EXT;
            result = create_hand_tracker(session, &tracker_info, &trackers[hand]);
            if (XR_FAILED(result)) {
                std::printf("xrCreateHandTrackerEXT: %s\n", ResultName(instance, result));
            }
        }

        // --- Swapchains --------------------------------------------------------------------
        constexpr XrViewConfigurationType ViewConfiguration =
            XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
        xrEnumerateViewConfigurationViews(instance, system, ViewConfiguration, 0, &count, nullptr);
        std::vector<XrViewConfigurationView> configuration_views(
            count, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
        xrEnumerateViewConfigurationViews(instance, system, ViewConfiguration, count, &count,
                                          configuration_views.data());

        xrEnumerateSwapchainFormats(session, 0, &count, nullptr);
        std::vector<int64_t> formats(count);
        xrEnumerateSwapchainFormats(session, count, &count, formats.data());
        DXGI_FORMAT format = DXGI_FORMAT_UNKNOWN;
        for (const DXGI_FORMAT wanted :
             {DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, DXGI_FORMAT_B8G8R8A8_UNORM_SRGB,
              DXGI_FORMAT_R8G8B8A8_UNORM, DXGI_FORMAT_B8G8R8A8_UNORM}) {
            if (std::find(formats.begin(), formats.end(), wanted) != formats.end()) {
                format = wanted;
                break;
            }
        }
        if (format == DXGI_FORMAT_UNKNOWN && !formats.empty()) {
            format = static_cast<DXGI_FORMAT>(formats.front());
        }

        std::vector<Eye> eyes(configuration_views.size());
        bool swapchains_ok = !eyes.empty();
        for (size_t i = 0; i < eyes.size() && swapchains_ok; ++i) {
            Eye& eye = eyes[i];
            eye.width = static_cast<int>(configuration_views[i].recommendedImageRectWidth);
            eye.height = static_cast<int>(configuration_views[i].recommendedImageRectHeight);
            XrSwapchainCreateInfo swapchain_info{XR_TYPE_SWAPCHAIN_CREATE_INFO};
            swapchain_info.usageFlags =
                XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
            swapchain_info.format = format;
            swapchain_info.sampleCount = 1;
            swapchain_info.width = eye.width;
            swapchain_info.height = eye.height;
            swapchain_info.faceCount = 1;
            swapchain_info.arraySize = 1;
            swapchain_info.mipCount = 1;
            result = xrCreateSwapchain(session, &swapchain_info, &eye.swapchain);
            if (XR_FAILED(result)) {
                std::printf("xrCreateSwapchain: %s\n", ResultName(instance, result));
                swapchains_ok = false;
                break;
            }
            xrEnumerateSwapchainImages(eye.swapchain, 0, &count, nullptr);
            std::vector<XrSwapchainImageD3D11KHR> images(count,
                                                         {XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR});
            xrEnumerateSwapchainImages(eye.swapchain, count, &count,
                                       reinterpret_cast<XrSwapchainImageBaseHeader*>(images.data()));
            for (const auto& image : images) {
                D3D11_RENDER_TARGET_VIEW_DESC target_desc{};
                target_desc.Format = format;
                target_desc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2D;
                ID3D11RenderTargetView* target = nullptr;
                if (FAILED(device->CreateRenderTargetView(image.texture, &target_desc, &target))) {
                    swapchains_ok = false;
                }
                eye.targets.push_back(target);
            }
            D3D11_TEXTURE2D_DESC depth_desc{};
            depth_desc.Width = eye.width;
            depth_desc.Height = eye.height;
            depth_desc.MipLevels = 1;
            depth_desc.ArraySize = 1;
            depth_desc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
            depth_desc.SampleDesc.Count = 1;
            depth_desc.Usage = D3D11_USAGE_DEFAULT;
            depth_desc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
            if (FAILED(device->CreateTexture2D(&depth_desc, nullptr, &eye.depth_texture)) ||
                FAILED(device->CreateDepthStencilView(eye.depth_texture, nullptr, &eye.depth))) {
                swapchains_ok = false;
            }
        }
        if (swapchains_ok) {
            std::printf("%zu views of %dx%d, format %d\n", eyes.size(), eyes[0].width,
                        eyes[0].height, static_cast<int>(format));
        }

        xrEnumerateEnvironmentBlendModes(instance, system, ViewConfiguration, 0, &count, nullptr);
        std::vector<XrEnvironmentBlendMode> blend_modes(count);
        xrEnumerateEnvironmentBlendModes(instance, system, ViewConfiguration, count, &count,
                                         blend_modes.data());
        const XrEnvironmentBlendMode blend_mode =
            blend_modes.empty() ? XR_ENVIRONMENT_BLEND_MODE_OPAQUE : blend_modes.front();

        std::printf("\nkeys (in this window): T/G tilt +-2 deg, A tilt to match the sensors, "
                    "S smoothing, R reset smoothing and sensors, F sensors/hands model, "
                    "H hand joints, Esc quit\n\n");

        // --- The gamepad's sensors ---------------------------------------------------------
        // (Read while the console, not a window of SDL's, has the focus.)
        SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
        Sensors sensors;
        if (!use_gamepad) {
            std::printf("gamepad: not looked for (--no-gamepad): the hands alone\n");
        } else if (!SDL_Init(SDL_INIT_GAMEPAD)) {
            std::printf("SDL: %s: the hands alone\n", SDL_GetError());
        } else {
            std::printf("gamepad: waiting for one with motion sensors (USB or Bluetooth)\n");
        }

        // --- Frame loop --------------------------------------------------------------------
        HeldPadFollow::Follower follower;
        HeldPadFusion::Agreement agreement;
        bool show_hands = true;
        bool sensors_first = true;
        // One hand.
        HeldPadOneHand::GripDetector grip_detectors[2];
        HeldPadOneHand::Grip grips[2]{HeldPadOneHand::Grip{0}, HeldPadOneHand::Grip{1}};
        HeldPadOneHand::Smoother one_smoother;
        int last_hand = -1;
        enum class Last { None, Two, One } last_kind = Last::None;
        XrTime previous_time = 0;
        XrSessionState state = XR_SESSION_STATE_UNKNOWN;
        bool running = false;
        bool exit_requested = false;
        bool done = !swapchains_ok;
        float floor = -1.6f;
        auto last_report = std::chrono::steady_clock::now();
        std::vector<XrView> views(eyes.size(), {XR_TYPE_VIEW});

        while (!done) {
            // (Also while the session waits: the gamepad is found, its sensors keep count.)
            if (use_gamepad) {
                sensors.Pump();
            }
            for (const WORD key : KeysPressed(console_input)) {
                switch (key) {
                case VK_ESCAPE:
                    quit_requested = true;
                    break;
                case 'T':
                case 'G':
                    follower.tilt += (key == 'T' ? 2.0f : -2.0f) * Degrees;
                    std::printf("tilt %+.0f deg\n", follower.tilt / Degrees);
                    break;
                case 'S':
                    follower.smoothing = !follower.smoothing;
                    std::printf("smoothing %s\n", follower.smoothing ? "on" : "off");
                    break;
                case 'A':
                    if (agreement.Any()) {
                        follower.tilt = std::round(agreement.Mean().pitch / Degrees) * Degrees;
                        std::printf("tilt %+.0f deg, to match the sensors\n",
                                    follower.tilt / Degrees);
                    } else {
                        std::printf("no tilt to match yet: hold the gamepad still in both hands, "
                                    "with its sensors on\n");
                    }
                    break;
                case 'R':
                    follower.Reset();
                    sensors.imu.Reset();
                    agreement.Reset();
                    grips[0].Forget(0);
                    grips[1].Forget(1);
                    one_smoother.Reset();
                    std::printf("smoothing and sensors reset\n");
                    break;
                case 'F':
                    sensors_first = !sensors_first;
                    std::printf("solid model posed by %s\n",
                                sensors_first ? "the sensors" : "the hands");
                    break;
                case 'H':
                    show_hands = !show_hands;
                    std::printf("hand joints %s\n", show_hands ? "shown" : "hidden");
                    break;
                default:
                    break;
                }
            }
            if (quit_requested && !exit_requested) {
                exit_requested = true;
                if (running) {
                    xrRequestExitSession(session);
                } else {
                    done = true;
                }
            }

            XrEventDataBuffer event{XR_TYPE_EVENT_DATA_BUFFER};
            while (xrPollEvent(instance, &event) == XR_SUCCESS) {
                if (event.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
                    state = reinterpret_cast<const XrEventDataSessionStateChanged&>(event).state;
                    static const char* const StateNames[] = {
                        "unknown", "idle (is the headset on?)", "ready", "synchronized",
                        "visible", "focused", "stopping", "loss pending", "exiting"};
                    std::printf("session: %s\n",
                                state >= 0 && state <= 8 ? StateNames[state] : "?");
                    if (state == XR_SESSION_STATE_READY) {
                        XrSessionBeginInfo begin_info{XR_TYPE_SESSION_BEGIN_INFO};
                        begin_info.primaryViewConfigurationType = ViewConfiguration;
                        result = xrBeginSession(session, &begin_info);
                        std::printf("session begins: %s\n", ResultName(instance, result));
                        running = XR_SUCCEEDED(result);
                    } else if (state == XR_SESSION_STATE_STOPPING) {
                        xrEndSession(session);
                        running = false;
                        if (exit_requested) {
                            done = true;
                        }
                    } else if (state == XR_SESSION_STATE_EXITING ||
                               state == XR_SESSION_STATE_LOSS_PENDING) {
                        done = true;
                    }
                } else if (event.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) {
                    done = true;
                }
                event = {XR_TYPE_EVENT_DATA_BUFFER};
            }
            if (done) {
                break;
            }
            if (!running) {
                Sleep(10);
                continue;
            }

            XrFrameWaitInfo wait_info{XR_TYPE_FRAME_WAIT_INFO};
            XrFrameState frame{XR_TYPE_FRAME_STATE};
            if (XR_FAILED(xrWaitFrame(session, &wait_info, &frame))) {
                Sleep(10);
                continue;
            }
            XrFrameBeginInfo frame_begin{XR_TYPE_FRAME_BEGIN_INFO};
            xrBeginFrame(session, &frame_begin);
            const XrTime time = frame.predictedDisplayTime;

            // Floor.
            if (stage_space != XR_NULL_HANDLE) {
                XrSpaceLocation stage{XR_TYPE_SPACE_LOCATION};
                if (XR_SUCCEEDED(xrLocateSpace(stage_space, local_space, time, &stage)) &&
                    (stage.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0) {
                    floor = stage.pose.position.y;
                }
            }

            // Hands -> pose.
            HandView hands[2];
            std::optional<HeldPadFollow::Palm> palms[2];
            for (int hand = 0; hand < 2; ++hand) {
                if (trackers[hand] == XR_NULL_HANDLE) {
                    continue;
                }
                XrHandJointLocationsEXT locations{XR_TYPE_HAND_JOINT_LOCATIONS_EXT};
                locations.jointCount = XR_HAND_JOINT_COUNT_EXT;
                locations.jointLocations = hands[hand].joints;
                XrHandJointsLocateInfoEXT locate_info{XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT};
                locate_info.baseSpace = local_space;
                locate_info.time = time;
                if (XR_SUCCEEDED(locate_hand_joints(trackers[hand], &locate_info, &locations))) {
                    hands[hand].active = locations.isActive == XR_TRUE;
                }
                const auto& palm = hands[hand].joints[XR_HAND_JOINT_PALM_EXT];
                palms[hand] = HeldPadFollow::PalmFrom(
                    hands[hand].active,
                    (palm.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0,
                    (palm.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0,
                    {palm.pose.position.x, palm.pose.position.y, palm.pose.position.z},
                    {palm.pose.orientation.x, palm.pose.orientation.y, palm.pose.orientation.z,
                     palm.pose.orientation.w});
            }
            const auto& shown =
                follower.Update(HeldPadFollow::HandsFrom(palms[0], palms[1]), time * 1e-9);

            // The sensors, with the heading the hands give.
            if (shown.live && shown.turned_known) {
                sensors.imu.HandYaw(shown.yaw, Sensors::Now());
            }
            const bool fused = sensors.gyro && sensors.imu.Valid() && shown.ever;
            const Quat fused_orientation = sensors.imu.Attitude();
            const Vec fused_position = HeldPadFusion::CentreFrom(shown.middle, fused_orientation);
            const float frame_time =
                previous_time != 0 && time > previous_time ? (time - previous_time) * 1e-9f : 0.0f;
            previous_time = time;
            if (fused && shown.live && shown.turned_known && shown.pitch_measured &&
                sensors.imu.Steady()) {
                agreement.Add({shown.yaw, shown.pitch, RollOf(shown.orientation)},
                              HeldPadFusion::AnglesOf(fused_orientation), frame_time);
            }

            // One hand: which hands are closed around something, and where each holds the
            // gamepad while both do.
            const bool sensors_on = sensors.gyro && sensors.imu.Valid();
            float curls[2] = {-1.0f, -1.0f};
            bool gripping[2] = {};
            std::optional<HeldPadOneHand::Palm> palm_poses[2];
            for (int hand = 0; hand < 2; ++hand) {
                if (hands[hand].active) {
                    HeldPadOneHand::Joint joints[HeldPadOneHand::JointCount];
                    for (int j = 0; j < HeldPadOneHand::JointCount; ++j) {
                        const auto& joint = hands[hand].joints[j];
                        joints[j] = {
                            (joint.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0,
                            {joint.pose.position.x, joint.pose.position.y, joint.pose.position.z}};
                    }
                    curls[hand] = HeldPadOneHand::GripCurl(joints);
                }
                gripping[hand] = grip_detectors[hand].Update(curls[hand]);
                if (palms[hand]) {
                    const auto& palm = hands[hand].joints[XR_HAND_JOINT_PALM_EXT];
                    HeldPadOneHand::Palm pose;
                    pose.position = palms[hand]->position;
                    pose.orientation_valid =
                        (palm.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0;
                    pose.orientation = {palm.pose.orientation.x, palm.pose.orientation.y,
                                        palm.pose.orientation.z, palm.pose.orientation.w};
                    palm_poses[hand] = pose;
                }
            }
            std::optional<HeldPadOneHand::Held> one;
            if (shown.live) {
                if (sensors_on || shown.turned_known) {
                    const Quat pad = sensors_on ? fused_orientation : shown.orientation;
                    const Vec centre = sensors_on ? fused_position : shown.position;
                    for (int hand = 0; hand < 2; ++hand) {
                        if (palm_poses[hand]) {
                            grips[hand].Learn(*palm_poses[hand], centre, pad, frame_time);
                        }
                    }
                }
                last_kind = Last::Two;
            } else {
                one = HeldPadOneHand::Locate(
                    palm_poses, gripping, grips,
                    sensors_on ? std::optional<Quat>{fused_orientation} : std::nullopt, last_hand);
                if (one) {
                    one_smoother.Update(one->position, one->orientation, time * 1e-9,
                                        follower.smoothing);
                    last_hand = one->hand;
                    last_kind = Last::One;
                    if (one->hand_turn_known) {
                        // The one hand keeps the sensors' heading from drifting.
                        sensors.imu.HandYaw(HeldPadFusion::AnglesOf(one->hand_orientation).yaw,
                                            Sensors::Now());
                    }
                }
            }

            // Readout.
            const auto now = std::chrono::steady_clock::now();
            if (now - last_report > std::chrono::milliseconds{200}) {
                last_report = now;
                char distance[16] = "  --  ";
                if (shown.palm_distance >= 0.0f) {
                    std::snprintf(distance, sizeof(distance), "%5.1fcm", shown.palm_distance * 100);
                }
                char grip_text[2][16];
                for (int hand = 0; hand < 2; ++hand) {
                    if (curls[hand] < 0.0f) {
                        std::snprintf(grip_text[hand], sizeof(grip_text[hand]), " -- %c ",
                                      grips[hand].Known() ? 'k' : ' ');
                    } else {
                        std::snprintf(grip_text[hand], sizeof(grip_text[hand]), "%.2f%c%c",
                                      curls[hand], gripping[hand] ? '*' : ' ',
                                      grips[hand].Known() ? 'k' : ' ');
                    }
                }
                const char* state = shown.live ? (shown.turned_known ? "turned" : "placed")
                                    : one      ? (one->hand == 0 ? "left" : "right")
                                    : last_kind != Last::None ? "held"
                                                              : "none";
                std::printf("hands L%c%c R%c%c  curl L%s R%s  palms %s  %-6s  yaw %+6.1f  "
                            "pitch %+6.1f (%s)  roll %+6.1f  at %+.2f %+.2f %+.2f\n",
                            hands[0].active ? '+' : '-',
                            palms[0] ? (palms[0]->pointing_known ? 'p' : 'o') : ' ',
                            hands[1].active ? '+' : '-',
                            palms[1] ? (palms[1]->pointing_known ? 'p' : 'o') : ' ',
                            grip_text[0], grip_text[1], distance, state,
                            shown.yaw / Degrees, shown.pitch / Degrees,
                            shown.pitch_measured ? "measured"
                            : shown.live         ? "assumed "
                                                 : "        ",
                            RollOf(shown.orientation) / Degrees, shown.position.x,
                            shown.position.y, shown.position.z);
                if (sensors.gyro) {
                    const auto imu = HeldPadFusion::AnglesOf(fused_orientation);
                    const auto& mean = agreement.Mean();
                    char difference[112] = "";
                    if (agreement.Any()) {
                        std::snprintf(difference, sizeof(difference),
                                      "  sensors-hands yaw %+5.1f pitch %+5.1f roll %+5.1f"
                                      "  (tilt to match %+.0f)",
                                      mean.yaw / Degrees, mean.pitch / Degrees,
                                      mean.roll / Degrees, mean.pitch / Degrees);
                    }
                    std::printf("  sensors %-8s %4d/s  yaw %+6.1f  pitch %+6.1f  roll %+6.1f  "
                                "%-6s %-7s%s\n",
                                !sensors.imu.Valid() ? "starting" : "on", sensors.gyro_readings * 5,
                                imu.yaw / Degrees, imu.pitch / Degrees, imu.roll / Degrees,
                                sensors.imu.Steady() ? "still" : "moving",
                                sensors.imu.HasYawReference(Sensors::Now()) ? "heading" : "",
                                difference);
                }
                sensors.gyro_readings = 0;
            }

            std::vector<XrCompositionLayerProjectionView> projection_views(
                eyes.size(), {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW});
            XrCompositionLayerProjection layer{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
            bool rendered = false;
            if (frame.shouldRender) {
                XrViewLocateInfo view_info{XR_TYPE_VIEW_LOCATE_INFO};
                view_info.viewConfigurationType = ViewConfiguration;
                view_info.displayTime = time;
                view_info.space = local_space;
                XrViewState view_state{XR_TYPE_VIEW_STATE};
                uint32_t view_count = 0;
                if (XR_SUCCEEDED(xrLocateViews(session, &view_info, &view_state,
                                               static_cast<uint32_t>(views.size()), &view_count,
                                               views.data())) &&
                    view_count == eyes.size() &&
                    (view_state.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) != 0 &&
                    (view_state.viewStateFlags & XR_VIEW_STATE_POSITION_VALID_BIT) != 0) {
                    Scene scene;
                    AddRoom(scene, floor);
                    if (show_hands) {
                        AddHands(scene, hands, palms);
                    }
                    const Color one_hand_color{0.72f, 0.42f, 0.95f};
                    if (!shown.live && last_kind == Last::One) {
                        // In one hand, or last seen in one: turned by the sensors if there
                        // are any, by the hand otherwise.
                        const Quat orientation =
                            sensors_on ? fused_orientation : one_smoother.Orientation();
                        AddGamepad(scene, one_smoother.Position(), orientation,
                                   one ? one_hand_color
                                   : sensors_on ? Color{0.4f, 0.46f, 0.58f}
                                                : Color{0.42f, 0.42f, 0.42f});
                        if (one && one->turned_by_sensors && one->hand_turn_known) {
                            AddAxes(scene, one_smoother.Position(), one->hand_orientation, 0.14f,
                                    0.002f, 0.5f);
                        }
                    } else if (fused && sensors_first) {
                        AddGamepad(scene, fused_position, fused_orientation, SensorsColor(shown));
                        AddAxes(scene, shown.position, shown.orientation, 0.14f, 0.002f, 0.5f);
                    } else if (shown.ever) {
                        AddGamepad(scene, shown.position, shown.orientation, HandsColor(shown));
                        if (fused) {
                            AddAxes(scene, fused_position, fused_orientation, 0.14f, 0.002f,
                                    0.5f);
                        }
                    }
                    renderer.Upload(context, scene);

                    for (size_t i = 0; i < eyes.size(); ++i) {
                        Eye& eye = eyes[i];
                        uint32_t index = 0;
                        XrSwapchainImageAcquireInfo acquire{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
                        xrAcquireSwapchainImage(eye.swapchain, &acquire, &index);
                        XrSwapchainImageWaitInfo image_wait{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
                        image_wait.timeout = XR_INFINITE_DURATION;
                        xrWaitSwapchainImage(eye.swapchain, &image_wait);
                        renderer.Draw(context, eye.targets[index], eye.depth, eye.width,
                                      eye.height,
                                      Multiply(Projection(views[i].fov, 0.02f, 50.0f),
                                               ViewFrom(views[i].pose)));
                        XrSwapchainImageReleaseInfo release{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
                        xrReleaseSwapchainImage(eye.swapchain, &release);

                        projection_views[i].pose = views[i].pose;
                        projection_views[i].fov = views[i].fov;
                        projection_views[i].subImage.swapchain = eye.swapchain;
                        projection_views[i].subImage.imageRect = {{0, 0}, {eye.width, eye.height}};
                    }
                    layer.space = local_space;
                    layer.viewCount = static_cast<uint32_t>(projection_views.size());
                    layer.views = projection_views.data();
                    rendered = true;
                }
            }
            const XrCompositionLayerBaseHeader* layers[] = {
                reinterpret_cast<const XrCompositionLayerBaseHeader*>(&layer)};
            XrFrameEndInfo end_info{XR_TYPE_FRAME_END_INFO};
            end_info.displayTime = time;
            end_info.environmentBlendMode = blend_mode;
            end_info.layerCount = rendered ? 1 : 0;
            end_info.layers = layers;
            result = xrEndFrame(session, &end_info);
            if (XR_FAILED(result)) {
                std::printf("xrEndFrame: %s\n", ResultName(instance, result));
            }
        }

        // --- Teardown ----------------------------------------------------------------------
        sensors.Close();
        SDL_Quit();
        for (Eye& eye : eyes) {
            for (auto*& target : eye.targets) {
                Release(target);
            }
            Release(eye.depth);
            Release(eye.depth_texture);
            if (eye.swapchain != XR_NULL_HANDLE) {
                xrDestroySwapchain(eye.swapchain);
            }
        }
        for (XrHandTrackerEXT tracker : trackers) {
            if (tracker != XR_NULL_HANDLE) {
                destroy_hand_tracker(tracker);
            }
        }
        if (stage_space != XR_NULL_HANDLE) {
            xrDestroySpace(stage_space);
        }
        if (local_space != XR_NULL_HANDLE) {
            xrDestroySpace(local_space);
        }
        xrDestroySession(session);
    }
    Release(context);
    Release(device);
    xrDestroyInstance(instance);
    finished = true;
    return 0;
}
