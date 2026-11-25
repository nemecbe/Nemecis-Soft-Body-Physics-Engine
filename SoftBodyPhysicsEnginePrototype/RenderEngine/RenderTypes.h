// Window Specific Types

#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace softphysics::renderengine
{
    struct LinearColor
    {
        float r;
        float g;
        float b;
        float a;
    };

    // Portable CPU-side mesh description. Positions are in model space and are
    // transformed by Transform3D before the camera projection is applied.
    struct MeshVertex
    {
        std::array<float, 3> position{};
        std::array<float, 3> normal{0.0f, 0.0f, 1.0f};
        LinearColor color{1.0f, 1.0f, 1.0f, 1.0f};
    };

    struct MeshData
    {
        std::vector<MeshVertex> vertices;
        // Empty indices means draw vertices in order; otherwise draw indexed triangles.
        std::vector<std::uint32_t> indices;
    };

    struct MeshHandle
    {
        std::uint32_t id = 0;

        [[nodiscard]] explicit operator bool() const noexcept { return id != 0; }
    };

    struct Vector3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct Transform3D
    {
        Vector3 position{};
        // Euler angles in radians, applied X then Y then Z.
        Vector3 rotationRadians{};
        Vector3 scale{1.0f, 1.0f, 1.0f};
    };

    struct Camera3D
    {
        // Engine coordinates are left-handed: +Y is up and +Z is forward.
        Vector3 position{0.0f, 0.0f, -3.5f};
        Vector3 target{};
        Vector3 up{0.0f, 1.0f, 0.0f};
        float verticalFovRadians = 1.04719755f;
        float nearPlane = 0.1f;
        float farPlane = 100.0f;
    };
};
