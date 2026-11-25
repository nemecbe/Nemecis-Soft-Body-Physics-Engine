#pragma once

#include <array>
#include <cstdint>

namespace softphysics::physics
{
    struct Vector3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct Aabb
    {
        Vector3 minimum{};
        Vector3 maximum{};
    };

    struct SphereCollider
    {
        Vector3 center{};
        float radius = 0.0f;
    };

    struct Contact
    {
        Vector3 point{};
        // Points from the torso surface toward the sphere; body response uses -normal.
        Vector3 normal{};
        Vector3 relativeVelocity{};
        float penetration = 0.0f;
        float normalImpulse = 0.0f;
        // Tangential impulse transferred to the body models the glove's hook/shear motion.
        Vector3 tangentialImpulse{};
        std::array<std::uint32_t, 3> surfaceVertices{};
        std::array<float, 3> surfaceWeights{};
        Vector3 colliderCenter{};
        float colliderRadius = 0.0f;
        float maximumContactForceNewtons = 12.0f;
        bool hasSurfaceFeature = false;
    };
}
