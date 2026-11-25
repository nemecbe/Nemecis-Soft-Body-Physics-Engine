#pragma once

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
    };
}
