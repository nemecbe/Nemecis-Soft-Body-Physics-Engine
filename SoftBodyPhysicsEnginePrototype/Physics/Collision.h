#pragma once

#include <optional>

#include "PhysicsTypes.h"

namespace softphysics::physics
{
    // Broad-phase proxy used for the first torso/glove collision experiment.
    [[nodiscard]] std::optional<Contact> collide_sphere_aabb(
        const SphereCollider& sphere,
        const Aabb& box,
        Vector3 sphereVelocity);
}
