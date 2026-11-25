#include "Collision.h"

#include <algorithm>
#include <cmath>

namespace softphysics::physics
{
    std::optional<Contact> collide_sphere_aabb(
        const SphereCollider& sphere,
        const Aabb& box,
        Vector3 sphereVelocity)
    {
        if (!std::isfinite(sphere.radius) || sphere.radius <= 0.0f)
        {
            return std::nullopt;
        }

        const Vector3 closest{
            std::clamp(sphere.center.x, box.minimum.x, box.maximum.x),
            std::clamp(sphere.center.y, box.minimum.y, box.maximum.y),
            std::clamp(sphere.center.z, box.minimum.z, box.maximum.z)};
        Vector3 offset{
            sphere.center.x - closest.x,
            sphere.center.y - closest.y,
            sphere.center.z - closest.z};
        float distanceSquared = offset.x * offset.x + offset.y * offset.y + offset.z * offset.z;
        Contact contact;
        contact.point = closest;
        contact.relativeVelocity = sphereVelocity;

        if (distanceSquared > 1.0e-12f)
        {
            const float distance = std::sqrt(distanceSquared);
            if (distance >= sphere.radius)
            {
                return std::nullopt;
            }
            const float inverseDistance = 1.0f / distance;
            contact.normal = {
                offset.x * inverseDistance,
                offset.y * inverseDistance,
                offset.z * inverseDistance};
            contact.penetration = sphere.radius - distance;
            return contact;
        }

        // If the sphere center is inside the proxy, resolve along its nearest face.
        const float faceDistances[] = {
            sphere.center.x - box.minimum.x,
            box.maximum.x - sphere.center.x,
            sphere.center.y - box.minimum.y,
            box.maximum.y - sphere.center.y,
            sphere.center.z - box.minimum.z,
            box.maximum.z - sphere.center.z};
        const auto nearestFace = static_cast<std::size_t>(
            std::min_element(std::begin(faceDistances), std::end(faceDistances)) - faceDistances);
        switch (nearestFace)
        {
        case 0: contact.normal = {-1.0f, 0.0f, 0.0f}; contact.point.x = box.minimum.x; break;
        case 1: contact.normal = {1.0f, 0.0f, 0.0f}; contact.point.x = box.maximum.x; break;
        case 2: contact.normal = {0.0f, -1.0f, 0.0f}; contact.point.y = box.minimum.y; break;
        case 3: contact.normal = {0.0f, 1.0f, 0.0f}; contact.point.y = box.maximum.y; break;
        case 4: contact.normal = {0.0f, 0.0f, -1.0f}; contact.point.z = box.minimum.z; break;
        default: contact.normal = {0.0f, 0.0f, 1.0f}; contact.point.z = box.maximum.z; break;
        }
        contact.penetration = sphere.radius + faceDistances[nearestFace];
        return contact;
    }
}
