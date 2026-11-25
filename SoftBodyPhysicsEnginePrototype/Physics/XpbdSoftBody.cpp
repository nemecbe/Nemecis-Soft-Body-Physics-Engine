#include "XpbdSoftBody.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace softphysics::physics
{
    namespace
    {
        Vector3 add(Vector3 a, Vector3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
        Vector3 subtract(Vector3 a, Vector3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
        Vector3 multiply(Vector3 value, float scalar)
        {
            return {value.x * scalar, value.y * scalar, value.z * scalar};
        }
        float dot(Vector3 a, Vector3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
        Vector3 cross(Vector3 a, Vector3 b)
        {
            return {
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
        }
        float length(Vector3 value) { return std::sqrt(dot(value, value)); }
        float tetrahedron_volume(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
        {
            return dot(subtract(b, a), cross(subtract(c, a), subtract(d, a))) / 6.0f;
        }

        struct TrianglePoint
        {
            Vector3 point{};
            std::array<float, 3> weights{};
        };

        TrianglePoint closest_point_on_triangle(Vector3 point, Vector3 a, Vector3 b, Vector3 c)
        {
            const Vector3 ab = subtract(b, a);
            const Vector3 ac = subtract(c, a);
            const Vector3 ap = subtract(point, a);
            const float d1 = dot(ab, ap);
            const float d2 = dot(ac, ap);
            if (d1 <= 0.0f && d2 <= 0.0f) return {a, {1.0f, 0.0f, 0.0f}};

            const Vector3 bp = subtract(point, b);
            const float d3 = dot(ab, bp);
            const float d4 = dot(ac, bp);
            if (d3 >= 0.0f && d4 <= d3) return {b, {0.0f, 1.0f, 0.0f}};

            const float vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
            {
                const float v = d1 / (d1 - d3);
                return {add(a, multiply(ab, v)), {1.0f - v, v, 0.0f}};
            }

            const Vector3 cp = subtract(point, c);
            const float d5 = dot(ab, cp);
            const float d6 = dot(ac, cp);
            if (d6 >= 0.0f && d5 <= d6) return {c, {0.0f, 0.0f, 1.0f}};

            const float vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
            {
                const float w = d2 / (d2 - d6);
                return {add(a, multiply(ac, w)), {1.0f - w, 0.0f, w}};
            }

            const float va = d3 * d6 - d5 * d4;
            if (va <= 0.0f && d4 - d3 >= 0.0f && d5 - d6 >= 0.0f)
            {
                const Vector3 bc = subtract(c, b);
                const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                return {add(b, multiply(bc, w)), {0.0f, 1.0f - w, w}};
            }

            const float inverse = 1.0f / (va + vb + vc);
            const float v = vb * inverse;
            const float w = vc * inverse;
            return {add(add(a, multiply(ab, v)), multiply(ac, w)), {1.0f - v - w, v, w}};
        }

        std::uint64_t edge_key(std::uint32_t a, std::uint32_t b)
        {
            if (a > b)
            {
                std::swap(a, b);
            }
            return (static_cast<std::uint64_t>(a) << 32) | b;
        }
    }

    bool XpbdSoftBody::initialize(
        std::span<const Vector3> surfaceVertices,
        std::span<const std::uint32_t> triangleIndices,
        const SoftBodyConfig& config,
        std::string& errorMessage)
    {
        initialized_ = false;
        if (surfaceVertices.size() < 4 || triangleIndices.size() < 12 ||
            triangleIndices.size() % 3 != 0 || !std::isfinite(config.totalMassKg) ||
            config.totalMassKg <= 0.0f || config.contactRadius <= 0.0f)
        {
            errorMessage = "A soft body needs a closed triangle surface and positive mass/radius.";
            return false;
        }

        config_ = config;
        surfaceVertexCount_ = surfaceVertices.size();
        surfaceIndices_.assign(triangleIndices.begin(), triangleIndices.end());
        pendingContacts_.clear();
        supportReleased_ = false;
        positions_.assign(surfaceVertices.begin(), surfaceVertices.end());
        restPositions_ = positions_;
        Vector3 center{};
        float minimumY = std::numeric_limits<float>::max();
        for (const Vector3 point : surfaceVertices)
        {
            center = add(center, point);
            minimumY = std::min(minimumY, point.y);
        }
        center = multiply(center, 1.0f / static_cast<float>(surfaceVertices.size()));
        positions_.push_back(center);
        restPositions_.push_back(center);
        groundY_ = minimumY;

        tetrahedra_.clear();
        tetrahedra_.reserve(triangleIndices.size() / 3);
        float totalVolume = 0.0f;
        const std::uint32_t centerIndex = static_cast<std::uint32_t>(surfaceVertexCount_);
        for (std::size_t index = 0; index < triangleIndices.size(); index += 3)
        {
            const auto a = triangleIndices[index];
            auto b = triangleIndices[index + 1];
            auto c = triangleIndices[index + 2];
            if (a >= surfaceVertexCount_ || b >= surfaceVertexCount_ || c >= surfaceVertexCount_)
            {
                errorMessage = "The surface mesh contains an index outside its vertex range.";
                return false;
            }

            float volume = tetrahedron_volume(center, positions_[a], positions_[b], positions_[c]);
            if (volume < 0.0f)
            {
                std::swap(b, c);
                volume = -volume;
            }
            if (!std::isfinite(volume) || volume < 1.0e-12f)
            {
                errorMessage = "The torso surface cannot form a stable center-fan tetrahedral mesh.";
                return false;
            }
            tetrahedra_.push_back({centerIndex, a, b, c, volume, 0.0f});
            totalVolume += volume;
        }

        if (!std::isfinite(totalVolume) || totalVolume <= 0.0f)
        {
            errorMessage = "The generated tetrahedral mesh has no usable volume.";
            return false;
        }

        // Distribute body mass by tetrahedron volume, then evenly across its four nodes.
        std::vector<float> masses(positions_.size(), 0.0f);
        for (const auto& tetrahedron : tetrahedra_)
        {
            const float nodeMass = config_.totalMassKg *
                                   (tetrahedron.restVolume / totalVolume) * 0.25f;
            masses[tetrahedron.a] += nodeMass;
            masses[tetrahedron.b] += nodeMass;
            masses[tetrahedron.c] += nodeMass;
            masses[tetrahedron.d] += nodeMass;
        }

        velocities_.assign(positions_.size(), {});
        externalForces_.assign(positions_.size(), {});
        inverseMasses_.resize(positions_.size());
        freeInverseMasses_.resize(positions_.size());
        anchoredVertices_.assign(positions_.size(), false);
        for (std::size_t index = 0; index < positions_.size(); ++index)
        {
            inverseMasses_[index] = masses[index] > 1.0e-8f ? 1.0f / masses[index] : 0.0f;
            freeInverseMasses_[index] = inverseMasses_[index];
            if (index < surfaceVertexCount_ &&
                positions_[index].y <= minimumY + config_.anchoredBottomBand)
            {
                // Temporary stand-in for attachment to a fighter's skeleton/legs.
                inverseMasses_[index] = 0.0f;
                anchoredVertices_[index] = true;
            }
        }

        distanceConstraints_.clear();
        std::unordered_set<std::uint64_t> knownEdges;
        const auto add_edge = [&](std::uint32_t a, std::uint32_t b)
        {
            if (knownEdges.insert(edge_key(a, b)).second)
            {
                distanceConstraints_.push_back({a, b, length(subtract(positions_[b], positions_[a])), 0.0f});
            }
        };
        for (const auto& tetrahedron : tetrahedra_)
        {
            const std::uint32_t points[] = {
                tetrahedron.a, tetrahedron.b, tetrahedron.c, tetrahedron.d};
            for (std::size_t first = 0; first < 4; ++first)
            {
                for (std::size_t second = first + 1; second < 4; ++second)
                {
                    add_edge(points[first], points[second]);
                }
            }
        }

        initialized_ = true;
        return true;
    }

    std::optional<Contact> XpbdSoftBody::collide_sphere(
        const SphereCollider& sphere,
        Vector3 sphereVelocity) const
    {
        if (!initialized_ || !std::isfinite(sphere.radius) || sphere.radius <= 0.0f)
        {
            return std::nullopt;
        }

        std::optional<Contact> closestContact;
        float closestDistanceSquared = sphere.radius * sphere.radius;
        for (std::size_t index = 0; index + 2 < surfaceIndices_.size(); index += 3)
        {
            const std::array<std::uint32_t, 3> vertices{
                surfaceIndices_[index], surfaceIndices_[index + 1], surfaceIndices_[index + 2]};
            if (vertices[0] >= surfaceVertexCount_ || vertices[1] >= surfaceVertexCount_ ||
                vertices[2] >= surfaceVertexCount_)
            {
                continue;
            }

            const auto closest = closest_point_on_triangle(
                sphere.center,
                positions_[vertices[0]],
                positions_[vertices[1]],
                positions_[vertices[2]]);
            const Vector3 separation = subtract(sphere.center, closest.point);
            const float distanceSquared = dot(separation, separation);
            if (distanceSquared >= closestDistanceSquared)
            {
                continue;
            }

            const float distance = std::sqrt(distanceSquared);
            Vector3 normal{};
            if (distance > 1.0e-7f)
            {
                normal = multiply(separation, 1.0f / distance);
            }
            else
            {
                normal = cross(
                    subtract(positions_[vertices[1]], positions_[vertices[0]]),
                    subtract(positions_[vertices[2]], positions_[vertices[0]]));
                const float normalLength = length(normal);
                if (normalLength <= 1.0e-8f)
                {
                    continue;
                }
                normal = multiply(normal, 1.0f / normalLength);
                const Vector3 triangleCenter = multiply(
                    add(add(positions_[vertices[0]], positions_[vertices[1]]),
                        positions_[vertices[2]]),
                    1.0f / 3.0f);
                if (dot(subtract(sphere.center, triangleCenter), normal) < 0.0f)
                {
                    normal = multiply(normal, -1.0f);
                }
            }

            Contact contact;
            contact.point = closest.point;
            contact.normal = normal;
            contact.relativeVelocity = sphereVelocity;
            contact.penetration = sphere.radius - distance;
            contact.surfaceVertices = vertices;
            contact.surfaceWeights = closest.weights;
            contact.colliderCenter = sphere.center;
            contact.colliderRadius = sphere.radius;
            contact.hasSurfaceFeature = true;
            closestDistanceSquared = distanceSquared;
            closestContact = contact;
        }
        return closestContact;
    }

    void XpbdSoftBody::release_support()
    {
        if (supportReleased_)
        {
            return;
        }
        supportReleased_ = true;
        for (std::size_t index = 0; index < anchoredVertices_.size(); ++index)
        {
            if (anchoredVertices_[index])
            {
                inverseMasses_[index] = freeInverseMasses_[index];
            }
        }
    }

    void XpbdSoftBody::add_contact(const Contact& contact)
    {
        if (!initialized_ || contact.penetration <= 0.0f)
        {
            return;
        }

        if (contact.hasSurfaceFeature)
        {
            pendingContacts_.push_back(contact);
        }

        const Vector3 totalImpulse = add(
            multiply(contact.normal, -contact.normalImpulse),
            contact.tangentialImpulse);
        if (length(totalImpulse) >= config_.balanceReleaseImpulseNs)
        {
            release_support();
        }

        Vector3 normal = contact.normal;
        const float normalLength = length(normal);
        if (!std::isfinite(normalLength) || normalLength < 1.0e-6f)
        {
            return;
        }
        normal = multiply(normal, 1.0f / normalLength);

        const float radiusSquared = config_.contactRadius * config_.contactRadius;
        std::vector<float> weights(surfaceVertexCount_, 0.0f);
        float totalWeight = 0.0f;
        for (std::size_t index = 0; index < surfaceVertexCount_; ++index)
        {
            if (inverseMasses_[index] == 0.0f)
            {
                continue;
            }
            const Vector3 offset = subtract(positions_[index], contact.point);
            const float distanceSquared = dot(offset, offset);
            if (distanceSquared > radiusSquared * 4.0f)
            {
                continue;
            }
            const float weight = std::exp(-distanceSquared / (2.0f * radiusSquared));
            weights[index] = weight;
            totalWeight += weight;
        }
        if (totalWeight <= 1.0e-8f)
        {
            return;
        }

        const Vector3 bodyDirection = multiply(normal, -1.0f);
        for (std::size_t index = 0; index < surfaceVertexCount_; ++index)
        {
            const float weight = weights[index] / totalWeight;
            if (weight <= 0.0f)
            {
                continue;
            }

            if (contact.normalImpulse > 0.0f)
            {
                const Vector3 impulse = add(
                    multiply(bodyDirection, contact.normalImpulse),
                    contact.tangentialImpulse);
                velocities_[index] = add(
                    velocities_[index],
                    multiply(impulse, weight * inverseMasses_[index]));
            }
            const float forceMagnitude = std::min(
                std::min(
                    config_.contactStiffness * contact.penetration,
                    config_.maximumContactForce),
                std::clamp(contact.maximumContactForceNewtons, 0.0f, config_.maximumContactForce));
            const Vector3 force = multiply(bodyDirection, forceMagnitude * weight);
            externalForces_[index] = add(externalForces_[index], force);
        }
    }

    void XpbdSoftBody::step(float fixedDeltaSeconds)
    {
        if (!initialized_ || !std::isfinite(fixedDeltaSeconds) || fixedDeltaSeconds <= 0.0f)
        {
            return;
        }

        const std::vector<Vector3> previousPositions = positions_;
        const float dampingFactor = std::exp(-std::max(config_.damping, 0.0f) * fixedDeltaSeconds);
        for (std::size_t index = 0; index < positions_.size(); ++index)
        {
            if (inverseMasses_[index] == 0.0f)
            {
                positions_[index] = restPositions_[index];
                velocities_[index] = {};
                externalForces_[index] = {};
                continue;
            }

            velocities_[index] = add(
                velocities_[index],
                multiply(externalForces_[index], inverseMasses_[index] * fixedDeltaSeconds));
            if (supportReleased_)
            {
                velocities_[index].y -=
                    config_.gravityMetersPerSecondSquared * fixedDeltaSeconds;
            }
            velocities_[index] = multiply(velocities_[index], dampingFactor);
            positions_[index] = add(positions_[index], multiply(velocities_[index], fixedDeltaSeconds));
            externalForces_[index] = {};
        }

        const float inverseDtSquared = 1.0f / (fixedDeltaSeconds * fixedDeltaSeconds);
        const float stretchAlpha = config_.stretchCompliance * inverseDtSquared;
        const float volumeAlpha = config_.volumeCompliance * inverseDtSquared;
        for (auto& constraint : distanceConstraints_)
        {
            constraint.lambda = 0.0f;
        }
        for (auto& tetrahedron : tetrahedra_)
        {
            tetrahedron.lambda = 0.0f;
        }

        for (std::uint32_t iteration = 0; iteration < config_.solverIterations; ++iteration)
        {
            for (auto& constraint : distanceConstraints_)
            {
                solve_distance(constraint, stretchAlpha);
            }
            for (auto& tetrahedron : tetrahedra_)
            {
                solve_volume(tetrahedron, volumeAlpha);
            }
            for (auto& contact : pendingContacts_)
            {
                solve_contact(contact);
            }
            if (supportReleased_)
            {
                for (std::size_t index = 0; index < surfaceVertexCount_; ++index)
                {
                    positions_[index].y = std::max(positions_[index].y, groundY_);
                }
            }
        }

        // Reconstruct velocity from corrected positions so constraints affect the next step.
        for (std::size_t index = 0; index < positions_.size(); ++index)
        {
            if (inverseMasses_[index] > 0.0f)
            {
                velocities_[index] = multiply(
                    subtract(positions_[index], previousPositions[index]),
                    dampingFactor / fixedDeltaSeconds);
            }
        }
        pendingContacts_.clear();
    }

    void XpbdSoftBody::reset() noexcept
    {
        if (!initialized_)
        {
            return;
        }

        std::copy(restPositions_.begin(), restPositions_.end(), positions_.begin());
        std::fill(velocities_.begin(), velocities_.end(), Vector3{});
        std::fill(externalForces_.begin(), externalForces_.end(), Vector3{});
        for (std::size_t index = 0; index < inverseMasses_.size(); ++index)
        {
            inverseMasses_[index] = anchoredVertices_[index]
                ? 0.0f
                : freeInverseMasses_[index];
        }
        for (auto& constraint : distanceConstraints_)
        {
            constraint.lambda = 0.0f;
        }
        for (auto& tetrahedron : tetrahedra_)
        {
            tetrahedron.lambda = 0.0f;
        }
        pendingContacts_.clear();
        supportReleased_ = false;
    }

    void XpbdSoftBody::solve_contact(Contact& contact)
    {
        if (!contact.hasSurfaceFeature)
        {
            return;
        }

        Vector3 surfacePoint{};
        float denominator = 0.0f;
        for (std::size_t index = 0; index < 3; ++index)
        {
            const auto vertex = contact.surfaceVertices[index];
            if (vertex >= surfaceVertexCount_)
            {
                return;
            }
            const float barycentric = std::max(contact.surfaceWeights[index], 0.0f);
            surfacePoint = add(surfacePoint, multiply(positions_[vertex], barycentric));
            denominator += inverseMasses_[vertex] * barycentric * barycentric;
        }
        if (denominator <= 1.0e-12f)
        {
            return;
        }

        const Vector3 separation = subtract(contact.colliderCenter, surfacePoint);
        const float distance = length(separation);
        if (!std::isfinite(distance) || distance >= contact.colliderRadius)
        {
            return;
        }
        Vector3 normal = contact.normal;
        if (distance > 1.0e-8f)
        {
            normal = multiply(separation, 1.0f / distance);
        }
        const float correction = (contact.colliderRadius - distance) / denominator;
        for (std::size_t index = 0; index < 3; ++index)
        {
            const auto vertex = contact.surfaceVertices[index];
            const float barycentric = std::max(contact.surfaceWeights[index], 0.0f);
            positions_[vertex] = subtract(
                positions_[vertex],
                multiply(normal, inverseMasses_[vertex] * barycentric * correction));
        }
    }

    void XpbdSoftBody::solve_distance(DistanceConstraint& constraint, float alpha)
    {
        const Vector3 difference = subtract(positions_[constraint.b], positions_[constraint.a]);
        const float currentLength = length(difference);
        if (currentLength < 1.0e-8f)
        {
            return;
        }
        const Vector3 direction = multiply(difference, 1.0f / currentLength);
        const float weightA = inverseMasses_[constraint.a];
        const float weightB = inverseMasses_[constraint.b];
        const float denominator = weightA + weightB + alpha;
        if (denominator <= 1.0e-12f)
        {
            return;
        }
        const float constraintValue = currentLength - constraint.restLength;
        const float deltaLambda = (-constraintValue - alpha * constraint.lambda) / denominator;
        constraint.lambda += deltaLambda;
        positions_[constraint.a] = subtract(
            positions_[constraint.a], multiply(direction, weightA * deltaLambda));
        positions_[constraint.b] = add(
            positions_[constraint.b], multiply(direction, weightB * deltaLambda));
    }

    void XpbdSoftBody::solve_volume(Tetrahedron& tetrahedron, float alpha)
    {
        const Vector3 a = positions_[tetrahedron.a];
        const Vector3 b = positions_[tetrahedron.b];
        const Vector3 c = positions_[tetrahedron.c];
        const Vector3 d = positions_[tetrahedron.d];
        const float currentVolume = tetrahedron_volume(a, b, c, d);
        const Vector3 gradientB = multiply(cross(subtract(c, a), subtract(d, a)), 1.0f / 6.0f);
        const Vector3 gradientC = multiply(cross(subtract(d, a), subtract(b, a)), 1.0f / 6.0f);
        const Vector3 gradientD = multiply(cross(subtract(b, a), subtract(c, a)), 1.0f / 6.0f);
        const Vector3 gradientA = multiply(add(add(gradientB, gradientC), gradientD), -1.0f);
        const Vector3 gradients[] = {gradientA, gradientB, gradientC, gradientD};
        const std::uint32_t indices[] = {
            tetrahedron.a, tetrahedron.b, tetrahedron.c, tetrahedron.d};

        float denominator = alpha;
        for (std::size_t index = 0; index < 4; ++index)
        {
            denominator += inverseMasses_[indices[index]] * dot(gradients[index], gradients[index]);
        }
        if (denominator <= 1.0e-16f)
        {
            return;
        }

        const float constraintValue = currentVolume - tetrahedron.restVolume;
        const float deltaLambda = (-constraintValue - alpha * tetrahedron.lambda) / denominator;
        tetrahedron.lambda += deltaLambda;
        for (std::size_t index = 0; index < 4; ++index)
        {
            positions_[indices[index]] = add(
                positions_[indices[index]],
                multiply(gradients[index], inverseMasses_[indices[index]] * deltaLambda));
        }
    }

    std::span<const Vector3> XpbdSoftBody::surface_positions() const noexcept
    {
        return {positions_.data(), surfaceVertexCount_};
    }

    Aabb XpbdSoftBody::surface_bounds() const noexcept
    {
        if (!initialized_ || surfaceVertexCount_ == 0)
        {
            return {};
        }
        Aabb bounds{positions_[0], positions_[0]};
        for (std::size_t index = 1; index < surfaceVertexCount_; ++index)
        {
            const auto point = positions_[index];
            bounds.minimum.x = std::min(bounds.minimum.x, point.x);
            bounds.minimum.y = std::min(bounds.minimum.y, point.y);
            bounds.minimum.z = std::min(bounds.minimum.z, point.z);
            bounds.maximum.x = std::max(bounds.maximum.x, point.x);
            bounds.maximum.y = std::max(bounds.maximum.y, point.y);
            bounds.maximum.z = std::max(bounds.maximum.z, point.z);
        }
        return bounds;
    }
}
