#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "PhysicsTypes.h"

namespace softphysics::physics
{
    struct SoftBodyConfig
    {
        // SI units: kilograms, meters, seconds, and newtons.
        float totalMassKg = 18.0f;
        float damping = 2.0f;
        float stretchCompliance = 0.0001f;
        float volumeCompliance = 1.0e-11f;
        float contactStiffness = 300.0f;
        float maximumContactForce = 12.0f;
        float contactRadius = 0.13f;
        float anchoredBottomBand = 0.025f;
        std::uint32_t solverIterations = 6;
    };

    // A first volumetric XPBD body; the current center-fan tetrahedralization
    // assumes a closed surface that is approximately star-shaped.
    class XpbdSoftBody final
    {
    public:
        bool initialize(
            std::span<const Vector3> surfaceVertices,
            std::span<const std::uint32_t> triangleIndices,
            const SoftBodyConfig& config,
            std::string& errorMessage);

        void add_contact(const Contact& contact);
        void step(float fixedDeltaSeconds);

        [[nodiscard]] std::span<const Vector3> surface_positions() const noexcept;
        [[nodiscard]] Aabb surface_bounds() const noexcept;

    private:
        struct DistanceConstraint
        {
            std::uint32_t a = 0;
            std::uint32_t b = 0;
            float restLength = 0.0f;
            float lambda = 0.0f;
        };

        struct Tetrahedron
        {
            std::uint32_t a = 0;
            std::uint32_t b = 0;
            std::uint32_t c = 0;
            std::uint32_t d = 0;
            float restVolume = 0.0f;
            float lambda = 0.0f;
        };

        void solve_distance(DistanceConstraint& constraint, float alpha);
        void solve_volume(Tetrahedron& tetrahedron, float alpha);

        SoftBodyConfig config_{};
        std::vector<Vector3> positions_;
        std::vector<Vector3> restPositions_;
        std::vector<Vector3> velocities_;
        std::vector<Vector3> externalForces_;
        std::vector<float> inverseMasses_;
        std::vector<DistanceConstraint> distanceConstraints_;
        std::vector<Tetrahedron> tetrahedra_;
        std::size_t surfaceVertexCount_ = 0;
        bool initialized_ = false;
    };
}
