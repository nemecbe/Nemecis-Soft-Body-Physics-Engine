#include <softphysics/EngineApplication.h>

#include "../Assets/Gltf/GlbMeshLoader.h"
#include "../RenderEngine/IRenderEngine.h"
#include "../Window/SdlWindowSystem/SdlWindowSystem.h"

#if defined(SOFTPHYSICS_HAS_D3D12_RENDERER)
#include "../RenderEngine/Adapters/Windows/D3D12/D3D12RenderEngine.h"
#endif

#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <variant>

namespace
{
    using softphysics::renderengine::LinearColor;
    using softphysics::renderengine::MeshData;
    using softphysics::renderengine::MeshVertex;
    using softphysics::renderengine::Vector3;
    using softphysics::window::KeyCode;

    struct CameraNavigation
    {
        static constexpr std::size_t KeyCount = static_cast<std::size_t>(KeyCode::Count);
        std::array<bool, KeyCount> keysDown{};
        float yaw = 0.0f;
        float pitch = 0.0f;
        float focusDistance = 1.0f;
    };

    std::size_t key_index(KeyCode key)
    {
        return static_cast<std::size_t>(key);
    }

    Vector3 camera_forward(float yaw, float pitch)
    {
        const float horizontal = std::cos(pitch);
        return {std::sin(yaw) * horizontal, std::sin(pitch), std::cos(yaw) * horizontal};
    }

    void initialize_camera_navigation(
        const softphysics::renderengine::Camera3D& camera,
        CameraNavigation& navigation)
    {
        Vector3 direction{
            camera.target.x - camera.position.x,
            camera.target.y - camera.position.y,
            camera.target.z - camera.position.z};
        const float lengthSquared = direction.x * direction.x + direction.y * direction.y +
                                   direction.z * direction.z;
        navigation.focusDistance = std::sqrt(lengthSquared);
        if (std::isfinite(lengthSquared) && lengthSquared > 1.0e-12f)
        {
            const float inverseLength = 1.0f / navigation.focusDistance;
            direction.x *= inverseLength;
            direction.y *= inverseLength;
            direction.z *= inverseLength;
            navigation.yaw = std::atan2(direction.x, direction.z);
            navigation.pitch = std::asin(std::clamp(direction.y, -1.0f, 1.0f));
        }
        else
        {
            navigation.focusDistance = 1.0f;
        }
    }

    bool update_camera_navigation(
        softphysics::renderengine::Camera3D& camera,
        CameraNavigation& navigation,
        float deltaSeconds)
    {
        // WASD moves in view-relative directions; arrow keys change yaw and pitch.
        const auto down = [&navigation](KeyCode key)
        {
            return navigation.keysDown[key_index(key)];
        };

        const float yawInput = static_cast<float>(down(KeyCode::ArrowRight)) -
                               static_cast<float>(down(KeyCode::ArrowLeft));
        const float pitchInput = static_cast<float>(down(KeyCode::ArrowUp)) -
                                 static_cast<float>(down(KeyCode::ArrowDown));
        constexpr float turnSpeed = 1.5f;
        constexpr float moveSpeed = 0.8f;
        navigation.yaw += yawInput * turnSpeed * deltaSeconds;
        navigation.pitch = std::clamp(
            navigation.pitch + pitchInput * turnSpeed * deltaSeconds,
            -1.45f,
            1.45f);

        const float forwardInput = static_cast<float>(down(KeyCode::W)) -
                                   static_cast<float>(down(KeyCode::S));
        const float strafeInput = static_cast<float>(down(KeyCode::D)) -
                                  static_cast<float>(down(KeyCode::A));
        const Vector3 forward = camera_forward(navigation.yaw, navigation.pitch);
        const Vector3 right{std::cos(navigation.yaw), 0.0f, -std::sin(navigation.yaw)};
        Vector3 movement{
            forward.x * forwardInput + right.x * strafeInput,
            forward.y * forwardInput,
            forward.z * forwardInput + right.z * strafeInput};

        const bool hasRotationInput = yawInput != 0.0f || pitchInput != 0.0f;
        const bool hasMovementInput = forwardInput != 0.0f || strafeInput != 0.0f;
        if (!hasRotationInput && !hasMovementInput)
        {
            return false;
        }

        const float movementLengthSquared = movement.x * movement.x +
                                            movement.y * movement.y +
                                            movement.z * movement.z;
        if (hasMovementInput && std::isfinite(movementLengthSquared) &&
            movementLengthSquared > 1.0e-12f)
        {
            const float inverseLength = 1.0f / std::sqrt(movementLengthSquared);
            movement.x *= inverseLength;
            movement.y *= inverseLength;
            movement.z *= inverseLength;
            camera.position.x += movement.x * moveSpeed * deltaSeconds;
            camera.position.y += movement.y * moveSpeed * deltaSeconds;
            camera.position.z += movement.z * moveSpeed * deltaSeconds;
        }
        const Vector3 newForward = camera_forward(navigation.yaw, navigation.pitch);
        camera.target = {
            camera.position.x + newForward.x * navigation.focusDistance,
            camera.position.y + newForward.y * navigation.focusDistance,
            camera.position.z + newForward.z * navigation.focusDistance};
        return true;
    }

    softphysics::renderengine::MeshData make_glove_sphere()
    {
        // Temporary procedural stand-in so the impact experiment needs no glove asset.
        constexpr float radius = 0.075f;
        constexpr std::uint32_t ringCount = 12;
        constexpr std::uint32_t segmentCount = 20;
        MeshData mesh;
        mesh.vertices.reserve((ringCount + 1) * (segmentCount + 1));
        mesh.indices.reserve(ringCount * segmentCount * 6);
        const LinearColor gloveColor{0.9f, 0.035f, 0.06f, 1.0f};

        for (std::uint32_t ring = 0; ring <= ringCount; ++ring)
        {
            const float latitude = 3.14159265f * static_cast<float>(ring) /
                                   static_cast<float>(ringCount);
            const float y = std::cos(latitude);
            const float radial = std::sin(latitude);
            for (std::uint32_t segment = 0; segment <= segmentCount; ++segment)
            {
                const float longitude = 6.28318531f * static_cast<float>(segment) /
                                        static_cast<float>(segmentCount);
                const Vector3 normal{
                    radial * std::cos(longitude),
                    y,
                    radial * std::sin(longitude)};
                mesh.vertices.push_back({
                    {normal.x * radius, normal.y * radius, normal.z * radius},
                    {normal.x, normal.y, normal.z},
                    gloveColor});
            }
        }

        for (std::uint32_t ring = 0; ring < ringCount; ++ring)
        {
            for (std::uint32_t segment = 0; segment < segmentCount; ++segment)
            {
                const std::uint32_t first = ring * (segmentCount + 1) + segment;
                const std::uint32_t nextRing = first + segmentCount + 1;
                mesh.indices.insert(
                    mesh.indices.end(),
                    {first, nextRing, first + 1,
                     first + 1, nextRing, nextRing + 1});
            }
        }
        return mesh;
    }

    float smooth_step(float edge0, float edge1, float value)
    {
        const float t = std::clamp((value - edge0) / (edge1 - edge0), 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }

    Vector3 subtract(Vector3 a, Vector3 b)
    {
        return {a.x - b.x, a.y - b.y, a.z - b.z};
    }

    Vector3 cross(Vector3 a, Vector3 b)
    {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
    }

    bool normalize(Vector3& value)
    {
        const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
        if (!std::isfinite(lengthSquared) || lengthSquared < 1.0e-12f)
        {
            return false;
        }
        const float inverseLength = 1.0f / std::sqrt(lengthSquared);
        value.x *= inverseLength;
        value.y *= inverseLength;
        value.z *= inverseLength;
        return true;
    }

    void deform_torso_for_impact(
        const MeshData& baseMesh,
        float impactAmount,
        std::vector<MeshVertex>& deformedVertices)
    {
        // Start from the undeformed mesh each frame so dents do not accumulate or drift.
        deformedVertices = baseMesh.vertices;
        if (impactAmount <= 0.001f)
        {
            return;
        }

        constexpr float contactX = 0.0f;
        constexpr float contactY = 0.025f;
        constexpr float radius = 0.075f;
        constexpr float radiusSquared = radius * radius;
        constexpr float maximumDent = 0.045f;

        // A Gaussian radial falloff makes the scripted indentation smooth around contact.
        for (std::size_t index = 0; index < baseMesh.vertices.size(); ++index)
        {
            const auto& base = baseMesh.vertices[index];
            const float dx = base.position[0] - contactX;
            const float dy = base.position[1] - contactY;
            const float distanceSquared = dx * dx + dy * dy;
            const float radialWeight = std::exp(-distanceSquared / (2.0f * radiusSquared));
            const float frontWeight = std::clamp(base.normal[2], 0.0f, 1.0f);
            deformedVertices[index].position[2] -=
                maximumDent * impactAmount * radialWeight * frontWeight;
        }

        // Rebuild vertex normals from the deformed triangles for correct lighting.
        std::vector<Vector3> accumulatedNormals(deformedVertices.size());
        for (std::size_t index = 0; index + 2 < baseMesh.indices.size(); index += 3)
        {
            const auto i0 = baseMesh.indices[index];
            const auto i1 = baseMesh.indices[index + 1];
            const auto i2 = baseMesh.indices[index + 2];
            const auto point = [&deformedVertices](std::uint32_t vertex)
            {
                const auto& position = deformedVertices[vertex].position;
                return Vector3{position[0], position[1], position[2]};
            };
            const Vector3 edgeA = subtract(point(i1), point(i0));
            const Vector3 edgeB = subtract(point(i2), point(i0));
            const Vector3 faceNormal = cross(edgeA, edgeB);
            for (const auto vertex : {i0, i1, i2})
            {
                auto& normal = accumulatedNormals[vertex];
                normal.x += faceNormal.x;
                normal.y += faceNormal.y;
                normal.z += faceNormal.z;
            }
        }

        for (std::size_t index = 0; index < deformedVertices.size(); ++index)
        {
            if (normalize(accumulatedNormals[index]))
            {
                deformedVertices[index].normal = {
                    accumulatedNormals[index].x,
                    accumulatedNormals[index].y,
                    accumulatedNormals[index].z};
            }
        }
    }
}

namespace softphysics::engine
{
    struct EngineApplication::Impl
    {
        std::unique_ptr<softphysics::window::SdlWindowSystem> windowSystem;
        std::unique_ptr<softphysics::renderengine::IRenderEngine> renderer;
        std::optional<softphysics::window::WindowHandle> window;
        softphysics::renderengine::MeshHandle torsoMesh;
        softphysics::renderengine::MeshHandle gloveMesh;
        softphysics::renderengine::MeshData torsoBaseMesh;
        std::vector<softphysics::renderengine::MeshVertex> torsoDeformedVertices;
        softphysics::renderengine::Camera3D camera;
        CameraNavigation cameraNavigation;
        bool initialized = false;
        bool running = false;
    };

    EngineApplication::EngineApplication()
        : impl_(std::make_unique<Impl>())
    {
    }

    EngineApplication::~EngineApplication()
    {
        shutdown();
    }

    bool EngineApplication::initialize(const EngineConfig& config)
    {
        if (impl_->initialized)
        {
            return false;
        }

        // Platform objects are owned behind interfaces; this prototype currently selects SDL + D3D12.
        impl_->windowSystem = std::make_unique<softphysics::window::SdlWindowSystem>();

        softphysics::window::WindowDesc windowDesc;
        windowDesc.title = config.windowTitle != nullptr ? config.windowTitle : "Soft Physics Engine";
        windowDesc.size = {config.windowWidth, config.windowHeight};
        windowDesc.resizable = config.resizable;
        windowDesc.high_dpi = config.highDpi;

        impl_->window = impl_->windowSystem->create_window(windowDesc);
        if (!impl_->window)
        {
            std::cerr << "Could not create the application window.\n";
            shutdown();
            return false;
        }

        const auto drawableSize = impl_->windowSystem->get_drawable_size(*impl_->window);
        std::cout << "Window created at "
                  << drawableSize.width << " x " << drawableSize.height
                  << " drawable pixels. Close the window to exit.\n";

#if defined(SOFTPHYSICS_HAS_D3D12_RENDERER)
        impl_->renderer = std::make_unique<softphysics::renderengine::windows::d3d12::D3D12RenderEngine>(
            *impl_->windowSystem);
        if (!impl_->renderer->initialize(*impl_->window, drawableSize))
        {
            std::cerr << "Could not initialize the Direct3D 12 renderer.\n";
            shutdown();
            return false;
        }

        softphysics::renderengine::Camera3D camera;
        camera.position = {0.65f, 0.28f, 1.25f};
        camera.target = {0.0f, 0.0f, 0.0f};
        impl_->camera = camera;
        initialize_camera_navigation(impl_->camera, impl_->cameraNavigation);
        if (!impl_->renderer->set_camera(camera))
        {
            std::cerr << "Could not set up the test camera.\n";
            shutdown();
            return false;
        }

        // Resolve assets beside the built executable, where CMake stages the test mesh.
        const char* executableDirectory = SDL_GetBasePath();
        if (executableDirectory == nullptr)
        {
            std::cerr << "Could not locate the testbed executable directory: "
                      << SDL_GetError() << "\n";
            shutdown();
            return false;
        }
        const std::filesystem::path assetDirectory =
            std::filesystem::u8path(executableDirectory) /
            std::filesystem::u8path(config.assetDirectory != nullptr ? config.assetDirectory : "assets");
        const auto torsoPath = assetDirectory / "meshes" / "Torso_Test_Verticies.glb";
        std::string assetError;
        auto torsoData = softphysics::assets::load_glb_mesh(torsoPath, assetError);
        if (!torsoData)
        {
            std::cerr << "Could not load torso mesh at " << torsoPath.string()
                      << ": " << assetError << "\n";
            shutdown();
            return false;
        }
        impl_->torsoBaseMesh = std::move(*torsoData);
        impl_->torsoDeformedVertices = impl_->torsoBaseMesh.vertices;
        impl_->torsoMesh = impl_->renderer->create_mesh(impl_->torsoBaseMesh);
        if (!impl_->torsoMesh)
        {
            std::cerr << "Could not upload the imported torso mesh.\n";
            shutdown();
            return false;
        }

        impl_->gloveMesh = impl_->renderer->create_mesh(make_glove_sphere());
        if (!impl_->gloveMesh)
        {
            std::cerr << "Could not create the procedural glove sphere.\n";
            shutdown();
            return false;
        }
        std::cout << "Loaded torso mesh with " << impl_->torsoBaseMesh.vertices.size()
                  << " vertices and " << impl_->torsoBaseMesh.indices.size() << " indices.\n";
#endif

        impl_->initialized = true;
        return true;
    }

    int EngineApplication::run()
    {
        if (!impl_->initialized)
        {
            std::cerr << "EngineApplication must be initialized before run().\n";
            return 1;
        }

        impl_->running = true;
        int exitCode = 0;
        const auto animationStart = std::chrono::steady_clock::now();
        // This timed kinematic cycle is a stand-in until collision-driven motion exists.
        constexpr float cycleDuration = 2.8f;
        constexpr float approachDuration = 0.75f;
        constexpr float contactHoldDuration = 0.18f;
        constexpr float releaseDuration = 0.42f;
        constexpr float gloveStartZ = 0.72f;
        constexpr float gloveContactZ = 0.12f;
        auto previousCameraUpdate = std::chrono::steady_clock::now();
        while (impl_->running)
        {
            softphysics::window::WindowEvent event;
            while (impl_->windowSystem->poll_event(event))
            {
                if (std::holds_alternative<softphysics::window::ApplicationQuitRequested>(event))
                {
                    impl_->running = false;
                }
                else if (const auto* close =
                             std::get_if<softphysics::window::WindowCloseRequested>(&event))
                {
                    if (close->window.id == impl_->window->id)
                    {
                        impl_->running = false;
                    }
                }
                else if (const auto* resized =
                             std::get_if<softphysics::window::WindowResized>(&event))
                {
                    if (resized->window.id == impl_->window->id)
                    {
                        std::cout << "Drawable size changed to "
                                  << resized->drawable_size.width << " x "
                                  << resized->drawable_size.height << " pixels.\n";
                        if (impl_->renderer && !impl_->renderer->resize(resized->drawable_size))
                        {
                            std::cerr << "Could not resize the renderer.\n";
                            impl_->running = false;
                            exitCode = 1;
                        }
                    }
                }
                else if (const auto* key = std::get_if<softphysics::window::KeyChanged>(&event))
                {
                    if (key->window.id == impl_->window->id && key->key != KeyCode::Count)
                    {
                        impl_->cameraNavigation.keysDown[key_index(key->key)] = key->is_down;
                    }
                }
                else if (const auto* focusLost =
                             std::get_if<softphysics::window::WindowFocusLost>(&event))
                {
                    if (focusLost->window.id == impl_->window->id)
                    {
                        impl_->cameraNavigation.keysDown.fill(false);
                    }
                }
            }

            if (!impl_->running)
            {
                break;
            }

            if (impl_->renderer)
            {
                const auto cameraUpdateTime = std::chrono::steady_clock::now();
                const float cameraDeltaSeconds = std::clamp(
                    std::chrono::duration<float>(cameraUpdateTime - previousCameraUpdate).count(),
                    0.0f,
                    0.1f);
                previousCameraUpdate = cameraUpdateTime;
                if (update_camera_navigation(
                        impl_->camera,
                        impl_->cameraNavigation,
                        cameraDeltaSeconds) &&
                    !impl_->renderer->set_camera(impl_->camera))
                {
                    std::cerr << "Could not update the camera from keyboard input.\n";
                    impl_->running = false;
                    exitCode = 1;
                    continue;
                }

                if (!impl_->renderer->begin_frame())
                {
                    std::cerr << "Could not begin a renderer frame.\n";
                    impl_->running = false;
                    exitCode = 1;
                    continue;
                }

                const float elapsedSeconds = std::chrono::duration<float>(
                    std::chrono::steady_clock::now() - animationStart).count();
                // A deterministic kinematic punch drives the first deformation pass;
                // collision detection and force-based response will replace this later.
                const float phase = std::fmod(elapsedSeconds, cycleDuration);
                float gloveProgress = 0.0f;
                float impactAmount = 0.0f;
                if (phase < approachDuration)
                {
                    const float approach = phase / approachDuration;
                    gloveProgress = smooth_step(0.0f, 1.0f, approach);
                    impactAmount = smooth_step(0.68f, 1.0f, approach);
                }
                else if (phase < approachDuration + contactHoldDuration)
                {
                    gloveProgress = 1.0f;
                    impactAmount = 1.0f;
                }
                else if (phase < approachDuration + contactHoldDuration + releaseDuration)
                {
                    const float release =
                        (phase - approachDuration - contactHoldDuration) / releaseDuration;
                    gloveProgress = 1.0f - smooth_step(0.0f, 1.0f, release);
                    impactAmount = 1.0f - smooth_step(0.0f, 0.55f, release);
                }

                deform_torso_for_impact(
                    impl_->torsoBaseMesh,
                    impactAmount,
                    impl_->torsoDeformedVertices);
                if (!impl_->renderer->update_mesh_vertices(
                        impl_->torsoMesh,
                        impl_->torsoDeformedVertices))
                {
                    std::cerr << "Could not update the deformed torso vertices.\n";
                    impl_->running = false;
                    exitCode = 1;
                    continue;
                }

                impl_->renderer->clear({0.25f, 0.65f, 0.95f, 1.0f});
                impl_->renderer->draw_mesh(impl_->torsoMesh, {});
                softphysics::renderengine::Transform3D gloveTransform;
                gloveTransform.position = {
                    0.0f,
                    0.025f,
                    gloveStartZ + (gloveContactZ - gloveStartZ) * gloveProgress};
                impl_->renderer->draw_mesh(impl_->gloveMesh, gloveTransform);
                if (!impl_->renderer->present())
                {
                    std::cerr << "Could not present a renderer frame.\n";
                    impl_->running = false;
                    exitCode = 1;
                }
            }
            else
            {
                // Other platform renderer adapters will replace this idle wait.
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        return exitCode;
    }

    void EngineApplication::shutdown()
    {
        if (!impl_)
        {
            return;
        }

        impl_->running = false;
        if (impl_->renderer)
        {
            if (impl_->torsoMesh)
            {
                impl_->renderer->destroy_mesh(impl_->torsoMesh);
                impl_->torsoMesh = {};
            }
            if (impl_->gloveMesh)
            {
                impl_->renderer->destroy_mesh(impl_->gloveMesh);
                impl_->gloveMesh = {};
            }
            impl_->renderer->shutdown();
            impl_->renderer.reset();
        }

        if (impl_->windowSystem && impl_->window)
        {
            impl_->windowSystem->destroy_window(*impl_->window);
        }
        impl_->window.reset();
        impl_->windowSystem.reset();
        impl_->torsoBaseMesh = {};
        impl_->torsoDeformedVertices.clear();
        impl_->initialized = false;
    }
}
