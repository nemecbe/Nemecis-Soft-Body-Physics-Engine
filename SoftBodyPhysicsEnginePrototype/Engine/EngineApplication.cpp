#include <softphysics/EngineApplication.h>

#include "../RenderEngine/IRenderEngine.h"
#include "../Window/SdlWindowSystem/SdlWindowSystem.h"

#if defined(SOFTPHYSICS_HAS_D3D12_RENDERER)
#include "../RenderEngine/Adapters/Windows/D3D12/D3D12RenderEngine.h"
#endif

#include <chrono>
#include <array>
#include <iostream>
#include <memory>
#include <optional>
#include <thread>
#include <variant>

namespace
{
    softphysics::renderengine::MeshData make_colored_cube()
    {
        using softphysics::renderengine::LinearColor;
        using softphysics::renderengine::MeshData;
        using softphysics::renderengine::Vector3;

        constexpr float halfExtent = 0.5f;
        MeshData mesh;
        mesh.vertices.reserve(24);
        mesh.indices.reserve(36);

        const auto addFace = [&mesh](
                                 const std::array<Vector3, 4>& corners,
                                 Vector3 faceNormal,
                                 LinearColor faceColor)
        {
            const auto first = static_cast<std::uint32_t>(mesh.vertices.size());
            for (const auto& corner : corners)
            {
                mesh.vertices.push_back(
                    {{corner.x, corner.y, corner.z},
                     {faceNormal.x, faceNormal.y, faceNormal.z},
                     faceColor});
            }
            mesh.indices.insert(
                mesh.indices.end(),
                {first, first + 1, first + 2, first, first + 2, first + 3});
        };

        addFace({{{-halfExtent, -halfExtent, -halfExtent},
                  {-halfExtent, halfExtent, -halfExtent},
                  {halfExtent, halfExtent, -halfExtent},
                  {halfExtent, -halfExtent, -halfExtent}}},
                {0.0f, 0.0f, -1.0f},
                {0.95f, 0.25f, 0.2f, 1.0f});
        addFace({{{halfExtent, -halfExtent, -halfExtent},
                  {halfExtent, -halfExtent, halfExtent},
                  {halfExtent, halfExtent, halfExtent},
                  {halfExtent, halfExtent, -halfExtent}}},
                {1.0f, 0.0f, 0.0f},
                {0.2f, 0.8f, 0.35f, 1.0f});
        addFace({{{halfExtent, -halfExtent, halfExtent},
                  {-halfExtent, -halfExtent, halfExtent},
                  {-halfExtent, halfExtent, halfExtent},
                  {halfExtent, halfExtent, halfExtent}}},
                {0.0f, 0.0f, 1.0f},
                {0.2f, 0.4f, 0.95f, 1.0f});
        addFace({{{-halfExtent, -halfExtent, halfExtent},
                  {-halfExtent, -halfExtent, -halfExtent},
                  {-halfExtent, halfExtent, -halfExtent},
                  {-halfExtent, halfExtent, halfExtent}}},
                {-1.0f, 0.0f, 0.0f},
                {0.95f, 0.75f, 0.2f, 1.0f});
        addFace({{{-halfExtent, halfExtent, -halfExtent},
                  {-halfExtent, halfExtent, halfExtent},
                  {halfExtent, halfExtent, halfExtent},
                  {halfExtent, halfExtent, -halfExtent}}},
                {0.0f, 1.0f, 0.0f},
                {0.75f, 0.3f, 0.9f, 1.0f});
        addFace({{{-halfExtent, -halfExtent, halfExtent},
                  {-halfExtent, -halfExtent, -halfExtent},
                  {halfExtent, -halfExtent, -halfExtent},
                  {halfExtent, -halfExtent, halfExtent}}},
                {0.0f, -1.0f, 0.0f},
                {0.2f, 0.8f, 0.9f, 1.0f});
        return mesh;
    }
}

namespace softphysics::engine
{
    struct EngineApplication::Impl
    {
        std::unique_ptr<softphysics::window::SdlWindowSystem> windowSystem;
        std::unique_ptr<softphysics::renderengine::IRenderEngine> renderer;
        std::optional<softphysics::window::WindowHandle> window;
        softphysics::renderengine::MeshHandle cubeMesh;
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
        camera.position = {2.2f, 1.7f, -4.5f};
        camera.target = {0.0f, 0.0f, 0.0f};
        if (!impl_->renderer->set_camera(camera))
        {
            std::cerr << "Could not set up the test camera.\n";
            shutdown();
            return false;
        }

        impl_->cubeMesh = impl_->renderer->create_mesh(make_colored_cube());
        if (!impl_->cubeMesh)
        {
            std::cerr << "Could not create the test cube mesh.\n";
            shutdown();
            return false;
        }
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
            }

            if (!impl_->running)
            {
                break;
            }

            if (impl_->renderer)
            {
                if (!impl_->renderer->begin_frame())
                {
                    std::cerr << "Could not begin a renderer frame.\n";
                    impl_->running = false;
                    exitCode = 1;
                    continue;
                }

                impl_->renderer->clear({0.25f, 0.65f, 0.95f, 1.0f});
                const float elapsedSeconds = std::chrono::duration<float>(
                    std::chrono::steady_clock::now() - animationStart).count();
                softphysics::renderengine::Transform3D cubeTransform;
                cubeTransform.rotationRadians = {0.35f, elapsedSeconds * 0.65f, 0.1f};
                impl_->renderer->draw_mesh(impl_->cubeMesh, cubeTransform);
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
            if (impl_->cubeMesh)
            {
                impl_->renderer->destroy_mesh(impl_->cubeMesh);
                impl_->cubeMesh = {};
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
        impl_->initialized = false;
    }
}
