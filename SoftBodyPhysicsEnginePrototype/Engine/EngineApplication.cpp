#include <softphysics/EngineApplication.h>

#include "../RenderEngine/IRenderEngine.h"
#include "../Window/SdlWindowSystem/SdlWindowSystem.h"

#if defined(SOFTPHYSICS_HAS_D3D12_RENDERER)
#include "../RenderEngine/Adapters/Windows/D3D12/D3D12RenderEngine.h"
#endif

#include <chrono>
#include <iostream>
#include <memory>
#include <optional>
#include <thread>
#include <variant>

namespace softphysics::engine
{
    struct EngineApplication::Impl
    {
        std::unique_ptr<softphysics::window::SdlWindowSystem> windowSystem;
        std::unique_ptr<softphysics::renderengine::IRenderEngine> renderer;
        std::optional<softphysics::window::WindowHandle> window;
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
