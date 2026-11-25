#include <iostream>
#include <limits>
#include <memory>

#include "SdlWindowSystem.h"

namespace softphysics::window
{
    SdlWindowSystem::~SdlWindowSystem()
    {
        for (const auto& [id, window] : windowsByHandle)
        {
            SDL_DestroyWindow(window);
        }

        windowsByHandle.clear();
        handlesBySdlId.clear();

        if (sdlInitialized)
        {
            SDL_Quit();
        }
    }

    std::optional<WindowHandle> SdlWindowSystem::create_window(const WindowDesc& specs)
    {
        if (specs.size.width == 0 || specs.size.height == 0 ||
            specs.size.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
            specs.size.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()))
        {
            std::cerr << "Window dimensions must fit in a positive int.\n";
            return std::nullopt;
        }

        if (!sdlInitialized)
        {
            if (!SDL_Init(SDL_INIT_VIDEO))
            {
                std::cerr << "SDL initialization failed: " << SDL_GetError() << '\n';
                return std::nullopt;
            }

            sdlInitialized = true;
        }

        SDL_WindowFlags flags = 0;
        if (specs.resizable)
        {
            flags |= SDL_WINDOW_RESIZABLE;
        }
        if (specs.high_dpi)
        {
            flags |= SDL_WINDOW_HIGH_PIXEL_DENSITY;
        }

        auto quit_sdl_if_unused = [this]()
        {
            if (windowsByHandle.empty() && sdlInitialized)
            {
                SDL_Quit();
                sdlInitialized = false;
            }
        };

        std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)> window(
            SDL_CreateWindow(
                specs.title.c_str(),
                static_cast<int>(specs.size.width),
                static_cast<int>(specs.size.height),
                flags),
            &SDL_DestroyWindow);

        if (window == nullptr)
        {
            std::cerr << "SDL window creation failed: " << SDL_GetError() << '\n';
            quit_sdl_if_unused();
            return std::nullopt;
        }

        const SDL_WindowID sdlId = SDL_GetWindowID(window.get());
        if (sdlId == 0)
        {
            std::cerr << "SDL returned an invalid window ID: " << SDL_GetError() << '\n';
            window.reset();
            quit_sdl_if_unused();
            return std::nullopt;
        }

        const WindowHandle handle{nextWindowId++};
        const auto [windowIt, windowInserted] = windowsByHandle.emplace(handle.id, window.get());
        if (!windowInserted)
        {
            std::cerr << "Could not register the SDL window handle.\n";
            window.reset();
            quit_sdl_if_unused();
            return std::nullopt;
        }

        try
        {
            const auto sdlIdInsertion = handlesBySdlId.emplace(sdlId, handle);
            if (!sdlIdInsertion.second)
            {
                std::cerr << "SDL returned a window ID that is already registered.\n";
                windowsByHandle.erase(windowIt);
                window.reset();
                quit_sdl_if_unused();
                return std::nullopt;
            }
        }
        catch (...)
        {
            handlesBySdlId.erase(sdlId);
            windowsByHandle.erase(windowIt);
            window.reset();
            quit_sdl_if_unused();
            throw;
        }

        window.release();
        return handle;
    }

    bool SdlWindowSystem::poll_event(WindowEvent& outEvent)
    {
        SDL_Event event{};

        while (SDL_PollEvent(&event))
        {
            switch (event.type)
            {
			case SDL_EVENT_QUIT:
				outEvent = ApplicationQuitRequested{};
				return true;

            case SDL_EVENT_WINDOW_CLOSE_REQUESTED:
            {
                const auto handleIt = handlesBySdlId.find(event.window.windowID);
                if (handleIt == handlesBySdlId.end())
                {
                    break;
                }

                outEvent = WindowCloseRequested{handleIt->second};
                return true;
            }

            case SDL_EVENT_WINDOW_RESIZED:
            case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
            {
                const auto handleIt = handlesBySdlId.find(event.window.windowID);
                if (handleIt == handlesBySdlId.end())
                {
                    break;
                }

                const auto windowIt = windowsByHandle.find(handleIt->second.id);
                if (windowIt == windowsByHandle.end())
                {
                    break;
                }

                int logicalWidth = 0;
                int logicalHeight = 0;
                int pixelWidth = 0;
                int pixelHeight = 0;

                if (!SDL_GetWindowSize(windowIt->second, &logicalWidth, &logicalHeight) ||
                    !SDL_GetWindowSizeInPixels(windowIt->second, &pixelWidth, &pixelHeight) ||
                    logicalWidth < 0 || logicalHeight < 0 || pixelWidth < 0 || pixelHeight < 0)
                {
                    break;
                }

                const softphysics::core::Extent2D logicalSize{
                    static_cast<std::uint32_t>(logicalWidth),
                    static_cast<std::uint32_t>(logicalHeight)};
                const softphysics::core::Extent2D drawableSize{
                    static_cast<std::uint32_t>(pixelWidth),
                    static_cast<std::uint32_t>(pixelHeight)};

                outEvent = WindowResized{handleIt->second, logicalSize, drawableSize};
                return true;
            }

            default:
                break;
            }
        }

        return false;
    }

    softphysics::core::Extent2D SdlWindowSystem::get_drawable_size(WindowHandle windowRef)
    {
        const auto windowIt = windowsByHandle.find(windowRef.id);
        if (windowIt == windowsByHandle.end())
        {
            return {};
        }

        int width = 0;
        int height = 0;
        if (!SDL_GetWindowSizeInPixels(windowIt->second, &width, &height) ||
            width < 0 || height < 0)
        {
            return {};
        }

        return {
            static_cast<std::uint32_t>(width),
            static_cast<std::uint32_t>(height)};
    }

    std::optional<void*> SdlWindowSystem::get_win32_hwnd(WindowHandle windowRef) const
    {
        const auto windowIt = windowsByHandle.find(windowRef.id);
        if (windowIt == windowsByHandle.end())
        {
            return std::nullopt;
        }

#if defined(SDL_PLATFORM_WIN32)
        const SDL_PropertiesID properties = SDL_GetWindowProperties(windowIt->second);
        if (properties == 0)
        {
            return std::nullopt;
        }

        void* hwnd = SDL_GetPointerProperty(
            properties,
            SDL_PROP_WINDOW_WIN32_HWND_POINTER,
            nullptr);
        if (hwnd == nullptr)
        {
            return std::nullopt;
        }

        return hwnd;
#else
        // Native handles for other platforms will be added with their renderer adapters.
        return std::nullopt;
#endif
    }

    void SdlWindowSystem::destroy_window(WindowHandle windowRef)
    {
        const auto windowIt = windowsByHandle.find(windowRef.id);
        if (windowIt == windowsByHandle.end())
        {
            return;
        }

        const SDL_WindowID sdlId = SDL_GetWindowID(windowIt->second);
        SDL_DestroyWindow(windowIt->second);
        windowsByHandle.erase(windowIt);
        handlesBySdlId.erase(sdlId);

        if (windowsByHandle.empty() && sdlInitialized)
        {
            SDL_Quit();
            sdlInitialized = false;
        }
    }
}
