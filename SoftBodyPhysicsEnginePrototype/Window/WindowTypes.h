// Window Specific Types

#pragma once

#include <cstdint>
#include <string>
#include <variant>

#include "../Core/Types/Types.h"


namespace softphysics::window
{
    using softphysics::core::Extent2D;

    // Window structure and reference types
    struct WindowDesc
    {
        std::string title;
        Extent2D size{1280, 720};
        bool resizable = true;
        bool high_dpi = true;
    };

    struct WindowHandle
    {
        std::uint64_t id = 0;
    };

    // Window Event Types
    struct WindowCloseRequested
    {
        WindowHandle window{};
    };

    struct WindowResized
    {
        WindowHandle window{};
        Extent2D logical_size{};
        Extent2D drawable_size{};
    };

    struct ApplicationQuitRequested
    {
    };

    using WindowEvent = std::variant<
        WindowCloseRequested,
        WindowResized,
        ApplicationQuitRequested
    >;
};
