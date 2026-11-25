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

    // Events use our own handles so callers do not depend on SDL's identifiers.
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

    enum class KeyCode : std::uint8_t
    {
        W,
        A,
        S,
        D,
        ArrowUp,
        ArrowDown,
        ArrowLeft,
        ArrowRight,
        Count
    };

    struct KeyChanged
    {
        WindowHandle window{};
        KeyCode key = KeyCode::W;
        bool is_down = false;
        bool repeat = false;
    };

    struct WindowFocusLost
    {
        WindowHandle window{};
    };

    struct ApplicationQuitRequested
    {
    };

    using WindowEvent = std::variant<
        WindowCloseRequested,
        WindowResized,
        KeyChanged,
        WindowFocusLost,
        ApplicationQuitRequested
    >;
};
