// Common, renderer- and window-system-independent value types.

#pragma once

#include <cstdint>

namespace softphysics::core
{
    struct Extent2D
    {
        std::uint32_t width = 1280;
        std::uint32_t height = 720;
    };

}
