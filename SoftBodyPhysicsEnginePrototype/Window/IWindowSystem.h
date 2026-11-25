// Main interface for managing the user's window

#pragma once

#include <optional>

#include "../Core/Types/Types.h"
#include "WindowTypes.h"

namespace softphysics::window
{
	class IWindowSystem {
	public:
		virtual std::optional<WindowHandle> create_window(const WindowDesc& specs) = 0;
		virtual bool poll_event(WindowEvent& outEvent) = 0;
		virtual softphysics::core::Extent2D get_drawable_size(WindowHandle windowRef) = 0;
		virtual void destroy_window(WindowHandle windowRef) = 0;

		virtual ~IWindowSystem() = default;
	};
};
