// Contains the Engine interface, to be implemented by different adapters

#pragma once

#include "../Core/Types/Types.h"
#include "../Window/WindowTypes.h"
#include "RenderTypes.h"

namespace softphysics::renderengine
{
	class IRenderEngine {
	public:
		virtual bool initialize(
			softphysics::window::WindowHandle window,
			softphysics::core::Extent2D initialDrawableSize) = 0;
		virtual bool resize(softphysics::core::Extent2D updatedSize) = 0;
		virtual bool begin_frame() = 0;
		virtual void clear(softphysics::renderengine::LinearColor color) = 0;
		virtual bool present() = 0;
		virtual void shutdown() = 0;

		virtual ~IRenderEngine() = default;
	};
}
