#pragma once

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <SDL3/SDL.h>

#include "../IWindowSystem.h"
#include "../Platform/Win32/IWin32WindowAccess.h"

namespace softphysics::window {
	class SdlWindowSystem final : public IWindowSystem, public IWin32WindowAccess {
	private:
		std::uint64_t nextWindowId = 1;
		bool sdlInitialized = false;
		// Public handles are stable engine IDs; SDL IDs are only used to map incoming events.
		std::unordered_map<std::uint64_t, SDL_Window*> windowsByHandle;
		std::unordered_map<SDL_WindowID, WindowHandle> handlesBySdlId;

	public:
		~SdlWindowSystem() override;
		std::optional<WindowHandle> create_window(const WindowDesc& specs) override;
		bool poll_event(WindowEvent& outEvent) override;
		softphysics::core::Extent2D get_drawable_size(WindowHandle windowRef) override;
		// Returns a borrowed HWND as void* on Windows. The value is valid while
		// the corresponding SDL window remains alive. Other platforms return nullopt.
		std::optional<void*> get_win32_hwnd(WindowHandle windowRef) const override;
		void destroy_window(WindowHandle windowRef) override;
	};
}
