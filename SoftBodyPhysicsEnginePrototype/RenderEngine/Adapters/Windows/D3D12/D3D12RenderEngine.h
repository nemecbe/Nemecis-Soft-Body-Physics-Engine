#pragma once

#include <memory>

#include "../../../IRenderEngine.h"
#include "../../../../Window/Platform/Win32/IWin32WindowAccess.h"

namespace softphysics::renderengine::windows::d3d12
{
    class D3D12RenderEngine final : public IRenderEngine
    {
    public:
        explicit D3D12RenderEngine(softphysics::window::IWin32WindowAccess& windowAccess);
        ~D3D12RenderEngine() override;

        D3D12RenderEngine(const D3D12RenderEngine&) = delete;
        D3D12RenderEngine& operator=(const D3D12RenderEngine&) = delete;

        bool initialize(
            softphysics::window::WindowHandle window,
            softphysics::core::Extent2D initialDrawableSize) override;
        bool resize(softphysics::core::Extent2D updatedSize) override;
        bool set_camera(const Camera3D& camera) override;
        bool begin_frame() override;
        void clear(LinearColor color) override;
        MeshHandle create_mesh(const MeshData& mesh) override;
        void destroy_mesh(MeshHandle mesh) override;
        void draw_mesh(MeshHandle mesh, const Transform3D& transform) override;
        bool present() override;
        void shutdown() override;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
