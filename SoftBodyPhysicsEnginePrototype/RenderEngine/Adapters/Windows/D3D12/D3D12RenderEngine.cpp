#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#include <d3d12.h>
#include <d3d12sdklayers.h>
#include <dxgi1_6.h>
#include <wrl/client.h>

#include <array>
#include <iostream>
#include <limits>

#include "D3D12RenderEngine.h"

namespace softphysics::renderengine::windows::d3d12
{
    using Microsoft::WRL::ComPtr;

    namespace
    {
        constexpr UINT FrameCount = 2;
        constexpr DXGI_FORMAT BackBufferFormat = DXGI_FORMAT_R8G8B8A8_UNORM;

        bool report_failure(HRESULT result, const char* operation)
        {
            if (SUCCEEDED(result))
            {
                return false;
            }

            std::cerr << operation << " failed (HRESULT 0x"
                      << std::hex << static_cast<unsigned long>(result)
                      << std::dec << ").\n";
            return true;
        }
    }

    struct D3D12RenderEngine::Impl
    {
        explicit Impl(softphysics::window::IWin32WindowAccess& access)
            : windowAccess(access)
        {
        }

        softphysics::window::IWin32WindowAccess& windowAccess;
        ComPtr<IDXGIFactory4> factory;
        ComPtr<ID3D12Device> device;
        ComPtr<ID3D12CommandQueue> commandQueue;
        ComPtr<IDXGISwapChain3> swapChain;
        ComPtr<ID3D12DescriptorHeap> rtvHeap;
        std::array<ComPtr<ID3D12CommandAllocator>, FrameCount> commandAllocators;
        std::array<ComPtr<ID3D12Resource>, FrameCount> renderTargets;
        std::array<UINT64, FrameCount> fenceValues{};
        ComPtr<ID3D12GraphicsCommandList> commandList;
        ComPtr<ID3D12Fence> fence;
        HANDLE fenceEvent = nullptr;
        UINT rtvDescriptorSize = 0;
        UINT frameIndex = 0;
        UINT64 nextFenceValue = 1;
        UINT width = 0;
        UINT height = 0;
        bool initialized = false;
        bool frameOpen = false;
        bool renderTargetActive = false;

        bool wait_for_gpu()
        {
            if (!commandQueue || !fence || fenceEvent == nullptr)
            {
                return false;
            }

            const UINT64 value = nextFenceValue++;
            if (report_failure(commandQueue->Signal(fence.Get(), value), "Signal GPU fence"))
            {
                return false;
            }

            const UINT64 completedValue = fence->GetCompletedValue();
            if (completedValue == std::numeric_limits<UINT64>::max())
            {
                std::cerr << "The D3D12 device was removed while waiting for the GPU.\n";
                return false;
            }
            if (completedValue < value)
            {
                if (report_failure(fence->SetEventOnCompletion(value, fenceEvent), "Set fence event") ||
                    WaitForSingleObject(fenceEvent, INFINITE) != WAIT_OBJECT_0)
                {
                    std::cerr << "Waiting for the GPU fence failed.\n";
                    return false;
                }
            }

            fenceValues.fill(0);
            return true;
        }

        bool wait_for_frame(UINT index)
        {
            const UINT64 value = fenceValues[index];
            const UINT64 completedValue = fence->GetCompletedValue();
            if (completedValue == std::numeric_limits<UINT64>::max())
            {
                std::cerr << "The D3D12 device was removed while waiting for a frame.\n";
                return false;
            }
            if (value == 0 || completedValue >= value)
            {
                return true;
            }

            if (report_failure(fence->SetEventOnCompletion(value, fenceEvent), "Set frame fence event"))
            {
                return false;
            }

            if (WaitForSingleObject(fenceEvent, INFINITE) != WAIT_OBJECT_0)
            {
                std::cerr << "Waiting for the frame fence failed.\n";
                return false;
            }
            return true;
        }

        bool create_render_targets()
        {
            D3D12_CPU_DESCRIPTOR_HANDLE handle = rtvHeap->GetCPUDescriptorHandleForHeapStart();
            for (UINT index = 0; index < FrameCount; ++index)
            {
                if (report_failure(
                        swapChain->GetBuffer(index, IID_PPV_ARGS(&renderTargets[index])),
                        "Get swap-chain back buffer"))
                {
                    return false;
                }

                device->CreateRenderTargetView(renderTargets[index].Get(), nullptr, handle);
                handle.ptr += rtvDescriptorSize;
            }
            return true;
        }
    };

    D3D12RenderEngine::D3D12RenderEngine(softphysics::window::IWin32WindowAccess& windowAccess)
        : impl_(std::make_unique<Impl>(windowAccess))
    {
    }

    D3D12RenderEngine::~D3D12RenderEngine()
    {
        shutdown();
    }

    bool D3D12RenderEngine::initialize(
        softphysics::window::WindowHandle window,
        softphysics::core::Extent2D initialDrawableSize)
    {
        shutdown();

        if (initialDrawableSize.width == 0 || initialDrawableSize.height == 0)
        {
            std::cerr << "Cannot initialize D3D12 with a zero-sized drawable.\n";
            return false;
        }

        const auto nativeWindow = impl_->windowAccess.get_win32_hwnd(window);
        if (!nativeWindow || *nativeWindow == nullptr)
        {
            std::cerr << "Could not retrieve the Win32 HWND for the SDL window.\n";
            return false;
        }
        const HWND hwnd = static_cast<HWND>(*nativeWindow);

        UINT factoryFlags = 0;
#if defined(_DEBUG)
        ComPtr<ID3D12Debug> debugController;
        if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&debugController))))
        {
            debugController->EnableDebugLayer();
            factoryFlags |= DXGI_CREATE_FACTORY_DEBUG;
        }
#endif

        if (report_failure(
                CreateDXGIFactory2(factoryFlags, IID_PPV_ARGS(&impl_->factory)),
                "Create DXGI factory"))
        {
            return false;
        }
        if (report_failure(
                D3D12CreateDevice(nullptr, D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&impl_->device)),
                "Create D3D12 device"))
        {
            return false;
        }

        D3D12_COMMAND_QUEUE_DESC queueDesc{};
        queueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
        if (report_failure(
                impl_->device->CreateCommandQueue(&queueDesc, IID_PPV_ARGS(&impl_->commandQueue)),
                "Create D3D12 command queue"))
        {
            return false;
        }

        DXGI_SWAP_CHAIN_DESC1 swapChainDesc{};
        swapChainDesc.Width = initialDrawableSize.width;
        swapChainDesc.Height = initialDrawableSize.height;
        swapChainDesc.Format = BackBufferFormat;
        swapChainDesc.SampleDesc.Count = 1;
        swapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        swapChainDesc.BufferCount = FrameCount;
        swapChainDesc.Scaling = DXGI_SCALING_STRETCH;
        swapChainDesc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
        swapChainDesc.AlphaMode = DXGI_ALPHA_MODE_IGNORE;

        ComPtr<IDXGISwapChain1> initialSwapChain;
        if (report_failure(
                impl_->factory->CreateSwapChainForHwnd(
                    impl_->commandQueue.Get(),
                    hwnd,
                    &swapChainDesc,
                    nullptr,
                    nullptr,
                    &initialSwapChain),
                "Create D3D12 swap chain"))
        {
            return false;
        }
        if (report_failure(initialSwapChain.As(&impl_->swapChain), "Get IDXGISwapChain3"))
        {
            return false;
        }
        impl_->factory->MakeWindowAssociation(hwnd, DXGI_MWA_NO_ALT_ENTER);

        D3D12_DESCRIPTOR_HEAP_DESC rtvHeapDesc{};
        rtvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
        rtvHeapDesc.NumDescriptors = FrameCount;
        if (report_failure(
                impl_->device->CreateDescriptorHeap(&rtvHeapDesc, IID_PPV_ARGS(&impl_->rtvHeap)),
                "Create render-target descriptor heap"))
        {
            return false;
        }
        impl_->rtvDescriptorSize = impl_->device->GetDescriptorHandleIncrementSize(
            D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

        for (UINT index = 0; index < FrameCount; ++index)
        {
            if (report_failure(
                    impl_->device->CreateCommandAllocator(
                        D3D12_COMMAND_LIST_TYPE_DIRECT,
                        IID_PPV_ARGS(&impl_->commandAllocators[index])),
                    "Create D3D12 command allocator"))
            {
                return false;
            }
        }
        if (report_failure(
                impl_->device->CreateCommandList(
                    0,
                    D3D12_COMMAND_LIST_TYPE_DIRECT,
                    impl_->commandAllocators[0].Get(),
                    nullptr,
                    IID_PPV_ARGS(&impl_->commandList)),
                "Create D3D12 command list") ||
            report_failure(impl_->commandList->Close(), "Close initial command list"))
        {
            return false;
        }

        if (report_failure(
                impl_->device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&impl_->fence)),
                "Create D3D12 fence"))
        {
            return false;
        }
        impl_->fenceEvent = CreateEventW(nullptr, FALSE, FALSE, nullptr);
        if (impl_->fenceEvent == nullptr)
        {
            std::cerr << "Could not create the D3D12 fence event.\n";
            return false;
        }

        impl_->width = initialDrawableSize.width;
        impl_->height = initialDrawableSize.height;
        impl_->frameIndex = impl_->swapChain->GetCurrentBackBufferIndex();
        if (!impl_->create_render_targets())
        {
            return false;
        }

        impl_->initialized = true;
        return true;
    }

    bool D3D12RenderEngine::resize(softphysics::core::Extent2D updatedSize)
    {
        if (!impl_->initialized)
        {
            return false;
        }
        if (updatedSize.width == 0 || updatedSize.height == 0)
        {
            // Keep the existing swap chain while minimized; a later resize event restores it.
            return true;
        }
        if (updatedSize.width == impl_->width && updatedSize.height == impl_->height)
        {
            return true;
        }
        if (!impl_->wait_for_gpu())
        {
            return false;
        }

        for (auto& renderTarget : impl_->renderTargets)
        {
            renderTarget.Reset();
        }

        if (report_failure(
                impl_->swapChain->ResizeBuffers(
                    FrameCount,
                    updatedSize.width,
                    updatedSize.height,
                    BackBufferFormat,
                    0),
                "Resize D3D12 swap chain"))
        {
            return false;
        }

        impl_->width = updatedSize.width;
        impl_->height = updatedSize.height;
        impl_->frameIndex = impl_->swapChain->GetCurrentBackBufferIndex();
        return impl_->create_render_targets();
    }

    bool D3D12RenderEngine::begin_frame()
    {
        if (!impl_->initialized || impl_->frameOpen)
        {
            return false;
        }

        impl_->frameIndex = impl_->swapChain->GetCurrentBackBufferIndex();
        if (!impl_->wait_for_frame(impl_->frameIndex))
        {
            return false;
        }
        if (report_failure(
                impl_->commandAllocators[impl_->frameIndex]->Reset(),
                "Reset D3D12 command allocator") ||
            report_failure(
                impl_->commandList->Reset(impl_->commandAllocators[impl_->frameIndex].Get(), nullptr),
                "Reset D3D12 command list"))
        {
            return false;
        }

        impl_->renderTargetActive = false;
        impl_->frameOpen = true;
        return true;
    }

    void D3D12RenderEngine::clear(LinearColor color)
    {
        if (!impl_->frameOpen)
        {
            return;
        }

        auto* renderTarget = impl_->renderTargets[impl_->frameIndex].Get();
        if (!impl_->renderTargetActive)
        {
            D3D12_RESOURCE_BARRIER barrier{};
            barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            barrier.Transition.pResource = renderTarget;
            barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_PRESENT;
            barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_RENDER_TARGET;
            impl_->commandList->ResourceBarrier(1, &barrier);

            D3D12_CPU_DESCRIPTOR_HANDLE rtv = impl_->rtvHeap->GetCPUDescriptorHandleForHeapStart();
            rtv.ptr += static_cast<SIZE_T>(impl_->frameIndex) * impl_->rtvDescriptorSize;
            impl_->commandList->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
            impl_->renderTargetActive = true;
        }

        D3D12_CPU_DESCRIPTOR_HANDLE rtv = impl_->rtvHeap->GetCPUDescriptorHandleForHeapStart();
        rtv.ptr += static_cast<SIZE_T>(impl_->frameIndex) * impl_->rtvDescriptorSize;
        const float clearColor[] = {color.r, color.g, color.b, color.a};
        impl_->commandList->ClearRenderTargetView(rtv, clearColor, 0, nullptr);
    }

    bool D3D12RenderEngine::present()
    {
        if (!impl_->initialized || !impl_->frameOpen)
        {
            return false;
        }

        if (impl_->renderTargetActive)
        {
            D3D12_RESOURCE_BARRIER barrier{};
            barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            barrier.Transition.pResource = impl_->renderTargets[impl_->frameIndex].Get();
            barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
            barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_PRESENT;
            impl_->commandList->ResourceBarrier(1, &barrier);
        }

        if (report_failure(impl_->commandList->Close(), "Close D3D12 command list"))
        {
            impl_->frameOpen = false;
            return false;
        }

        ID3D12CommandList* commandLists[] = {impl_->commandList.Get()};
        impl_->commandQueue->ExecuteCommandLists(1, commandLists);
        const HRESULT presentResult = impl_->swapChain->Present(1, 0);

        const UINT64 submittedFenceValue = impl_->nextFenceValue++;
        const HRESULT signalResult = impl_->commandQueue->Signal(
            impl_->fence.Get(),
            submittedFenceValue);
        if (SUCCEEDED(signalResult))
        {
            impl_->fenceValues[impl_->frameIndex] = submittedFenceValue;
        }
        impl_->frameOpen = false;
        impl_->frameIndex = impl_->swapChain->GetCurrentBackBufferIndex();

        const bool presentFailed = report_failure(presentResult, "Present D3D12 frame");
        const bool signalFailed = report_failure(signalResult, "Signal D3D12 frame fence");
        return !presentFailed && !signalFailed;
    }

    void D3D12RenderEngine::shutdown()
    {
        if (!impl_)
        {
            return;
        }

        impl_->frameOpen = false;
        if (impl_->commandQueue && impl_->fence && impl_->fenceEvent != nullptr)
        {
            impl_->wait_for_gpu();
        }

        for (auto& renderTarget : impl_->renderTargets)
        {
            renderTarget.Reset();
        }
        for (auto& allocator : impl_->commandAllocators)
        {
            allocator.Reset();
        }
        impl_->commandList.Reset();
        impl_->rtvHeap.Reset();
        impl_->swapChain.Reset();
        impl_->commandQueue.Reset();
        impl_->fence.Reset();

#if defined(_DEBUG)
        if (impl_->device)
        {
            ComPtr<ID3D12DebugDevice> debugDevice;
            if (SUCCEEDED(impl_->device.As(&debugDevice)))
            {
                constexpr auto liveObjectFlags = static_cast<D3D12_RLDO_FLAGS>(
                    D3D12_RLDO_DETAIL | D3D12_RLDO_IGNORE_INTERNAL);
                debugDevice->ReportLiveDeviceObjects(liveObjectFlags);
            }
        }
#endif

        impl_->device.Reset();
        impl_->factory.Reset();

        if (impl_->fenceEvent != nullptr)
        {
            CloseHandle(impl_->fenceEvent);
            impl_->fenceEvent = nullptr;
        }

        impl_->fenceValues.fill(0);
        impl_->frameIndex = 0;
        impl_->nextFenceValue = 1;
        impl_->rtvDescriptorSize = 0;
        impl_->width = 0;
        impl_->height = 0;
        impl_->initialized = false;
        impl_->renderTargetActive = false;
    }
}
