#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#include <d3d12.h>
#include <d3d12sdklayers.h>
#include <d3dcompiler.h>
#include <dxgi1_6.h>
#include <wrl/client.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <utility>

#include "D3D12RenderEngine.h"

namespace softphysics::renderengine::windows::d3d12
{
    using Microsoft::WRL::ComPtr;

    namespace
    {
        constexpr UINT FrameCount = 2;
        constexpr DXGI_FORMAT BackBufferFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
        constexpr DXGI_FORMAT DepthBufferFormat = DXGI_FORMAT_D32_FLOAT;

        struct Matrix4
        {
            std::array<float, 16> values{};

            float& at(std::size_t row, std::size_t column) { return values[row * 4 + column]; }
            float at(std::size_t row, std::size_t column) const { return values[row * 4 + column]; }
        };

        struct DrawConstants
        {
            Matrix4 modelViewProjection;
            // HLSL row-major float3x3 rows each occupy four 32-bit constants.
            std::array<float, 12> normalMatrix{};
        };

        Matrix4 identity_matrix()
        {
            Matrix4 result{};
            result.at(0, 0) = 1.0f;
            result.at(1, 1) = 1.0f;
            result.at(2, 2) = 1.0f;
            result.at(3, 3) = 1.0f;
            return result;
        }

        Matrix4 multiply(const Matrix4& left, const Matrix4& right)
        {
            Matrix4 result{};
            for (std::size_t row = 0; row < 4; ++row)
            {
                for (std::size_t column = 0; column < 4; ++column)
                {
                    for (std::size_t index = 0; index < 4; ++index)
                    {
                        result.at(row, column) += left.at(row, index) * right.at(index, column);
                    }
                }
            }
            return result;
        }

        float dot(softphysics::renderengine::Vector3 left, softphysics::renderengine::Vector3 right)
        {
            return left.x * right.x + left.y * right.y + left.z * right.z;
        }

        softphysics::renderengine::Vector3 subtract(
            softphysics::renderengine::Vector3 left,
            softphysics::renderengine::Vector3 right)
        {
            return {left.x - right.x, left.y - right.y, left.z - right.z};
        }

        softphysics::renderengine::Vector3 cross(
            softphysics::renderengine::Vector3 left,
            softphysics::renderengine::Vector3 right)
        {
            return {
                left.y * right.z - left.z * right.y,
                left.z * right.x - left.x * right.z,
                left.x * right.y - left.y * right.x};
        }

        bool normalize(softphysics::renderengine::Vector3& value)
        {
            const float lengthSquared = dot(value, value);
            if (!std::isfinite(lengthSquared) || lengthSquared < 1.0e-8f)
            {
                return false;
            }
            const float inverseLength = 1.0f / std::sqrt(lengthSquared);
            value.x *= inverseLength;
            value.y *= inverseLength;
            value.z *= inverseLength;
            return true;
        }

        Matrix4 make_model_matrix(const softphysics::renderengine::Transform3D& transform)
        {
            const float cx = std::cos(transform.rotationRadians.x);
            const float sx = std::sin(transform.rotationRadians.x);
            const float cy = std::cos(transform.rotationRadians.y);
            const float sy = std::sin(transform.rotationRadians.y);
            const float cz = std::cos(transform.rotationRadians.z);
            const float sz = std::sin(transform.rotationRadians.z);

            Matrix4 scale = identity_matrix();
            scale.at(0, 0) = transform.scale.x;
            scale.at(1, 1) = transform.scale.y;
            scale.at(2, 2) = transform.scale.z;

            Matrix4 rotationX = identity_matrix();
            rotationX.at(1, 1) = cx;
            rotationX.at(1, 2) = -sx;
            rotationX.at(2, 1) = sx;
            rotationX.at(2, 2) = cx;

            Matrix4 rotationY = identity_matrix();
            rotationY.at(0, 0) = cy;
            rotationY.at(0, 2) = sy;
            rotationY.at(2, 0) = -sy;
            rotationY.at(2, 2) = cy;

            Matrix4 rotationZ = identity_matrix();
            rotationZ.at(0, 0) = cz;
            rotationZ.at(0, 1) = -sz;
            rotationZ.at(1, 0) = sz;
            rotationZ.at(1, 1) = cz;

            Matrix4 translation = identity_matrix();
            translation.at(0, 3) = transform.position.x;
            translation.at(1, 3) = transform.position.y;
            translation.at(2, 3) = transform.position.z;

            return multiply(
                translation,
                multiply(rotationZ, multiply(rotationY, multiply(rotationX, scale))));
        }

        bool make_normal_matrix(const Matrix4& model, std::array<float, 12>& normalMatrix)
        {
            const float a = model.at(0, 0);
            const float b = model.at(0, 1);
            const float c = model.at(0, 2);
            const float d = model.at(1, 0);
            const float e = model.at(1, 1);
            const float f = model.at(1, 2);
            const float g = model.at(2, 0);
            const float h = model.at(2, 1);
            const float i = model.at(2, 2);

            const float determinant = a * (e * i - f * h) -
                                      b * (d * i - f * g) +
                                      c * (d * h - e * g);
            if (!std::isfinite(determinant) || std::abs(determinant) < 1.0e-8f)
            {
                return false;
            }

            const float inverseDeterminant = 1.0f / determinant;
            normalMatrix = {
                (e * i - f * h) * inverseDeterminant,
                (f * g - d * i) * inverseDeterminant,
                (d * h - e * g) * inverseDeterminant,
                0.0f,
                (c * h - b * i) * inverseDeterminant,
                (a * i - c * g) * inverseDeterminant,
                (b * g - a * h) * inverseDeterminant,
                0.0f,
                (b * f - c * e) * inverseDeterminant,
                (c * d - a * f) * inverseDeterminant,
                (a * e - b * d) * inverseDeterminant,
                0.0f};
            return true;
        }

        bool make_view_projection(
            const softphysics::renderengine::Camera3D& camera,
            float aspectRatio,
            Matrix4& viewProjection)
        {
            if (!std::isfinite(camera.verticalFovRadians) ||
                camera.verticalFovRadians <= 0.01f || camera.verticalFovRadians >= 3.13f ||
                !std::isfinite(aspectRatio) || aspectRatio <= 0.0f ||
                !std::isfinite(camera.nearPlane) || !std::isfinite(camera.farPlane) ||
                camera.nearPlane <= 0.0f || camera.farPlane <= camera.nearPlane)
            {
                return false;
            }

            auto forward = subtract(camera.target, camera.position);
            auto right = cross(camera.up, forward);
            if (!normalize(forward) || !normalize(right))
            {
                return false;
            }
            const auto up = cross(forward, right);

            Matrix4 view = identity_matrix();
            view.at(0, 0) = right.x;
            view.at(0, 1) = right.y;
            view.at(0, 2) = right.z;
            view.at(0, 3) = -dot(right, camera.position);
            view.at(1, 0) = up.x;
            view.at(1, 1) = up.y;
            view.at(1, 2) = up.z;
            view.at(1, 3) = -dot(up, camera.position);
            view.at(2, 0) = forward.x;
            view.at(2, 1) = forward.y;
            view.at(2, 2) = forward.z;
            view.at(2, 3) = -dot(forward, camera.position);

            const float yScale = 1.0f / std::tan(camera.verticalFovRadians * 0.5f);
            const float xScale = yScale / aspectRatio;
            const float depthRange = camera.farPlane - camera.nearPlane;
            Matrix4 projection{};
            projection.at(0, 0) = xScale;
            projection.at(1, 1) = yScale;
            projection.at(2, 2) = camera.farPlane / depthRange;
            projection.at(2, 3) = -(camera.nearPlane * camera.farPlane) / depthRange;
            projection.at(3, 2) = 1.0f;

            viewProjection = multiply(projection, view);
            return true;
        }

        constexpr char MeshVertexShader[] = R"(
cbuffer DrawConstants : register(b0)
{
    row_major float4x4 modelViewProjection;
    row_major float3x3 normalMatrix;
};

struct VertexInput
{
    float3 position : POSITION;
    float3 normal : NORMAL;
    float4 color : COLOR;
};

struct VertexOutput
{
    float4 position : SV_POSITION;
    float3 normal : NORMAL;
    float4 color : COLOR;
};

VertexOutput VSMain(VertexInput input)
{
    VertexOutput output;
    output.position = mul(modelViewProjection, float4(input.position, 1.0));
    output.normal = mul(normalMatrix, input.normal);
    output.color = input.color;
    return output;
}
)";

        constexpr char MeshPixelShader[] = R"(
float4 PSMain(
    float4 position : SV_POSITION,
    float3 interpolatedNormal : NORMAL,
    float4 color : COLOR) : SV_TARGET
{
    float3 normal = normalize(interpolatedNormal);
    float3 lightDirection = normalize(float3(-0.4, 0.8, 0.6));
    float diffuse = saturate(dot(normal, lightDirection));
    float lighting = 0.25 + 0.75 * diffuse;
    return float4(color.rgb * lighting, color.a);
}
)";

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
        struct MeshGpuData
        {
            std::array<ComPtr<ID3D12Resource>, FrameCount> vertexBuffers;
            ComPtr<ID3D12Resource> indexBuffer;
            UINT vertexCount = 0;
            UINT indexCount = 0;
        };

        explicit Impl(softphysics::window::IWin32WindowAccess& access)
            : windowAccess(access)
        {
        }

        softphysics::window::IWin32WindowAccess& windowAccess;
        ComPtr<IDXGIFactory4> factory;
        ComPtr<ID3D12Device> device;
        ComPtr<ID3D12RootSignature> rootSignature;
        ComPtr<ID3D12PipelineState> pipelineState;
        ComPtr<ID3D12DescriptorHeap> dsvHeap;
        ComPtr<ID3D12Resource> depthBuffer;
        std::unordered_map<std::uint32_t, MeshGpuData> meshes;
        std::uint32_t nextMeshId = 1;
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
        softphysics::renderengine::Camera3D camera;
        Matrix4 viewProjection{};
        UINT width = 0;
        UINT height = 0;
        bool initialized = false;
        bool frameOpen = false;
        bool renderTargetActive = false;

        bool update_view_projection()
        {
            if (height == 0)
            {
                return false;
            }
            return make_view_projection(
                camera,
                static_cast<float>(width) / static_cast<float>(height),
                viewProjection);
        }

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

        bool create_depth_buffer()
        {
            if (!dsvHeap)
            {
                D3D12_DESCRIPTOR_HEAP_DESC heapDesc{};
                heapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
                heapDesc.NumDescriptors = 1;
                if (report_failure(
                        device->CreateDescriptorHeap(&heapDesc, IID_PPV_ARGS(&dsvHeap)),
                        "Create depth-stencil descriptor heap"))
                {
                    return false;
                }
            }

            depthBuffer.Reset();
            D3D12_HEAP_PROPERTIES heapProperties{};
            heapProperties.Type = D3D12_HEAP_TYPE_DEFAULT;
            heapProperties.CreationNodeMask = 1;
            heapProperties.VisibleNodeMask = 1;

            D3D12_RESOURCE_DESC depthDesc{};
            depthDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
            depthDesc.Width = width;
            depthDesc.Height = height;
            depthDesc.DepthOrArraySize = 1;
            depthDesc.MipLevels = 1;
            depthDesc.Format = DepthBufferFormat;
            depthDesc.SampleDesc.Count = 1;
            depthDesc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
            depthDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;

            D3D12_CLEAR_VALUE clearValue{};
            clearValue.Format = DepthBufferFormat;
            clearValue.DepthStencil.Depth = 1.0f;
            clearValue.DepthStencil.Stencil = 0;
            if (report_failure(
                    device->CreateCommittedResource(
                        &heapProperties,
                        D3D12_HEAP_FLAG_NONE,
                        &depthDesc,
                        D3D12_RESOURCE_STATE_DEPTH_WRITE,
                        &clearValue,
                        IID_PPV_ARGS(&depthBuffer)),
                    "Create depth buffer"))
            {
                return false;
            }

            device->CreateDepthStencilView(
                depthBuffer.Get(),
                nullptr,
                dsvHeap->GetCPUDescriptorHandleForHeapStart());
            return true;
        }

        bool create_upload_buffer(const void* source, UINT64 size, ComPtr<ID3D12Resource>& buffer)
        {
            D3D12_HEAP_PROPERTIES heapProperties{};
            heapProperties.Type = D3D12_HEAP_TYPE_UPLOAD;
            heapProperties.CreationNodeMask = 1;
            heapProperties.VisibleNodeMask = 1;

            D3D12_RESOURCE_DESC resourceDesc{};
            resourceDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
            resourceDesc.Width = size;
            resourceDesc.Height = 1;
            resourceDesc.DepthOrArraySize = 1;
            resourceDesc.MipLevels = 1;
            resourceDesc.SampleDesc.Count = 1;
            resourceDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

            if (report_failure(
                    device->CreateCommittedResource(
                        &heapProperties,
                        D3D12_HEAP_FLAG_NONE,
                        &resourceDesc,
                        D3D12_RESOURCE_STATE_GENERIC_READ,
                        nullptr,
                        IID_PPV_ARGS(&buffer)),
                    "Create mesh upload buffer"))
            {
                return false;
            }

            D3D12_RANGE noReadRange{0, 0};
            void* mappedData = nullptr;
            if (report_failure(buffer->Map(0, &noReadRange, &mappedData), "Map mesh upload buffer"))
            {
                return false;
            }
            std::memcpy(mappedData, source, static_cast<std::size_t>(size));
            buffer->Unmap(0, nullptr);
            return true;
        }

        bool create_mesh_pipeline()
        {
            ComPtr<ID3DBlob> vertexShader;
            ComPtr<ID3DBlob> pixelShader;
            ComPtr<ID3DBlob> errors;
            HRESULT result = D3DCompile(
                MeshVertexShader,
                sizeof(MeshVertexShader) - 1,
                nullptr,
                nullptr,
                nullptr,
                "VSMain",
                "vs_5_0",
                D3DCOMPILE_ENABLE_STRICTNESS,
                0,
                &vertexShader,
                &errors);
            if (FAILED(result))
            {
                if (errors)
                {
                    std::cerr << "Compile mesh vertex shader failed: "
                              << static_cast<const char*>(errors->GetBufferPointer()) << "\n";
                }
                else
                {
                    report_failure(result, "Compile mesh vertex shader");
                }
                return false;
            }

            errors.Reset();
            result = D3DCompile(
                MeshPixelShader,
                sizeof(MeshPixelShader) - 1,
                nullptr,
                nullptr,
                nullptr,
                "PSMain",
                "ps_5_0",
                D3DCOMPILE_ENABLE_STRICTNESS,
                0,
                &pixelShader,
                &errors);
            if (FAILED(result))
            {
                if (errors)
                {
                    std::cerr << "Compile mesh pixel shader failed: "
                              << static_cast<const char*>(errors->GetBufferPointer()) << "\n";
                }
                else
                {
                    report_failure(result, "Compile mesh pixel shader");
                }
                return false;
            }

            D3D12_ROOT_PARAMETER rootParameter{};
            rootParameter.ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
            rootParameter.ShaderVisibility = D3D12_SHADER_VISIBILITY_VERTEX;
            rootParameter.Constants.ShaderRegister = 0;
            rootParameter.Constants.RegisterSpace = 0;
            rootParameter.Constants.Num32BitValues = 28;

            D3D12_ROOT_SIGNATURE_DESC rootSignatureDesc{};
            rootSignatureDesc.NumParameters = 1;
            rootSignatureDesc.pParameters = &rootParameter;
            rootSignatureDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;
            ComPtr<ID3DBlob> serializedRootSignature;
            errors.Reset();
            result = D3D12SerializeRootSignature(
                &rootSignatureDesc,
                D3D_ROOT_SIGNATURE_VERSION_1,
                &serializedRootSignature,
                &errors);
            if (FAILED(result))
            {
                if (errors)
                {
                    std::cerr << "Serialize mesh root signature failed: "
                              << static_cast<const char*>(errors->GetBufferPointer()) << "\n";
                }
                else
                {
                    report_failure(result, "Serialize mesh root signature");
                }
                return false;
            }

            if (report_failure(
                    device->CreateRootSignature(
                        0,
                        serializedRootSignature->GetBufferPointer(),
                        serializedRootSignature->GetBufferSize(),
                        IID_PPV_ARGS(&rootSignature)),
                    "Create mesh root signature"))
            {
                return false;
            }

            D3D12_GRAPHICS_PIPELINE_STATE_DESC pipelineDesc{};
            pipelineDesc.pRootSignature = rootSignature.Get();
            pipelineDesc.VS = {vertexShader->GetBufferPointer(), vertexShader->GetBufferSize()};
            pipelineDesc.PS = {pixelShader->GetBufferPointer(), pixelShader->GetBufferSize()};
            const D3D12_INPUT_ELEMENT_DESC inputElements[] = {
                {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0,
                 D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
                {"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12,
                 D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
                {"COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 24,
                 D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0}
            };
            pipelineDesc.InputLayout = {
                inputElements,
                static_cast<UINT>(sizeof(inputElements) / sizeof(inputElements[0]))};
            pipelineDesc.BlendState.AlphaToCoverageEnable = FALSE;
            pipelineDesc.BlendState.IndependentBlendEnable = FALSE;
            auto& renderTargetBlend = pipelineDesc.BlendState.RenderTarget[0];
            renderTargetBlend.BlendEnable = FALSE;
            renderTargetBlend.LogicOpEnable = FALSE;
            renderTargetBlend.SrcBlend = D3D12_BLEND_ONE;
            renderTargetBlend.DestBlend = D3D12_BLEND_ZERO;
            renderTargetBlend.BlendOp = D3D12_BLEND_OP_ADD;
            renderTargetBlend.SrcBlendAlpha = D3D12_BLEND_ONE;
            renderTargetBlend.DestBlendAlpha = D3D12_BLEND_ZERO;
            renderTargetBlend.BlendOpAlpha = D3D12_BLEND_OP_ADD;
            renderTargetBlend.LogicOp = D3D12_LOGIC_OP_NOOP;
            renderTargetBlend.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;

            pipelineDesc.SampleMask = std::numeric_limits<UINT>::max();
            pipelineDesc.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
            pipelineDesc.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
            pipelineDesc.RasterizerState.FrontCounterClockwise = FALSE;
            pipelineDesc.RasterizerState.DepthBias = D3D12_DEFAULT_DEPTH_BIAS;
            pipelineDesc.RasterizerState.DepthBiasClamp = D3D12_DEFAULT_DEPTH_BIAS_CLAMP;
            pipelineDesc.RasterizerState.SlopeScaledDepthBias = D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS;
            pipelineDesc.RasterizerState.DepthClipEnable = TRUE;
            pipelineDesc.RasterizerState.MultisampleEnable = FALSE;
            pipelineDesc.RasterizerState.AntialiasedLineEnable = FALSE;
            pipelineDesc.RasterizerState.ForcedSampleCount = 0;
            pipelineDesc.RasterizerState.ConservativeRaster = D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF;

            pipelineDesc.DepthStencilState.DepthEnable = TRUE;
            pipelineDesc.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
            pipelineDesc.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_LESS;
            pipelineDesc.DepthStencilState.StencilEnable = FALSE;
            pipelineDesc.DepthStencilState.StencilReadMask = D3D12_DEFAULT_STENCIL_READ_MASK;
            pipelineDesc.DepthStencilState.StencilWriteMask = D3D12_DEFAULT_STENCIL_WRITE_MASK;
            pipelineDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
            pipelineDesc.NumRenderTargets = 1;
            pipelineDesc.RTVFormats[0] = BackBufferFormat;
            pipelineDesc.DSVFormat = DepthBufferFormat;
            pipelineDesc.SampleDesc.Count = 1;

            return !report_failure(
                device->CreateGraphicsPipelineState(&pipelineDesc, IID_PPV_ARGS(&pipelineState)),
                "Create mesh pipeline state");
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
        if (!impl_->create_mesh_pipeline())
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
        if (!impl_->create_depth_buffer() || !impl_->update_view_projection())
        {
            std::cerr << "Could not initialize the camera or depth buffer.\n";
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
        impl_->depthBuffer.Reset();

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
        return impl_->create_render_targets() &&
               impl_->create_depth_buffer() &&
               impl_->update_view_projection();
    }

    bool D3D12RenderEngine::set_camera(const Camera3D& camera)
    {
        if (!impl_->initialized)
        {
            return false;
        }

        const auto previousCamera = impl_->camera;
        impl_->camera = camera;
        if (!impl_->update_view_projection())
        {
            impl_->camera = previousCamera;
            impl_->update_view_projection();
            std::cerr << "Camera parameters are invalid.\n";
            return false;
        }
        return true;
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
            const auto dsv = impl_->dsvHeap->GetCPUDescriptorHandleForHeapStart();
            impl_->commandList->OMSetRenderTargets(1, &rtv, FALSE, &dsv);
            impl_->renderTargetActive = true;
        }

        D3D12_CPU_DESCRIPTOR_HANDLE rtv = impl_->rtvHeap->GetCPUDescriptorHandleForHeapStart();
        rtv.ptr += static_cast<SIZE_T>(impl_->frameIndex) * impl_->rtvDescriptorSize;
        const float clearColor[] = {color.r, color.g, color.b, color.a};
        impl_->commandList->ClearRenderTargetView(rtv, clearColor, 0, nullptr);
        impl_->commandList->ClearDepthStencilView(
            impl_->dsvHeap->GetCPUDescriptorHandleForHeapStart(),
            D3D12_CLEAR_FLAG_DEPTH,
            1.0f,
            0,
            0,
            nullptr);
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

    MeshHandle D3D12RenderEngine::create_mesh(const MeshData& mesh)
    {
        if (!impl_->initialized || mesh.vertices.size() < 3 ||
            mesh.vertices.size() > std::numeric_limits<UINT>::max() ||
            mesh.vertices.size() > std::numeric_limits<UINT>::max() / sizeof(MeshVertex) ||
            mesh.indices.size() > std::numeric_limits<UINT>::max() ||
            mesh.indices.size() > std::numeric_limits<UINT>::max() / sizeof(std::uint32_t) ||
            (mesh.indices.empty() && mesh.vertices.size() % 3 != 0) ||
            (!mesh.indices.empty() && mesh.indices.size() % 3 != 0))
        {
            std::cerr << "Mesh must contain complete triangles and fit in D3D12 draw counts.\n";
            return {};
        }

        for (const std::uint32_t index : mesh.indices)
        {
            if (index >= mesh.vertices.size())
            {
                std::cerr << "Mesh index refers to a vertex outside the mesh.\n";
                return {};
            }
        }

        static_assert(sizeof(MeshVertex) == 40);
        static_assert(offsetof(MeshVertex, normal) == 12);
        static_assert(offsetof(MeshVertex, color) == 24);

        Impl::MeshGpuData gpuMesh;
        gpuMesh.vertexCount = static_cast<UINT>(mesh.vertices.size());
        gpuMesh.indexCount = static_cast<UINT>(mesh.indices.size());
        const UINT64 vertexBytes = static_cast<UINT64>(mesh.vertices.size() * sizeof(MeshVertex));
        for (auto& vertexBuffer : gpuMesh.vertexBuffers)
        {
            if (!impl_->create_upload_buffer(mesh.vertices.data(), vertexBytes, vertexBuffer))
            {
                return {};
            }
        }
        if (!mesh.indices.empty())
        {
            const UINT64 indexBytes = static_cast<UINT64>(mesh.indices.size() * sizeof(std::uint32_t));
            if (!impl_->create_upload_buffer(mesh.indices.data(), indexBytes, gpuMesh.indexBuffer))
            {
                return {};
            }
        }

        std::uint32_t id = impl_->nextMeshId;
        while (id == 0 || impl_->meshes.contains(id))
        {
            ++id;
            if (id == impl_->nextMeshId)
            {
                std::cerr << "No mesh handles are available.\n";
                return {};
            }
        }
        impl_->nextMeshId = id + 1;
        impl_->meshes.emplace(id, std::move(gpuMesh));
        return {id};
    }

    bool D3D12RenderEngine::update_mesh_vertices(
        MeshHandle mesh,
        const std::vector<MeshVertex>& vertices)
    {
        if (!impl_->initialized || !impl_->frameOpen || !mesh)
        {
            return false;
        }
        const auto found = impl_->meshes.find(mesh.id);
        if (found == impl_->meshes.end() || vertices.size() != found->second.vertexCount)
        {
            return false;
        }

        const UINT64 byteSize = static_cast<UINT64>(vertices.size() * sizeof(MeshVertex));
        D3D12_RANGE noReadRange{0, 0};
        void* mappedData = nullptr;
        auto& vertexBuffer = found->second.vertexBuffers[impl_->frameIndex];
        if (report_failure(vertexBuffer->Map(0, &noReadRange, &mappedData), "Map dynamic mesh vertex buffer"))
        {
            return false;
        }
        std::memcpy(mappedData, vertices.data(), static_cast<std::size_t>(byteSize));
        vertexBuffer->Unmap(0, nullptr);
        return true;
    }

    void D3D12RenderEngine::destroy_mesh(MeshHandle mesh)
    {
        if (!mesh || !impl_->meshes.contains(mesh.id))
        {
            return;
        }
        if (impl_->frameOpen)
        {
            std::cerr << "Cannot destroy a mesh while a renderer frame is open.\n";
            return;
        }
        if (impl_->initialized && !impl_->wait_for_gpu())
        {
            std::cerr << "Could not safely wait for GPU before destroying mesh.\n";
            return;
        }
        impl_->meshes.erase(mesh.id);
    }

    void D3D12RenderEngine::draw_mesh(MeshHandle mesh, const Transform3D& transform)
    {
        if (!impl_->initialized || !impl_->frameOpen || !impl_->renderTargetActive ||
            !impl_->pipelineState || !impl_->rootSignature || !mesh)
        {
            return;
        }

        const auto found = impl_->meshes.find(mesh.id);
        if (found == impl_->meshes.end())
        {
            return;
        }
        const auto& gpuMesh = found->second;
        const Matrix4 model = make_model_matrix(transform);
        DrawConstants constants{};
        constants.modelViewProjection = multiply(impl_->viewProjection, model);
        if (!make_normal_matrix(model, constants.normalMatrix))
        {
            std::cerr << "Cannot draw a mesh with a zero or invalid scale.\n";
            return;
        }
        static_assert(sizeof(DrawConstants) == 28 * sizeof(std::uint32_t));

        const D3D12_VIEWPORT viewport{
            0.0f,
            0.0f,
            static_cast<float>(impl_->width),
            static_cast<float>(impl_->height),
            0.0f,
            1.0f};
        const D3D12_RECT scissor{0, 0, static_cast<LONG>(impl_->width), static_cast<LONG>(impl_->height)};
        impl_->commandList->RSSetViewports(1, &viewport);
        impl_->commandList->RSSetScissorRects(1, &scissor);
        impl_->commandList->SetGraphicsRootSignature(impl_->rootSignature.Get());
        impl_->commandList->SetGraphicsRoot32BitConstants(
            0,
            static_cast<UINT>(sizeof(DrawConstants) / sizeof(std::uint32_t)),
            &constants,
            0);
        impl_->commandList->SetPipelineState(impl_->pipelineState.Get());
        impl_->commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

        const D3D12_VERTEX_BUFFER_VIEW vertexBufferView{
            gpuMesh.vertexBuffers[impl_->frameIndex]->GetGPUVirtualAddress(),
            gpuMesh.vertexCount * static_cast<UINT>(sizeof(MeshVertex)),
            static_cast<UINT>(sizeof(MeshVertex))};
        impl_->commandList->IASetVertexBuffers(0, 1, &vertexBufferView);
        if (gpuMesh.indexCount > 0)
        {
            const D3D12_INDEX_BUFFER_VIEW indexBufferView{
                gpuMesh.indexBuffer->GetGPUVirtualAddress(),
                gpuMesh.indexCount * static_cast<UINT>(sizeof(std::uint32_t)),
                DXGI_FORMAT_R32_UINT};
            impl_->commandList->IASetIndexBuffer(&indexBufferView);
            impl_->commandList->DrawIndexedInstanced(gpuMesh.indexCount, 1, 0, 0, 0);
        }
        else
        {
            impl_->commandList->DrawInstanced(gpuMesh.vertexCount, 1, 0, 0);
        }
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
        impl_->meshes.clear();
        impl_->nextMeshId = 1;
        impl_->depthBuffer.Reset();
        impl_->dsvHeap.Reset();
        impl_->pipelineState.Reset();
        impl_->rootSignature.Reset();
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
