#pragma once

#include <cstdint>
#include <memory>

#include "EngineExport.h"

namespace softphysics::engine
{
    struct EngineConfig
    {
        const char* windowTitle = "Soft Physics Engine Test Bed";
        std::uint32_t windowWidth = 1280;
        std::uint32_t windowHeight = 720;
        bool resizable = true;
        bool highDpi = true;
    };

    // Public runtime entry point. Platform systems and renderer implementations
    // stay behind this DLL boundary.
    class SOFTPHYSICS_ENGINE_API EngineApplication final
    {
    public:
        EngineApplication();
        ~EngineApplication();

        EngineApplication(const EngineApplication&) = delete;
        EngineApplication& operator=(const EngineApplication&) = delete;

        bool initialize(const EngineConfig& config = {});
        int run();
        void shutdown();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
