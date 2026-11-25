#if defined(_MSC_VER) && defined(_DEBUG)
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#endif

#include <softphysics/EngineApplication.h>

#include <iostream>

int main()
{
#if defined(_MSC_VER) && defined(_DEBUG)
    const int debugFlags = _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG);
    _CrtSetDbgFlag(debugFlags | _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif

    // Keep this executable thin: games should call the same public DLL API.
    softphysics::engine::EngineApplication application;
    if (!application.initialize())
    {
        std::cerr << "Engine initialization failed.\n";
        return 1;
    }

    const int exitCode = application.run();
    application.shutdown();
    return exitCode;
}
