#pragma once

#if defined(_WIN32)
#    if defined(SOFTPHYSICS_ENGINE_EXPORTS)
#        define SOFTPHYSICS_ENGINE_API __declspec(dllexport)
#    else
#        define SOFTPHYSICS_ENGINE_API __declspec(dllimport)
#    endif
#elif defined(__GNUC__) || defined(__clang__)
#    define SOFTPHYSICS_ENGINE_API __attribute__((visibility("default")))
#else
#    define SOFTPHYSICS_ENGINE_API
#endif
