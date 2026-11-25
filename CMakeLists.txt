# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.21)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("SoftBodyPhysicsEnginePrototype")

# Enable standard C++ exception unwinding for every C++ target in this build,
# including the test-bed host and any dependencies that compile C++ sources.
add_compile_options ("$<$<AND:$<CXX_COMPILER_ID:MSVC>,$<COMPILE_LANGUAGE:CXX>>:/EHsc>")

include (FetchContent)

set (SDL_TESTS OFF CACHE BOOL "" FORCE)
set (SDL_EXAMPLES OFF CACHE BOOL "" FORCE)

FetchContent_Declare (
  SDL3
  GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
  GIT_TAG release-3.4.20
  GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable (SDL3)

# cgltf is a small single-header glTF 2.0 parser used by the engine's asset loader.
FetchContent_Declare (
  cgltf
  GIT_REPOSITORY https://github.com/jkuhlmann/cgltf.git
  GIT_TAG v1.15
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable (cgltf)

# Include sub-projects.
add_subdirectory ("SoftBodyPhysicsEnginePrototype" "engine")
