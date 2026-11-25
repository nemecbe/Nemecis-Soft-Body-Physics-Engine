# Shared engine library. Platform adapters and system graphics dependencies
# are selected below for the target operating system.
add_library (
  SoftPhysicsEngine SHARED
    "Engine/EngineApplication.cpp"
    "Assets/Gltf/GlbMeshLoader.h"
    "Assets/Gltf/GlbMeshLoader.cpp"
    "include/softphysics/EngineApplication.h"
    "include/softphysics/EngineExport.h"
    "RenderEngine/IRenderEngine.h"
    "RenderEngine/RenderTypes.h"
    "Window/IWindowSystem.h"
    "Window/WindowTypes.h"
    "Window/Platform/Win32/IWin32WindowAccess.h"
    "Window/SdlWindowSystem/SdlWindowSystem.h"
    "Window/SdlWindowSystem/SdlWindowSystem.cpp"
    "Core/Types/Types.h"
)

target_compile_features (SoftPhysicsEngine PRIVATE cxx_std_20)
target_compile_definitions (SoftPhysicsEngine PRIVATE SOFTPHYSICS_ENGINE_EXPORTS=1)
target_include_directories (
  SoftPhysicsEngine
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_include_directories (SoftPhysicsEngine PRIVATE "${cgltf_SOURCE_DIR}")
target_link_libraries (SoftPhysicsEngine PRIVATE SDL3::SDL3)

# Keep graphics dependencies limited to the platform being built.
add_library (SoftPhysicsPlatformGraphicsDependencies INTERFACE)

if (WIN32)
  set (SOFTPHYSICS_GRAPHICS_BACKEND "Direct3D 12")
  target_sources (
    SoftPhysicsEngine
    PRIVATE
      "RenderEngine/Adapters/Windows/D3D12/D3D12RenderEngine.h"
      "RenderEngine/Adapters/Windows/D3D12/D3D12RenderEngine.cpp"
  )
  target_compile_definitions (SoftPhysicsEngine PRIVATE SOFTPHYSICS_HAS_D3D12_RENDERER=1)
  target_link_libraries (SoftPhysicsPlatformGraphicsDependencies INTERFACE d3d12 dxgi d3dcompiler)
elseif (APPLE)
  set (SOFTPHYSICS_GRAPHICS_BACKEND "Metal")
  find_library (SOFTPHYSICS_METAL_FRAMEWORK Metal REQUIRED)
  find_library (SOFTPHYSICS_QUARTZCORE_FRAMEWORK QuartzCore REQUIRED)
  find_library (SOFTPHYSICS_APPKIT_FRAMEWORK AppKit REQUIRED)
  find_library (SOFTPHYSICS_FOUNDATION_FRAMEWORK Foundation REQUIRED)
  target_link_libraries (
    SoftPhysicsPlatformGraphicsDependencies
    INTERFACE
      ${SOFTPHYSICS_METAL_FRAMEWORK}
      ${SOFTPHYSICS_QUARTZCORE_FRAMEWORK}
      ${SOFTPHYSICS_APPKIT_FRAMEWORK}
      ${SOFTPHYSICS_FOUNDATION_FRAMEWORK}
  )
elseif (CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set (SOFTPHYSICS_GRAPHICS_BACKEND "Vulkan")
  find_package (Vulkan REQUIRED)
  target_link_libraries (SoftPhysicsPlatformGraphicsDependencies INTERFACE Vulkan::Vulkan)
else ()
  message (FATAL_ERROR "No graphics backend dependencies are configured for this target platform.")
endif ()

target_link_libraries (SoftPhysicsEngine PRIVATE SoftPhysicsPlatformGraphicsDependencies)
message (STATUS "Soft Physics graphics backend dependencies: ${SOFTPHYSICS_GRAPHICS_BACKEND}")

# The test bed is a separate host that consumes the same public API as a game.
add_executable (SoftPhysicsTestBed "SoftBodyPhysicsEnginePrototype.cpp")
target_compile_features (SoftPhysicsTestBed PRIVATE cxx_std_20)
target_link_libraries (SoftPhysicsTestBed PRIVATE SoftPhysicsEngine)

set (SOFTPHYSICS_TEST_TORSO_ASSET
  "${CMAKE_CURRENT_SOURCE_DIR}/../assets/meshes/Torso_Test_Verticies.glb")
add_custom_command (
  TARGET SoftPhysicsTestBed POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory
    "$<TARGET_FILE_DIR:SoftPhysicsTestBed>/assets/meshes"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${SOFTPHYSICS_TEST_TORSO_ASSET}"
    "$<TARGET_FILE_DIR:SoftPhysicsTestBed>/assets/meshes/Torso_Test_Verticies.glb"
  VERBATIM
)

if (WIN32)
  add_custom_command (
    TARGET SoftPhysicsTestBed POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:SDL3::SDL3>
      $<TARGET_FILE_DIR:SoftPhysicsTestBed>
    VERBATIM
  )
endif ()
