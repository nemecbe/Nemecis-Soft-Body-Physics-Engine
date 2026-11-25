# Soft Physics Engine Prototype

## Project goal

This project is an experiment in building a 3D game engine with a strong focus on
soft-body collision and deformation. The intended first game is a boxing game
with two fighters, wearables, and a boxing ring. Longer term, the engine should
support grappling and no-rules MMA mechanics.

The important gameplay goal is contact that changes the shape and motion of a
body: a glove should compress a torso on impact, and later grappling should
produce sustained squeezing and deformation. The target is a playable 60 FPS
game, but performance will be measured and improved as the simulation grows.

## Current state

The project is at the rendering and deformation proof-of-concept stage. The
Windows test bed currently:

- Opens an SDL window and renders through a Direct3D 12 adapter.
- Loads the torso GLB from `assets/meshes/Torso_Test_Verticies.glb`.
- Creates a simple red sphere in code to stand in for a glove.
- Animates the sphere toward and away from the torso.
- Applies a small procedural dent near a fixed contact point, then recalculates
  mesh normals so the dent is visible under lighting.
- Exposes the engine through a DLL entry point, separate from the test-bed
  executable.

The punch motion and dent are currently scripted. There is not yet a physics
simulation, collision detection, force-based response, mass or material model,
skeletal animation system, or grappling logic. The current importer supports a
single root-level GLB mesh with triangle primitives and position/normal data; it
is intentionally a small first step rather than a general-purpose asset
pipeline.

## Architecture direction

The code separates portable engine-facing types and interfaces from the current
Windows implementations:

- `Core/Types` holds common values such as drawable extents.
- `Window` defines window handles and events, with SDL providing the current
  window-system implementation.
- `RenderEngine` defines CPU-side mesh, camera, transform, and renderer
  interfaces. `Adapters/Windows/D3D12` is the current graphics backend.
- `Assets/Gltf` contains the first GLB mesh loader.
- `Engine/EngineApplication` owns the current application loop and demo setup.
- The top-level executable in `SoftBodyPhysicsEnginePrototype.cpp` is only a
  test host that calls the public DLL API.

These interfaces are an initial seam for future adapters, not a claim of
cross-platform support yet. Windows is the only implemented platform. Linux and
macOS will need their own window and rendering adapters, and the interfaces may
change as real use cases emerge.

## Build and run on Windows

Open the repository with Visual Studio 2022 as a CMake project, select the
`x64-debug` configure preset, then build and run `SoftPhysicsTestBed`. CMake
fetches SDL3 and cgltf during configuration. The build stages the torso GLB next
to the executable under `assets/meshes`.

The Windows renderer uses Direct3D 12 from the Windows SDK. The current Windows
implementation is the only renderer adapter configured in this prototype.

## Next engineering steps

1. Move the demo punch and deformation code out of the application loop into
   small simulation components.
2. Add basic collision shapes and contact data for the glove and torso.
3. Drive a localized deformation from the contact point, normal, and penetration
   or impact strength rather than a hard-coded point and timeline.
4. Introduce mass and material parameters, then tune stability and frame cost.
5. Build up from the torso test mesh toward a rigged fighter and more complete
   character/arena assets.

Prototype geometry is treated as meter-scale, with +Y up and +Z forward. The GLB
loader converts glTF's right-handed coordinates into that engine convention.

The early work should favor clear, inspectable experiments over premature
generalization. Physics APIs and renderer interfaces can expand as those
experiments reveal what the engine needs.
