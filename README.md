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

The project now has a first physics and deformation proof of concept. The
Windows test bed currently:

- Opens an SDL window and renders through a Direct3D 12 adapter.
- Loads the torso GLB from `assets/meshes/Torso_Test_Verticies.glb`.
- Creates a simple red sphere in code to stand in for a glove.
- Animates a fixed straight punch by default; press **R** to toggle a seeded
  sequence of straight punches, left/right hooks, uppercuts, and overhands.
- Varies hit locations and approach vectors in randomized mode. The same seed
  makes each run repeatable for debugging.
- Randomizes each randomized punch's impact force from 0 to 1 N; that setting
  also scales its impact impulse, while the fixed straight punch keeps its
  baseline strength.
- Lets you move the test camera with **WASD** and rotate it with the **arrow keys**.
- Uses a fixed 120 Hz simulation step while rendering through the D3D12 adapter.
- Detects glove contact against the current deforming torso triangles and
  applies a non-penetration constraint during the XPBD solve.
- Keeps normal and tangential impact impulses, so punch speed and direction
  affect both the local deformation and the torso's motion.
- Releases the torso's temporary lower-body support above a configurable impact
  impulse; gravity and a simple floor constraint then allow it to move and tip.
- Welds coincident render vertices into a simulation surface, creates a simple
  center-fan tetrahedral volume, and solves edge-length and volume constraints
  with XPBD.
- Applies a localized contact force to nearby body vertices and updates the
  rendered mesh positions and normals.
- Exposes the engine through a DLL entry point, separate from the test-bed
  executable.

### Testbed controls

- **W / A / S / D** — move the camera forward, left, backward, and right.
- **Arrow keys** — rotate the camera view (left/right yaw, up/down pitch).
- **R** — toggle randomized punch profiles. The selected mode is used when
  the next punch profile is queued; the test bed reports the new mode in the
  console.
- **T** — reset the torso, restore its support and physics state, and restart
  the punch cycle.
- **Close the window** — exit the test bed.

The glove still follows a scripted kinematic path (fixed straight by default,
with randomized attack profiles available). The balance response is a simple
support-release threshold and floor plane, not a full fighter skeleton,
articulated balance controller, or rigid-body character simulation. The
generated tetrahedra use one interior center point; this is a prototype for a
star-shaped, closed mesh, not a general tetrahedral mesher. There is no
self-collision, glove reaction, skeletal attachment, or grappling simulation
yet. The current importer supports a single root-level GLB mesh with triangle
primitives and position/normal data; it is intentionally a small first step
rather than a general-purpose asset pipeline.

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

1. Replace the torso AABB with a collider that better follows the body shape and
   can update as the body deforms.
2. Inspect and tune contact, stiffness, damping, and mass parameters against
   repeatable punch cases.
3. Replace center-fan tetrahedra with a proper interior tetrahedral mesh and
   validate that the solver handles the torso without inverted elements.
4. Add self-collision and connect the body to a simple skeletal/rigid support.
5. Profile the fixed-step solver, then expand toward a full fighter and arena.

Prototype geometry is treated as meter-scale, with +Y up and +Z forward. The GLB
loader converts glTF's right-handed coordinates into that engine convention.

The early work should favor clear, inspectable experiments over premature
generalization. Physics APIs and renderer interfaces can expand as those
experiments reveal what the engine needs.
