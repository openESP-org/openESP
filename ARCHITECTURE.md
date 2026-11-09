# openESP architecture

openESP is a flight simulator built to run FSX and Prepar3D-era content:
aircraft, scenery, gauges and panel configuration from those ecosystems load
unmodified. This document describes how the code is arranged and which rules
hold permanently. Code standards live in `CODE_STANDARDS.md`.

## Layers

The workspace is a cargo workspace of small crates arranged in three layers.
A crate may only depend on crates at the same layer or below. No crate above
the app layer is allowed to depend on another crate above it.

```
app layer
    openesp        binary: command line, frame loop, input, camera, wiring

engine layer
    esp-render     wgpu renderer
    esp-audio      sound mixing and output
    esp-physics    reusable, session-free solver models
    esp-sim        flight session, loading, systems, animation, picking and variables
    esp-assets     aircraft, model and gauge loading
    esp-scenery    airport surfaces and placed scenery objects
    esp-terrain    elevation mesh, land class and photoreal ground
    esp-nav        VOR, ILS, DME and waypoint facilities
    esp-script     gauge calculator and script interpreter

foundation layer
    esp-formats    binary decoders: BGL, MDL, AIR, DDS, I8N
    esp-cfg        ini and xml configuration files from FSX and Prepar3D
    esp-installs   detection and inspection of installed simulator versions
    esp-geo        coordinate frames and precision-critical math
```

These rules follow from this arrangement and are not negotiable:

- `esp-physics` and `esp-sim` never depend on `esp-render` or `esp-ui`.
- `esp-ui` and `esp-launcher` never contain simulation logic.
- The app layer is the only place that knows every crate. Everything below it
  can be built, tested and used on its own.

The partition remains in progress: several physics solver constructors still
read aircraft content. Those constructors must move behind content adapters
before the physics crate is fully content-agnostic.
The propeller, piston-cycle and turbine content adapters live in `esp-sim::content`;
they decode AIR and configuration data into parameters for `esp-physics`.
The same runtime adapter selects the appropriate propulsion model from content.
Physics lookup curves and grids accept numerical values; the remaining AIR
record conversions are isolated in the transitional `content_tables` module.

The session owns its flight state. Physics and navigation consume `Motion`
snapshots; rigid-body integration returns a new snapshot and the session
commits it only on success. A mutable snapshot convenience method remains for
existing solver tests and callers.

Crates that do not exist yet are listed in their scheduled order below. When a
domain earns its promotion, it moves out of its current module with no
functional change, in its own commit.

## Size before the first split

| crate       | files | lines |
|-------------|------:|------:|
| esp-physics     |    30 | 13005 |
| esp-formats |    57 |  9995 |
| openesp     |     7 |  8600 |
| esp-assets  |    13 |  6408 |
| esp-render  |     3 |  1434 |
| esp-script  |     3 |   976 |

The app bootstrap, CLI and flight code are split into modules. Navigation and
VOR calculations live in `esp-nav`; cross-system integration tests stay in
the app because they exercise flight and weather together.

`app/scenery_folder.rs` resolves a CLI add-on folder to the existing airport
and object loaders before `app/spawn.rs` resolves the selected runway or parking
stand. It reads content without modifying the add-on. Airport surface and
object loading will move together into `esp-scenery` as that domain grows.
Placed objects use the scene's mean-sea-level datum, and above-ground placements
sample the same terrain and airport surface footprints used by flight contact.

`esp-terrain` decodes and selects geographic imagery tiles independently of
the renderer. The app projects selected bounds into its local scene; the
renderer uploads and composites the imagery beneath airport surfaces.
The initial elevation mesh samples registered simulator DEM tiles over a fixed
area around the starting position; photo tiles are subdivided onto that mesh.
Terrain elevation is rebuilt on a worker as the aircraft travels and swapped
into both the renderer and flight runtime in the same frame. This uses a moving
local mesh rather than a full worldwide tile cache. Base land-class rasters
are sampled into the mesh and given approximate fallback colours. The terrain
engine resolves region and land class to base texture sets, which the renderer
tiles over the mesh, with interpolated transitions between adjacent texture
classes. Monthly geographic season rasters select the matching seasonal
texture variant, with summer as a fallback; streaming airport scenery and
photos remains unfinished. The flight runtime
uses runway, taxiway and apron footprints for flat airport surfaces.
Flight positions remain local to the scene datum; instruments, navigation,
autopilot and atmospheric calculations add that datum to obtain mean-sea-level
altitude, while ground clearance is measured against the terrain mesh.
`esp-render::ground` validates and uploads either the terrain geometry or the
fallback grid. `esp-render::draw` creates mesh buffers and material bindings.

`esp-physics::solar` computes the sun's local direction from UTC and position.
The app updates that direction from the live flight position; `esp-render`
uses it for surface lighting and atmospheric sky scattering.
Legacy and PBR emissive maps are loaded as colour textures and added to the
material lighting. Legacy emissive modes and panel-light control are not yet
decoded; the current additive treatment is an approximation.

Aircraft sound resources are prepared on the aircraft loading worker. The app
owns the audio output and supplies engine state and view selection to the
`esp-audio` mixer; device or content failures leave the flight running silently.
Exterior engine layers use aircraft-configured source locations and camera
position for stereo panning and distance attenuation. Authored directional
cones apply in aircraft coordinates, including their inside/outside angles and
outside volume. This is a stereo model, not HRTF or an acoustic simulation.
Gear and flap sound lists trigger from actual actuator movement, with state
committed only after audio controls are queued successfully. Individual wheel
contacts trigger their configured touchdown sound categories. Airport BGL
surface types and airspeed select and modulate ground-roll sound layers; terrain
outside airport surfaces currently falls back to the first ground-roll layer.
Wind layers use true airspeed and their authored cockpit/exterior viewpoints,
speed curves and sound lists.

`esp-installs` reads simulator installation paths from the Windows registry;
it does not modify the registry or simulator directories. The app uses a
registered install for base terrain when no explicit simulator root is set.

`esp-formats` builds the MIT-licensed flightsimlib PTC decoder from vendored C
sources under `native/ptc`. A fixed-size photo bridge validates container,
frame and tile ranges before decoding 256x256 ARGB1555 images. Rust owns the
input and output buffers. Other PTC formats are rejected explicitly. Building
from source therefore requires a C compiler alongside the Rust toolchain.

## First steps

1. Done: `main.rs` is a short bootstrap file. Its contents moved to `cli.rs`
   (argument definitions and parsers) and `app/` feature modules (spawn,
   input, camera).
2. Done: `flight.rs` became a `flight/` directory, with flight tests grouped
   by subsystem.
3. Done: `navigation.rs` and `vor.rs` moved to the `esp-nav` crate.
4. `esp-render`'s `lib.rs` splits into `mesh.rs`, `pipeline.rs`, `ground.rs`,
   `runway.rs` and `objects.rs` as each area grows.

## New crates, in build order

- `esp-cfg` and `esp-installs` (foundation). Reading `fsx.cfg`, `scenery.cfg`,
  `dll.xml`, `addon.xml` and registry entries for FSX, FSX-SE and Prepar3D
  versions. Everything else consumes these two; build them before the launcher.
- `esp-scenery` and `esp-terrain` (engine). Airport surfaces and placed objects
  move out of `openesp`; terrain arrives behind the same interface the flat
  ground already implements, so grid and terrain worlds are interchangeable.
- `esp-nav` (engine, created). Adds the magnetic model and flight plan support.
- `esp-input` (engine). Device axes, bindings and mapping data.
- `esp-options`, `esp-content`, `esp-ui`, `esp-launcher` (app side). Settings
  model with persistence; a content manager that installs and enables add-ons
  without touching the base simulator directories; the launcher with install
  selection, controls setup and graphics options.
- `esp-weather` (engine). Weather state and data for physics and rendering.
- Effects start as an `esp-render` module; promote them to `esp-effects` only
  if the post-processing chain grows beyond the file cap.
- `tools/esp-check`. A validation binary that loads aircraft and scenery
  packages and reports every decode failure, for content authors and debugging.

## Precision rules

The rules behind the ground shimmer fix apply to everything written from now
on, including terrain and scenery code.

- World positions are `f64` until the last possible moment. Anything handed
  to the GPU is camera-relative `f32`.
- Repeating surfaces (ground grids, runway markings) anchor to integer world
  offsets, snapped once per frame.
- The simulator advances fixed `1/60` second ticks. Rendering interpolates
  between ticks with a frame alpha. Ticking and rendering are changed together
  or not at all.
