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
    esp-sim        flight session runtime, autopilot and simulator variables
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

Two rules follow from this arrangement and are not negotiable:

- `esp-physics` and `esp-sim` never depend on `esp-render` or `esp-ui`.
- `esp-ui` and `esp-launcher` never contain simulation logic.
- The app layer is the only place that knows every crate. Everything below it
  can be built, tested and used on its own.

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
object loading will move together into `esp-scenery` when terrain is introduced.

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
- `esp-options`, `esp-content`, `esp-ui`, `esp-launcher` (app side). Settings
  model with persistence; a content manager that installs and enables add-ons
  without touching the base simulator directories; the launcher with install
  selection, controls setup and graphics options.
- `esp-effects` (engine). Post-processing passes (tone mapping, bloom, glare,
  weather) as an optional chain on top of `esp-render`.
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
