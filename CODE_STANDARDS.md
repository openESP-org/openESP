# Code standards

These apply to every commit. A change that only holds outside them is a change
that has to justify itself first.

## Format and lint

- `cargo fmt --all` and `cargo clippy --workspace` pass before every push.
- Errors use `anyhow`, with enough context where they are raised that the
  message identifies the file and the thing that failed. Panic only in tests
  and for genuinely impossible states.

## Layout

- The dependency direction in `ARCHITECTURE.md` is binding. A crate never
  depends on the app layer.
- Feature logic lives in feature modules. A file crossing about 1,500 lines,
   or a module serving several clearly different purposes, is split. `main.rs`
  stays a bootstrap file.
- Long files are split by job, not by size: one file per subsystem with its
  tests beside it, not one giant file with a test appendix.

## Tests

- Correctness that can be stated as an invariant is asserted against a test in
  the same crate. Content-format code is tested against real FSX and Prepar3D
  content through `tests/support/content.rs`; the aircraft path environment
  (`OPENESP_AIRCRAFT_PATHS`) is the standard way to run those.
- A fix to a rendering or physics bug adds a test that fails without the fix.
- Instrumentation added to diagnose a bug is removed before the commit. A
  `dbg!` or logging statement reaching main is a review failing point.

## Numerics

- `f64` for all simulation and world geometry. `f32` only at the point where
  data enters the GPU, already made camera-relative.
- Physical quantities keep SI units in structure fields; conversions to
  display units happen at the presentation edge, never inside solvers.

## Simulation changes

- Physics changes must not break the fixed-timestep contract: content and
  saved states are modified only when the plan in `ARCHITECTURE.md` says so.
- Lifecycle, fuel, gear and electrical changes are verified against the
  included aircraft with `cargo test -p openesp --bin openesp` before push.

## Commits

- One commit covers one purpose. Mixed feature and fix commits are split.
- Messages are one line, lowercase, `type(scope): summary`:

      fix(render): keep ground grid anchored under camera motion
      feat(sim): model alternator output from engine speed
      refactor(openesp): move spawn handling into app module

- Types are `feat`, `fix`, `refactor`, `improve`, `test`, `docs`, `build`.
  No scope when the crate is obvious; parentheses empty otherwise.
- Nothing half-working is pushed: code either passes its tests and starts, or
  it stays uncommitted. Work in progress belongs on a branch, not on main
  untested and uncommitted at once.
- Strangers' uncommitted changes in the tree stay out of commits you do not
  own. Check `git status` before staging anything.

## Documentation

- `ARCHITECTURE.md` records structure decisions; update it in the same commit
  that adds, removes or renames a crate.
- Source comments explain why, not what. Header comment blocks do not narrate
  the history of the file.
