# Loading add-on scenery

Pass an add-on root containing a `Scenery` directory, or pass the directory
containing its BGL files directly. Directory and extension matching accepts
Windows-style capitalization. The loader selects the requested airport and
reads placements and model libraries from every discovered object BGL.
Textures resolve from the add-on's sibling `Texture` directory.

```powershell
cargo run --release -p openesp -- --aircraft C:\Users\user\Downloads\B737_800 --cockpit --airspeed 0 --engines-running --fuel-percent 100 --payload --scenery C:\Users\user\Downloads\LTFO --airport LTFO --runway 24
```

Use `--parking 0` in place of `--runway 24` to select a parking stand by its
zero-based index in the airport BGL. Runway names include a two-digit number
and optional designator, such as `06`, `24L`, or `09R`. These spawn options set
the starting airspeed and height above the local ground to zero and use the
authored heading. Omit `--engines-running` for the existing cold-start mode.

Folder loading centres the local world on the airport. Runway and parking
spawning then move the origin to the selected location. Fuel and payload
options continue to apply. Explicit BGL options remain available as an
alternative to folder discovery.

The current scene uses a flat ground plane. Discovering a BGL does not imply
that every section renders: terrain elevation, photoreal terrain and CVX
polygons still require the terrain pipeline; raster imagery and masks decode
but are not yet drawn by that pipeline. PBR models are reported as
unsupported. Supported MDL ground polygons load as placed library models.
All content access is read-only.
