# PTC raster decoder

The `ptc` directory comes from `seanisom/flightsimlib` commit
`fc17bec8e20770da3344eea10f25ecac281ee09f`, `external/PTC`, under its MIT license
(retained in `ptc/LICENSE`). The upstream readme describes the codec and its
testing limitations.

Local changes add range and multiplication checks to stream reads in `PTC.c`.
`ptc_bridge.c` restricts the entry point to a single 256x256 ARGB1555 photo
frame and checks frame lengths, tile lengths, dimensions and offsets. The
Rust wrapper limits input size and allocates an exact output buffer. Upstream
assertions are disabled for malformed-input errors to return through the bridge.

Real-content validation decodes all LTFO photo tiles and rejects truncated
PTC payloads. This is not a completed fuzzing audit of the native entropy
decoder. Broader formats and untrusted-input hardening remain work in progress.
