fn main() {
    let root = "native/ptc";
    println!("cargo:rerun-if-changed=native");
    let mut build = cc::Build::new();
    build.include(root).warnings(false).define("NDEBUG", None);
    for source in [
        "PTC",
        "PTCAdaptiveDecoder",
        "PTCColorMapper",
        "PTCRow",
        "PTCTransform",
    ] {
        build.file(format!("{root}/{source}.c"));
    }
    build.file("native/ptc_bridge.c").compile("openesp_ptc");
}
