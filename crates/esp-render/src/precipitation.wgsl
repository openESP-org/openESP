// Screen-space precipitation is an interim view effect; cockpit surfaces are
// excluded by the caller so instruments remain readable.
fn precipitation_hash(p: vec2<f32>) -> f32 {
    var q = fract(vec3(p.xyx) * 0.1031);
    q += vec3(dot(q, q.yzx + vec3(33.33)));
    return fract((q.x + q.y) * q.z);
}

fn precipitation(colour: vec3<f32>, screen: vec2<f32>) -> vec3<f32> {
    var result = colour;
    var height_factor = 1.0;
    var cloud_base = 1000000.0;
    for (var i = 0; i < 3; i += 1) {
        if camera.clouds[i].z > 0.0 { cloud_base = min(cloud_base, camera.clouds[i].x); }
    }
    if cloud_base < 1000000.0 { height_factor = 1.0 - smoothstep(cloud_base, cloud_base + 600.0, camera.sun.w); }
    if camera.lighting.z > 0.0 && height_factor > 0.0 {
        let p = vec2((screen.x + screen.y * 0.15) / 28.0, screen.y / 48.0 - camera.world.z * 2.0);
        let cell = floor(p);
        let seed = precipitation_hash(cell);
        let x = abs(fract(p.x) - precipitation_hash(cell + vec2(7.1, 3.7)));
        let y = abs(fract(p.y) - 0.5);
        let streak = (1.0 - smoothstep(0.015, 0.05, x)) * (1.0 - smoothstep(0.15, 0.4, y));
        let mask = select(0.0, streak, seed < camera.lighting.z * height_factor * 0.75);
        result = mix(result, vec3(0.75, 0.82, 0.9), mask * 0.5);
    }
    if camera.lighting.w > 0.0 && height_factor > 0.0 {
        let p = vec2(screen.x / 32.0, screen.y / 32.0 - camera.world.z * 0.4);
        let cell = floor(p);
        let seed = precipitation_hash(cell + vec2(13.0, 17.0));
        let centre = vec2(precipitation_hash(cell), precipitation_hash(cell + vec2(3.0, 7.0)));
        let flake = 1.0 - smoothstep(0.04, 0.11, length(fract(p) - centre));
        let mask = select(0.0, flake, seed < camera.lighting.w * height_factor * 0.75);
        result = mix(result, vec3(0.95, 0.97, 1.0), mask * 0.8);
    }
    return result;
}
