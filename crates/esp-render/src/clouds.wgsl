fn cloud_hash(p: vec3<f32>) -> f32 {
    var q = fract(p * 0.1031);
    q += vec3(dot(q, q.yzx + vec3(33.33)));
    return fract((q.x + q.y) * q.z);
}

fn cloud_noise(p: vec3<f32>) -> f32 {
    let cell = floor(p);
    let f = fract(p);
    let s = f * f * (vec3(3.0) - 2.0 * f);
    let a = mix(cloud_hash(cell), cloud_hash(cell + vec3(1.0, 0.0, 0.0)), s.x);
    let b = mix(cloud_hash(cell + vec3(0.0, 1.0, 0.0)), cloud_hash(cell + vec3(1.0, 1.0, 0.0)), s.x);
    let c = mix(cloud_hash(cell + vec3(0.0, 0.0, 1.0)), cloud_hash(cell + vec3(1.0, 0.0, 1.0)), s.x);
    let d = mix(cloud_hash(cell + vec3(0.0, 1.0, 1.0)), cloud_hash(cell + vec3(1.0, 1.0, 1.0)), s.x);
    return mix(mix(a, b, s.y), mix(c, d, s.y), s.z);
}

fn cloud_density(position: vec3<f32>, layer: vec4<f32>) -> f32 {
    let local = (position.y - layer.x) / layer.y;
    let profile = smoothstep(0.0, 0.16, local) * (1.0 - smoothstep(0.70, 1.0, local));
    let p = position * vec3(0.00045, 0.0014, 0.00045) + vec3(17.3, 11.7, 29.1);
    let noise = 0.7 * cloud_noise(p) + 0.3 * cloud_noise(p * 2.1);
    return max(smoothstep(0.76 - layer.z * 0.48, 0.88 - layer.z * 0.48, noise), max((layer.z - 0.9) * 3.0, 0.0)) * profile;
}

// Integrate the same density used by visible clouds along a short solar ray.
fn cloud_sunlight(position: vec3<f32>, sun: vec3<f32>) -> f32 {
    if sun.y <= 0.02 { return 1.0; }
    let origin = vec3(camera.world.x + position.x, camera.sun.w + position.y, camera.world.y + position.z);
    var optical_depth = 0.0;
    for (var i = 0; i < 3; i += 1) {
        let layer = camera.clouds[i];
        if layer.z <= 0.0 || layer.y <= 0.0 || origin.y >= layer.x + layer.y { continue; }
        let start = max((layer.x - origin.y) / sun.y, 0.0);
        let end = (layer.x + layer.y - origin.y) / sun.y;
        let step = (end - start) / 3.0;
        for (var j = 0; j < 3; j += 1) {
            let p = origin + sun * (start + (f32(j) + 0.5) * step);
            optical_depth += cloud_density(p, layer) * step * 0.0018;
        }
    }
    return exp(-optical_depth);
}

// A bounded ray march through a procedural volume for each reported cloud layer.
fn cloud_layer(ray: vec3<f32>, sun: vec3<f32>, layer: vec4<f32>, max_distance: f32) -> vec4<f32> {
    if layer.z <= 0.0 { return vec4(0.0); }
    var start = 0.0;
    var end = max_distance;
    if abs(ray.y) < 0.001 {
        if camera.sun.w < layer.x || camera.sun.w > layer.x + layer.y { return vec4(0.0); }
    } else {
        let a = (layer.x - camera.sun.w) / ray.y;
        let b = (layer.x + layer.y - camera.sun.w) / ray.y;
        start = max(min(a, b), 0.0);
        end = min(max(a, b), max_distance);
    }
    if end <= start { return vec4(0.0); }
    let step = (end - start) / 8.0;
    var opacity = 0.0;
    var colour = vec3(0.0);
    let daylight = smoothstep(-0.105, 0.087, sun.y);
    for (var i = 0; i < 8; i += 1) {
        let distance = start + (f32(i) + 0.5) * step;
        let height = camera.sun.w + ray.y * distance;
        let local = (height - layer.x) / layer.y;
        let position = vec3(camera.world.x + ray.x * distance, height, camera.world.y + ray.z * distance);
        let density = cloud_density(position, layer);
        let alpha = 1.0 - exp(-density * step * 0.0018);
        let brightness = mix(0.55, 1.0, local) * (0.65 + 0.35 * max(sun.y, 0.0));
        let light = mix(vec3(0.035, 0.045, 0.07), vec3(0.95, 0.96, 0.97) * brightness, daylight);
        colour += (1.0 - opacity) * alpha * light;
        opacity += (1.0 - opacity) * alpha;
    }
    return vec4(colour, opacity);
}

