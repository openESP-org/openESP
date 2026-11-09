struct Camera {
    view_projection: mat4x4<f32>,
    inverse_view_projection: mat4x4<f32>,
    sun: vec4<f32>,
    lighting: vec4<f32>,
    clouds: array<vec4<f32>, 3>,
    world: vec4<f32>,
}
@group(0) @binding(0) var<uniform> camera: Camera;

struct Output {
    @builtin(position) position: vec4<f32>,
    @location(0) ndc: vec2<f32>,
}
@vertex fn vs_main(@builtin(vertex_index) index: u32) -> Output {
    let p = array(vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0))[index];
    var out: Output;
    out.position = vec4(p, 0.0, 1.0);
    out.ndc = p;
    return out;
}

// Single scattering in a spherical atmosphere. Distances and coefficients are
// in metres; the secondary optical depth uses a local exponential approximation.
@fragment fn fs_main(input: Output) -> @location(0) vec4<f32> {
    let projected = camera.inverse_view_projection * vec4(input.ndc, 1.0, 1.0);
    let ray = normalize(projected.xyz / projected.w);
    let sun = normalize(camera.sun.xyz);
    let earth = 6360000.0;
    let atmosphere = 6460000.0;
    let origin = vec3(0.0, earth + max(camera.sun.w, 1.0), 0.0);
    let b = dot(origin, ray);
    let distance = max(-b + sqrt(max(b*b - dot(origin, origin) + atmosphere*atmosphere, 0.0)), 0.0);
    let ground_disc = b*b - dot(origin, origin) + earth*earth;
    var limit = distance;
    if ground_disc >= 0.0 && -b - sqrt(ground_disc) > 0.0 {
        limit = min(limit, -b - sqrt(ground_disc));
    }
    let beta_r = vec3(5.8e-6, 13.5e-6, 33.1e-6);
    let beta_m = vec3(4e-6);
    var depth_r = 0.0;
    var depth_m = 0.0;
    var scatter_r = vec3(0.0);
    var scatter_m = vec3(0.0);
    for (var i = 0; i < 12; i += 1) {
        let t0 = f32(i) / 12.0;
        let t1 = f32(i + 1) / 12.0;
        let start = t0 * t0 * limit;
        let end = t1 * t1 * limit;
        let step = end - start;
        let p = origin + ray * ((start + end) * 0.5);
        let height = max(length(p) - earth, 0.0);
        let local_up = p / length(p);
        let density_r = exp(-height / 8000.0);
        let density_m = exp(-height / 1200.0);
        depth_r += density_r * step;
        depth_m += density_m * step;
        let sun_cos = dot(local_up, sun);
        let sun_ground = dot(p, sun) * dot(p, sun) - dot(p, p) + earth * earth;
        if sun_ground > 0.0 && dot(p, sun) < 0.0 { continue; }
        let air_mass = 1.0 / max(sun_cos + 0.08, 0.08);
        let sun_r = density_r * 8000.0 * air_mass;
        let sun_m = density_m * 1200.0 * air_mass;
        let attenuation = exp(-beta_r * (depth_r + sun_r) - beta_m * (depth_m + sun_m));
        scatter_r += attenuation * density_r * step;
        scatter_m += attenuation * density_m * step;
    }
    let mu = dot(ray, sun);
    let rayleigh_phase = 3.0 / (16.0 * 3.14159265) * (1.0 + mu * mu);
    let g = 0.76;
    let mie_phase = 3.0 / (8.0 * 3.14159265) * (1.0 - g*g) * (1.0 + mu*mu)
        / ((2.0 + g*g) * pow(max(1.0 + g*g - 2.0*g*mu, 0.001), 1.5));
    let radiance = 20.0 * (scatter_r * beta_r * rayleigh_phase + scatter_m * beta_m * mie_phase);
    let sun_disc = smoothstep(0.99990, 0.99996, mu) * smoothstep(-0.02, 0.01, sun.y);
    var color = vec3(1.0) - exp(-radiance - vec3(sun_disc * 6.0));
    for (var i = 0; i < 3; i += 1) {
        let index = select(2 - i, i, ray.y < 0.0);
        let cloud = cloud_layer(ray, sun, camera.clouds[index], 40000.0);
        color = color * (1.0 - cloud.a) + cloud.rgb;
    }
    if camera.lighting.y > 0.0 {
        let daylight = smoothstep(-0.105, 0.087, sun.y);
        let haze = (1.0 - exp(-5000.0 / max(camera.lighting.y, 1.0))) * exp(-max(ray.y, 0.0) * 6.0);
        color = mix(color, mix(vec3(0.015, 0.02, 0.03), vec3(0.57, 0.64, 0.70), daylight), haze);
    }
    return vec4(precipitation(color, input.position.xy), 1.0);
}
