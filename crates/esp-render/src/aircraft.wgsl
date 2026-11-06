struct Camera {
    view_projection: mat4x4<f32>,
    inverse_view_projection: mat4x4<f32>,
    sun: vec4<f32>,
}
struct Draw {
    model: mat4x4<f32>,
    normal: mat4x4<f32>,
    diffuse: vec4<f32>,
    settings: vec4<f32>,
    pbr: vec4<f32>,
    normal_settings: vec4<f32>,
}
@group(0) @binding(0) var<uniform> camera: Camera;
@group(1) @binding(0) var<uniform> draw: Draw;
@group(1) @binding(1) var diffuse_texture: texture_2d<f32>;
@group(1) @binding(2) var texture_sampler: sampler;
@group(1) @binding(3) var metallic_texture: texture_2d<f32>;
@group(1) @binding(4) var normal_texture: texture_2d<f32>;
struct Output {
    @builtin(position) position: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) uv: vec2<f32>,
    @location(2) camera_relative_position: vec3<f32>,
    @location(3) tangent: vec4<f32>,
}
@vertex fn vs_main(@location(0) position: vec3<f32>, @location(1) normal: vec3<f32>, @location(2) uv: vec2<f32>, @location(3) tangent: vec4<f32>) -> Output {
    var output: Output;
    output.position = camera.view_projection * draw.model * vec4(position, 1.0);
    output.camera_relative_position = (draw.model * vec4(position, 1.0)).xyz;
    output.normal = (draw.normal * vec4(normal, 0.0)).xyz;
    let model3 = mat3x3(draw.model[0].xyz, draw.model[1].xyz, draw.model[2].xyz);
    let orientation = select(-1.0, 1.0, determinant(model3) >= 0.0);
    output.tangent = vec4(model3 * tangent.xyz, tangent.w * orientation);
    output.uv = vec2(uv.x, select(uv.y, 1.0-uv.y, draw.settings.x > 0.0));
    return output;
}
@fragment fn fs_main(input: Output) -> @location(0) vec4<f32> {
    var colour = textureSample(diffuse_texture, texture_sampler, input.uv) * draw.diffuse;
    var metal_sample = vec4(1.0);
    if draw.pbr.w > 0.5 {
        let metal_uv = vec2(input.uv.x, select(input.uv.y, 1.0 - input.uv.y, draw.pbr.w > 1.5));
        metal_sample = textureSample(metallic_texture, texture_sampler, metal_uv);
    }
    var normal_sample = vec4(0.5, 0.5, 1.0, 0.5);
    if draw.normal_settings.x > 0.5 {
        let normal_uv = vec2(input.uv.x, select(input.uv.y, 1.0 - input.uv.y, draw.normal_settings.y > 0.5));
        normal_sample = textureSample(normal_texture, texture_sampler, normal_uv);
    }
    let alpha = colour.a;
    let threshold = draw.settings.z;
    var passes = true;
    switch u32(draw.settings.w) {
        case 1u: { passes = false; }
        case 2u: { passes = alpha < threshold; }
        case 3u: { passes = alpha == threshold; }
        case 4u: { passes = alpha <= threshold; }
        case 5u: { passes = alpha > threshold; }
        case 6u: { passes = alpha != threshold; }
        case 7u: { passes = alpha >= threshold; }
        default: {}
    }
    if !passes { discard; }
    if draw.settings.y > 0.0 && draw.settings.y < 1.5 {
        let distance = abs(fract(input.uv - 0.5) - 0.5);
        let width = max(fwidth(input.uv), vec2(0.0001));
        let lines = 1.0 - smoothstep(vec2(0.008), vec2(0.008) + width * 0.5, distance);
        let fade = 1.0 - smoothstep(0.2, 0.5, max(width.x, width.y));
        colour = vec4(mix(draw.diffuse.rgb, vec3(0.9), max(lines.x,lines.y) * fade), 1.0);
        return colour;
    }
    let daylight = smoothstep(-0.105, 0.087, camera.sun.y);
    let air_mass = 1.0 / max(camera.sun.y + 0.08, 0.08);
    let sun_transmittance = exp(-vec3(0.0512, 0.1128, 0.2696) * air_mass);
    if draw.pbr.z > 0.5 {
        var n = normalize(input.normal);
        if draw.normal_settings.x > 0.5 {
            let t = normalize(input.tangent.xyz - n * dot(n, input.tangent.xyz));
            let b = cross(n, t) * input.tangent.w;
            let xy = vec2(select(normal_sample.r, normal_sample.a, draw.normal_settings.z > 0.5), normal_sample.g) * 2.0 - vec2(1.0);
            let z = sqrt(max(0.0, 1.0 - dot(xy, xy)));
            n = normalize(t * xy.x + b * xy.y + n * z);
        }
        let l = normalize(camera.sun.xyz);
        let v = normalize(-input.camera_relative_position);
        let h = normalize(l + v);
        let nl = max(dot(n, l), 0.0);
        let nv = max(dot(n, v), 0.001);
        let nh = max(dot(n, h), 0.0);
        let vh = max(dot(v, h), 0.0);
        var metal = draw.pbr.x;
        var roughness = draw.pbr.y;
        if draw.pbr.w > 0.5 {
            metal = metal_sample.r;
            roughness = 1.0 - metal_sample.a;
        }
        let rough = max(roughness, 0.045);
        let a = rough * rough;
        let a2 = a * a;
        let d = a2 / max(3.14159265 * pow(nh * nh * (a2 - 1.0) + 1.0, 2.0), 0.000001);
        let k = (rough + 1.0) * (rough + 1.0) / 8.0;
        let g = (nl / (nl * (1.0 - k) + k)) * (nv / (nv * (1.0 - k) + k));
        let f0 = mix(vec3(0.04), colour.rgb, metal);
        let f = f0 + (vec3(1.0) - f0) * pow(1.0 - vh, 5.0);
        let specular = d * g * f / max(4.0 * nl * nv, 0.001);
        let diffuse = (vec3(1.0) - f) * (1.0 - metal) * colour.rgb / 3.14159265;
        return vec4(colour.rgb * mix(0.025, 0.2, daylight) + (diffuse + specular) * sun_transmittance * nl * 2.0 * daylight, colour.a);
    }
    let diffuse_light = 0.65 * max(dot(normalize(input.normal), normalize(camera.sun.xyz)),0.0) * daylight;
    return vec4(colour.rgb * (mix(0.025, 0.35, daylight) + diffuse_light * sun_transmittance), colour.a);
}
