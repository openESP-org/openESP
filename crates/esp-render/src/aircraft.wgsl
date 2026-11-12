struct Camera {
    view_projection: mat4x4<f32>,
    inverse_view_projection: mat4x4<f32>,
    sun: vec4<f32>,
    lighting: vec4<f32>,
    clouds: array<vec4<f32>, 3>,
    world: vec4<f32>,
    spotlights: array<mat4x4<f32>, 8>,
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
@group(1) @binding(5) var emissive_texture: texture_2d<f32>;
struct Output {
    @builtin(position) position: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) uv: vec2<f32>,
    @location(2) camera_relative_position: vec3<f32>,
    @location(3) tangent: vec4<f32>,
}
fn weather_fog(lit: vec3<f32>, distance: f32) -> vec3<f32> {
    if camera.lighting.y <= 0.0 { return lit; }
    let daylight = smoothstep(-0.105, 0.087, camera.sun.y);
    let fog = 1.0 - exp(-3.912 * distance / max(camera.lighting.y, 1.0));
    let fog_colour = mix(vec3(0.015, 0.02, 0.03), vec3(0.57, 0.64, 0.70), daylight);
    return mix(lit, fog_colour, fog);
}
fn weather_volume(lit: vec3<f32>, position: vec3<f32>) -> vec3<f32> {
    let distance = length(position);
    if distance < 0.001 { return lit; }
    let ray = position / distance;
    let sun = normalize(camera.sun.xyz);
    var result = lit;
    for (var i = 0; i < 3; i += 1) {
        let index = select(2 - i, i, ray.y < 0.0);
        let cloud = cloud_layer(ray, sun, camera.clouds[index], min(distance, 40000.0));
        result = result * (1.0 - cloud.a) + cloud.rgb;
    }
    return result;
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
@fragment fn fs_main(input: Output, @builtin(front_facing) front_facing: bool) -> @location(0) vec4<f32> {
    var colour = textureSample(diffuse_texture, texture_sampler, input.uv) * draw.diffuse;
    var metal_sample = vec4(1.0);
    if draw.pbr.w > 0.5 {
        let metal_uv = vec2(input.uv.x, select(input.uv.y, 1.0 - input.uv.y, draw.pbr.w > 1.5));
        metal_sample = textureSample(metallic_texture, texture_sampler, metal_uv);
    }
    var normal_sample = vec4(0.5, 0.5, 1.0, 0.5);
    if draw.normal_settings.x > 0.5 {
        let normal_uv = vec2(input.uv.x, select(input.uv.y, 1.0 - input.uv.y, draw.normal_settings.y > 0.0)) * abs(draw.normal_settings.y);
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
        let fogged = weather_fog(weather_volume(colour.rgb, input.camera_relative_position), length(input.camera_relative_position));
        return vec4(precipitation(fogged, input.position.xy), colour.a);
    }
    if draw.settings.y > 2.5 && draw.settings.y < 3.5 {
        colour = vec4(input.tangent.xyz, 1.0);
    }
    if draw.settings.y > 4.5 && draw.settings.y < 5.5 {
        colour.a = clamp(input.tangent.w, 0.0, 1.0);
    }
    let daylight = smoothstep(-0.105, 0.087, camera.sun.y);
    var emission = vec3(0.0);
    var emission_mode = 0u;
    var emission_enabled = false;
    if draw.normal_settings.w > 0.5 {
        let encoded = u32(draw.normal_settings.w - 1.0);
        emission_mode = (encoded / 2u) % 4u;
        emission_enabled = encoded < 8u || camera.lighting.x > 0.5;
        let emissive_uv = vec2(input.uv.x, select(input.uv.y, 1.0 - input.uv.y, encoded % 2u == 1u));
        emission = textureSample(emissive_texture, texture_sampler, emissive_uv).rgb;
    }
    let air_mass = 1.0 / max(camera.sun.y + 0.08, 0.08);
    let sun_transmittance = exp(-vec3(0.0512, 0.1128, 0.2696) * air_mass);
    let cloud_light = cloud_sunlight(input.camera_relative_position, normalize(camera.sun.xyz));
    var lit = vec3(0.0);
    var n = normalize(input.normal);
    if draw.normal_settings.x > 0.5 {
        let t = normalize(input.tangent.xyz - n * dot(n, input.tangent.xyz));
        let b = cross(n, t) * input.tangent.w;
        let xy = vec2(select(normal_sample.r, normal_sample.a, draw.normal_settings.z > 0.5), normal_sample.g) * 2.0 - vec2(1.0);
        let z = sqrt(max(0.0, 1.0 - dot(xy, xy)));
        n = normalize(t * xy.x + b * xy.y + n * z);
    }
    n *= select(-1.0, 1.0, front_facing);
    if draw.pbr.z > 0.5 {
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
        lit = colour.rgb * mix(0.025, 0.2, daylight) + (diffuse + specular) * sun_transmittance * nl * 2.0 * daylight * cloud_light;
    } else {
        let diffuse_light = 0.65 * max(dot(n, normalize(camera.sun.xyz)),0.0) * daylight * cloud_light;
        lit = colour.rgb * (mix(0.025, 0.35, daylight) + diffuse_light * sun_transmittance);
    }
    for (var i = 0; i < 8; i += 1) {
        let light = camera.spotlights[i];
        if light[2].w <= 0.0 { continue; }
        let offset = light[0].xyz - input.camera_relative_position;
        let distance = length(offset);
        if distance < 0.001 || distance >= light[0].w { continue; }
        let direction = offset / distance;
        let cone = smoothstep(light[1].w, light[3].x, dot(-direction, light[1].xyz));
        let edge = pow(max(1.0 - distance / light[0].w, 0.0), 2.0);
        let falloff = edge / pow(max(distance, 1.0), light[3].y);
        let diffuse = max(dot(n, direction), 0.0) * cone * falloff;
        var response = colour.rgb;
        if draw.pbr.z > 0.5 {
            let metal = clamp(select(draw.pbr.x, metal_sample.r, draw.pbr.w > 0.5), 0.0, 1.0);
            let rough = max(select(draw.pbr.y, 1.0 - metal_sample.a, draw.pbr.w > 0.5), 0.045);
            let view = normalize(-input.camera_relative_position);
            let half_vector = normalize(direction + view);
            let nl = max(dot(n, direction), 0.0);
            let nv = max(dot(n, view), 0.001);
            let nh = max(dot(n, half_vector), 0.0);
            let vh = max(dot(view, half_vector), 0.0);
            let a2 = pow(rough, 4.0);
            let d = a2 / max(3.14159265 * pow(nh * nh * (a2 - 1.0) + 1.0, 2.0), 0.000001);
            let k = (rough + 1.0) * (rough + 1.0) / 8.0;
            let g = (nl / (nl * (1.0 - k) + k)) * (nv / (nv * (1.0 - k) + k));
            let f0 = mix(vec3(0.04), colour.rgb, metal);
            let f = f0 + (vec3(1.0) - f0) * pow(1.0 - vh, 5.0);
            response = (vec3(1.0) - f) * (1.0 - metal) * colour.rgb / 3.14159265
                + d * g * f / max(4.0 * nl * nv, 0.001);
        }
        lit += response * light[2].rgb * light[2].w * diffuse / 12.56637;
    }
    if emission_enabled {
        let night = 1.0 - daylight;
        switch emission_mode {
            case 0u: { lit += emission; }
            case 1u: { lit += emission * night; }
            case 2u: { lit = mix(lit, emission, night); }
            default: { lit = mix(lit, lit * emission, night); }
        }
    }
    let fogged = weather_fog(weather_volume(lit, input.camera_relative_position), length(input.camera_relative_position));
    let in_cockpit = camera.world.w > 0.5 && draw.settings.y < 0.5;
    var result = fogged;
    if !in_cockpit { result = precipitation(result, input.position.xy); }
    return vec4(result, colour.a);
}
