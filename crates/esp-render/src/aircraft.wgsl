struct Camera { view_projection: mat4x4<f32> }
struct Draw {
    model: mat4x4<f32>,
    normal: mat4x4<f32>,
    diffuse: vec4<f32>,
    settings: vec4<f32>,
}
@group(0) @binding(0) var<uniform> camera: Camera;
@group(1) @binding(0) var<uniform> draw: Draw;
@group(1) @binding(1) var diffuse_texture: texture_2d<f32>;
@group(1) @binding(2) var texture_sampler: sampler;
struct Output {
    @builtin(position) position: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) uv: vec2<f32>,
}
@vertex fn vs_main(@location(0) position: vec3<f32>, @location(1) normal: vec3<f32>, @location(2) uv: vec2<f32>) -> Output {
    var output: Output;
    output.position = camera.view_projection * draw.model * vec4(position, 1.0);
    output.normal = (draw.normal * vec4(normal, 0.0)).xyz;
    output.uv = vec2(uv.x, select(uv.y, 1.0-uv.y, draw.settings.x > 0.0));
    return output;
}
@fragment fn fs_main(input: Output) -> @location(0) vec4<f32> {
    var colour = textureSample(diffuse_texture, texture_sampler, input.uv) * draw.diffuse;
    if colour.a < draw.settings.z { discard; }
    if draw.settings.y > 0.0 {
        let distance = abs(fract(input.uv - 0.5) - 0.5);
        let width = max(fwidth(input.uv), vec2(0.0001));
        let lines = 1.0 - smoothstep(vec2(0.008), vec2(0.008) + width * 0.5, distance);
        let fade = 1.0 - smoothstep(0.2, 0.8, max(width.x, width.y));
        colour = vec4(mix(draw.diffuse.rgb, vec3(0.9), max(lines.x,lines.y) * fade), 1.0);
        return colour;
    }
    let light = 0.35 + 0.65 * max(dot(normalize(input.normal), normalize(vec3(-0.4,0.8,0.6))),0.0);
    return vec4(colour.rgb * light, colour.a);
}
