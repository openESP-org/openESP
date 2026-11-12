struct Camera {
    view_projection: mat4x4<f32>,
    inverse_view_projection: mat4x4<f32>,
    sun: vec4<f32>,
    lighting: vec4<f32>,
    clouds: array<vec4<f32>, 3>,
    world: vec4<f32>,
    spotlights: array<mat4x4<f32>, 8>,
}
struct Sprite {
    position: vec4<f32>,
    right: vec4<f32>,
    up: vec4<f32>,
    colour: vec4<f32>,
    uv: vec4<f32>,
}
@group(0) @binding(0) var<uniform> camera: Camera;
@group(1) @binding(0) var<uniform> sprite: Sprite;
@group(1) @binding(1) var image: texture_2d<f32>;
@group(1) @binding(2) var image_sampler: sampler;
struct VertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(0) uv: vec2<f32>,
}
@vertex fn vs_main(@builtin(vertex_index) index: u32) -> VertexOutput {
    let corners = array(vec2(-1.0,-1.0), vec2(1.0,-1.0), vec2(1.0,1.0),
                        vec2(-1.0,-1.0), vec2(1.0,1.0), vec2(-1.0,1.0));
    let corner = corners[index];
    var out: VertexOutput;
    out.position = camera.view_projection * vec4(sprite.position.xyz + corner.x * sprite.right.xyz + corner.y * sprite.up.xyz, 1.0);
    let uv = mix(sprite.uv.xy, sprite.uv.zw, vec2((corner.x + 1.0) * 0.5, (1.0 - corner.y) * 0.5));
    out.uv = vec2(uv.x, select(uv.y, 1.0 - uv.y, sprite.position.w > 0.5));
    return out;
}
@fragment fn fs_main(input: VertexOutput) -> @location(0) vec4<f32> {
    let texel = textureSample(image, image_sampler, input.uv);
    let alpha = select(texel.a * sprite.colour.a, sprite.colour.a, sprite.right.w > 0.5);
    return vec4(texel.rgb * sprite.colour.rgb, alpha);
}
