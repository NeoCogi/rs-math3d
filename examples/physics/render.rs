// Copyright 2020-Present (c) Raja Lehtihet & Wael El Oraiby
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//! SDL2-owned GLES3 application. GPU resources share an immutable glow context
//! on the main thread and are dropped before the SDL context/window. Simulation
//! has no dependency on these types; conversion to f32 happens only for drawing.

use crate::physics::{scene, Shape, World};
use glow::HasContext;
use rs_math3d::{transforms, Matrix4, RealScalar, TryCastFrom, Vector3};
use sdl2::{event::Event, keyboard::Keycode, video::GLProfile};
use std::{
    fs,
    io::{BufWriter, Write},
    path::PathBuf,
    rc::Rc,
    time::Instant,
};

/// Explicit application options, including deterministic finite render runs.
pub struct Options {
    /// Simulation backend; rendering always uses GLES float vertex data.
    pub scalar: String,
    /// Optional finite frame count; these runs advance two fixed ticks per frame.
    frames: Option<usize>,
    /// Optional final-frame PPM screenshot, written before swapping buffers.
    capture: Option<PathBuf>,
    /// Hidden window for driver/render smoke tests.
    hidden: bool,
}
impl Options {
    /// Parses bounded demo options; unsupported flags are an explicit usage error.
    pub fn parse() -> Result<Self, String> {
        let mut result = Self {
            scalar: "f32".into(),
            frames: None,
            capture: None,
            hidden: false,
        };
        let mut args = std::env::args().skip(1);
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--scalar" => result.scalar = args.next().ok_or("missing scalar")?,
                "--frames" => {
                    let n = args
                        .next()
                        .ok_or("missing frame count")?
                        .parse::<usize>()
                        .map_err(|e| e.to_string())?;
                    if n == 0 {
                        return Err("frame count must be positive".into());
                    }
                    result.frames = Some(n);
                }
                "--capture" => {
                    result.capture = Some(args.next().ok_or("missing capture path")?.into());
                }
                "--hidden" => result.hidden = true,
                _ => return Err(
                    "usage: physics_demo [--scalar f32|fixed32|fixed64] [--frames N] [--capture image.ppm] [--hidden]".into(),
                ),
            }
        }
        Ok(result)
    }
}

const VERTEX: &str = r#"#version 300 es
precision highp float;
layout(location=0) in vec3 a_position;
layout(location=1) in vec3 a_normal;
layout(location=2) in float a_end;
uniform mat4 u_vp;
uniform mat4 u_model;
uniform vec3 u_dimensions;
uniform float u_half_height;
out vec3 v_world;
out vec3 v_normal;
void main(){
 vec3 p=a_position*u_dimensions;
 p.y+=a_end*u_half_height;
 vec4 world=u_model*vec4(p,1.0);
 v_world=world.xyz;v_normal=mat3(u_model)*a_normal;
 gl_Position=u_vp*world;
}
"#;
const FRAGMENT: &str = r#"#version 300 es
precision highp float;
in vec3 v_world;
in vec3 v_normal;
uniform vec3 u_color;
uniform float u_ground;
uniform float u_unlit;
out vec4 out_color;
void main(){
 if(u_unlit>0.5){out_color=vec4(u_color,1.0);return;}
 vec3 n=normalize(v_normal);if(!gl_FrontFacing)n=-n;
 float diffuse=max(dot(n,normalize(vec3(-0.5,1.0,0.4))),0.0);
 vec3 base=u_color;
 if(u_ground>0.5){
  vec2 grid=abs(fract(v_world.xz-0.5)-0.5)/max(fwidth(v_world.xz),vec2(0.001));
  float line=1.0-min(min(grid.x,grid.y),1.0);
  float checker=mod(floor(v_world.x*0.25)+floor(v_world.z*0.25),2.0);
  base*=0.82+0.18*checker;base=mix(base,base*1.65,line*0.45);
 }
 vec3 color=base*(0.35+0.65*diffuse);
 float fog=clamp(length(v_world)*0.008,0.0,0.35);
 color=mix(color,vec3(0.035,0.045,0.065),fog);
 out_color=vec4(pow(color,vec3(0.85)),1.0);
}
"#;

/// One immutable/dynamic vertex allocation. Rc confines context ownership to the
/// UI thread; native GL lifetime is bounded by run's SDL context drop order.
struct Mesh {
    /// Current main-thread GL entry points, shared by renderer resources.
    gl: Rc<glow::Context>,
    /// Vertex array describing position/normal/capsule-end attributes.
    vao: glow::NativeVertexArray,
    /// Interleaved seven-float vertex buffer.
    buffer: glow::NativeBuffer,
    /// Number of initialized vertices available to draw.
    count: i32,
}
impl Mesh {
    /// Uploads owned CPU data, with cleanup if resource creation fails halfway.
    fn new(gl: Rc<glow::Context>, data: &[f32]) -> Result<Self, String> {
        // SAFETY: caller owns a current GLES3 context on this thread. All vertex
        // offsets/stride refer to the initialized seven-float packed buffer.
        unsafe {
            let vao = gl.create_vertex_array()?;
            let buffer = match gl.create_buffer() {
                Ok(x) => x,
                Err(e) => {
                    gl.delete_vertex_array(vao);
                    return Err(e);
                }
            };
            gl.bind_vertex_array(Some(vao));
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(buffer));
            for (i, size, offset) in [(0, 3, 0), (1, 3, 12), (2, 1, 24)] {
                gl.enable_vertex_attrib_array(i);
                gl.vertex_attrib_pointer_f32(i, size, glow::FLOAT, false, 28, offset);
            }
            let mut result = Self {
                gl,
                vao,
                buffer,
                count: 0,
            };
            result.set(data);
            Ok(result)
        }
    }
    /// Replaces initialized vertex contents; no raw slice cast or alias is used.
    fn set(&mut self, data: &[f32]) {
        assert_eq!(data.len() % 7, 0);
        let bytes: Vec<u8> = data.iter().flat_map(|x| x.to_ne_bytes()).collect();
        // SAFETY: buffer is live in the current context; bytes cover every vertex.
        unsafe {
            self.gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.buffer));
            self.gl
                .buffer_data_u8_slice(glow::ARRAY_BUFFER, &bytes, glow::DYNAMIC_DRAW);
        }
        self.count = (data.len() / 7) as i32;
    }
    /// Draws only the initialized vertex prefix in the current context.
    fn draw(&self, mode: u32) {
        // SAFETY: this VAO/buffer is live and count covers initialized vertices;
        // the private renderer invokes it only with its SDL context current.
        unsafe {
            self.gl.bind_vertex_array(Some(self.vao));
            self.gl.draw_arrays(mode, 0, self.count);
        }
    }
}
impl Drop for Mesh {
    fn drop(&mut self) {
        // SAFETY: all Mesh owners are local to run and drop before its GL context.
        unsafe {
            self.gl.delete_buffer(self.buffer);
            self.gl.delete_vertex_array(self.vao);
        }
    }
}
/// Linked shader and its required uniform bindings.
struct Program {
    /// Current thread's context, retained through program destruction.
    gl: Rc<glow::Context>,
    /// Successfully linked program.
    id: glow::NativeProgram,
    /// Required uniforms; shader construction rejects optimized-away/missing ones.
    vp: glow::NativeUniformLocation,
    /// Object-to-world rotation/translation binding.
    model: glow::NativeUniformLocation,
    /// Collider dimensions used by the reusable primitive vertex meshes.
    dimensions: glow::NativeUniformLocation,
    /// Capsule centerline half-height; zero for other draw types.
    half: glow::NativeUniformLocation,
    /// Per-object diffuse/unlit color.
    color: glow::NativeUniformLocation,
    /// Selects procedural floor-grid shading.
    ground: glow::NativeUniformLocation,
    /// Selects HUD/debug colors without lighting.
    unlit: glow::NativeUniformLocation,
}
impl Program {
    /// Compiles and links actual GLSL ES 3.00, preserving driver error logs.
    fn new(gl: Rc<glow::Context>) -> Result<Self, String> {
        // SAFETY: current GLES3 context; created shaders/program are deleted on
        // each error path, and linked ownership transfers to Program on success.
        unsafe {
            let id = gl.create_program()?;
            let mut shaders = Vec::new();
            for (kind, text) in [
                (glow::VERTEX_SHADER, VERTEX),
                (glow::FRAGMENT_SHADER, FRAGMENT),
            ] {
                let shader = match gl.create_shader(kind) {
                    Ok(x) => x,
                    Err(e) => {
                        for s in shaders {
                            gl.delete_shader(s);
                        }
                        gl.delete_program(id);
                        return Err(e);
                    }
                };
                gl.shader_source(shader, text);
                gl.compile_shader(shader);
                if !gl.get_shader_compile_status(shader) {
                    let error = gl.get_shader_info_log(shader);
                    gl.delete_shader(shader);
                    for s in shaders {
                        gl.delete_shader(s);
                    }
                    gl.delete_program(id);
                    return Err(error);
                }
                gl.attach_shader(id, shader);
                shaders.push(shader);
            }
            gl.link_program(id);
            for shader in shaders {
                gl.detach_shader(id, shader);
                gl.delete_shader(shader);
            }
            if !gl.get_program_link_status(id) {
                let error = gl.get_program_info_log(id);
                gl.delete_program(id);
                return Err(error);
            }
            let uniform = |name| {
                gl.get_uniform_location(id, name)
                    .ok_or_else(|| format!("missing shader uniform {name}"))
            };
            let bindings = (|| {
                Ok::<_, String>((
                    uniform("u_vp")?,
                    uniform("u_model")?,
                    uniform("u_dimensions")?,
                    uniform("u_half_height")?,
                    uniform("u_color")?,
                    uniform("u_ground")?,
                    uniform("u_unlit")?,
                ))
            })();
            let (vp, model, dimensions, half, color, ground, unlit) = match bindings {
                Ok(x) => x,
                Err(e) => {
                    gl.delete_program(id);
                    return Err(e);
                }
            };
            Ok(Self {
                gl,
                id,
                vp,
                model,
                dimensions,
                half,
                color,
                ground,
                unlit,
            })
        }
    }
    /// Sets frame/object uniforms; all matrices are column-major, as in rs-math3d.
    fn bind(
        &self,
        vp: &[f32; 16],
        model: &[f32; 16],
        dimensions: [f32; 3],
        half: f32,
        color: [f32; 3],
        ground: bool,
        unlit: bool,
    ) {
        // SAFETY: locations belong to this linked current program; slices contain
        // the complete matrices and only supported scalar/vector uniform types.
        unsafe {
            let gl = &self.gl;
            gl.use_program(Some(self.id));
            gl.uniform_matrix_4_f32_slice(Some(&self.vp), false, vp);
            gl.uniform_matrix_4_f32_slice(Some(&self.model), false, model);
            gl.uniform_3_f32(
                Some(&self.dimensions),
                dimensions[0],
                dimensions[1],
                dimensions[2],
            );
            gl.uniform_1_f32(Some(&self.half), half);
            gl.uniform_3_f32(Some(&self.color), color[0], color[1], color[2]);
            gl.uniform_1_f32(Some(&self.ground), if ground { 1.0 } else { 0.0 });
            gl.uniform_1_f32(Some(&self.unlit), if unlit { 1.0 } else { 0.0 });
        }
    }
}
impl Drop for Program {
    fn drop(&mut self) {
        // SAFETY: the owned program drops before run's SDL context/window.
        unsafe {
            self.gl.delete_program(self.id);
        }
    }
}

/// Appends one interleaved position/normal/centerline-end vertex.
fn vertex(out: &mut Vec<f32>, p: [f32; 3], normal: [f32; 3], end: f32) {
    out.extend(p);
    out.extend(normal);
    out.push(end);
}
/// Tessellates a sphere or capsule. The duplicate equator rows form the capsule
/// cylinder; hemisphere normals and endpoints share the collider's dimensions.
fn round_mesh(capsule: bool) -> Vec<f32> {
    let mut rings = Vec::new();
    let steps = 10;
    for i in 0..=steps * 2 {
        let angle =
            -std::f32::consts::FRAC_PI_2 + i as f32 * std::f32::consts::PI / (steps * 2) as f32;
        let end = if capsule {
            if i <= steps {
                -1.0
            } else {
                1.0
            }
        } else {
            0.0
        };
        rings.push((angle, end));
        if capsule && i == steps {
            rings.push((angle, 1.0));
        }
    }
    let mut out = Vec::new();
    let sides = 24;
    for row in 0..rings.len() - 1 {
        for side in 0..sides {
            for (r, s) in [
                (row, side),
                (row + 1, side),
                (row + 1, side + 1),
                (row, side),
                (row + 1, side + 1),
                (row, side + 1),
            ] {
                let (latitude, end) = rings[r];
                let longitude = s as f32 * std::f32::consts::TAU / sides as f32;
                let p = [
                    latitude.cos() * longitude.cos(),
                    latitude.sin(),
                    latitude.cos() * longitude.sin(),
                ];
                vertex(&mut out, p, p, end);
            }
        }
    }
    out
}
/// Unit box with independent face normals, scaled by the actual half extents.
fn box_mesh() -> Vec<f32> {
    let mut out = Vec::new();
    for axis in 0..3 {
        for sign in [-1.0, 1.0] {
            let u = (axis + 1) % 3;
            let v = (axis + 2) % 3;
            let mut points = [[0.0; 3]; 4];
            for (i, (a, b)) in [(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)]
                .iter()
                .enumerate()
            {
                points[i][axis] = sign;
                points[i][u] = *a;
                points[i][v] = *b;
            }
            let mut normal = [0.; 3];
            normal[axis] = sign;
            let indices = if sign > 0.0 {
                [0, 1, 2, 0, 2, 3]
            } else {
                [0, 2, 1, 0, 3, 2]
            };
            for i in indices {
                vertex(&mut out, points[i], normal, 0.0);
            }
        }
    }
    out
}
/// Converts one physics scalar at the rendering boundary.
fn f<T>(x: T) -> f32
where
    f32: TryCastFrom<T>,
{
    f32::try_cast_from(x)
        .ok()
        .expect("render coordinate fits f32")
}
/// Converts a vector without routing simulation data back through floats.
fn vf<T: RealScalar>(p: Vector3<T>) -> [f32; 3]
where
    f32: TryCastFrom<T>,
{
    [f(p.x), f(p.y), f(p.z)]
}
/// Column-major flattening of the library's matrix representation.
fn matrix(m: Matrix4<f32>) -> [f32; 16] {
    let mut result = [0.; 16];
    for (i, c) in m.col.iter().enumerate() {
        result[i * 4..i * 4 + 4].copy_from_slice(&[c.x, c.y, c.z, c.w]);
    }
    result
}
/// Identity transform used for baked world geometry and screen overlays.
fn identity() -> [f32; 16] {
    matrix(Matrix4::identity())
}

/// GPU resources for static soups, reusable primitive geometry and debug/UI draws.
struct Renderer {
    /// Current main-thread context and linked shader.
    gl: Rc<glow::Context>,
    /// Linked shader shared by geometry, debugging and screen overlays.
    program: Program,
    /// Shared primitive meshes: sphere, box, capsule.
    shapes: [Mesh; 3],
    /// Immutable world-space triangle soups, same order as World::meshes().
    meshes: Vec<Mesh>,
    /// Reused line, text and panel buffers; no GL resources are created per frame.
    lines: Mesh,
    /// Reused screen text vertices.
    text: Mesh,
    /// Reused opaque HUD panel vertices.
    panel: Mesh,
}
impl Renderer {
    /// Creates GPU geometry from the exact simulation colliders/triangles.
    fn new<T: RealScalar + TryCastFrom<f64>>(
        gl: glow::Context,
        world: &World<T>,
    ) -> Result<Self, String>
    where
        f32: TryCastFrom<T>,
    {
        let gl = Rc::new(gl);
        let program = Program::new(gl.clone())?;
        let shapes = [
            Mesh::new(gl.clone(), &round_mesh(false))?,
            Mesh::new(gl.clone(), &box_mesh())?,
            Mesh::new(gl.clone(), &round_mesh(true))?,
        ];
        let mut meshes = Vec::new();
        for mesh in world.meshes() {
            let mut data = Vec::new();
            for i in 0..mesh.tree.info().triangles {
                let p = mesh.tree.triangle(i).unwrap().vertices().map(vf);
                let a = Vector3::new(p[0][0], p[0][1], p[0][2]);
                let b = Vector3::new(p[1][0], p[1][1], p[1][2]);
                let c = Vector3::new(p[2][0], p[2][1], p[2][2]);
                let normal = crate::collision::contact::unit(rs_math3d::CrossProduct::cross(
                    &(b - a),
                    &(c - a),
                ))
                .unwrap_or(Vector3::new(0., 1., 0.));
                let normal = [normal.x, normal.y, normal.z];
                for point in p {
                    vertex(&mut data, point, normal, 0.0);
                }
            }
            meshes.push(Mesh::new(gl.clone(), &data)?);
        }
        let lines = Mesh::new(gl.clone(), &[])?;
        let text = Mesh::new(gl.clone(), &[])?;
        let panel = Mesh::new(gl.clone(), &[])?;
        Ok(Self {
            gl,
            program,
            shapes,
            meshes,
            lines,
            text,
            panel,
        })
    }
    /// Draws lit collider geometry, optional debugging, and a native bitmap HUD.
    fn draw<T: RealScalar + TryCastFrom<f64>>(
        &mut self,
        world: &World<T>,
        camera: &Camera,
        size: (u32, u32),
        debug: (bool, bool),
        label: &str,
        paused: bool,
    ) -> Result<(), String>
    where
        f32: TryCastFrom<T>,
    {
        let (w, h) = size;
        if w == 0 || h == 0 {
            return Ok(());
        }
        let eye = camera.eye();
        let vp = matrix(
            transforms::perspective(50f32.to_radians(), w as f32 / h as f32, 0.1, 120.)
                * transforms::lookat(&eye, &camera.target, &Vector3::new(0., 1., 0.)),
        );
        // SAFETY: context current; viewport matches drawable size. Every draw
        // binds its owned VAO, initialized buffers, linked program and uniforms.
        unsafe {
            self.gl.viewport(0, 0, w as i32, h as i32);
            self.gl.enable(glow::DEPTH_TEST);
            self.gl.disable(glow::CULL_FACE);
            self.gl.clear_color(0.025, 0.035, 0.055, 1.);
            self.gl
                .clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);
        }
        for (i, mesh) in self.meshes.iter().enumerate() {
            let color = match i {
                0 => [0.17, 0.21, 0.27],
                1 => [0.64, 0.29, 0.12],
                _ => [0.69, 0.74, 0.68],
            };
            self.program
                .bind(&vp, &identity(), [1.; 3], 0., color, i == 0, false);
            mesh.draw(glow::TRIANGLES);
        }
        for (_, body) in world.bodies() {
            if !body.enabled() {
                continue;
            }
            let pose = body.pose();
            let axes = pose.orientation.mat3().col.map(vf);
            let position = vf(pose.position);
            let mut model = identity();
            for i in 0..3 {
                model[i * 4..i * 4 + 3].copy_from_slice(&axes[i]);
            }
            model[12..15].copy_from_slice(&position);
            let (index, dimensions, half, color) = match body.shape() {
                Shape::Sphere { radius } => (0, [f(radius); 3], 0., [0.12, 0.52, 0.95]),
                Shape::Box { half } => (1, vf(half), 0., [0.8, 0.18, 0.4]),
                Shape::Capsule {
                    half_height,
                    radius,
                } => (2, [f(radius); 3], f(half_height), [0.95, 0.62, 0.1]),
            };
            let color = if body.sleeping() {
                color.map(|x| x * 0.65)
            } else {
                color
            };
            self.program
                .bind(&vp, &model, dimensions, half, color, false, false);
            self.shapes[index].draw(glow::TRIANGLES);
        }
        let mut lines = Vec::new();
        if debug.0 {
            for (_, body) in world.bodies() {
                if !body.enabled() {
                    continue;
                }
                let b = body.bounds();
                let lo = vf(b.min);
                let hi = vf(b.max);
                let points: [[f32; 3]; 8] = core::array::from_fn(|i| {
                    core::array::from_fn(|j| if i & (1 << j) == 0 { lo[j] } else { hi[j] })
                });
                for i in 0..8 {
                    for bit in [1, 2, 4] {
                        if i & bit == 0 {
                            vertex(&mut lines, points[i], [0.; 3], 0.);
                            vertex(&mut lines, points[i | bit], [0.; 3], 0.);
                        }
                    }
                }
            }
        }
        if debug.1 {
            for (p, n) in world.contacts() {
                let p = vf(p);
                let n = vf(n);
                vertex(&mut lines, p, [0.; 3], 0.);
                vertex(
                    &mut lines,
                    core::array::from_fn(|i| p[i] + n[i] * 0.45),
                    [0.; 3],
                    0.,
                );
            }
        }
        self.lines.set(&lines);
        self.program
            .bind(&vp, &identity(), [1.; 3], 0., [0.2, 1., 0.55], false, true);
        self.lines.draw(glow::LINES);
        let stats = world.stats();
        let status = if paused { "PAUSED" } else { "RUNNING" };
        let rows = [
            format!("TWO PHASE PHYSICS  {label}  {status}"),
            format!(
                "AWAKE {}  SLEEP {}  CONTACTS {}  PAIRS {}",
                stats.awake, stats.sleeping, stats.contacts, stats.body_pairs
            ),
            "SPACE PAUSE  R RESET  1 SPHERE  2 BOX  3 CAPSULE".into(),
            "DRAG ORBIT  WHEEL ZOOM  B BOUNDS  C CONTACTS".into(),
        ];
        let mut text = Vec::new();
        for (row, line) in rows.iter().enumerate() {
            text_mesh(&mut text, line, 16., 14. + row as f32 * 18., w, h);
        }
        let mut panel = Vec::new();
        quad(&mut panel, [8., 6.], [590., 89.], w, h);
        self.panel.set(&panel);
        self.text.set(&text);
        // SAFETY: this renderer's SDL context remains current for the HUD pass.
        unsafe {
            self.gl.disable(glow::DEPTH_TEST);
        }
        self.program.bind(
            &identity(),
            &identity(),
            [1.; 3],
            0.,
            [0.025, 0.035, 0.055],
            false,
            true,
        );
        self.panel.draw(glow::TRIANGLES);
        self.program.bind(
            &identity(),
            &identity(),
            [1.; 3],
            0.,
            [0.83, 0.9, 1.],
            false,
            true,
        );
        self.text.draw(glow::TRIANGLES);
        // SAFETY: querying error state uses the same current context as drawing.
        let error = unsafe { self.gl.get_error() };
        if error != glow::NO_ERROR {
            return Err(format!("GLES render error 0x{error:x}"));
        }
        Ok(())
    }
    /// Captures the initialized back buffer as top-down RGB PPM, with explicit
    /// pack alignment and a buffer covering the entire drawable.
    fn capture(&self, size: (u32, u32), path: &PathBuf) -> Result<(), String> {
        let (w, h) = size;
        if w == 0 || h == 0 {
            return Err("cannot capture a minimized drawable".into());
        }
        let bytes = (w as usize)
            .checked_mul(h as usize)
            .and_then(|n| n.checked_mul(4))
            .ok_or("capture too large")?;
        let mut pixels = vec![0u8; bytes];
        // SAFETY: current framebuffer size is w*h; GLES guarantees the supported
        // RGBA/UNSIGNED_BYTE read pair. The allocation covers four bytes/pixel.
        unsafe {
            self.gl.finish();
            self.gl.pixel_store_i32(glow::PACK_ALIGNMENT, 1);
            self.gl.read_pixels(
                0,
                0,
                w as i32,
                h as i32,
                glow::RGBA,
                glow::UNSIGNED_BYTE,
                glow::PixelPackData::Slice(Some(&mut pixels)),
            );
        }
        // SAFETY: the capture context is still current after pixel readback.
        let error = unsafe { self.gl.get_error() };
        if error != glow::NO_ERROR {
            return Err(format!("GLES capture error 0x{error:x}"));
        }
        let mut file = BufWriter::new(fs::File::create(path).map_err(|e| e.to_string())?);
        write!(file, "P6\n{w} {h}\n255\n").map_err(|e| e.to_string())?;
        for row in pixels.chunks_exact(w as usize * 4).rev() {
            for pixel in row.chunks_exact(4) {
                file.write_all(&pixel[..3]).map_err(|e| e.to_string())?;
            }
        }
        file.flush().map_err(|e| e.to_string())?;
        Ok(())
    }
}

/// Main-thread orbit camera; independent of simulation scalar and tick cadence.
struct Camera {
    /// Horizontal orbit angle in radians.
    yaw: f32,
    /// Clamped vertical orbit angle in radians.
    pitch: f32,
    /// Clamped eye-to-target distance.
    distance: f32,
    /// World-space look target, changed only by camera input.
    target: Vector3<f32>,
}
impl Camera {
    /// Starts with both static meshes and the falling shape groups visible.
    fn new() -> Self {
        Self {
            yaw: 0.65,
            pitch: 0.5,
            distance: 28.,
            target: Vector3::new(0., 4.5, 0.),
        }
    }
    /// World eye position from the orbit parameters.
    fn eye(&self) -> Vector3<f32> {
        self.target
            + Vector3::new(
                self.yaw.sin() * self.pitch.cos(),
                self.pitch.sin(),
                self.yaw.cos() * self.pitch.cos(),
            ) * self.distance
    }
}
/// Screen-aligned quad, directly in clip space after pixel-coordinate conversion.
fn quad(out: &mut Vec<f32>, lo: [f32; 2], hi: [f32; 2], w: u32, h: u32) {
    let p = |x: f32, y: f32| [x * 2. / w as f32 - 1., 1. - y * 2. / h as f32, 0.];
    for (x, y) in [
        (lo[0], lo[1]),
        (hi[0], lo[1]),
        (hi[0], hi[1]),
        (lo[0], lo[1]),
        (hi[0], hi[1]),
        (lo[0], hi[1]),
    ] {
        vertex(out, p(x, y), [0.; 3], 0.);
    }
}
/// Built-in five-by-seven font, requiring no platform font/text dependencies.
fn glyph(c: char) -> [u8; 7] {
    match c {
        'A' => [14, 17, 17, 31, 17, 17, 17],
        'B' => [30, 17, 17, 30, 17, 17, 30],
        'C' => [14, 17, 16, 16, 16, 17, 14],
        'D' => [30, 17, 17, 17, 17, 17, 30],
        'E' => [31, 16, 16, 30, 16, 16, 31],
        'F' => [31, 16, 16, 30, 16, 16, 16],
        'G' => [14, 17, 16, 23, 17, 17, 15],
        'H' => [17, 17, 17, 31, 17, 17, 17],
        'I' => [14, 4, 4, 4, 4, 4, 14],
        'J' => [7, 2, 2, 2, 2, 18, 12],
        'K' => [17, 18, 20, 24, 20, 18, 17],
        'L' => [16, 16, 16, 16, 16, 16, 31],
        'M' => [17, 27, 21, 21, 17, 17, 17],
        'N' => [17, 25, 21, 19, 17, 17, 17],
        'O' => [14, 17, 17, 17, 17, 17, 14],
        'P' => [30, 17, 17, 30, 16, 16, 16],
        'Q' => [14, 17, 17, 17, 21, 18, 13],
        'R' => [30, 17, 17, 30, 20, 18, 17],
        'S' => [15, 16, 16, 14, 1, 1, 30],
        'T' => [31, 4, 4, 4, 4, 4, 4],
        'U' => [17, 17, 17, 17, 17, 17, 14],
        'V' => [17, 17, 17, 17, 17, 10, 4],
        'W' => [17, 17, 17, 21, 21, 21, 10],
        'X' => [17, 17, 10, 4, 10, 17, 17],
        'Y' => [17, 17, 10, 4, 4, 4, 4],
        'Z' => [31, 1, 2, 4, 8, 16, 31],
        '0' => [14, 17, 19, 21, 25, 17, 14],
        '1' => [4, 12, 4, 4, 4, 4, 14],
        '2' => [14, 17, 1, 2, 4, 8, 31],
        '3' => [30, 1, 1, 14, 1, 1, 30],
        '4' => [2, 6, 10, 18, 31, 2, 2],
        '5' => [31, 16, 16, 30, 1, 1, 30],
        '6' => [14, 16, 16, 30, 17, 17, 14],
        '7' => [31, 1, 2, 4, 8, 8, 8],
        '8' => [14, 17, 17, 14, 17, 17, 14],
        '9' => [14, 17, 17, 15, 1, 1, 14],
        _ => [0; 7],
    }
}
/// Appends pixel-snapped bitmap text into the existing dynamic HUD allocation.
fn text_mesh(out: &mut Vec<f32>, text: &str, x: f32, y: f32, w: u32, h: u32) {
    for (i, c) in text.to_ascii_uppercase().chars().enumerate() {
        for (row, bits) in glyph(c).iter().enumerate() {
            for col in 0..5 {
                if bits & (1 << (4 - col)) != 0 {
                    let x = x + i as f32 * 12. + col as f32 * 2.;
                    let y = y + row as f32 * 2.;
                    quad(out, [x, y], [x + 2., y + 2.], w, h);
                }
            }
        }
    }
}

/// Runs a typed simulation with SDL owning the GLES3 context for every GPU call.
/// Finite-frame mode is deterministic and useful with SDL's offscreen driver.
pub fn run<T: RealScalar + TryCastFrom<f64>>(options: Options) -> Result<(), String>
where
    f32: TryCastFrom<T>,
{
    let sdl = sdl2::init()?;
    let video = sdl.video()?;
    let attr = video.gl_attr();
    attr.set_context_profile(GLProfile::GLES);
    attr.set_context_version(3, 0);
    attr.set_depth_size(24);
    attr.set_double_buffer(true);
    let mut builder = video.window("Two-phase physics | SDL2 + GLES3", 1280, 800);
    builder
        .opengl()
        .resizable()
        .allow_highdpi()
        .position_centered();
    if options.hidden {
        builder.hidden();
    }
    let window = builder.build().map_err(|e| e.to_string())?;
    let context = window.gl_create_context()?;
    window.gl_make_current(&context)?;
    let _ = video.gl_set_swap_interval(if options.frames.is_some() { 0 } else { 1 });
    // SAFETY: SDL has made this GLES context current; loader pointers come from
    // that same video subsystem and resources drop before context/window below.
    let gl = unsafe {
        glow::Context::from_loader_function(|s| video.gl_get_proc_address(s) as *const _)
    };
    // SAFETY: SDL made this context current before loading its entry points.
    let version = unsafe { gl.get_parameter_string(glow::VERSION) };
    if !version.contains("OpenGL ES 3.") {
        return Err(format!("GLES3 required, received {version}"));
    }
    println!("{version}; simulation={}", options.scalar);
    let mut world = scene::demo::<T>()?;
    let mut renderer = Renderer::new(gl, &world)?;
    let mut camera = Camera::new();
    let mut events = sdl.event_pump()?;
    let (mut paused, mut debug, mut frame, mut accumulator) =
        (false, (false, false), 0usize, 0.0f64);
    let mut last = Instant::now();
    'running: loop {
        let now = Instant::now();
        let elapsed = (now - last).as_secs_f64().min(0.1);
        last = now;
        let mut single = false;
        for event in events.poll_iter() {
            match event {
                Event::Quit { .. }
                | Event::KeyDown {
                    keycode: Some(Keycode::Escape),
                    ..
                } => break 'running,
                Event::KeyDown {
                    keycode: Some(key),
                    repeat: false,
                    ..
                } => match key {
                    Keycode::Space => {
                        paused = !paused;
                        accumulator = 0.;
                    }
                    Keycode::N => single = true,
                    Keycode::R => {
                        world = scene::demo::<T>()?;
                        accumulator = 0.;
                    }
                    Keycode::B => debug.0 = !debug.0,
                    Keycode::C => debug.1 = !debug.1,
                    Keycode::Num1 | Keycode::Num2 | Keycode::Num3 => {
                        let kind = if key == Keycode::Num1 {
                            0
                        } else if key == Keycode::Num2 {
                            1
                        } else {
                            2
                        };
                        let eye = camera.eye();
                        let direction =
                            crate::collision::contact::unit(camera.target - eye).unwrap();
                        let p = camera.target - direction * 7. + Vector3::new(0., 4., 0.);
                        let position = [
                            p.x.clamp(-14., 14.) as f64,
                            p.y.clamp(3., 18.) as f64,
                            p.z.clamp(-14., 14.) as f64,
                        ];
                        let mut body = scene::body::<T>(kind, position, world.bodies().count())
                            .map_err(|e| format!("spawn: {e:?}"))?;
                        body.set_velocity(
                            scene::vector([
                                direction.x as f64 * 8.,
                                direction.y as f64 * 8.,
                                direction.z as f64 * 8.,
                            ]),
                            scene::vector([0.1, 0.3, 0.2]),
                        )
                        .map_err(|e| format!("launch: {e:?}"))?;
                        let _ = world.add_body(body);
                    }
                    _ => {}
                },
                Event::MouseMotion {
                    mousestate,
                    xrel,
                    yrel,
                    ..
                } => {
                    if mousestate.left() {
                        camera.yaw -= xrel as f32 * 0.006;
                        camera.pitch = (camera.pitch + yrel as f32 * 0.006).clamp(0.12, 1.3);
                    }
                    if mousestate.right() {
                        camera.target.x -= xrel as f32 * 0.02;
                        camera.target.y += yrel as f32 * 0.02;
                    }
                }
                Event::MouseWheel { y, .. } => {
                    camera.distance = (camera.distance * (-y as f32 * 0.1).exp()).clamp(8., 60.)
                }
                _ => {}
            }
        }
        if single {
            world.step();
        } else if !paused {
            if options.frames.is_some() {
                world.step();
                world.step();
            } else {
                accumulator = (accumulator + elapsed).min(8. / 120.);
                while accumulator >= 1. / 120. {
                    world.step();
                    accumulator -= 1. / 120.;
                }
            }
        }
        let size = window.drawable_size();
        renderer.draw(&world, &camera, size, debug, &options.scalar, paused)?;
        frame += 1;
        let done = options.frames.is_some_and(|limit| frame >= limit);
        if done {
            if let Some(path) = &options.capture {
                renderer.capture(size, path)?;
            }
        }
        window.gl_swap_window();
        if done {
            break;
        }
    }
    println!("frames={frame} stats={:?}", world.stats());
    // Resource destruction order is part of the unsafe GL ownership contract.
    drop(renderer);
    drop(context);
    drop(window);
    Ok(())
}
