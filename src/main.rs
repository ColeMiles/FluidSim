#[macro_use]
extern crate glium;
extern crate fluidsim;

// TEMP - REMOVE
fn printout(fbox: &fluidsim::FluidGrid) {
    let mut maxxval = 0.0;
    let mut maxxval_inds = (0, 0);
    let mut maxyval = 0.0;
    let mut maxyval_inds = (0, 0);
    let mut maxrho = 0.0;
    let mut maxrho_inds = (0, 0);
    for i in 0..fbox.nx {
        for j in 0..fbox.ny {
            let ind = i * fbox.nx + j;
            if fbox.vx[ind].abs() > maxxval {
                maxxval = fbox.vx[ind].abs();
                maxxval_inds = (i, j);
            }
            if fbox.vy[ind].abs() > maxyval {
                maxyval = fbox.vy[ind].abs();
                maxyval_inds = (i, j);
            }
            if fbox.rho[ind] > maxrho {
                maxrho = fbox.rho[ind];
                maxrho_inds = (i, j);
            }
        }
    }
    println!("Max x vel: {} at indices (i, j) = ({}, {})", maxxval, maxxval_inds.0, maxxval_inds.1);
    println!("Max y vel: {} at indices (i, j) = ({}, {})", maxyval, maxyval_inds.0, maxxval_inds.1);
    println!("Max rho: {} at indices (i, j) = ({}, {})", maxrho, maxrho_inds.0, maxrho_inds.1);
}

fn main() {
    use glium::{glutin, Surface, uniforms::AsUniformValue};

    // Set up display window
    let mut events_loop = glutin::EventsLoop::new();
    let window = glutin::WindowBuilder::new();
    let context = glutin::ContextBuilder::new();
    let display = glium::Display::new(window, context, &events_loop).unwrap();

    // Set up the fluid simulation
    let mut fluidbox = fluidsim::FluidGrid::new(100);
    fluidbox.test_init();

    // Make shader programs
    let vector_program = glium::Program::from_source(
        &display,
        fluidsim::shaders::VECTOR_VERT_SHADER_SRC,
        fluidsim::shaders::VECTOR_FRAG_SHADER_SRC,
        Some(fluidsim::shaders::VECTOR_GEOM_SHADER_SRC),
    ).unwrap();
    let density_program = glium::Program::from_source(
        &display,
        fluidsim::shaders::DENSITY_VERT_SHADER_SRC,
        fluidsim::shaders::DENSITY_FRAG_SHADER_SRC,
        None,
    ).unwrap();
    let mouse_program = glium::Program::from_source(
        &display, 
        fluidsim::shaders::MOUSE_VERT_SHADER_SRC,
        fluidsim::shaders::MOUSE_FRAG_SHADER_SRC,
        None,
    ).unwrap();

    // Main loop
    let mut closed = false;
    let mut paused = true;
    let mut draw_vectors = true;
    let mut draw_density = true;
    let mut mouse_position = glutin::dpi::LogicalPosition::new(0.0, 0.0);
    while !closed {
        let mut target = display.draw();
        target.clear_color(0.0, 0.0, 0.0, 1.0);

        if draw_density {
            let (density_verts, density_inds) = fluidsim::gen_density_verts(&fluidbox);
            let density_vertex_buffer = glium::VertexBuffer::new(&display, &density_verts).unwrap();
            let density_index_buffer = glium::IndexBuffer::new(
                &display, 
                glium::index::PrimitiveType::TrianglesList, 
                &density_inds
            ).unwrap();
            target.draw(&density_vertex_buffer, &density_index_buffer, &density_program,
                        &glium::uniforms::EmptyUniforms, &Default::default()).unwrap();
        }

        if draw_vectors {
            let vector_verts = fluidsim::gen_vector_verts(&fluidbox);
            let vector_vertex_buffer = glium::VertexBuffer::new(&display, &vector_verts).unwrap();
            let vector_index_buffer = glium::index::NoIndices(glium::index::PrimitiveType::Points);
            target.draw(&vector_vertex_buffer, &vector_index_buffer, &vector_program, 
                        &glium::uniforms::EmptyUniforms, &Default::default()).unwrap();
        }

        // let mouse_vertex_buffer = glium::VertexBuffer::new(&display, MY_VEC).unwrap();
        // let mouse_index_buffer = glium::index::NoIndices(glium::index::PrimitiveType::LineStrip);
        

        // let dims_uniform = target.get_dimensions().as_uniform_value();
        // let dims_uniform_buffer = glium::uniforms::UniformBuffer::new(&display, dims_uniform).unwrap();

        events_loop.poll_events(|ev| {
            match ev {
                glutin::Event::WindowEvent{event, ..} => match event {
                    glutin::WindowEvent::CloseRequested => closed = true,
                    glutin::WindowEvent::KeyboardInput{input, ..} => {
                        if input.state == glutin::ElementState::Pressed {
                            match input.virtual_keycode {
                                Some(glutin::VirtualKeyCode::Escape) => closed = true,
                                Some(glutin::VirtualKeyCode::R) => fluidbox.test_init(),
                                Some(glutin::VirtualKeyCode::N) => fluidbox.vel_step(None, 0.005, 0.005),
                                Some(glutin::VirtualKeyCode::V) => draw_vectors = !draw_vectors,
                                Some(glutin::VirtualKeyCode::D) => draw_density = !draw_density,
                                Some(glutin::VirtualKeyCode::Key1) => {
                                    fluidsim::diffuse(&mut fluidbox.vx, &mut fluidbox.old_vx, fluidbox.nx, 0.005, 0.005, true);
                                    fluidsim::diffuse(&mut fluidbox.vy, &mut fluidbox.old_vy, fluidbox.ny, 0.005, 0.005, false);
                                },
                                Some(glutin::VirtualKeyCode::Key2) => {
                                    fluidsim::project(&mut fluidbox.vx, &mut fluidbox.vy, &mut fluidbox.old_vx, &mut fluidbox.old_vy, fluidbox.nx);
                                },
                                Some(glutin::VirtualKeyCode::Key3) => {
                                    fluidsim::advect(&mut fluidbox.vx, &fluidbox.old_vx, &fluidbox.old_vx, &fluidbox.old_vy, fluidbox.nx, 0.005, true);
                                    fluidsim::advect(&mut fluidbox.vy, &fluidbox.old_vy, &fluidbox.old_vx, &fluidbox.old_vy, fluidbox.nx, 0.005, false);
                                },
                                Some(glutin::VirtualKeyCode::Key4) => {
                                    std::mem::swap(&mut fluidbox.vx, &mut fluidbox.old_vx);
                                    std::mem::swap(&mut fluidbox.vy, &mut fluidbox.old_vy);
                                },
                                Some(glutin::VirtualKeyCode::Key5) => {
                                    printout(&fluidbox);
                                },
                                Some(glutin::VirtualKeyCode::Space) => paused = !paused,
                                _ => (),
                            }
                        }
                    },
                    glutin::WindowEvent::CursorMoved{position, ..} => {
                        mouse_position = position;
                    },
                    glutin::WindowEvent::MouseInput{state, button, ..} => {
                        if (state == glutin::ElementState::Pressed) && (button == glutin::MouseButton::Left) {
                            target.clear_color(1.0, 0.0, 0.0, 1.0);
                        }
                    },
                    _ => (),
                },
                _ => (),
            }
        });
        target.finish().unwrap();
        if !paused {
            fluidbox.dens_step(None, 0.0005, 0.005);
            fluidbox.vel_step(None, 0.005, 0.005);            
        }
    }
}
