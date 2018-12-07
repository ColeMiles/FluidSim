#[macro_use]
extern crate glium;
use std::f32::consts::PI;
use std::mem;
pub mod shaders;

pub struct Rect {
    lx: f32,
    ly: f32,
    ux: f32,
    uy: f32,
}

pub struct Force {
    rect: Rect,
    fx: f32,
    fy: f32,
}

pub struct FluidGrid {
    pub vx: Vec<f32>,       // x velocity at each grid point
    pub old_vx: Vec<f32>,
    pub vy: Vec<f32>,       // y velocity at each grid point
    pub old_vy: Vec<f32>,
    pub rho: Vec<f32>,      // current density at each grid point
    pub old_rho: Vec<f32>,  // previous density at each grid point
    pub nx: usize,          // number of grid points in x (inc. boundary points)
    pub ny: usize,          // number of grid points in y (inc. boundary points)
    walls: Vec<bool>        // boolean mask which shows where walls are
}

// Simulation box -- currently restricted to be a square
impl FluidGrid {
    pub fn new(size: usize) -> FluidGrid {
        FluidGrid {
            vx: vec![0.0; (size+2) * (size+2)],
            old_vx: vec![0.0; (size+2) * (size+2)],
            vy: vec![0.0; (size+2) * (size+2)],
            old_vy: vec![0.0; (size+2) * (size+2)],
            rho: vec![0.0; (size+2) * (size+2)],
            old_rho: vec![0.0; (size+2) * (size+2)],
            walls: vec![false; (size+2) * (size+2)],
            nx: size+2,
            ny: size+2
        }
    }
    pub fn dens_step(&mut self, source: Option<Rect>, diff: f32, dt: f32) {
        match source {
            Some(r) => add_source(&mut self.rho, r, 1.0, dt),
            None => (),
        };
        diffuse(&mut self.rho, &self.old_rho, self.nx, diff, dt, true);
        mem::swap(&mut self.rho, &mut self.old_rho);
        advect(&mut self.rho, &self.old_rho, &self.vx, &self.vy, self.nx, dt, true);
        mem::swap(&mut self.rho, &mut self.old_rho);
    }

    pub fn vel_step(&mut self, force: Option<Force>, visc: f32, dt: f32) {
        mem::swap(&mut self.vx, &mut self.old_vx);
        mem::swap(&mut self.vy, &mut self.old_vy);
        diffuse(&mut self.vx, &self.old_vx, self.nx, visc, dt, true);
        diffuse(&mut self.vy, &self.old_vy, self.ny, visc, dt, false);
        project(&mut self.vx, &mut self.vy, &mut self.old_vx, &mut self.old_vy, self.nx);
        mem::swap(&mut self.vx, &mut self.old_vx);
        mem::swap(&mut self.vy, &mut self.old_vy);
        advect(&mut self.vx, &self.old_vx, &self.old_vx, &self.old_vy, self.nx, dt, true);
        advect(&mut self.vy, &self.old_vy, &self.old_vx, &self.old_vy, self.nx, dt, false);
        project(&mut self.vx, &mut self.vy, &mut self.old_vx, &mut self.old_vy, self.nx);
    }

    pub fn test_init(&mut self) {
        for i in 0..self.nx {
            for j in 0..self.ny {
                let ind = self.nx * i + j;
                let x = i as f32 / self.nx as f32;
                let y = j as f32 / self.ny as f32;
                self.vx[ind] = 0.0;
                self.vy[ind] = 0.0;
                self.old_vx[ind] = 0.0;
                self.old_vy[ind] = 0.0;
                self.rho[ind] = 0.0;
                self.old_rho[ind] = 0.0;
            }
        }
        flow_bdy_conds(&mut self.vx, self.nx);
        flow_bdy_conds(&mut self.old_vx, self.nx);
        flow_bdy_conds(&mut self.rho, self.nx);
        flow_bdy_conds(&mut self.old_rho, self.nx);
        // Set up a small (square for now) barrier for the fluid to flow around
        let start_ind = 2 * self.nx / 5;
        let end_ind = 3 * self.nx / 5;
        for i in start_ind..end_ind {
            for j in start_ind..end_ind {
                let ind = i * self.nx + j;
                self.walls[ind] = true;
            }
        }
    }
}

fn add_source(field: &mut Vec<f32>, rect: Rect, rate: f32, dt: f32) {
    for i in rect.lx.round() as usize..rect.ux.round() as usize {
        for j in rect.ly.round() as usize..rect.uy.round() as usize {
            let size = field.len();
            let ind = size * i + j;
            field[ind] += rate * dt;
        }
    }
}

// Gauss-Seidel relaxation of diffusion term
// If flow is set, then this is a field which needs the flow boundary conditions on it
// (Gross, I know...)
pub fn diffuse(field: &mut Vec<f32>, old_field: &Vec<f32>, N: usize, diff: f32, dt: f32, flow: bool) {
    let gam: f32 = dt * diff * N as f32 * N as f32;
    for k in 0..20 {
        for i in 1..N-1 {
            for j in 1..N-1 {
                let ind = N * i + j;
                field[ind] = (old_field[ind] +
                    gam * (field[ind - 1] + field[ind + 1] + field[ind - N] +
                           field[ind + N])) / (1.0 + 4.0 * gam);
            }
        }
        open_bdy_conds(field, N);
        if flow {
            flow_bdy_conds(field, N);
        }
    }
}

pub fn advect(field: &mut Vec<f32>, old_field: &Vec<f32>, 
              velx: &Vec<f32>, vely: &Vec<f32>, N: usize, dt: f32, flow: bool) {
    let fsize = N as f32;
    let gam = dt * fsize;
    for i in 1..N-1 {
        for j in 1..N-1 {
            let ind = i * N + j;
            let mut x = i as f32 - gam * velx[ind];
            let mut y = j as f32 - gam * vely[ind];
            if x < 0.5 {
                x = 0.5;
            } else if x > fsize + 0.5 {
                x = fsize + 0.5;
            }
            if y < 0.5 {
                y = 0.5;
            } else if y > fsize + 0.5 {
                y = fsize + 0.5;
            }
            let i0 = x as usize;
            let j0 = y as usize;
            let s1 = x - i0 as f32;
            let s0 = 1.0 - s1;
            let t1 = y - j0 as f32;
            let t0 = 1.0 - t1;
            field[ind] = s0 * (t0 * old_field[N * i0 + j0] +
                               t1 * old_field[N * i0 + j0 + 1]) +
                         s1 * (t0 * old_field[N * (i0 + 1) + j0] +
                               t1 * old_field[N * (i0 + 1) + j0 + 1]);
        }
    }
    open_bdy_conds(field, N);
    if flow {
        flow_bdy_conds(field, N);
    }
}

pub fn project(velx: &mut Vec<f32>, vely: &mut Vec<f32>, 
           p: &mut Vec<f32>, div: &mut Vec<f32>, N: usize) {
    let dx = 1.0 / N as f32;
    for i in 1..N-1 {
        for j in 1..N-1 {
            let ind = i * N + j;
            div[ind] = -0.5 * dx * (velx[ind + N] - velx[ind - N] +
                                    vely[ind + 1] - vely[ind - 1]);
            p[ind] = 0.0;
        }
    }
    open_bdy_conds(div, N);
    open_bdy_conds(p, N);

    for k in 0..20 {
        for i in 1..N-1 {
            for j in 1..N-1 {
                let ind = i * N + j;
                p[ind] = (div[ind] + p[ind - N] + p[ind + N] +
                                     p[ind - 1] + p[ind + 1]) / 4.0;
            }
        }
        open_bdy_conds(p, N);
    }
    for i in 1..N-1 {
        for j in 1..N-1 {
            let ind = i * N + j;
            velx[ind] -= 0.5 * (p[ind + N] - p[ind - N]) / dx;
            vely[ind] -= 0.5 * (p[ind + 1] - p[ind - 1]) / dx;
        }
    }
    open_bdy_conds(velx, N);
    open_bdy_conds(vely, N);
    flow_bdy_conds(velx, N);
}

// Copies wall values to be the values of adjacent grid points
fn open_bdy_conds(field: &mut Vec<f32>, N: usize) {
    for i in 1..N-1 {
        field[i] = field[N+i];
        field[N*(N-1) + i] = field[N*(N-2) + i];
        field[N*i] = field[N*i + 1];
        field[N*i + N] = field[N*i + N - 1];
    }
    // Set corners to average values of two adjacent walls
    field[0] = 0.5 * (field[1] + field[N-1]);
    field[N-1] = 0.5 * (field[N-2] + field[2*N - 1]);
    field[N*(N-1)] = 0.5 * (field[N*(N-2)] + field[N*(N-1) + 1]);
    field[N*(N-1) + N-1] = 0.5 * (field[N*(N-2) + N-1] + field[N*(N-1) + N-2]);
}

// For use on the velocity fields to set a small inlet fixed velocity
fn flow_bdy_conds(field: &mut Vec<f32>, N: usize) {
    // Small in-flow of width 1/6 of the grid
    let start_ind = 5 * N / 12 ;
    let end_ind = 7 * N / 12;
    let dx = 1.0 / (N - 1) as f32;
    for j in start_ind..end_ind {
        field[j] = 5.0 * (PI * dx * (j - start_ind) as f32 / (((end_ind - start_ind) as f32) * dx)).sin();
    }
}

#[derive(Copy, Clone)]
pub struct VectorVert {
    pos: [f32; 2],
    vel: [f32; 2],
}
implement_vertex!(VectorVert, pos, vel);

#[derive(Copy, Clone)]
pub struct DensityVert {
    pos: [f32; 2],
    density: f32,
}
implement_vertex!(DensityVert, pos, density);

#[derive(Copy, Clone)]
pub struct WallVert {
    pos: [f32; 2],
}
implement_vertex!(WallVert, pos);

// TODO: Swap both of these to output fixed size arrays
// Generates a Vec of vertices that can be fed into the shaders for drawing
//  the vectors at each gridpoint.
pub fn gen_vector_verts(fluidgrid: &FluidGrid) -> Vec<VectorVert> {
    let mut vector_verts: Vec<VectorVert> = Vec::with_capacity(
        (fluidgrid.nx - 2) * (fluidgrid.ny - 2)
    );
    let dx = 2.0 / ((fluidgrid.nx - 2) as f32);
    // Not drawing boundary points at the moment    
    let mut pos = [-1.0 - dx / 2.0 , -1.0 - dx / 2.0];
    for i in 1..fluidgrid.nx-1 {
        pos[0] += dx;
        pos[1] = -1.0 - dx / 2.0;
        for j in 1..fluidgrid.ny-1 {
            pos[1] += dx;
            let ind = fluidgrid.ny * i + j;
            vector_verts.push(
                VectorVert{
                    pos,
                    vel: [fluidgrid.vx[ind], fluidgrid.vy[ind]]
                }
            );
        }
    }
    vector_verts
}

// Generates a Vec of vertices that can be fed into the shaders for drawing
//  the density visualization. Also returns the Vec of indices to link the vertices,
//  in the format expected by TrianglesList. Could probably be the more memory efficient
//  TriangleStrip if I change iteration order to "snake" across grid.
pub fn gen_density_verts(fluidgrid: &FluidGrid) -> (Vec<DensityVert>, Vec<u32>) {
    let mut density_verts: Vec<DensityVert> = Vec::with_capacity(
        fluidgrid.nx * fluidgrid.ny
    );
    let mut indices: Vec<u32> = Vec::with_capacity(
        6 * (fluidgrid.nx - 1) * (fluidgrid.ny - 1)
    );
    let dx = 2.0 / (fluidgrid.nx as f32 - 2.0);
    let mut pos = [-1.0 - 3.0 * dx / 2.0, -1.0 - 3.0 * dx / 2.0];
    for i in 0..fluidgrid.nx {
        pos[0] += dx;
        pos[1] = -1.0 - 3.0 * dx / 2.0;
        for j in 0..fluidgrid.ny {
            pos[1] += dx;
            let ind = fluidgrid.ny * i + j;
            density_verts.push(
                DensityVert{
                    pos,
                    density: fluidgrid.rho[ind]
                }
            );
            if (i < fluidgrid.nx - 1) && (j < fluidgrid.ny -1) {
                indices.push(ind as u32);
                indices.push((ind + 1) as u32);
                indices.push((ind + fluidgrid.ny + 1) as u32);
                indices.push(ind as u32);
                indices.push((ind + fluidgrid.ny) as u32);
                indices.push((ind + fluidgrid.ny + 1) as u32);                
            }
        }
    }
    (density_verts, indices)
}
