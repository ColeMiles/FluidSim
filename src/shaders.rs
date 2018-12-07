pub const VECTOR_VERT_SHADER_SRC: &'static str = r#"
    #version 330 core
    in vec2 pos;
    in vec2 vel;
    out vec2 vert_vel;
    void main() {
        gl_Position = vec4(pos, 0.0, 1.0);
        vert_vel = vel;
    }
"#;

// Pass in screen height and width to normalize vector length by screen dimensions
pub const VECTOR_GEOM_SHADER_SRC: &'static str = r#"
    #version 330 core
    layout (points) in;
    layout (line_strip, max_vertices = 4) out;
    in vec2 vert_vel[];
    
    mat2 rotate(float theta) {
        mat2 rotmat = mat2(cos(theta), sin(theta), 
                           -sin(theta), cos(theta));
        return rotmat;
    }

    void main() {
        vec2 arrow_center = gl_in[0].gl_Position.xy;
        vec2 arrow_zw = gl_in[0].gl_Position.zw;
        float theta = atan(vert_vel[0].y, vert_vel[0].x);
        float len = 2.0 * length(vert_vel[0]);
        mat2 rotmat = rotate(theta);
        vec2 pos = arrow_center + rotmat * len * vec2(-0.01, 0.0);
        gl_Position = vec4(pos, arrow_zw);
        EmitVertex();
        pos = arrow_center + rotmat * len * vec2(0.01, 0.0);
        gl_Position = vec4(pos, arrow_zw);
        EmitVertex();
        pos = arrow_center + rotmat * len * vec2(0.0075, -0.005);
        gl_Position = vec4(pos, arrow_zw);
        EmitVertex();
        pos = arrow_center + rotmat * len * vec2(0.0075, 0.005);
        gl_Position = vec4(pos, arrow_zw);
        EmitVertex();
        pos = arrow_center + rotmat * len * vec2(0.01, 0.0);
        gl_Position = vec4(pos, arrow_zw);
        EmitVertex();
        EndPrimitive();
    }
"#;

pub const VECTOR_FRAG_SHADER_SRC: &'static str = r#"
    #version 330 core
    void main() {
        gl_FragColor = vec4(1.0, 1.0, 1.0, 1.0);
    }
"#;

pub const DENSITY_VERT_SHADER_SRC: &'static str = r#"
    #version 330 core
    in vec2 pos;
    in float density;
    out vec4 vert_color;
    void main() {
        gl_Position = vec4(pos, 0.0, 1.0);
        vert_color = vec4(0.0, (density / 2.0), 0.0, 1.0);
    }
"#;

pub const DENSITY_FRAG_SHADER_SRC: &'static str = r#"
    #version 330 core
    noperspective in vec4 vert_color;
    void main() {
        gl_FragColor = vert_color;
    }
"#;

pub const MOUSE_VERT_SHADER_SRC: &'static str = r#"
    #version 330 core
    in vec2 pos;
    void main() {
        gl_Position = vec4(pos, 0.0, 1.0);
    }
"#;

pub const MOUSE_FRAG_SHADER_SRC: &'static str = r#"
    #version 330 core
    void main() {
        gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);
    }
"#;