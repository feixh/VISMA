// Basic depth linearization fragment shader.
// http://roxlu.com/2014/036/rendering-the-depth-buffer
#version 430 core

in vec2 uv;
out vec3 color;
uniform sampler2D depth_texture;

// Ideally, z_near and z_far should be uniform variables and
// set by user outside the shader.
// For instance, the following chunk of code sets values for znear and zfar:

// GLint znear_location = glGetUniformLocation(shader->Program, "z_near");
// GLint zfar_location = glGetUniformLocation(edge_shader_->Program, "z_far");
// glUniform1f(znear_location, (float)znear_value);
// glUniform1f(zfar_location, (float)zfar_value);

uniform float z_near = 0.05;
uniform float z_far = 5.0;

void main(){
    float z = texture(depth_texture, uv).r;
    float c = z_far * z_near / (z_far - z*(z_far - z_near) );
    color = vec3(c);
}
