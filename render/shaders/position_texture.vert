// basic position + texture coordinates vertex shader
#version 430 core
layout (location = 0) in vec3 position;
//layout (location = 1) in vec3 aColor;
layout (location = 1) in vec2 texture_position;

//out vec3 color;
out vec2 uv;

void main()
{
	gl_Position = vec4(position, 1.0);
//	ourColor = aColor;
	uv = vec2(texture_position.x, texture_position.y);
}