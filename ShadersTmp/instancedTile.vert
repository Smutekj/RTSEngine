
#version 330 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in vec4 aCol;
layout (location = 2) in vec2 aTexCoords;

// instanced stuff
layout (location = 3) in vec2 translation;
layout (location = 4) in vec2 tex_coord;
layout (location = 5) in vec4 color;

out vec2 texCoords;
out vec4 vertexColor;

uniform mat4 view;
uniform float cell_size = 5;

void main()
{
    vec4 transformed_pos =  vec4(aPos.x*cell_size, aPos.y*cell_size, 0, 1.0);
    transformed_pos.xy += translation;

    gl_Position = view * transformed_pos ;
    
    texCoords = tex_coord;
    vertexColor = color;
}