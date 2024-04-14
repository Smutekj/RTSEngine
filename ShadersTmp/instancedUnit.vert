
#version 330 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in vec4 aCol;
layout (location = 2) in vec2 aTexCoords;

// instanced stuff
layout (location = 3) in float angle;
layout (location = 4) in float scale;
layout (location = 5) in vec2 translation;
layout (location = 6) in vec2 tex_coord;
layout (location = 7) in vec4 color;

out vec2 texCoords;
out vec4 vertexColor;

uniform mat4 view;

void main()
{
    float angle_rad = angle * 3.141592/180.;
    vec4 transformed_pos =  vec4(aPos.x * cos(angle_rad) - aPos.y * sin(angle_rad), aPos.x * sin(angle_rad) + aPos.y * cos(angle_rad), 0, 1.0);
    transformed_pos.x *= scale;
    transformed_pos.y *= scale;
    transformed_pos.xy += translation;

    gl_Position = view * transformed_pos;
    
    texCoords = tex_coord;
    vertexColor = color;
}