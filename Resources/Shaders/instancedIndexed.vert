
#version 330 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in vec4 aCol;
layout (location = 2) in vec2 aTexCoords;

// instanced stuff
layout (location = 3) in float angle;
layout (location = 4) in vec2 scale;
layout (location = 5) in vec2 translation;

out vec2 texCoords;
out vec4 vertexColor;

uniform mat4 view;

void main()
{
    

    vec4 transformed_pos =  vec4(aPos.x * cos(angle) - aPos.y * sin(angle), aPos.x * sin(angle) + aPos.y * cos(angle), 0, 1.0);
    transformed_pos.xy *= scale;
    transformed_pos.xy += translation;

    gl_Position = view * transformed_pos;
    
    texCoords = aTexCoords;
    vertexColor = aCol;
}