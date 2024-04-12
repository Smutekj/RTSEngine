
#version 330 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in vec4 aCol;
layout (location = 2) in vec2 aTexCoords;

out vec2 texCoords;
out vec4 vertexColor;

uniform mat4 view;
uniform mat4 transform;

void main()
{
    gl_Position =  view * transform * vec4(aPos.x, aPos.y, 0.0, 1.0);
    texCoords = aTexCoords;
    vertexColor = aCol;
}