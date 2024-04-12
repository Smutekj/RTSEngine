#version 330 core


in vec4 vertexColor;
in vec2 texCoords;

uniform sampler2D renderedTexture;
uniform float time;

out vec4 FragColor;

void main()
{
    FragColor = vertexColor;
}
