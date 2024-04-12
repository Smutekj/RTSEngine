#version 330 core


uniform sampler2D image;

in vec4 vertexColor;
in vec2 texCoords;

out vec4 FragColor;

void main()
{
    FragColor = vec4(texture2D(image, texCoords).rgb, 0.5);
}
