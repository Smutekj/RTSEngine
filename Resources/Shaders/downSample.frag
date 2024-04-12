#version 330 core

uniform sampler2D source;
uniform vec2 		sourceSize = vec2(1920, 1080);

in vec4 vertexColor;
in vec2 texCoords;

out vec4 FragmentColor;

void main(void)
{
	vec2 pixelSize = vec2(1.0 / sourceSize.x, 1.0 / sourceSize.y);

	vec4 color = texture2D(source, texCoords);
	color     += texture2D(source, texCoords + vec2( 1.0,  0.0) * pixelSize);
	color     += texture2D(source, texCoords + vec2(-1.0,  0.0) * pixelSize);
	color     += texture2D(source, texCoords + vec2( 0.0,  1.0) * pixelSize);
	color     += texture2D(source, texCoords + vec2( 0.0, -1.0) * pixelSize);
	color     += texture2D(source, texCoords + vec2( 1.0,  1.0) * pixelSize);
	color     += texture2D(source, texCoords + vec2(-1.0, -1.0) * pixelSize);
	color     += texture2D(source, texCoords + vec2( 1.0, -1.0) * pixelSize);
	color     += texture2D(source, texCoords + vec2(-1.0,  1.0) * pixelSize);
	FragmentColor = color / 9.0;

	FragmentColor.a = 1-(FragmentColor.r + FragmentColor.g + FragmentColor.b)/3;
}
