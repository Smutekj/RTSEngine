#version 330 core

uniform sampler2D image;

in vec4 vertexColor;
in vec2 texCoords;

out vec4 FragmentColor;

uniform float offset[5] = float[]( 0.0, 1.0, 2.0, 3.0, 4.0 );
uniform float weight[5] = float[]( 0.2270270270, 0.1945945946, 0.1216216216, 0.0540540541, 0.0162162162 );

void main(void)
{
	FragmentColor = texture2D( image, texCoords ) * weight[0]; //  vec4((texCoords.x + 1.f)/2., 0, 0, 1); //
	for (int i=1; i<5; i++) {
		FragmentColor += texture2D( image, ( texCoords+vec2(offset[i]/800, 0.0) ) ) * weight[i];
 		FragmentColor += texture2D( image, ( texCoords-vec2(offset[i]/800, 0.0) ) ) * weight[i];
	}
	FragmentColor.a = 1-(FragmentColor.r + FragmentColor.g + FragmentColor.b)/3;
}
