#version 330 core


in vec4 vertexColor;
in vec2 texCoords;

uniform vec2 u_resolution = vec2(800, 600);
uniform float u_time;

const vec4 base_color = vec4(0.167780,0.158743,0.841060,1.000000); 
const vec4 spots_color = vec4(0.000000,0.000000,0.000000,1.000000); 

vec2 random2(vec2 st){
    st = vec2( dot(st,vec2(127.1,311.7)),
              dot(st,vec2(269.5,183.3)) );
    return -1.0 + 2.0*fract(sin(st)*43758.5453123);
}

// Gradient Noise by Inigo Quilez - iq/2013
// https://www.shadertoy.com/view/XdXGW8
float noise(vec2 st) {
    vec2 i = floor(st);
    vec2 f = fract(st);

    vec2 u = f*f*(3.0-2.0*f);

    return mix( mix( dot( random2(i + vec2(0.0,0.0) ), f - vec2(0.0,0.0) ),
                     dot( random2(i + vec2(1.0,0.0) ), f - vec2(1.0,0.0) ), u.x),
                mix( dot( random2(i + vec2(0.0,1.0) ), f - vec2(0.0,1.0) ),
                     dot( random2(i + vec2(1.0,1.0) ), f - vec2(1.0,1.0) ), u.x), u.y);
}

void main() {
    vec2 st = texCoords;
    vec3 color = base_color.rgb;

    vec2 pos = vec2(st*10.0);

    color +=  (1 - spots_color.rgb) * (noise(pos)*.5+.5) ;

    gl_FragColor = vec4(color,1.0);
}
