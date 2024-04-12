#version 330 core

uniform float u_time;
uniform vec2 u_resolution = vec2(800, 600);

const int detail_steps_ = 13;
const vec3 mod3_  =    vec3(.1031, .11369, .13787);


vec3  hash3_3(vec3 p3);
float perlin_noise3(vec3 p);
float noise_sum_abs3(vec3 p);
vec2  domain(vec2 uv, float s);


in vec4 vertexColor;
in vec2 texCoords;

out vec4 FragColor;


vec3 hash3_3(vec3 p3) {
	p3 = fract(p3 * mod3_);
    p3 += dot(p3, p3.yxz + 19.19);
    return -1. + 2. * fract(vec3((p3.x + p3.y) * p3.z, (p3.x+p3.z) * p3.y, (p3.y+p3.z) * p3.x));
}

float perlin_noise3(vec3 p) {
    vec3 pi = floor(p);
    vec3 pf = p - pi;
    
    vec3 w = pf * pf * (3. - 2. * pf);
    
    return 	mix(
    	mix(
            mix(
                dot(pf - vec3(0, 0, 0), hash3_3(pi + vec3(0, 0, 0))), 
                dot(pf - vec3(1, 0, 0), hash3_3(pi + vec3(1, 0, 0))),
                w.x),
            mix(
                dot(pf - vec3(0, 0, 1), hash3_3(pi + vec3(0, 0, 1))), 
                dot(pf - vec3(1, 0, 1), hash3_3(pi + vec3(1, 0, 1))),
                w.x),
    	w.z),
        mix(
            mix(
                dot(pf - vec3(0, 1, 0), hash3_3(pi + vec3(0, 1, 0))), 
                dot(pf - vec3(1, 1, 0), hash3_3(pi + vec3(1, 1, 0))),
                w.x),
            mix(
                dot(pf - vec3(0, 1, 1), hash3_3(pi + vec3(0, 1, 1))), 
                dot(pf - vec3(1, 1, 1), hash3_3(pi + vec3(1, 1, 1))),
                w.x),
     	w.z),
	w.y);
}


float noise_sum_abs3(vec3 p) {
    float f = 0.;
    p = p * 3.;
    f += 1.0000 * abs(perlin_noise3(p)); p = 2. * p;
    f += 0.5000 * abs(perlin_noise3(p)); p = 3. * p;
	f += 0.2500 * abs(perlin_noise3(p)); p = 4. * p;
	f += 0.1250 * abs(perlin_noise3(p)); p = 5. * p;
	f += 0.0625 * abs(perlin_noise3(p)); p = 6. * p;
    
    return f;
}

vec2 domain(vec2 uv, float s) {
    return (2.*uv.xy-u_resolution.xy) / u_resolution.y*s;
}


void main() {
	vec2 p = texCoords * 2.; //domain(texCoords, 2.5);
   	
    float electric_density = .9;
    float electric_radius  = length(p) - .4;
    float velocity = .1;
    
    float moving_coord = sin(velocity * u_time) / .2 * cos(velocity * u_time);
    vec3  electric_local_domain = vec3(p, moving_coord);
    float electric_field = electric_density * noise_sum_abs3(electric_local_domain); 
    
    vec3 col = vec3(107, 148, 196) / 255.;
 	col += (1. - (electric_field + electric_radius));
    for(int i = 0; i < detail_steps_; i++) {
    	if(length(col) >= 2.1 + float(i) / 2.)
            col -= .3;
    }
    col += 1. - 4.2*electric_field;
    
    float alpha = 1.;
    FragColor = vec4(col, alpha);
}
