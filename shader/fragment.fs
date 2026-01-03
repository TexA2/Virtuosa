#version 450 core

in vec3 intensityColor;


out vec4 FragColor;

uniform  vec4 ourColor;
uniform bool useIntensityColor;

void main(){

    if (useIntensityColor)
        FragColor = vec4(intensityColor, 1.0);
    else
        FragColor = ourColor;
}