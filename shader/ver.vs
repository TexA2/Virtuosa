#version 450 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

uniform mat4 mvpMatrix;

out vec3 intensityColor;


void main(){
    gl_Position = mvpMatrix * vec4(aPos, 1.0);
    intensityColor = aColor;
}