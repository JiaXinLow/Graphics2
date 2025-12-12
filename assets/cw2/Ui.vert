#version 430 core
layout(location = 0) in vec2 aPos;   // screen-space position in pixels
layout(location = 1) in vec4 aColor; // RGBA

out vec4 vColor;

uniform vec2 uResolution;

void main()
{
    // convert from [0, width]x[0, height] (origin bottom-left)
    // to NDC [-1,1]x[-1,1] (origin center)
    vec2 ndc;
    ndc.x = (aPos.x / uResolution.x) * 2.0 - 1.0;
    ndc.y = (aPos.y / uResolution.y) * 2.0 - 1.0;

    gl_Position = vec4(ndc, 0.0, 1.0);
    vColor = aColor;
}