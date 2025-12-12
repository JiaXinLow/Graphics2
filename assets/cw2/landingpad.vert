#version 430

// Vertex inputs
layout(location = 0) in vec3 inPosition; // Vertex position
layout(location = 1) in vec3 inNormal; // Vertex normal

// Transformation matrices
uniform mat4 uModel; // Model matrix
uniform mat4 uView; // View matrix
uniform mat4 uProj; // Projection matrix

// Outputs to fragment shader
out vec3 vNormal; // Transformed normal
out vec3 vWorldPos; // World-space position

void main()
{
    // Transform normal (ignore translation)
    vNormal = mat3(uModel) * inNormal;

    // Compute world-space position
    vec4 worldPos = uModel * vec4(inPosition, 1.0);
    vWorldPos = worldPos.xyz;

    // Final clip-space position
    gl_Position = uProj * uView * uModel * vec4(inPosition, 1.0);
}
