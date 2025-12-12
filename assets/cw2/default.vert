#version 430

// Vertex inputs
layout(location = 0) in vec3 inPosition; // Vertex position
layout(location = 1) in vec3 inNormal; // Vertex normal
layout(location = 2) in vec2 inUV; // Texture coordinates
layout(location = 3) in float inSize; // Particle-specific attribute (bound in particle VAO)

// Outputs to fragment shader
out vec3 vNormal;
out vec3 vWorldPos;
out vec2 vUV;                     
out float vPointSize; // Pass particle size

// Uniforms
uniform mat4 uModel; // Model transform
uniform mat4 uView; // View transform
uniform mat4 uProj; // Projection transform

// Particle-specific uniform
uniform int uIsParticle; // 1 = particle, 0 = normal mesh
uniform sampler2D uTexture;

void main()
{
    vec4 worldPos = uModel * vec4(inPosition, 1.0); // Transform to world space
    vWorldPos = worldPos.xyz;

    if (uIsParticle == 1) {
        // Particle path
        gl_Position = uProj * uView * worldPos; // Project position
        gl_PointSize = inSize; // Set point size

        // No manual UVs; fragment shader uses gl_PointCoord
        vUV = vec2(0.0);
        vNormal = vec3(0.0);
        vPointSize = inSize;
    } else {
        // Mesh path
        vNormal = mat3(uModel) * inNormal; // Transform normal
        vUV = inUV; // Pass UVs
        gl_Position = uProj * uView * worldPos; // Project position
        vPointSize = 0.0;
    }
}
