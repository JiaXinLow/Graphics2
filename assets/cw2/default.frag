#version 430

// Inputs from vertex shader
in vec3 vNormal; // Interpolated normal
in vec3 vWorldPos; // World-space position
in vec2 vUV; // Texture coordinates

// Output color
out vec4 fragColor;

// Uniforms
uniform sampler2D uTexture;  // Bound texture
uniform vec3 uLightDir; // Directional light direction
uniform int uDirEnabled; // 1 = on, 0 = off
uniform vec3 uViewPos;  // Camera position
uniform int uIsParticle;  //1 = particle, 0 = normal mesh

// Point light struct
struct PointLight {
    vec3 position;
    vec3 color;
    int enabled; // 1 = on, 0 = off
};

uniform PointLight uLights[3]; // Array of point lights

// Task 1.6 Helper: Blinn-Phong with attenuation
vec3 blinnPhongPoint(
    vec3 normal, // Surface normal (N)
    vec3 viewDir, // View direction (V)
    vec3 fragPos, // Fragment world position (P)
    vec3 Kd, // Diffuse colour (material albedo)
    vec3 lightPos, // Point light position
    vec3 lightColor, // Point light colour
    int lightEnabled, // Toggle flag
    float Ns // shininess (specular exponent)
)
{
    if (lightEnabled == 0) return vec3(0.0); // skip if disabled

    // Light direction and attenuation
    vec3 L = lightPos - fragPos;
    float dist = length(L);
    L = normalize(L);
    float attenuation = 1.0 / (dist * dist);
    float diffuse = max(dot(normal, L), 0.0); // Diffuse term
    vec3 H = normalize(L + viewDir); // Half-vector
    float specular = pow(max(dot(normal, H), 0.0), Ns); // Blinn–Phong specular

    // Combine diffuse and specular, scaled by light colour and attenuation
    return (Kd * diffuse + vec3(specular)) * lightColor * attenuation;
}

void main()
{
    // Particle rendering path
    if (uIsParticle == 1) {
        vec4 texColor = texture(uTexture, gl_PointCoord);
        if (texColor.a < 0.1) discard;
        fragColor = texColor; // Discard transparent pixels
    }

    // Normal mesh path
    vec3 N = normalize(vNormal); // Normalized normal
    vec3 V = normalize(uViewPos - vWorldPos); // View direction
    vec3 albedo = texture(uTexture, vUV).rgb; // Base color from texture
    vec3 lit = vec3(0.0); // Accumulated lighting

    // Directional light toggle
    if (uDirEnabled == 1) {
        float diff = max(dot(N, uLightDir), 0.0);
        vec3 ambient = 0.25 * albedo;
        vec3 lightColor = vec3(0.90);
        lit += ambient + diff * albedo * lightColor;
    }

    // Point lights (shininess fixed at 32)
    lit += blinnPhongPoint(N, V, vWorldPos, albedo,
                           uLights[0].position, uLights[0].color, uLights[0].enabled, 32.0);
    lit += blinnPhongPoint(N, V, vWorldPos, albedo,
                           uLights[1].position, uLights[1].color, uLights[1].enabled, 32.0);
    lit += blinnPhongPoint(N, V, vWorldPos, albedo,
                           uLights[2].position, uLights[2].color, uLights[2].enabled, 32.0);

    fragColor = vec4(lit, 1.0); // Final shaded color
}