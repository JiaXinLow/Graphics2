#version 430

// Inputs
in vec3 vNormal; // interpolated normal
in vec3 vWorldPos; // world-space position

// Uniforms
uniform vec3 uLightDir; // directional light direction
uniform vec3 uColor; // per-instance colour
uniform int uDirEnabled; // toggle for directional light
uniform vec3 Kd; // diffuse colour from material
uniform float Ns; // shininess exponent
uniform vec3 uViewPos; // camera position

// Point light struct
struct PointLight {
    vec3 position; // light position
    vec3 color; // light colour
    int enabled; // toggle flag
};
uniform PointLight uLights[3]; // array of 3 point lights

// Output
out vec4 fragColor;

// Blinn–Phong point light with inverse-square attenuation
vec3 blinnPhongPoint(
    vec3 normal,          // surface normal
    vec3 viewDir,         // view direction
    vec3 fragPos,         // fragment position
    vec3 Kd,              // diffuse albedo
    vec3 lightPos,        // point light position
    vec3 lightColor,      // point light colour
    int lightEnabled,     // toggle flag
    float Ns              // shininess exponent
)
{
    if (lightEnabled == 0) return vec3(0.0); // skip if disabled

    vec3 L = lightPos - fragPos; // light vector
    float dist = length(L); // distance
    L = normalize(L); // normalize
    float attenuation = 1.0 / (dist * dist); // inverse-square falloff

    float diffuse = max(dot(normal, L), 0.0); // diffuse term

    vec3 H = normalize(L + viewDir); // half-vector
    float specular = pow(max(dot(normal, H), 0.0), Ns); // specular term

    return (Kd * diffuse + vec3(specular)) * lightColor * attenuation;
}

void main()
{
    vec3 N = normalize(vNormal); // normalized normal
    vec3 V = normalize(uViewPos - vWorldPos); // view direction
    vec3 albedo = Kd; // base colour

    vec3 lit = vec3(0.0); // accumulated lighting

    // Directional light contribution
    if (uDirEnabled == 1) {
        float diff = max(dot(N, uLightDir), 0.0);
        vec3 ambient = 0.25 * albedo;
        vec3 lightColor = vec3(0.90);
        lit += ambient + diff * albedo * lightColor;
    }

    // Point lights contribution
    lit += blinnPhongPoint(N, V, vWorldPos, albedo,
                           uLights[0].position, uLights[0].color, uLights[0].enabled, Ns);
    lit += blinnPhongPoint(N, V, vWorldPos, albedo,
                           uLights[1].position, uLights[1].color, uLights[1].enabled, Ns);
    lit += blinnPhongPoint(N, V, vWorldPos, albedo,
                           uLights[2].position, uLights[2].color, uLights[2].enabled, Ns);

    fragColor = vec4(lit, 1.0); // final shaded colour
}
