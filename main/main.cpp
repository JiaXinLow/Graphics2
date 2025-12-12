#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <print>
#include <numbers>
#include <typeinfo>
#include <stdexcept>
#include <vector> // added
#include <cmath> // added
#include <cstdlib>

#include "../support/error.hpp"
#include "../support/program.hpp"
#include "../support/checkpoint.hpp"
#include "../support/debug_output.hpp"

#include "../vmlib/vec3.hpp" // added
#include "../vmlib/vec4.hpp"
#include "../vmlib/mat44.hpp"

#include "../third_party/rapidobj/include/rapidobj/rapidobj.hpp" // added
#define STB_IMAGE_IMPLEMENTATION // added
#include "../third_party/stb/include/stb_image.h" // added

#define FONTSTASH_IMPLEMENTATION // added
#include "../third_party/fontstash/include/fontstash.h" // added

#define STB_TRUETYPE_IMPLEMENTATION // added
#define STBTT_STATIC // added

#include "defaults.hpp"

namespace
{
    constexpr char const* kWindowTitle = "COMP3811 - CW2";

    void glfw_callback_error_(int, char const*);
    void glfw_callback_key_(GLFWwindow*, int, int, int, int);

    struct GLFWCleanupHelper { ~GLFWCleanupHelper(); };
    struct GLFWWindowDeleter { ~GLFWWindowDeleter(); GLFWwindow* window; };

	// For Task 1.2: Mouse state
    bool mouseLookEnabled = false;
    bool firstMouse = true;
    float lastX = 0.0f, lastY = 0.0f;
    
    // For Task 1.2: Camera state
    struct CameraState {
        float x = 0.0f;
        float y = 50.0f;
        float z = 200.0f;
        float yaw = 0.0f;
        float pitch = -0.3f;
        float baseSpeed = 50.0f;
    };
    CameraState camera;

    // Task 1.2: Moving state
    struct InputState {
        bool forward = false;   // W
        bool backward = false;  // S
        bool left = false;      // A
        bool right = false;     // D
        bool up = false;        // E
        bool down = false;      // Q
        bool shift = false;
        bool ctrl = false;
    };
    InputState input;

	// Task 1.3: Vertex structure
    struct Vertex {
        float px, py, pz;   // position
        float nx, ny, nz;   // normal
        float u, v;         // texture coordinates
    };

    // Task 1.4: Spaceship pads
    Vec3f padPos1;
    Vec3f padPos2;
    Mat44f modelPad1;
    Mat44f modelPad2;

	// Task 1.5: Geometry generation helpers
    inline Vec3f cross(const Vec3f& a, const Vec3f& b) {
        return Vec3f{
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }

    // Task 1.5: Generate a hexagonal prism centered at origin
    void makeHexPrism(std::vector<Vertex>& verts,
        std::vector<unsigned>& idx,
        float radius, float height)
    {
        float halfH = height * 0.5f; // Half of prism height
        std::vector<Vec3f> top, bottom; // Store top and bottom hexagon vertices

        // Compute hexagon vertices for top and bottom faces
        for (int i = 0; i < 6; ++i) {
            float angle = i * (2.0f * std::numbers::pi_v<float> / 6.0f); // 60° step
            float x = radius * std::cos(angle);
            float z = radius * std::sin(angle);
            top.push_back({ x, halfH, z }); // Top vertex
            bottom.push_back({ x, -halfH, z }); // Bottom vertex
        }

        // Top face (triangle fan)
        Vec3f nTop{ 0,1,0 }; // Upward normal
        unsigned baseTop = static_cast<unsigned int>(verts.size());
        for (int i = 0; i < 6; ++i)
            verts.push_back({ top[i].x, top[i].y, top[i].z, nTop.x, nTop.y, nTop.z, 0,0 });
        for (int i = 1; i < 5; ++i) { // Triangulate top hexagon
            idx.push_back(baseTop);
            idx.push_back(baseTop + i + 1);
            idx.push_back(baseTop + i);
        }

        // Bottom face (triangle fan)
        Vec3f nBottom{ 0,-1,0 }; // Downward normal
        unsigned baseBottom = static_cast<unsigned int>(verts.size());
        for (int i = 0; i < 6; ++i)
            verts.push_back({ bottom[i].x, bottom[i].y, bottom[i].z, nBottom.x, nBottom.y, nBottom.z, 0,0 });
        for (int i = 1; i < 5; ++i) { // Triangulate bottom hexagon
            idx.push_back(baseBottom);
            idx.push_back(baseBottom + i);
            idx.push_back(baseBottom + i + 1);
        }

        // Side faces (quads split into two triangles)
        for (int i = 0; i < 6; ++i) {
            int j = (i + 1) % 6; // Next vertex index (wrap around)
            Vec3f a = top[i], b = top[j], c = bottom[j];
            Vec3f normal = normalize(cross(b - a, c - a)); // Compute face normal

            unsigned baseSide = static_cast<unsigned int>(verts.size());
            verts.push_back({ a.x,a.y,a.z, normal.x,normal.y,normal.z, 0,0 }); // Top-left
            verts.push_back({ b.x,b.y,b.z, normal.x,normal.y,normal.z, 0,0 }); // Top-right
            verts.push_back({ c.x,c.y,c.z, normal.x,normal.y,normal.z, 0,0 }); // Bottom-right
            verts.push_back({ bottom[i].x,bottom[i].y,bottom[i].z, normal.x,normal.y,normal.z, 0,0 }); // Bottom-left
            // Two triangles per quad
            idx.insert(idx.end(), { baseSide, baseSide + 1, baseSide + 2,
                                    baseSide, baseSide + 2, baseSide + 3 });
        }
    }

	// Task 1.5: Generate a square pyramid centered at origin
    void makeSquarePyramid(std::vector<Vertex>& verts,
        std::vector<unsigned>& idx,
        float size, float height)
    {
        float h = size * 0.5f; // Half side length of base
        std::vector<Vec3f> base = { // Base square vertices (y=0 plane)
            {-h,0,-h},{h,0,-h},{h,0,h},{-h,0,h}
        };
        Vec3f apex{ 0,height,0 }; // Apex point above origin

        // Base face (single quad split into two triangles)
        Vec3f nBase{ 0,-1,0 }; // Downward normal
        unsigned b = static_cast<unsigned int>(verts.size());
        for (auto& p : base)
            verts.push_back({ p.x,p.y,p.z,nBase.x,nBase.y,nBase.z,0,0 });
        idx.insert(idx.end(), { b,b + 2,b + 1, b,b + 3,b + 2 }); // Two triangles for base

        // Side faces (each base edge and apex forms a triangle)
        for (int i = 0; i < 4; ++i) {
            int j = (i + 1) % 4; // Next vertex index (wrap around)
            Vec3f a = base[i], b2 = base[j];
            Vec3f normal = normalize(cross(b2 - a, apex - a)); // Face normal
             
            unsigned s = static_cast<unsigned int>(verts.size());
            verts.push_back({ a.x,a.y,a.z,normal.x,normal.y,normal.z,0,0 }); // Base corner
            verts.push_back({ b2.x,b2.y,b2.z,normal.x,normal.y,normal.z,0,0 }); // Next corner
            verts.push_back({ apex.x,apex.y,apex.z,normal.x,normal.y,normal.z,0,0 }); // Apex
            idx.insert(idx.end(), { s,s + 1,s + 2 }); // One triangle per sid
        }
    }

	// Task 1.5: Generate a cuboid centered at origin
    void makeCuboid(std::vector<Vertex>& verts,
        std::vector<unsigned>& idx,
        float w, float h, float d)
    {
        // Half dimensions for width, height, depth
        float hw = w * 0.5f, hh = h * 0.5f, hd = d * 0.5f;
        
        // 8 corner vertices of cuboid
        std::vector<Vec3f> v = {
            {-hw,-hh,-hd},{hw,-hh,-hd},{hw,-hh,hd},{-hw,-hh,hd},
            {-hw,hh,-hd},{hw,hh,-hd},{hw,hh,hd},{-hw,hh,hd}
        };
        
        // Each face defined by 4 vertex indices
        int faces[6][4] = { {0,1,2,3},{4,5,6,7},{0,1,5,4},
                            {2,3,7,6},{0,3,7,4},{1,2,6,5} };
        
        // Normals for each face
        Vec3f norms[6] = { {0,-1,0},{0,1,0},{0,0,-1},
                           {0,0,1},{-1,0,0},{1,0,0} };

        // Build faces
        for (int f = 0; f < 6; ++f) {
            unsigned b = static_cast<unsigned int>(verts.size()); // Base index for this face
            Vec3f n = normalize(norms[f]); // Normal for current face
            // Add 4 vertices of the face with same normal
            for (int k = 0; k < 4; ++k) {
                auto& p = v[faces[f][k]];
                verts.push_back({ p.x,p.y,p.z,n.x,n.y,n.z,0,0 });
            }
            // Two triangles per quad face
            idx.insert(idx.end(), { b,b + 1,b + 2, b,b + 2,b + 3 });
        }
    }

	// Task 1.5: Generate a cylinder centered at origin
    void makeCylinder(std::vector<Vertex>& verts,
        std::vector<unsigned>& idx,
        float radius, float height,
        int segments = 16)
    {
        float hh = height * 0.5f; // Half height

        // Top circle
        unsigned topStart = static_cast<unsigned int>(verts.size());  // Starting index for top vertices
        for (int i = 0; i < segments; ++i) {
            float ang = i * (2.0f * std::numbers::pi_v<float> / segments); // Angle step
            float x = radius * std::cos(ang);
            float z = radius * std::sin(ang);
            verts.push_back({ x, hh, z, 0, 1, 0, 0, 0 }); // Top vertex with upward normal
        }
        // Triangulate top face (fan from first vertex)
        for (int i = 1; i < segments - 1; ++i) {
            idx.insert(idx.end(), { topStart, topStart + i + 1, topStart + i });
        }

        // Bottom circle
        unsigned botStart = static_cast<unsigned int>(verts.size());// Starting index for bottom vertices
        for (int i = 0; i < segments; ++i) {
            float ang = i * (2.0f * std::numbers::pi_v<float> / segments);
            float x = radius * std::cos(ang);
            float z = radius * std::sin(ang);
			verts.push_back({ x, -hh, z, 0, -1, 0, 0, 0 }); // Bottom vertex with downward normal
        }
        // Triangulate bottom face (fan from first vertex)
        for (int i = 1; i < segments - 1; ++i) {
            idx.insert(idx.end(), { botStart, botStart + i, botStart + i + 1 });
        }

        // Side faces
        for (int i = 0; i < segments; ++i) {
            int j = (i + 1) % segments; // Next vertex index (wrap around)

            // Top edge vertices
            Vec3f a{ radius * std::cos(i * 2 * std::numbers::pi_v<float> / segments), hh,
                     radius * std::sin(i * 2 * std::numbers::pi_v<float> / segments) };
            Vec3f b{ radius * std::cos(j * 2 * std::numbers::pi_v<float> / segments), hh,
                     radius * std::sin(j * 2 * std::numbers::pi_v<float> / segments) };
            
            // Bottom edge vertices
            Vec3f c{ b.x, -hh, b.z };
            Vec3f d{ a.x, -hh, a.z };

            Vec3f normal = normalize(cross(b - a, c - a)); // Compute side face normal

            unsigned s = static_cast<unsigned int>(verts.size()); // Base index for side quad
            verts.push_back({ a.x,a.y,a.z,normal.x,normal.y,normal.z,0,0 }); // Top-left
            verts.push_back({ b.x,b.y,b.z,normal.x,normal.y,normal.z,0,0 }); // Top-right
            verts.push_back({ c.x,c.y,c.z,normal.x,normal.y,normal.z,0,0 }); // Bottom-right
            verts.push_back({ d.x,d.y,d.z,normal.x,normal.y,normal.z,0,0 }); // Bottom-left

            // Two triangles per quad
            idx.insert(idx.end(), { s, s + 1, s + 2, s, s + 2, s + 3 });
        }
    }

	// Task 1.6: Lighting state
    bool dirEnabled = true;
    struct LightState {
        Vec3f pos[3] = {
            { 18.3f, 2.0f, 38.0f },   // Above pad1
            { 19.0f, 1.5f, 37.0f },   // Side offset
            { 17.5f, 1.8f, 39.0f }    // Another offset
        };
        Vec3f color[3] = {
            { 1.0f, 0.3f, 0.3f },     // Red
            { 0.3f, 1.0f, 0.3f },     // Green
            { 0.3f, 0.3f, 1.0f }      // Blue
        };
        bool enabled[3] = { true, true, true };
    } lights;

    Vec3f lightOffsets[3] = {
    { 0.0f, 2.0f, 0.0f },   // Above ship
    { 0.5f, 1.0f, -0.5f },  // Side offset
    { -0.5f, 1.0f, 0.5f }   // Another offset
    };

    // Task 1.7: Animation state
    bool animRunning = false; // true when F pressed
    bool animPaused = false; // toggled by F
    float animTime = 0.0f; // elapsed time since start
    float currentSpeed = 0.5f;   
    float distanceTravelled = 0.0f;
    Vec3f shipPos;
    Vec3f shipVel = { 0,0,0 };
    Vec3f travelDirection{ 0.0f, 0.0f, 1.0f };
    Mat44f shipModel;

	// Task 1.7: Make rotation matrix to face given forward direction
    inline Mat44f make_rotation_to_face(Vec3f forward) noexcept
    {
        forward = normalize(forward); // Ensure forward vector is unit length

        // Define world +X axis as reference for right vector
        Vec3f worldRight{ 1.0f, 0.0f, 0.0f };
        
        // Compute right vector (perpendicular to forward and worldRight)
        Vec3f right = normalize(cross(forward, worldRight));
        
        // Compute up vector (perpendicular to right and forward)
        Vec3f up = normalize(cross(right, forward));

        // Construct rotation matrix
        return { {
            right.x, right.y, right.z, 0.0f,
            forward.x, forward.y, forward.z, 0.0f, // forward aligned with local +Y
            up.x, up.y, up.z, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f
        } };
    }

    // Task 1.8: Camera modes
    enum CameraMode { FREE, TRACKING, GROUND };
    // CameraMode camMode = FREE;

	// Task 1.8: Make lookAt view matrix
    inline Mat44f make_lookAt(Vec3f eye, Vec3f target, Vec3f up) noexcept {
        Vec3f f = normalize(target - eye);   // Forward direction
        Vec3f r = normalize(cross(f, up));   // Right vector
        Vec3f u = cross(r, f);               // Corrected up vector

        // Build view matrix from basis vectors and eye position
        return { {
            r.x,  r.y,  r.z, -dot(r, eye),
            u.x,  u.y,  u.z, -dot(u, eye),
           -f.x, -f.y, -f.z,  dot(f, eye),
            0.0f, 0.0f, 0.0f, 1.0f
        } };
    }

	// Task 1.8: Make free camera view matrix
    inline Mat44f make_freeCameraView(CameraState const& cam) noexcept {
        // Build yaw and pitch rotations
        Mat44f rotY = make_rotation_y(cam.yaw);
        Mat44f rotX = make_rotation_x(cam.pitch);

        // Translate the world by the negative camera position
        Mat44f trans = make_translation(Vec3f{ -cam.x, -cam.y, -cam.z });

        // Rotate then translate
        return (rotX * rotY) * trans;
    }

	// Task 1.8: Make tracking camera view matrix
    inline Mat44f make_trackingCameraView(Vec3f shipPosition, Vec3f travelDir) noexcept {
        Vec3f dir = (length(travelDir) > 0.001f) ? normalize(travelDir) : Vec3f{ 0.0f, 0.0f, 1.0f };
        Vec3f offsetBehind = dir * 10.0f; // Camera 10 units behind ship
        Vec3f offsetAbove = { 0.0f, 5.0f, 0.0f }; // Camera 5 units above

        Vec3f eye = shipPosition + offsetBehind + offsetAbove; // Camera position
        Vec3f target = shipPosition; // Look at ship
        Vec3f up{ 0.0f, 1.0f, 0.0f }; // World up

        return make_lookAt(eye, target, up); // Build view matrix
    }

	// Task 1.8: Make ground camera view matrix
    inline Mat44f make_groundCameraView(Vec3f shipPosition, Vec3f groundPos) noexcept {
        Vec3f eye = groundPos + Vec3f{ -20.0f, 2.0f, -10.0f }; // Camera offset above/behind ground
        Vec3f target = shipPosition; // Look at ship
        Vec3f up{ 0.0f, 1.0f, 0.0f };  // World up

        // Build view matrix
        return make_lookAt(eye, target, up);
    }

	// Task 1.9: Split-screen state
    bool splitScreenEnabled = false;
    CameraMode camModeLeft = FREE;
    CameraMode camModeRight = TRACKING;

	// Task 1.9: Get view matrix for given camera mode
    Mat44f getView(CameraMode mode) {
        switch (mode) {
        case FREE: {
            Vec3f camPos = { camera.x, camera.y, camera.z }; // Camera position
            Vec3f camTarget = camPos + Vec3f{ // Target based on yaw/pitch
                std::cos(camera.pitch) * std::sin(camera.yaw),
                std::sin(camera.pitch),
                -std::cos(camera.pitch) * std::cos(camera.yaw)
            };
            return make_lookAt(camPos, camTarget, Vec3f{ 0.0f, 1.0f, 0.0f }); // LookAt view
        }
        case TRACKING:
            return make_trackingCameraView(shipPos, travelDirection); // Follow ship
        case GROUND:
            return make_groundCameraView(shipPos, padPos1); // Fixed ground view
        }
        return kIdentity44f; // Default identity matrix fallback
    }

	// Task 1.10: Particle system for exhaust
    struct Particle {
        Vec3f pos; // world-space position
        Vec3f vel; // world-space velocity
        float age; // seconds since spawn
        float lifetime; // seconds until death
        float size; // pixel size (point sprite) or world size (billboard)
    };

	// Task 1.10: Minimal GPU-side particle data
    struct ParticleGPU {
        Vec3f pos;
        float size;
    };

    // Task 1.10: Global particle system state
    constexpr int kMaxParticles = 512;
    std::array<Particle, kMaxParticles> particles; // CPU-side simulation pool
    std::array<ParticleGPU, kMaxParticles> gpuBuffer; // GPU staging buffer (minimal data for rendering)
    int aliveCount = 0; // Number of currently active particles
    float spawnAccumulator = 0.0f; // Fractional spawn counter for frame-rate independence

    // Task 1.10 Exhaust emitter configuration
    Vec3f exhaustOffset = { 0.0f, -0.3f, 0.0f }; // local offset from shipModel
    float spawnRate = 100.0f; // particles per second
    float minLifetime = 0.5f; // seconds
    float maxLifetime = 1.0f; // seconds
    float minSize = 50.0f; // pixels
    float maxSize = 100.0f; // pixels
    float exhaustSpeed = 5.0f; // average velocity (m/s)

    // Task 1.10 Utility: random float helper
    float randf(float a, float b) {
        return a + (b - a) * (float(rand()) / float(RAND_MAX));
    }

    // Task 1.11: Simple 2D UI state
    struct UIState {
        float fbWidth = 1280.0f;
        float fbHeight = 720.0f;

        double mouseX = 0.0;
        double mouseY = 0.0;
        bool mouseLeftDown = false;
        bool mouseLeftClicked = false; // becomes true for one frame on release over a button
    };

    UIState ui;

    struct UIButton {
        float x, y;    // bottom-left in screen space
        float w, h;    // width / height in pixels
        const char* label;

        bool hovered = false;
        bool pressed = false;

        UIButton()
            : x(0.0f), y(0.0f), w(0.0f), h(0.0f), label("") // explicitly initialize all members
        {
        }

        // convenience helper: hit test
        bool hit(float mx, float my) const {
            return (mx >= x && mx <= x + w && my >= y && my <= y + h);
        }
    };

    UIButton launchButton;
    UIButton resetButton;

    void updateButton(UIButton& b) {
        // Convert GLFW mouse coordinates (origin top-left) to our bottom-left system:
        float mx = static_cast<float>(ui.mouseX);
        float my = ui.fbHeight - static_cast<float>(ui.mouseY);

        bool inside = b.hit(mx, my);
        b.hovered = inside;

        if (inside && ui.mouseLeftDown) {
            b.pressed = true;
        }
        else if (!ui.mouseLeftDown) {
            b.pressed = false;
        }
    }

    void pushButtonQuad(std::vector<float>& v, UIButton const& b,
        float r, float g, float bl, float a)
    {
        float x = b.x;
        float y = b.y;
        float w = b.w;
        float h = b.h;

        // 2 triangles: (x,y) (x+w,y) (x+w,y+h) and (x,y) (x+w,y+h) (x,y+h)
        auto addVert = [&](float px, float py) {
            v.push_back(px);
            v.push_back(py);
            v.push_back(r);
            v.push_back(g);
            v.push_back(bl);
            v.push_back(a);
            };

        addVert(x, y);
        addVert(x + w, y);
        addVert(x + w, y + h);
        addVert(x, y);
        addVert(x + w, y + h);
        addVert(x, y + h);
    }

    //void pushButtonOutline(std::vector<float>& vertices,
    //    const UIButton& button,
    //    float red, float green, float blue, float alpha) {
    //    float left = button.x;
    //    float bottom = button.y;
    //    float width = button.w;
    //    float height = button.h;
    //    float thickness = 2.0f; // outline thickness in pixels

    //    auto addVertex = [&](float posX, float posY) {
    //        vertices.push_back(posX);
    //        vertices.push_back(posY);
    //        vertices.push_back(red);
    //        vertices.push_back(green);
    //        vertices.push_back(blue);
    //        vertices.push_back(alpha);
    //        };

    //    // Top edge
    //    addVertex(left, bottom + height);
    //    addVertex(left + width, bottom + height);
    //    addVertex(left + width, bottom + height - thickness);
    //    addVertex(left, bottom + height);
    //    addVertex(left + width, bottom + height - thickness);
    //    addVertex(left, bottom + height - thickness);

    //    // Bottom edge
    //    addVertex(left, bottom);
    //    addVertex(left + width, bottom);
    //    addVertex(left + width, bottom + thickness);
    //    addVertex(left, bottom);
    //    addVertex(left + width, bottom + thickness);
    //    addVertex(left, bottom + thickness);

    //    // Left edge
    //    addVertex(left, bottom);
    //    addVertex(left + thickness, bottom);
    //    addVertex(left + thickness, bottom + height);
    //    addVertex(left, bottom);
    //    addVertex(left + thickness, bottom + height);
    //    addVertex(left, bottom + height);

    //    // Right edge
    //    addVertex(left + width - thickness, bottom);
    //    addVertex(left + width, bottom);
    //    addVertex(left + width, bottom + height);
    //    addVertex(left + width - thickness, bottom);
    //    addVertex(left + width, bottom + height);
    //    addVertex(left + width - thickness, bottom + height);
    //}

    //void drawButtonLabel(const UIButton& button, FONScontext* fontContext) {
    //    fonsSetSize(fontContext, 18.0f);
    //    fonsSetColor(fontContext, 0xFFFFFFFF); // white text

    //    float bounds[4];
    //    fonsTextBounds(fontContext, 0, 0, button.label, nullptr, bounds);
    //    float textWidth = bounds[2] - bounds[0];
    //    float textHeight = bounds[3] - bounds[1];

    //    float textX = button.x + (button.w - textWidth) * 0.5f;
    //    float textY = button.y + (button.h + textHeight) * 0.5f;

    //    fonsDrawText(fontContext, textX, textY, button.label, nullptr);
    //}

    //void drawButton(UIButton& button,
    //    std::vector<float>& vertices,
    //    FONScontext* fontContext) {
    //    float red, green, blue, alpha;
    //    if (button.pressed) {
    //        red = 0.2f; green = 0.6f; blue = 0.9f; alpha = 0.8f;
    //    }
    //    else if (button.hovered) {
    //        red = 0.2f; green = 0.6f; blue = 0.9f; alpha = 0.5f;
    //    }
    //    else {
    //        red = 0.1f; green = 0.4f; blue = 0.7f; alpha = 0.4f;
    //    }

    //    // Filled rectangle
    //    pushButtonQuad(vertices, button, red, green, blue, alpha);

    //    // Solid outline (white)
    //    pushButtonOutline(vertices, button, 1.0f, 1.0f, 1.0f, 1.0f);

    //    // Centered label
    //    drawButtonLabel(button, fontContext);
    //}

    //Task 1.11 Fontstash OpenGL Backend
    static int atlasWidth = 0;
    static int atlasHeight = 0;

    // GL texture used for glyph atlas
    static GLuint fsTex = 0;

    // Shader & VAO/VBO for rendering glyph quads
    static GLuint fsShader = 0;
    static GLuint fsVAO = 0;
    static GLuint fsVBO = 0;
    static size_t fsVBOSize = 0;


    // Uniform locations
    static GLint fsLocTexture = -1;
    static GLint fsLocResolution = -1;

    // Screen resolution for transforming glyph quads
    static float fsScreenW = 1280.0f;
    static float fsScreenH = 720.0f;

    //shader source strings
    static const char* fsVertSrc = R"(#version 430 core
    layout(location = 0) in vec2 aPos;
    layout(location = 1) in vec2 aUV;
    layout(location = 2) in vec4 aColor;

    out vec2 vUV;
    out vec4 vColor;

    uniform vec2 uResolution;

    void main()
    {
        vec2 ndc;
        ndc.x = (aPos.x / uResolution.x) * 2.0 - 1.0;
        ndc.y = 1.0 - (aPos.y / uResolution.y) * 2.0;  

        gl_Position = vec4(ndc, 0.0, 1.0);
        vUV = aUV;
        vColor = aColor;
    }
    )";

    static const char* fsFragSrc = R"(#version 430 core
    in vec2 vUV;
    in vec4 vColor;       // text color from vertex
    uniform sampler2D uTex;
    out vec4 outColor;

    void main() {
        float alpha = texture(uTex, vUV).r;   // sample red directly
        outColor = vec4(vColor.rgb, vColor.a * alpha);
    }
    )";

    int fsRenderCreate(void* userPtr, int width, int height)
    {
        (void)userPtr;
        atlasWidth = width;
        atlasHeight = height;
        // ---- Compile vertex shader ----
        GLuint vs = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vs, 1, &fsVertSrc, nullptr);
        glCompileShader(vs);

        // ---- Compile fragment shader ----
        GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fs, 1, &fsFragSrc, nullptr);
        glCompileShader(fs);

        GLint status;
        glGetShaderiv(vs, GL_COMPILE_STATUS, &status);

        // ---- Link program ----
        fsShader = glCreateProgram();
        glAttachShader(fsShader, vs);
        glAttachShader(fsShader, fs);
        glLinkProgram(fsShader);

        glDeleteShader(vs);
        glDeleteShader(fs);

        // ---- Create atlas texture ----
        glGenTextures(1, &fsTex);
        glBindTexture(GL_TEXTURE_2D, fsTex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, width, height, 0,
            GL_RED, GL_UNSIGNED_BYTE, nullptr);

        // try
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        // IMPORTANT: swizzle red channel into alpha
        //GLint swizzleMask[] = { GL_ONE, GL_ONE, GL_ONE, GL_RED };
        //glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzleMask);

        // ---- Uniforms ----
        fsLocTexture = glGetUniformLocation(fsShader, "uTex");
        fsLocResolution = glGetUniformLocation(fsShader, "uResolution");

        // ---- VAO / VBO ----
        glGenVertexArrays(1, &fsVAO);
        glGenBuffers(1, &fsVBO);

        glBindVertexArray(fsVAO);
        glBindBuffer(GL_ARRAY_BUFFER, fsVBO);
        fsVBOSize = 6 * 8 * sizeof(float);
        glBufferData(GL_ARRAY_BUFFER, fsVBOSize, nullptr, GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(2 * sizeof(float)));
        glEnableVertexAttribArray(1);

        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
        glEnableVertexAttribArray(2);

        glBindVertexArray(0);

        return 1;
    }


    //Update atlas size
    int fsRenderResize(void* up, int width, int height)
    {
        (void)up;
        atlasWidth = width;
        atlasHeight = height;
        glBindTexture(GL_TEXTURE_2D, fsTex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, width, height, 0,
            GL_RED, GL_UNSIGNED_BYTE, nullptr);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

        // IMPORTANT: swizzle red channel into alpha
        //GLint swizzleMask[] = { GL_ONE, GL_ONE, GL_ONE, GL_RED };
        //glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzleMask);
        return 1;
    }

    //update atlas region when new glyphs are baked
    void fsRenderUpdate(void* up, int* rect, const unsigned char* data)
    {
        (void)up;
        int x = rect[0];
        int y = rect[1];
        int w = rect[2];
        int h = rect[3];
        glBindTexture(GL_TEXTURE_2D, fsTex);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexSubImage2D(GL_TEXTURE_2D, 0, x, y, w, h,
            GL_RED, GL_UNSIGNED_BYTE, data);

        std::printf("Uploading glyph rect: %d %d %d %d\n", x, y, w, h);

    }

    //draw glyph quad batch
    void fsRenderDraw(void* up, const float* verts, const float* tcoords,
        const unsigned int* colors, int nverts)
    {
        (void)up;
        static bool once = false;
        if (!once) {
            std::print("Fontstash: fsRenderDraw called\n");
            once = true;
        }

        glUseProgram(fsShader);
        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        glUniform2f(fsLocResolution, float(vp[2]), float(vp[3]));

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, fsTex);
        glUniform1i(fsLocTexture, 0);

        // Build VBO (pos, uv, color) for each vertex
        std::vector<float> buffer;
        buffer.reserve(nverts * 8);

        for (int i = 0; i < nverts; ++i)
        {
            float a = ((colors[i] >> 24) & 0xFF) / 255.0f;
            float b = ((colors[i] >> 16) & 0xFF) / 255.0f;
            float g = ((colors[i] >> 8) & 0xFF) / 255.0f;
            float r = ((colors[i] >> 0) & 0xFF) / 255.0f;

            buffer.push_back(verts[i * 2 + 0]);
            buffer.push_back(verts[i * 2 + 1]);

            float u = tcoords[i * 2 + 0] / float(atlasWidth);
            float v = tcoords[i * 2 + 1] / float(atlasHeight);
            buffer.push_back(u);
            buffer.push_back(v);

            buffer.push_back(r);
            buffer.push_back(g);
            buffer.push_back(b);
            buffer.push_back(a);
        }
        glBindVertexArray(fsVAO);
        glBindBuffer(GL_ARRAY_BUFFER, fsVBO);

        size_t requiredSize = buffer.size() * sizeof(float);
        if (requiredSize > fsVBOSize) {
            // Reallocate buffer with enough space
            fsVBOSize = requiredSize;
            glBufferData(GL_ARRAY_BUFFER, fsVBOSize, buffer.data(), GL_DYNAMIC_DRAW);
        }
        else {
            // Update existing buffer
            glBufferSubData(GL_ARRAY_BUFFER, 0, requiredSize, buffer.data());
        }

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glDrawArrays(GL_TRIANGLES, 0, nverts);

        glDisable(GL_BLEND);
        glBindVertexArray(0);
    }

    //cleanup
    void fsRenderDelete(void* up)
    {
        (void)up;
        glDeleteTextures(1, &fsTex);
        glDeleteBuffers(1, &fsVBO);
        glDeleteVertexArrays(1, &fsVAO);
        glDeleteProgram(fsShader);
    }
}

int main() try
{
    // Initialize GLFW
    if (GLFW_TRUE != glfwInit()) {
        char const* msg = nullptr;
        int ecode = glfwGetError(&msg);
        throw Error("glfwInit() failed with '{}' ({})", msg, ecode);
    }

    // Ensure that we call glfwTerminate() at the end of the program.
    GLFWCleanupHelper cleanupHelper;

    // Configure GLFW and create window
    glfwSetErrorCallback(&glfw_callback_error_);

    glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_TRUE);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);

    //glfwWindowHint( GLFW_RESIZABLE, GLFW_FALSE );

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    glfwWindowHint(GLFW_DEPTH_BITS, 24);

#	if !defined(NDEBUG)
    // When building in debug mode, request an OpenGL debug context. This
    // enables additional debugging features. However, this can carry extra
    // overheads. We therefore do not do this for release builds.
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);
#	endif // ~ !NDEBUG

    GLFWwindow* window = glfwCreateWindow(
        1280, 
        720, 
        kWindowTitle, 
        nullptr, nullptr);
    if (!window) {
        char const* msg = nullptr;
        int ecode = glfwGetError(&msg);
        throw Error("glfwCreateWindow() failed with '{}' ({})", msg, ecode);
    }

    GLFWWindowDeleter windowDeleter{ window };

    // Set up event handling
    // TODO: Additional event handling setup
    // Task 1.2: Cursor position callback, handles mouse look
    glfwSetCursorPosCallback(window,
        [](GLFWwindow* window, double xpos, double ypos) {
            (void)window;
            //Task 1.11 store info for UI
            ui.mouseX = xpos;
            ui.mouseY = ypos;
            
            if (!mouseLookEnabled) return; // Ignore if mouse look disabled
            
            // Initialize on first movement
            if (firstMouse) { lastX = float(xpos); lastY = float(ypos); firstMouse = false; }
            float xoffset = float(xpos) - lastX; // Horizontal delta
            float yoffset = lastY - float(ypos); // Vertical delta (inverted)
            lastX = float(xpos); 
            lastY = float(ypos); // Update last position
            float sensitivity = 0.002f; // Mouse sensitivity
            xoffset *= sensitivity; yoffset *= sensitivity;
            camera.yaw += xoffset; // Adjust yaw
            camera.pitch += yoffset; // Adjust pitch
            
            // Clamp pitch to avoid flipping
            if (camera.pitch > 1.5f) camera.pitch = 1.5f;
            if (camera.pitch < -1.5f) camera.pitch = -1.5f;
        }
    );

	// Task 1.2: Mouse button callback, toggles mouse look on right-click
    glfwSetMouseButtonCallback(window,
        [](GLFWwindow* window, int button, int action, int mods) {
            (void)mods;
            if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
                mouseLookEnabled = !mouseLookEnabled; // Toggle mouse look
                firstMouse = true; // Reset first mouse flag
                if (mouseLookEnabled)
                    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED); // Hide cursor
                else
                    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL); // Show cursor
            }
            if (button == GLFW_MOUSE_BUTTON_LEFT) {
                if (action == GLFW_PRESS) {
                    ui.mouseLeftDown = true;
                    ui.mouseLeftClicked = false;
                }
                else if (action == GLFW_RELEASE) {
                    // click = release after being down
                    ui.mouseLeftClicked = ui.mouseLeftDown;
                    ui.mouseLeftDown = false;
                }
            }
        }
    );
    
    glfwSetKeyCallback(window, &glfw_callback_key_);
    
    // Set up drawing stuff
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // V-Sync is on.

    // Initialize GLAD
    // This will load the OpenGL API. We mustn't make any OpenGL calls before this!
    if (!gladLoadGLLoader((GLADloadproc)&glfwGetProcAddress))
        throw Error("gladLoadGLLoader() failed - cannot load GL API!");

    std::print("RENDERER {}\n", (char const*)glGetString(GL_RENDERER));
    std::print("VENDOR {}\n", (char const*)glGetString(GL_VENDOR));
    std::print("VERSION {}\n", (char const*)glGetString(GL_VERSION));
    std::print("SHADING_LANGUAGE_VERSION {}\n", (char const*)glGetString(GL_SHADING_LANGUAGE_VERSION));

    // Debug output
#	if !defined(NDEBUG)
    setup_gl_debug_output();
#	endif // ~ !NDEBUG

    // TODO: global GL setup goes here
    OGL_CHECKPOINT_ALWAYS();
    glEnable(GL_DEPTH_TEST); // Enable depth testing
    glDepthFunc(GL_LESS); // Pass if fragment is closer
    glDisable(GL_CULL_FACE); // Disable face culling (Task 1.5: show all faces)
    glCullFace(GL_BACK); // Define back faces for culling
    glFrontFace(GL_CCW); // Counter-clockwise winding (front face)
    glEnable(GL_FRAMEBUFFER_SRGB); // Enable sRGB for correct gamma correction
    glClearColor(0.1f, 0.12f, 0.18f, 1.0f); // Set background clear color (dark bluish tone)

    OGL_CHECKPOINT_ALWAYS();

    // Get actual framebuffer size.
    // This can be different from the window size, as standard window
    // decorations (title bar, borders, ...) may be included in the window size
    // but not be part of the drawable surface area.
    int iwidth, iheight;
    glfwGetFramebufferSize(window, &iwidth, &iheight);

    glViewport(0, 0, iwidth, iheight);

    // Other initialization & loading
    OGL_CHECKPOINT_ALWAYS();

    // TODO: global GL setup goes here
	// Task 1.2: Terrain setup
	GLuint terrainVao = 0, terrainVbo = 0, terrainEbo = 0; // Terrain VAO/VBO/EBO
	GLsizei terrainIndexCount = 0; // Number of terrain indices
	float lastFrameTime = 0.0f; // Time of last frame (seconds)
	float fovY = std::numbers::pi_v<float> / 4.0f; // 45 degrees
	float zNear = 1.0f, zFar = 5000.0f; // Near and far clipping planes
	// Create shader program for terrain rendering
    ShaderProgram terrainShader({
        { GL_VERTEX_SHADER,   "assets/cw2/default.vert" },
        { GL_FRAGMENT_SHADER, "assets/cw2/default.frag" }
        });
	GLuint terrainProgramId = terrainShader.programId(); // Get program ID
	// Get uniform locations
    GLint locModel = glGetUniformLocation(terrainProgramId, "uModel");
    GLint locView = glGetUniformLocation(terrainProgramId, "uView");       // mat4
    GLint locViewPos = glGetUniformLocation(terrainProgramId, "uViewPos"); // vec3
    GLint locProj = glGetUniformLocation(terrainProgramId, "uProj");
    GLint locLight = glGetUniformLocation(terrainProgramId, "uLightDir");
    GLint locDirEnabled = glGetUniformLocation(terrainProgramId, "uDirEnabled");
    GLint locTex = glGetUniformLocation(terrainProgramId, "uTexture");
    GLint locIsParticle = glGetUniformLocation(terrainProgramId, "uIsParticle");
    
    // Task 1.2: Original struct vertex definition
	std::vector<Vertex> vertices; // Vertex array
	std::vector<unsigned int> indices; // Index array
    {
        auto result = rapidobj::ParseFile("assets/cw2/parlahti.obj");
        if (result.error.code) throw std::runtime_error("Failed to load OBJ file");
        rapidobj::Triangulate(result);
        auto const& attrib = result.attributes;
        for (auto const& shape : result.shapes)
        {
            for (auto const& idx : shape.mesh.indices)
            {
                Vertex v{};

				// Position
                v.px = attrib.positions[3 * idx.position_index + 0];
                v.py = attrib.positions[3 * idx.position_index + 1];
                v.pz = attrib.positions[3 * idx.position_index + 2];

                // Normal 
                if (idx.normal_index >= 0) {
                    v.nx = attrib.normals[3 * idx.normal_index + 0];
                    v.ny = attrib.normals[3 * idx.normal_index + 1];
                    v.nz = attrib.normals[3 * idx.normal_index + 2];
                }
                else {
                    v.nx = 0; v.ny = 1; v.nz = 0;
                }

                // UV (texture coordinates)
                if (idx.texcoord_index >= 0) {
                    v.u = attrib.texcoords[2 * idx.texcoord_index + 0];
                    v.v = attrib.texcoords[2 * idx.texcoord_index + 1];
                }
                else {
                    v.u = v.v = 0.0f;
                }

				// Add vertex and index
                vertices.push_back(v);
                indices.push_back(static_cast<unsigned int>(indices.size()));
            }
        }
		// Store index count
        terrainIndexCount = static_cast<GLsizei>(indices.size());
    }

	// Task 1.2: Load terrain model from OBJ file
    glGenVertexArrays(1, &terrainVao);
    glGenBuffers(1, &terrainVbo);
    glGenBuffers(1, &terrainEbo);
    glBindVertexArray(terrainVao);
    glBindBuffer(GL_ARRAY_BUFFER, terrainVbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, terrainEbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, px)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, nx)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, u)));
    glEnableVertexAttribArray(2);
    glBindVertexArray(0);

	// Task 1.3: Load terrain texture
	GLuint terrainTexture = 0; // Terrain texture ID
	glGenTextures(1, &terrainTexture); // Generate texture
	glBindTexture(GL_TEXTURE_2D, terrainTexture); // Bind texture

	int texW, texH, texC; // Texture width, height, channels
	stbi_set_flip_vertically_on_load(true); // Flip image vertically on load
	unsigned char* data = stbi_load("assets/cw2/L4343A-4k.jpeg", &texW, &texH, &texC, STBI_rgb); // Load image
	// Check if loading succeeded
    if (!data) { 
        throw std::runtime_error("Failed to load terrain texture!");
    }
	// Upload texture data to GPU
    glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8, texW, texH, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glGenerateMipmap(GL_TEXTURE_2D);
	stbi_image_free(data); // Free image memory

	// Task 1.4: Landing pad setup
	GLuint padVao = 0, padVbo = 0, padEbo = 0; // Landing pad VAO/VBO/EBO
	//GLsizei padIndexCount = 0; // Number of landing pad indices
    // Simple untextured shader for landing pad
    ShaderProgram padShader({
        { GL_VERTEX_SHADER,   "assets/cw2/landingpad.vert" },
        { GL_FRAGMENT_SHADER, "assets/cw2/landingpad.frag" }
        });
    GLuint padProgramId = padShader.programId();
    GLint padLocModel = glGetUniformLocation(padProgramId, "uModel");
    GLint padLocView = glGetUniformLocation(padProgramId, "uView");
    GLint padLocViewPos = glGetUniformLocation(padProgramId, "uViewPos");
    GLint padLocProj = glGetUniformLocation(padProgramId, "uProj");
    GLint padLocLight = glGetUniformLocation(padProgramId, "uLightDir");
    GLint padLocDirEnabled = glGetUniformLocation(padProgramId, "uDirEnabled");

	// Task 1.4: Load landing pad model from OBJ file
    struct PadVertex {
        float px, py, pz;
        float nx, ny, nz;
    };
	std::vector<PadVertex> padVertices; // Pad vertex array
	std::vector<unsigned int> padIndices; // Pad index array
	// Load OBJ file using rapidobj
    rapidobj::Result lpResult = rapidobj::ParseFile("assets/cw2/landingpad.obj");
	// Check for loading errors
    if (lpResult.error.code)
        throw std::runtime_error("Failed to load landingpad.obj");
    rapidobj::Triangulate(lpResult); // Triangulate faces
	auto const& attrib = lpResult.attributes; // Get attributes
    {
		for (auto const& shape : lpResult.shapes) // Iterate shapes
        {
			for (auto const& idx : shape.mesh.indices) // Iterate indices
            {
                PadVertex v{};
                v.px = attrib.positions[3 * idx.position_index + 0];
                v.py = attrib.positions[3 * idx.position_index + 1];
                v.pz = attrib.positions[3 * idx.position_index + 2];

                if (idx.normal_index >= 0) {
                    v.nx = attrib.normals[3 * idx.normal_index + 0];
                    v.ny = attrib.normals[3 * idx.normal_index + 1];
                    v.nz = attrib.normals[3 * idx.normal_index + 2];
                }
                else {
                    v.nx = 0.0f; v.ny = 1.0f; v.nz = 0.0f;
                }
				// Add vertex and index
                padVertices.push_back(v);
                padIndices.push_back(static_cast<unsigned int>(padIndices.size()));
            }
        }
		// Store index count
        // GLsizei padIndexCount = static_cast<GLsizei>(padIndices.size());
    }

	// Task 1.4: Create landing pad VAO/VBO/EBO
    glGenVertexArrays(1, &padVao); 
    glGenBuffers(1, &padVbo);
    glGenBuffers(1, &padEbo);
    glBindVertexArray(padVao);
    glBindBuffer(GL_ARRAY_BUFFER, padVbo);
    glBufferData(GL_ARRAY_BUFFER, padVertices.size() * sizeof(PadVertex), padVertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, padEbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, padIndices.size() * sizeof(unsigned int), padIndices.data(), GL_STATIC_DRAW);
    // layout(location = 0) in vec3 inPosition;
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(PadVertex),
        reinterpret_cast<void*>(offsetof(PadVertex, px)));
    glEnableVertexAttribArray(0);
    // layout(location = 1) in vec3 inNormal;
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(PadVertex),
        reinterpret_cast<void*>(offsetof(PadVertex, nx)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    // Task 1.4: Get water level from terrain mesh
    float waterLevel = 0.0f;
    for (auto const& v : vertices) { waterLevel = std::min(waterLevel, v.py);}

    // Task 1.4: Get lowest vertex of pad mesh
    float padMinY = +std::numeric_limits<float>::max();
    for (auto const& v : padVertices) { padMinY = std::min(padMinY, v.py);}

    // Task 1.4: Set pad positions in world (x, z chosen in sea)
    padPos1 = { 18.3f, 0.0f, 38.0f };
    padPos2 = { -14.3f, 0.0f, -35.4f };

    // Task 1.4: Translate pads so base touches water surface + small offset  
    modelPad1 = make_translation(Vec3f{ padPos1.x, waterLevel - padMinY + 0.05f, padPos1.z });
    modelPad2 = make_translation(Vec3f{ padPos2.x, waterLevel - padMinY + 0.05f, padPos2.z });

    // Task 1.5: Generate spaceship parts
    std::vector<Vertex> prismVerts, noseVerts, pyramidVerts, cuboidVerts, cylVerts;
    std::vector<unsigned> prismIdx, noseIdx, pyramidIdx, cuboidIdx, cylIdx;
    float cubeSize = 0.5f; // edge length of the cube
    makeHexPrism(prismVerts, prismIdx, 0.3f, 0.8f); // fuselage
    makeSquarePyramid(noseVerts, noseIdx, 0.4f, 1.0f); // nose
    makeSquarePyramid(pyramidVerts, pyramidIdx, cubeSize, 0.5f); // side spikes
    makeCuboid(cuboidVerts, cuboidIdx, cubeSize, cubeSize, cubeSize); // cuboid body
    makeCylinder(cylVerts, cylIdx, 0.1f, 0.5f); // landing struts

	// Task 1.5: Create VAO/VBO/EBO for each spaceship part
    GLuint prismVao, prismVbo, prismEbo;
    glGenVertexArrays(1, &prismVao);
    glGenBuffers(1, &prismVbo);
    glGenBuffers(1, &prismEbo);
    glBindVertexArray(prismVao);
    glBindBuffer(GL_ARRAY_BUFFER, prismVbo);
    glBufferData(GL_ARRAY_BUFFER, prismVerts.size() * sizeof(Vertex), prismVerts.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, prismEbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, prismIdx.size() * sizeof(unsigned), prismIdx.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, px)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, nx)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, u)));
    glEnableVertexAttribArray(2);
    glBindVertexArray(0);

	//Task 1.5: Nose cone
    GLuint noseVao, noseVbo, noseEbo;
    glGenVertexArrays(1, &noseVao);
    glGenBuffers(1, &noseVbo);
    glGenBuffers(1, &noseEbo);
    glBindVertexArray(noseVao);
    glBindBuffer(GL_ARRAY_BUFFER, noseVbo);
    glBufferData(GL_ARRAY_BUFFER, noseVerts.size() * sizeof(Vertex), noseVerts.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, noseEbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, noseIdx.size() * sizeof(unsigned), noseIdx.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, px)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, nx)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, u)));
    glEnableVertexAttribArray(2);
    glBindVertexArray(0);

	// Task 1.5: Pyramid spikes
    GLuint pyrVao, pyrVbo, pyrEbo;
    glGenVertexArrays(1, &pyrVao);
    glGenBuffers(1, &pyrVbo);
    glGenBuffers(1, &pyrEbo);
    glBindVertexArray(pyrVao);
    glBindBuffer(GL_ARRAY_BUFFER, pyrVbo);
    glBufferData(GL_ARRAY_BUFFER, pyramidVerts.size() * sizeof(Vertex), pyramidVerts.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pyrEbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, pyramidIdx.size() * sizeof(unsigned), pyramidIdx.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, px)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, nx)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, u)));
    glEnableVertexAttribArray(2);
    glBindVertexArray(0);

	// Task 1.5: Cuboid bodies
    GLuint cubVao, cubVbo, cubEbo;
    glGenVertexArrays(1, &cubVao);
    glGenBuffers(1, &cubVbo);
    glGenBuffers(1, &cubEbo);
    glBindVertexArray(cubVao);
    glBindBuffer(GL_ARRAY_BUFFER, cubVbo);
    glBufferData(GL_ARRAY_BUFFER, cuboidVerts.size() * sizeof(Vertex), cuboidVerts.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubEbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, cuboidIdx.size() * sizeof(unsigned), cuboidIdx.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, px)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, nx)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, u)));
    glEnableVertexAttribArray(2);
    glBindVertexArray(0);

	// Task 1.5: Cylindrical landing struts
    GLuint cylVao, cylVbo, cylEbo;
    glGenVertexArrays(1, &cylVao);
    glGenBuffers(1, &cylVbo);
    glGenBuffers(1, &cylEbo);
    glBindVertexArray(cylVao);
    glBindBuffer(GL_ARRAY_BUFFER, cylVbo);
    glBufferData(GL_ARRAY_BUFFER, cylVerts.size() * sizeof(Vertex), cylVerts.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cylEbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, cylIdx.size() * sizeof(unsigned), cylIdx.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, px)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, nx)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, u)));
    glEnableVertexAttribArray(2);
    glBindVertexArray(0);
    
    // Task 1.6: Material properties
	GLint padLocKd = glGetUniformLocation(padProgramId, "Kd"); // Diffuse color
	GLint padLocNs = glGetUniformLocation(padProgramId, "Ns"); // Specular exponent

	// Task 1.6: Lighting uniform locations for landing pad
    GLint padLightsPos[3], padLightsColor[3], padLightsEnabled[3];
    for (int i = 0; i < 3; ++i) {
        padLightsPos[i] = glGetUniformLocation(padProgramId, ("uLights[" + std::to_string(i) + "].position").c_str());
        padLightsColor[i] = glGetUniformLocation(padProgramId, ("uLights[" + std::to_string(i) + "].color").c_str());
        padLightsEnabled[i] = glGetUniformLocation(padProgramId, ("uLights[" + std::to_string(i) + "].enabled").c_str());
    }

	// Task 1.6: lighting uniform locations
    GLint locLightsPos[3], locLightsColor[3], locLightsEnabled[3];
    for (int i = 0; i < 3; ++i) {
        locLightsPos[i] = glGetUniformLocation(terrainProgramId, ("uLights[" + std::to_string(i) + "].position").c_str());
        locLightsColor[i] = glGetUniformLocation(terrainProgramId, ("uLights[" + std::to_string(i) + "].color").c_str());
        locLightsEnabled[i] = glGetUniformLocation(terrainProgramId, ("uLights[" + std::to_string(i) + "].enabled").c_str());
    }

    // Task 1.7: Initialise ship state here, after pad transforms are valid
    shipPos = padPos1;
    shipModel = modelPad1;

	// Tasl 1.10: Particle system setup
    GLuint particleVAO, particleVBO; // Create VAO/VBO
	glGenVertexArrays(1, &particleVAO); // Generate VAO
	glGenBuffers(1, &particleVBO); // Generate VBO
	glBindVertexArray(particleVAO); // Bind VAO
	glBindBuffer(GL_ARRAY_BUFFER, particleVBO); // Bind VBO
	glEnableVertexAttribArray(0); // Position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(ParticleGPU), (void*)0); // Position
	glEnableVertexAttribArray(3); // Size attribute
	glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(ParticleGPU), (void*)offsetof(ParticleGPU, size)); // Size
	glBindVertexArray(0); // Unbind VAO
	glBindBuffer(GL_ARRAY_BUFFER, particleVBO); // Bind VBO
	glBufferData(GL_ARRAY_BUFFER, kMaxParticles * sizeof(ParticleGPU), nullptr, GL_DYNAMIC_DRAW); // Allocate buffer

    // Task 1.10: Load Exhaust Texture
    GLuint exhaustTexture;
    glGenTextures(1, &exhaustTexture);
    glBindTexture(GL_TEXTURE_2D, exhaustTexture);

    int exhW = 0, exhH = 0, exhC = 0; // Exhaust texture width, height, channels
    stbi_set_flip_vertically_on_load(true); // Flip image vertically on load
    // Request RGBA so we keep the alpha channel
    unsigned char* exhData = stbi_load("assets/cw2/exhaust.png", &exhW, &exhH, &exhC, STBI_rgb_alpha);
    // Check if loading succeeded
    if (!exhData) {
        throw std::runtime_error("Failed to load exhaust texture: " + std::string(stbi_failure_reason()));
    }
    // Use GL_SRGB8_ALPHA8 so color is gamma‑correct and alpha is preserved
    glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8_ALPHA8, exhW, exhH, 0, GL_RGBA, GL_UNSIGNED_BYTE, exhData);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glGenerateMipmap(GL_TEXTURE_2D);
    stbi_image_free(exhData); // Free image memory 
    glBindTexture(GL_TEXTURE_2D, 0);

    // Task 1.10: Particle update function
    auto updateParticles = [&](float deltaTime, Mat44f const& shipModels) {
        // Spawn new particles only if animation is running
        if (animRunning && !animPaused) {
            spawnAccumulator += spawnRate * deltaTime; // Accumulate spawn quota
            int toSpawn = (int)floor(spawnAccumulator); // Number to spawn this frame
            spawnAccumulator -= toSpawn; // Carry over remainder

            for (int i = 0; i < toSpawn; ++i) {
                if (aliveCount >= kMaxParticles) break; // Cap particle count

                Vec4f local(exhaustOffset.x, exhaustOffset.y, exhaustOffset.z, 1.0f); // Local exhaust pos
                Vec4f world = shipModels * local; // Transform to world space

                Particle& p = particles[aliveCount++]; // Allocate new particle
                p.pos = { world.x, world.y, world.z }; // Set position

                //Vec3f baseVel = { 0.0f, -1.0f, 0.0f }; // Downward base velocity
                Vec3f jitter = { randf(-0.2f, 0.2f), randf(-0.1f, 0.1f), randf(-0.2f, 0.2f) }; // Random velocity variation

                p.vel = normalize(jitter) * exhaustSpeed; // Final velocity

                p.size = randf(minSize, maxSize); // Random size
                p.lifetime = randf(minLifetime, maxLifetime); // Random lifetime
                p.age = 0.0f; // Reset age
            }
        }

        // Update existing particles only if not paused
        if (!animPaused) {
            for (int i = 0; i < aliveCount; ++i) {
                Particle& p = particles[i];
                p.pos += p.vel * deltaTime;  // Move particle
                p.age += deltaTime; // Increase age
                if (p.age >= p.lifetime) { // Kill expired particle
                    particles[i] = particles[--aliveCount]; // Replace with last alive
                    --i; // Recheck current index
                }
            }
        }

        // Stream pos and size to GPU (reuse persistent buffer)
        for (int i = 0; i < aliveCount; ++i) {
            gpuBuffer[i].pos = particles[i].pos; // Copy position
            gpuBuffer[i].size = particles[i].size; // Copy size
        }

        glBindBuffer(GL_ARRAY_BUFFER, particleVBO); // Bind particle buffer
        // Update active particle data
        glBufferSubData(GL_ARRAY_BUFFER, 0, aliveCount * sizeof(ParticleGPU), gpuBuffer.data());
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        };

    // ===== Task 1.11: UI shader and buffers =====
    ShaderProgram uiShader({
        { GL_VERTEX_SHADER,   "assets/cw2/Ui.vert" },
        { GL_FRAGMENT_SHADER, "assets/cw2/Ui.frag" }
        });
    GLuint uiProgramId = uiShader.programId();
    GLint uiLocResolution = glGetUniformLocation(uiProgramId, "uResolution");

    GLuint uiVAO = 0, uiVBO = 0;
    glGenVertexArrays(1, &uiVAO);
    glGenBuffers(1, &uiVBO);

    glBindVertexArray(uiVAO);
    glBindBuffer(GL_ARRAY_BUFFER, uiVBO);

    // 6 vertices per button (two triangles), each vertex: vec2 pos + vec4 color = 6 floats
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);

	// Task 1.11: Initialize font rendering
    FONSparams fsParams{};
    fsParams.width = 1024;
    fsParams.height = 1024;
    fsParams.flags = FONS_ZERO_TOPLEFT;
    fsParams.renderCreate = fsRenderCreate;
    fsParams.renderResize = fsRenderResize;
    fsParams.renderUpdate = fsRenderUpdate;
    fsParams.renderDraw = fsRenderDraw;
    fsParams.renderDelete = fsRenderDelete;

    FONScontext* fs = fonsCreateInternal(&fsParams);
    int font = fonsAddFont(fs, "sans", "assets/cw2/DroidSansMonoDotted.ttf");
    if (font == FONS_INVALID) throw std::runtime_error("Font load failed");

	// Task 1.12: Query objects for rendering time
    const int QUERY_BUFFER_SIZE = 5; // Allocate 5 query slots for start/end per view
    int queryIndex = 0; // Current frame slot
    // Terrain
    GLuint terrainStartQueryLeft[QUERY_BUFFER_SIZE];
    GLuint terrainEndQueryLeft[QUERY_BUFFER_SIZE];
    GLuint terrainStartQueryRight[QUERY_BUFFER_SIZE];
    GLuint terrainEndQueryRight[QUERY_BUFFER_SIZE];
    glGenQueries(QUERY_BUFFER_SIZE, terrainStartQueryLeft);
    glGenQueries(QUERY_BUFFER_SIZE, terrainEndQueryLeft);
    glGenQueries(QUERY_BUFFER_SIZE, terrainStartQueryRight);
    glGenQueries(QUERY_BUFFER_SIZE, terrainEndQueryRight);
    // Pads
    GLuint padsStartQueryLeft[QUERY_BUFFER_SIZE];
    GLuint padsEndQueryLeft[QUERY_BUFFER_SIZE];
    GLuint padsStartQueryRight[QUERY_BUFFER_SIZE];
    GLuint padsEndQueryRight[QUERY_BUFFER_SIZE];
    glGenQueries(QUERY_BUFFER_SIZE, padsStartQueryLeft);
    glGenQueries(QUERY_BUFFER_SIZE, padsEndQueryLeft);
    glGenQueries(QUERY_BUFFER_SIZE, padsStartQueryRight);
    glGenQueries(QUERY_BUFFER_SIZE, padsEndQueryRight);
    // Spaceship
    GLuint shipStartQueryLeft[QUERY_BUFFER_SIZE];
    GLuint shipEndQueryLeft[QUERY_BUFFER_SIZE];
    GLuint shipStartQueryRight[QUERY_BUFFER_SIZE];
    GLuint shipEndQueryRight[QUERY_BUFFER_SIZE];
    glGenQueries(QUERY_BUFFER_SIZE, shipStartQueryLeft);
    glGenQueries(QUERY_BUFFER_SIZE, shipEndQueryLeft);
    glGenQueries(QUERY_BUFFER_SIZE, shipStartQueryRight);
    glGenQueries(QUERY_BUFFER_SIZE, shipEndQueryRight);
	// Full frame
    GLuint fullFrameStartQuery[QUERY_BUFFER_SIZE];
    GLuint fullFrameEndQuery[QUERY_BUFFER_SIZE];
    glGenQueries(QUERY_BUFFER_SIZE, fullFrameStartQuery);
    glGenQueries(QUERY_BUFFER_SIZE, fullFrameEndQuery);

	// For task 1.12: Store timing results
    std::vector<double> terrainLeftTimes, terrainRightTimes, terrainSingleTimes;
    std::vector<double> padsLeftTimes, padsRightTimes, padsSingleTimes;
    std::vector<double> shipLeftTimes, shipRightTimes, shipSingleTimes;
    std::vector<double> fullFrameGpuTimes;
    std::vector<double> cpuFrameTimes;

    OGL_CHECKPOINT_ALWAYS();

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        // Let GLFW process events
        glfwPollEvents();

        // Check if window was resized.
        int nwidth, nheight;
        glfwGetFramebufferSize(window, &nwidth, &nheight);
        if (nwidth == 0 || nheight == 0) {
            do {
                glfwWaitEvents();
                glfwGetFramebufferSize(window, &nwidth, &nheight);
            } while (nwidth == 0 || nheight == 0);
        }
        glViewport(0, 0, nwidth, nheight);
        float fbwidth = static_cast<float>(nwidth);
        float fbheight = static_cast<float>(nheight);

        // TODO: update state
        // Compute delta time
        float currentTime = static_cast<float>(glfwGetTime());
        float deltaTime = currentTime - lastFrameTime;
        lastFrameTime = currentTime;

        // Task 1.2: Forward vector from yaw/pitch
        float forwardX = std::cos(camera.pitch) * std::sin(camera.yaw);
        float forwardY = std::sin(camera.pitch);
        float forwardZ = -std::cos(camera.pitch) * std::cos(camera.yaw);

        // Right vector (perpendicular to forward, in XZ plane)
        float rightX = std::cos(camera.yaw);
        float rightZ = std::sin(camera.yaw);

		// Move camera based on input
        float speed = camera.baseSpeed;
        if (input.shift) speed *= 4.0f;
        if (input.ctrl)  speed *= 0.25f;
        if (input.forward) {
            camera.x += forwardX * speed * deltaTime;
            camera.y += forwardY * speed * deltaTime;
            camera.z += forwardZ * speed * deltaTime;
        }
        if (input.backward) {
            camera.x -= forwardX * speed * deltaTime;
            camera.y -= forwardY * speed * deltaTime;
            camera.z -= forwardZ * speed * deltaTime;
        }
        if (input.left) {
            camera.x -= rightX * speed * deltaTime;
            camera.z -= rightZ * speed * deltaTime;
        }
        if (input.right) {
            camera.x += rightX * speed * deltaTime;
            camera.z += rightZ * speed * deltaTime;
        }
        if (input.up)    camera.y += speed * deltaTime;
        if (input.down)  camera.y -= speed * deltaTime;

        // Task 1.11
        fsScreenW = fbwidth;
        fsScreenH = fbheight;
        ui.fbWidth = fbwidth;
        ui.fbHeight = fbheight;

        // Re-layout buttons for new size
        float btnW = 160.0f;
        float btnH = 40.0f;
        float spacing = 20.0f;
        float totalWidth = btnW * 2.0f + spacing;
        float startX = (ui.fbWidth - totalWidth) * 0.5f;
        float y = 20.0f;

        launchButton.x = startX;
        launchButton.y = y;
        launchButton.w = btnW;
        launchButton.h = btnH;

        resetButton.x = startX + btnW + spacing;
        resetButton.y = y;
        resetButton.w = btnW;
        resetButton.h = btnH;

        // TODO: draw frame
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Compute proj/view once
        // float aspect = fbwidth / fbheight;
        // Mat44f proj = make_perspective_projection(fovY, aspect, zNear, zFar);

		// View matrix from camera
        //Mat44f rotY = make_rotation_y(camera.yaw);
        //Mat44f rotX = make_rotation_x(camera.pitch);
        //Mat44f view = (rotX * rotY) * make_translation(Vec3f{ -camera.x, -camera.y, -camera.z });
        /*Mat44f view;*/

        // Camera lookAt matrix
        Vec3f lightDir{ 0.0f, 1.0f, -1.0f };
        float L = std::sqrt(lightDir.x * lightDir.x + lightDir.y * lightDir.y + lightDir.z * lightDir.z);
        if (L > 0) { lightDir.x /= L; lightDir.y /= L; lightDir.z /= L; }

        // Task 1.7: Update animation
        if (animRunning && !animPaused) {
            animTime += deltaTime;

            // Acceleration curve: ease‑in from standstill
            const float acceleration = 0.5f; // acceleration factor
            currentSpeed += acceleration * deltaTime; // v(t) = a * t
            distanceTravelled += currentSpeed * deltaTime; // s(t) = ∫ v dt

            // Curved path driven by distance
            const float s = distanceTravelled;
            shipPos.x = padPos1.x + 15.0f * sin(0.1f * s); // sideways weave
            shipPos.y = padPos1.y + 2.0f * s + 5.0f * sin(0.05f * s); // steady upward climb and gentle vertical wave
            shipPos.z = padPos1.z + 10.0f * s; // continuous forward motion

            // Velocity vector = derivative of path, scaled by ds/dt = speed
            shipVel = { 5.0f * 0.2f * cos(0.2f * animTime), // sideways derivative style
                        animTime, // upward component grows with time
                        currentSpeed // forward component grows with acceleration
            };

            // Face direction of travel
            travelDirection = normalize(shipVel);
            shipModel = make_translation(shipPos)
                * make_rotation_to_face(travelDirection)
                * make_rotation_x(-std::numbers::pi_v<float> / 4.0f)
                * make_rotation_z(-std::numbers::pi_v<float> / 2.0f);
        }

        // Task 1.10: Update particle system
        updateParticles(deltaTime, shipModel);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Task 1.12: Record start time of current frame
        auto frameStart = std::chrono::high_resolution_clock::now();

        // Render scene function
        auto renderScene = [&](Mat44f const& view, Mat44f const& proj, GLuint terrainStart, GLuint terrainEnd, 
                                GLuint padsStart, GLuint padsEnd, GLuint shipStart, GLuint shipEnd) {
            // Task 1.2: Render terrain
            glQueryCounter(terrainStart, GL_TIMESTAMP); // Task 1.12: Start terrain timing query
            glUseProgram(terrainProgramId);
            // Set all uniforms before drawing
            glUniformMatrix4fv(locView, 1, GL_TRUE, view.v);
            glUniformMatrix4fv(locProj, 1, GL_TRUE, proj.v);
            glUniform3f(locLight, lightDir.x, lightDir.y, lightDir.z);

            // Task 1.6 Point lights
            for (int i = 0; i < 3; ++i) {
                glUniform3f(locLightsPos[i], lights.pos[i].x, lights.pos[i].y, lights.pos[i].z);
                glUniform3f(locLightsColor[i], lights.color[i].x, lights.color[i].y, lights.color[i].z);
                glUniform1i(locLightsEnabled[i], lights.enabled[i] ? 1 : 0);
            }

            // Task 1.6 : Set light toggles
            glUniform3f(locViewPos, camera.x, camera.y, camera.z); // Camera position
            glUniform1i(locDirEnabled, dirEnabled ? 1 : 0); // Directional toggle

            glUniformMatrix4fv(locModel, 1, GL_TRUE, kIdentity44f.v);
            glUniform1i(locIsParticle, 0);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, terrainTexture);
            glUniform1i(locTex, 0); 
            glBindVertexArray(terrainVao);
            glDrawElements(GL_TRIANGLES, terrainIndexCount, GL_UNSIGNED_INT, nullptr);
            glBindVertexArray(0);
            glQueryCounter(terrainEnd, GL_TIMESTAMP); // Task 1.12: End terrain timing query

            // Task 1.7: Update ship position if animation is running
            for (int i = 0; i < 3; ++i) {
                Vec4f local(lightOffsets[i].x, lightOffsets[i].y, lightOffsets[i].z, 1.0f);
                Vec4f world = shipModel * local;   // transform offset into world space
                lights.pos[i] = { world.x, world.y, world.z };
            }

            // 1.4: Landing pads
            glQueryCounter(padsStart, GL_TIMESTAMP); // Task 1.12: Start pad timing query
            glUseProgram(padProgramId);
            glUniformMatrix4fv(padLocView, 1, GL_TRUE, view.v);
            glUniformMatrix4fv(padLocProj, 1, GL_TRUE, proj.v);
            glUniform3f(padLocLight, lightDir.x, lightDir.y, lightDir.z);
            glBindVertexArray(padVao);

            // Task 1.6 : Set light toggles for pad program
            glUniform3f(padLocViewPos, camera.x, camera.y, camera.z);// Camera position
            // GLint padLocDirEnabled = glGetUniformLocation(padProgramId, "uDirEnabled");
            glUniform1i(padLocDirEnabled, dirEnabled ? 1 : 0); // Directional toggle
            // Point lights
            for (int i = 0; i < 3; ++i) {
                glUniform3f(padLightsPos[i], lights.pos[i].x, lights.pos[i].y, lights.pos[i].z);
                glUniform3f(padLightsColor[i], lights.color[i].x, lights.color[i].y, lights.color[i].z);
                glUniform1i(padLightsEnabled[i], lights.enabled[i] ? 1 : 0);
            }

			// Task 1.4: Render pad 1 
            glUniformMatrix4fv(padLocModel, 1, GL_TRUE, modelPad1.v);
            {
                size_t indexOffset = 0;
                for (auto const& shape : lpResult.shapes) {
                    auto const& mesh = shape.mesh;
                    for (size_t face = 0; face < mesh.num_face_vertices.size(); ++face) {
                        int fv = mesh.num_face_vertices[face];
                        int matId = mesh.material_ids[face];
                        if (matId >= 0) {
                            auto const& mat = lpResult.materials[matId];
                            glUniform3f(padLocKd, mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]);
                            glUniform1f(padLocNs, mat.shininess);
                        }
                        glDrawElements(GL_TRIANGLES, fv, GL_UNSIGNED_INT,
                            reinterpret_cast<void*>(indexOffset * sizeof(unsigned int)));
                        indexOffset += fv;
                    }
                }
            }

            // Task 1.4: Render pad 2
            glUniformMatrix4fv(padLocModel, 1, GL_TRUE, modelPad2.v);
            {
                size_t indexOffset = 0;
                for (auto const& shape : lpResult.shapes) {
                    auto const& mesh = shape.mesh;
                    for (size_t face = 0; face < mesh.num_face_vertices.size(); ++face) {
                        int fv = mesh.num_face_vertices[face];
                        int matId = mesh.material_ids[face];
                        if (matId >= 0) {
                            auto const& mat = lpResult.materials[matId];
                            glUniform3f(padLocKd, mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]);
                            glUniform1f(padLocNs, mat.shininess);
                        }
                        glDrawElements(GL_TRIANGLES, fv, GL_UNSIGNED_INT,
                            reinterpret_cast<void*>(indexOffset * sizeof(unsigned int)));
                        indexOffset += fv;
                    }
                }
            }
            glBindVertexArray(0);
            glQueryCounter(padsEnd, GL_TIMESTAMP); // Task 1.12: End pad timing query

            // Task 1.5: Spaceship rendering, reuse padProgramId (untextured and color)
            glQueryCounter(shipStart, GL_TIMESTAMP); // Task 1.12: Start spaceship timing query
            glUseProgram(padProgramId);
            glUniformMatrix4fv(padLocView, 1, GL_TRUE, view.v);
            glUniformMatrix4fv(padLocProj, 1, GL_TRUE, proj.v);
            glUniform3f(padLocLight, lightDir.x, lightDir.y, lightDir.z);
            Mat44f modelCylinder = shipModel * make_translation(Vec3f{ 0.0f, 0.5f, 0.0f }); // Base on pad1, lift 0.5 units

            // Task 1.5: Start from bottom: Four cylinders (landing struts)
            for (int i = 0; i < 4; ++i) {
                float ang = i * (std::numbers::pi_v<float> / 2.0f);
                Vec3f offset{ 0.2f * std::cos(ang), -0.2f, 0.2f * std::sin(ang) };
                Mat44f modelCyl = modelCylinder * make_translation(offset);
                glUniformMatrix4fv(padLocModel, 1, GL_TRUE, modelCyl.v);
                //glUniform3f(padLocColor, 0.45f, 0.50f, 0.55f);
                glUniform3f(padLocKd, 0.45f, 0.50f, 0.55f); // diffuse colour
                glUniform1f(padLocNs, 32.0f);               // shininess
                glBindVertexArray(cylVao);
                glDrawElements(GL_TRIANGLES, (GLsizei)cylIdx.size(), GL_UNSIGNED_INT, nullptr);
            }

            float cylHeight = 0.5f;   // must match the height used in makeCylinder
            float cylYOffset = -0.2f; // the downward offset you applied

            // Task 1.5: Cube sits directly above cylinders, rotate cube around Y axis by 45 degrees
            Mat44f modelCub = modelCylinder
                * make_translation(Vec3f{ 0.0f, cylYOffset + cylHeight, 0.0f })
                * make_rotation_y(std::numbers::pi_v<float> / 4.0f);
            glUniformMatrix4fv(padLocModel, 1, GL_TRUE, modelCub.v);
            //glUniform3f(padLocColor, 0.20f, 0.45f, 0.50f);
            glUniform3f(padLocKd, 0.45f, 0.50f, 0.55f); // diffuse colour
            glUniform1f(padLocNs, 32.0f);               // shininess
            glBindVertexArray(cubVao);
            glDrawElements(GL_TRIANGLES, (GLsizei)cuboidIdx.size(), GL_UNSIGNED_INT, nullptr);

            // Task 1.5: Four pyramids on cube faces
            for (int i = 0; i < 4; ++i) {
                Vec3f offset;
                Mat44f orient;
                switch (i) {
                case 0: // +X face
                    offset = { cubeSize * 0.5f, 0.0f, 0.0f };
                    orient = make_rotation_z(-std::numbers::pi_v<float> / 2.0f); // tilt apex along +X
                    break;
                case 1: // -X face
                    offset = { -cubeSize * 0.5f, 0.0f, 0.0f };
                    orient = make_rotation_z(std::numbers::pi_v<float> / 2.0f);  // tilt apex along -X
                    break;
                case 2: // +Z face
                    offset = { 0.0f, 0.0f, cubeSize * 0.5f };
                    orient = make_rotation_x(std::numbers::pi_v<float> / 2.0f);  // tilt apex along +Z
                    break;
                case 3: // -Z face
                    offset = { 0.0f, 0.0f, -cubeSize * 0.5f };
                    orient = make_rotation_x(-std::numbers::pi_v<float> / 2.0f); // tilt apex along -Z
                    break;
                }

                Mat44f modelPyr = modelCub * make_translation(offset) * orient;

                glUniformMatrix4fv(padLocModel, 1, GL_TRUE, modelPyr.v);
                glUniform3f(padLocKd, 0.75f, 0.60f, 0.30f); // diffuse colour
                glUniform1f(padLocNs, 32.0f); // shininess
                glBindVertexArray(pyrVao);
                glDrawElements(GL_TRIANGLES, (GLsizei)pyramidIdx.size(), GL_UNSIGNED_INT, nullptr);
            }

            // Task 1.5: Prism sits directly above cube
            //float cubeSize = 0.5f;   // cube edge length
            float prismHeight = 1.0f;   // hex prism height
            Mat44f modelPrism = modelCub
                * make_translation(Vec3f{ 0.0f, cubeSize * 0.4f + prismHeight * 0.5f, 0.0f })
                * make_scaling(1.0f, 1.2f, 1.0f); // elongate fuselage
            glUniformMatrix4fv(padLocModel, 1, GL_TRUE, modelPrism.v);
            glUniform3f(padLocKd, 0.80f, 0.45f, 0.20f); // diffuse colour
            glUniform1f(padLocNs, 32.0f); // shininess
            glBindVertexArray(prismVao);
            glDrawElements(GL_TRIANGLES, (GLsizei)prismIdx.size(), GL_UNSIGNED_INT, nullptr);

            // Task 1.5: Nose pyramid
            Mat44f modelNose = modelPrism * make_translation(Vec3f{ 0.0f, prismHeight * 0.4f, 0.0f });
            glUniformMatrix4fv(padLocModel, 1, GL_TRUE, modelNose.v);
            glUniform3f(padLocKd, 0.80f, 0.45f, 0.20f); // diffuse colour
            glUniform1f(padLocNs, 32.0f); // shininess
            glBindVertexArray(noseVao);
            glDrawElements(GL_TRIANGLES, (GLsizei)noseIdx.size(), GL_UNSIGNED_INT, nullptr);

            glQueryCounter(shipEnd, GL_TIMESTAMP); // Task 1.12: End spaceship timing query

            // Task 1.10: Particle rendering
            glUseProgram(terrainProgramId);
            glUniform1i((locIsParticle), 1);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, exhaustTexture);
            glUniform1i(locTex, 0);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE);// addictive blending
            glDepthMask(GL_FALSE);
            glBindVertexArray(particleVAO);
            glEnable(GL_PROGRAM_POINT_SIZE);
            glDrawArrays(GL_POINTS, 0, aliveCount);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
         };
        
            // Task 1.9: Split-screen rendering
            if (splitScreenEnabled) {
				glQueryCounter(fullFrameStartQuery[queryIndex], GL_TIMESTAMP); // Task 1.12: Start full frame timing query (GPU)
                
                // Left half
                glViewport(0, 0, nwidth / 2, nheight);
                float aspectLeft = (nwidth / 2.0f) / float(nheight);
                Mat44f projLeft = make_perspective_projection(fovY, aspectLeft, zNear, zFar);
                renderScene(getView(camModeLeft), projLeft,
                    terrainStartQueryLeft[queryIndex], terrainEndQueryLeft[queryIndex],
                    padsStartQueryLeft[queryIndex], padsEndQueryLeft[queryIndex],
                    shipStartQueryLeft[queryIndex], shipEndQueryLeft[queryIndex]);

                // Right half
                glViewport(nwidth / 2, 0, nwidth / 2, nheight);
                float aspectRight = (nwidth / 2.0f) / float(nheight);
                Mat44f projRight = make_perspective_projection(fovY, aspectRight, zNear, zFar);
                renderScene(getView(camModeRight), projRight,
                    terrainStartQueryRight[queryIndex], terrainEndQueryRight[queryIndex],
                    padsStartQueryRight[queryIndex], padsEndQueryRight[queryIndex],
                    shipStartQueryRight[queryIndex], shipEndQueryRight[queryIndex]);

				glQueryCounter(fullFrameEndQuery[queryIndex], GL_TIMESTAMP); // Task 1.12: End full frame timing query (GPU)
            }
            else {
                glViewport(0, 0, nwidth, nheight);
                float aspect = float(nwidth) / float(nheight);
                Mat44f proj = make_perspective_projection(fovY, aspect, zNear, zFar);
				glQueryCounter(fullFrameStartQuery[queryIndex], GL_TIMESTAMP); // Task 1.12: Start full frame timing query (GPU)
                renderScene(getView(camModeLeft), proj,
                    terrainStartQueryLeft[queryIndex], terrainEndQueryLeft[queryIndex],
                    padsStartQueryLeft[queryIndex], padsEndQueryLeft[queryIndex],
                    shipStartQueryLeft[queryIndex], shipEndQueryLeft[queryIndex]);
				glQueryCounter(fullFrameEndQuery[queryIndex], GL_TIMESTAMP); // Task 1.12: End full frame timing query (GPU)
            }

            // RESET VIEWPORT FOR UI & TEXT
            glViewport(0, 0, nwidth, nheight);

            GLint vp[4];
            glGetIntegerv(GL_VIEWPORT, vp);
            //float vpW = float(vp[2]);
            //float vpH = float(vp[3]);

            // ===== Task 1.11: UI update & render =====
            // 1) Update button interaction state
            updateButton(launchButton);
            updateButton(resetButton);

            // 2) Decide button colors based on state
            auto colorFor = [](UIButton const& b, float& r, float& g, float& bl, float& a) {
                if (b.pressed) {
                    r = 0.2f; g = 0.6f; bl = 0.9f; a = 0.8f; // pressed
                }
                else if (b.hovered) {
                    r = 0.2f; g = 0.6f; bl = 0.9f; a = 0.5f; // hover
                }
                else {
                    r = 0.1f; g = 0.4f; bl = 0.7f; a = 0.4f; // normal
                }
                };

            std::vector<float> uiVerts;
            uiVerts.reserve(6 * 6 * 2); // 6 verts * 6 floats * 2 buttons

            float r, g, b, a;
            colorFor(launchButton, r, g, b, a);
            pushButtonQuad(uiVerts, launchButton, r, g, b, a);

            colorFor(resetButton, r, g, b, a);
            pushButtonQuad(uiVerts, resetButton, r, g, b, a);

            // 3) Draw rectangles
            glDisable(GL_DEPTH_TEST);
            glDisable(GL_CULL_FACE);

            glUseProgram(uiProgramId);
            glUniform2f(uiLocResolution, ui.fbWidth, ui.fbHeight);

            glBindVertexArray(uiVAO);
            glBindBuffer(GL_ARRAY_BUFFER, uiVBO);
            glBufferData(GL_ARRAY_BUFFER,
                uiVerts.size() * sizeof(float),
                uiVerts.data(),
                GL_DYNAMIC_DRAW);

            glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(uiVerts.size() / 6));

            glBindVertexArray(0);

            // 4) Handle clicks (fire actions once per release)
            // Convert mouse coords for hit-test
            float mx = static_cast<float>(ui.mouseX);
            float my = ui.fbHeight - static_cast<float>(ui.mouseY);

            if (ui.mouseLeftClicked) {
                if (launchButton.hit(mx, my)) {
                    // same as pressing F
                    if (!animRunning) { animRunning = true; animPaused = false; animTime = 0.0f; }
                    else { animPaused = !animPaused; }
                }
                else if (resetButton.hit(mx, my)) {
                    // same as pressing R
                    animRunning = false;
                    animPaused = false;
                    animTime = 0.0f;
                    shipPos = padPos1;
                    shipVel = { 0,0,0 };
                    shipModel = modelPad1;
                    travelDirection = Vec3f{ 0.0f, 0.0f, 1.0f };
                }
                ui.mouseLeftClicked = false; // consume
            }

            // ===== Task 1.11: Altitude text (top-left) =====
       // IMPORTANT: reset viewport for UI/text

            // Prepare GL state for text
            glDisable(GL_DEPTH_TEST);
            glDepthMask(GL_FALSE);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE);

            // Draw test text
            char altitudeBuffer[64];
            snprintf(altitudeBuffer, sizeof(altitudeBuffer), "Altitude: %.1f m", shipPos.y);

            fonsSetFont(fs, font);
            fonsSetSize(fs, 50.0f);
            fonsSetColor(fs, 0xFF00FFFF); // bright magenta
            fonsSetAlign(fs, FONS_ALIGN_LEFT | FONS_ALIGN_TOP);

            //fonsDrawText(fs, 20.0f, 20.0f, "HELLO", nullptr);
            fonsDrawText(fs, 10.0f, 10.0f, altitudeBuffer, nullptr);

            // Restore GL state
            glDisable(GL_BLEND);
            glDepthMask(GL_TRUE);
            glEnable(GL_DEPTH_TEST);

			// Task 1.12: Retrieve and print terrain rendering time
            int retrieveIndex = (queryIndex + QUERY_BUFFER_SIZE - 2) % QUERY_BUFFER_SIZE;
            if (splitScreenEnabled) {
                // Left view
                // Terrain
                GLint availableLeft = 0;
                glGetQueryObjectiv(terrainEndQueryLeft[retrieveIndex], GL_QUERY_RESULT_AVAILABLE, &availableLeft);
                if (availableLeft) {
                    GLuint64 startLeft = 0, endLeft = 0;
                    glGetQueryObjectui64v(terrainStartQueryLeft[retrieveIndex], GL_QUERY_RESULT, &startLeft);
                    glGetQueryObjectui64v(terrainEndQueryLeft[retrieveIndex], GL_QUERY_RESULT, &endLeft);
                    double terrainLeftMs = (endLeft - startLeft) / 1e6;
                    terrainLeftTimes.push_back(terrainLeftMs);
                    //printf("[Left view] Terrain GPU time: %.3f ms\n", terrainLeftMs);
                }
                else {
                    //printf("[Left view] Terrain GPU result not yet available (index %d)\n", retrieveIndex);
                }

                // Pads
                GLint availablePadsLeft = 0;
                glGetQueryObjectiv(padsEndQueryLeft[retrieveIndex], GL_QUERY_RESULT_AVAILABLE, &availablePadsLeft);
                if (availablePadsLeft) {
                    GLuint64 startPadsLeft = 0, endPadsLeft = 0;
                    glGetQueryObjectui64v(padsStartQueryLeft[retrieveIndex], GL_QUERY_RESULT, &startPadsLeft);
                    glGetQueryObjectui64v(padsEndQueryLeft[retrieveIndex], GL_QUERY_RESULT, &endPadsLeft);
                    double padsLeftMs = (endPadsLeft - startPadsLeft) / 1e6;
                    padsLeftTimes.push_back(padsLeftMs);
                    // printf("[Left view] Pads GPU time: %.3f ms\n", padsLeftMs);
                } else {
                    //printf("[Left view] Pads GPU result not yet available (index %d)\n", retrieveIndex);
                }

                // Spaceship
                GLint availableShipLeft = 0;
                glGetQueryObjectiv(shipEndQueryLeft[retrieveIndex], GL_QUERY_RESULT_AVAILABLE, &availableShipLeft);
                if (availableShipLeft) {
                    GLuint64 startShipLeft = 0, endShipLeft = 0;
                    glGetQueryObjectui64v(shipStartQueryLeft[retrieveIndex], GL_QUERY_RESULT, &startShipLeft);
                    glGetQueryObjectui64v(shipEndQueryLeft[retrieveIndex], GL_QUERY_RESULT, &endShipLeft);
                    double shipLeftMs = (endShipLeft - startShipLeft) / 1e6;
					shipLeftTimes.push_back(shipLeftMs);
                    // printf("[Left view] Spaceship GPU time: %.3f ms\n", shipLeftMs);
                } else {
                    //printf("[Left view] Spaceship GPU result not yet available (index %d)\n", retrieveIndex);
                }

                // Right view
                // Terrain
                GLint availableRight = 0;
                glGetQueryObjectiv(terrainEndQueryRight[retrieveIndex], GL_QUERY_RESULT_AVAILABLE, &availableRight);
                if (availableRight) {
                    GLuint64 startRight = 0, endRight = 0;
                    glGetQueryObjectui64v(terrainStartQueryRight[retrieveIndex], GL_QUERY_RESULT, &startRight);
                    glGetQueryObjectui64v(terrainEndQueryRight[retrieveIndex], GL_QUERY_RESULT, &endRight);
                    double terrainRightMs = (endRight - startRight) / 1e6;
					terrainRightTimes.push_back(terrainRightMs);
                    // printf("[Right view] Terrain GPU time: %.3f ms\n", terrainRightMs);
                } else {
                    //printf("[Right view] Terrain GPU result not yet available (index %d)\n", retrieveIndex);
                }

                // Pads
                GLint availablePadsRight = 0;
                glGetQueryObjectiv(padsEndQueryRight[retrieveIndex], GL_QUERY_RESULT_AVAILABLE, &availablePadsRight);
                if (availablePadsRight) {
                    GLuint64 startPadsRight = 0, endPadsRight = 0;
                    glGetQueryObjectui64v(padsStartQueryRight[retrieveIndex], GL_QUERY_RESULT, &startPadsRight);
                    glGetQueryObjectui64v(padsEndQueryRight[retrieveIndex], GL_QUERY_RESULT, &endPadsRight);
                    double padsRightMs = (endPadsRight - startPadsRight) / 1e6;
					padsRightTimes.push_back(padsRightMs);
                    // printf("[Right view] Pads GPU time: %.3f ms\n", padsRightMs);
                } else {
                    //printf("[Right view] Pads GPU result not yet available (index %d)\n", retrieveIndex);
                }

                // Spaceship
                GLint availableShipRight = 0;
                glGetQueryObjectiv(shipEndQueryRight[retrieveIndex], GL_QUERY_RESULT_AVAILABLE, &availableShipRight);
                if (availableShipRight) {
                    GLuint64 startShipRight = 0, endShipRight = 0;
                    glGetQueryObjectui64v(shipStartQueryRight[retrieveIndex], GL_QUERY_RESULT, &startShipRight);
                    glGetQueryObjectui64v(shipEndQueryRight[retrieveIndex], GL_QUERY_RESULT, &endShipRight);
                    double shipRightMs = (endShipRight - startShipRight) / 1e6;
					shipRightTimes.push_back(shipRightMs);
                    // printf("[Right view] Spaceship GPU time: %.3f ms\n", shipRightMs);
                } else {
                    //printf("[Right view] Spaceship GPU result not yet available (index %d)\n", retrieveIndex);
                }
            }
            else {
                // Single view
                // Terrain
                GLint available = 0;
                glGetQueryObjectiv(terrainEndQueryLeft[retrieveIndex], GL_QUERY_RESULT_AVAILABLE, &available);
                if (available) {
                    GLuint64 startTime = 0, endTime = 0;
                    glGetQueryObjectui64v(terrainStartQueryLeft[retrieveIndex], GL_QUERY_RESULT, &startTime);
                    glGetQueryObjectui64v(terrainEndQueryLeft[retrieveIndex], GL_QUERY_RESULT, &endTime);
                    double terrainTimeMs = (endTime - startTime) / 1e6;
                    terrainSingleTimes.push_back(terrainTimeMs);
                    // printf("[Single view] Terrain GPU time: %.3f ms\n", terrainTimeMs);
                } else {
                    //printf("[Single view] Terrain GPU result not yet available (index %d)\n", retrieveIndex);
                }

                // Pads
                GLint availablePads = 0;
                glGetQueryObjectiv(padsEndQueryLeft[retrieveIndex], GL_QUERY_RESULT_AVAILABLE, &availablePads);
                if (availablePads) {
                    GLuint64 startPads = 0, endPads = 0;
                    glGetQueryObjectui64v(padsStartQueryLeft[retrieveIndex], GL_QUERY_RESULT, &startPads);
                    glGetQueryObjectui64v(padsEndQueryLeft[retrieveIndex], GL_QUERY_RESULT, &endPads);
                    double padsTimeMs = (endPads - startPads) / 1e6;
					padsSingleTimes.push_back(padsTimeMs);
                    // printf("[Single view] Pads GPU time: %.3f ms\n", padsTimeMs);
                } else {
                    //printf("[Single view] Pads GPU result not yet available (index %d)\n", retrieveIndex);
                }

                // Spaceship
                GLint availableShip = 0;
                glGetQueryObjectiv(shipEndQueryLeft[retrieveIndex], GL_QUERY_RESULT_AVAILABLE, &availableShip);
                if (availableShip) {
                    GLuint64 startShip = 0, endShip = 0;
                    glGetQueryObjectui64v(shipStartQueryLeft[retrieveIndex], GL_QUERY_RESULT, &startShip);
                    glGetQueryObjectui64v(shipEndQueryLeft[retrieveIndex], GL_QUERY_RESULT, &endShip);
                    double shipTimeMs = (endShip - startShip) / 1e6;
					shipSingleTimes.push_back(shipTimeMs);
                    // printf("[Single view] Spaceship GPU time: %.3f ms\n", shipTimeMs);
                } else {
                    //printf("[Single view] Spaceship GPU result not yet available (index %d)\n", retrieveIndex);
                }
            }
            queryIndex = (queryIndex + 1) % QUERY_BUFFER_SIZE; // Advance to next slot

            // Task 1.12: Retrieve CPU frame time
            GLint availableFull = 0;
            glGetQueryObjectiv(fullFrameEndQuery[retrieveIndex], GL_QUERY_RESULT_AVAILABLE, &availableFull);
            if (availableFull) {
                GLuint64 startFull = 0, endFull = 0;
                glGetQueryObjectui64v(fullFrameStartQuery[retrieveIndex], GL_QUERY_RESULT, &startFull);
                glGetQueryObjectui64v(fullFrameEndQuery[retrieveIndex], GL_QUERY_RESULT, &endFull);
                double fullFrameGpuMs = (endFull - startFull) / 1e6;
                fullFrameGpuTimes.push_back(fullFrameGpuMs);
            }

            // CPU timing
            auto frameEnd = std::chrono::high_resolution_clock::now();
            double cpuFrameTimeMs =
                std::chrono::duration<double, std::milli>(frameEnd - frameStart).count();
            cpuFrameTimes.push_back(cpuFrameTimeMs);

        // Display results
        glfwSwapBuffers(window);
    }
    // Task 1.12: Compute and display statistics
    auto computeStats = [](const std::vector<double>& data, const char* label) {
        if (data.size() <= 50) {
            printf("%s: Not enough data collected.\n", label);
            return;
        }

        // Discard first 50 warm-up frames
        size_t startIndex = 50;
        size_t endIndex = std::min(data.size(), size_t(550)); // 50 warm-up and 500 steady frames

        double sum = 0, minVal = DBL_MAX, maxVal = DBL_MIN;
        for (size_t i = startIndex; i < endIndex; ++i) {
            double v = data[i];
            sum += v;
            if (v < minVal) minVal = v;
            if (v > maxVal) maxVal = v;
        }

        size_t count = endIndex - startIndex;
        double avg = sum / count;

        // Standard deviation
        double var = 0;
        for (size_t i = startIndex; i < endIndex; ++i) {
            double v = data[i];
            var += (v - avg) * (v - avg);
        }
        double stddev = std::sqrt(var / count);

        printf("%s -> Avg: %.3f ms, Min: %.3f ms, Max: %.3f ms, StdDev: %.3f ms (n=%zu)\n",
            label, avg, minVal, maxVal, stddev, count);
        };

    computeStats(terrainLeftTimes, "Terrain Left View");
    computeStats(terrainRightTimes, "Terrain Right View");
    computeStats(terrainSingleTimes, "Terrain Single View");
    computeStats(padsLeftTimes, "Pads Left View");
    computeStats(padsRightTimes, "Pads Right View");
    computeStats(padsSingleTimes, "Pads Single View");
    computeStats(shipLeftTimes, "Spaceship Left View");
    computeStats(shipRightTimes, "Spaceship Right View");
    computeStats(shipSingleTimes, "Spaceship Single View");

	computeStats(fullFrameGpuTimes, "Full Frame GPU Time");
	computeStats(cpuFrameTimes, "Full Frame CPU Time");

    // Cleanup.
    // TODO: additional cleanup
    // Terrain
    glDeleteVertexArrays(1, &terrainVao);
    glDeleteBuffers(1, &terrainVbo);
    glDeleteBuffers(1, &terrainEbo);

    // Landing pad
    glDeleteVertexArrays(1, &padVao);
    glDeleteBuffers(1, &padVbo);
    glDeleteBuffers(1, &padEbo);

    // Spaceship parts
    glDeleteVertexArrays(1, &prismVao);
    glDeleteBuffers(1, &prismVbo);
    glDeleteBuffers(1, &prismEbo);

	// Nose cone
    glDeleteVertexArrays(1, &noseVao);
    glDeleteBuffers(1, &noseVbo);
    glDeleteBuffers(1, &noseEbo);

	// Pyramid bodies
    glDeleteVertexArrays(1, &pyrVao);
    glDeleteBuffers(1, &pyrVbo);
    glDeleteBuffers(1, &pyrEbo);

	// Cuboid bodies
    glDeleteVertexArrays(1, &cubVao);
    glDeleteBuffers(1, &cubVbo);
    glDeleteBuffers(1, &cubEbo);

	// Cylindrical landing struts
    glDeleteVertexArrays(1, &cylVao);
    glDeleteBuffers(1, &cylVbo);
    glDeleteBuffers(1, &cylEbo);

    // Particle system
    glDeleteVertexArrays(1, &particleVAO);
    glDeleteBuffers(1, &particleVBO);

    // Textures
    glDeleteTextures(1, &terrainTexture);
    glDeleteTextures(1, &exhaustTexture);

	// Terrain queries
    glDeleteQueries(QUERY_BUFFER_SIZE, terrainStartQueryLeft);
    glDeleteQueries(QUERY_BUFFER_SIZE, terrainEndQueryLeft);
    glDeleteQueries(QUERY_BUFFER_SIZE, terrainStartQueryRight);
    glDeleteQueries(QUERY_BUFFER_SIZE, terrainEndQueryRight);

	// Pads queries
    glDeleteQueries(QUERY_BUFFER_SIZE, padsStartQueryLeft);
    glDeleteQueries(QUERY_BUFFER_SIZE, padsEndQueryLeft);
    glDeleteQueries(QUERY_BUFFER_SIZE, padsStartQueryRight);
    glDeleteQueries(QUERY_BUFFER_SIZE, padsEndQueryRight);

	// Spaceship queries
    glDeleteQueries(QUERY_BUFFER_SIZE, shipStartQueryLeft);
    glDeleteQueries(QUERY_BUFFER_SIZE, shipEndQueryLeft);
    glDeleteQueries(QUERY_BUFFER_SIZE, shipStartQueryRight);
    glDeleteQueries(QUERY_BUFFER_SIZE, shipEndQueryRight);

	// Full frame queries
    glDeleteQueries(QUERY_BUFFER_SIZE, fullFrameStartQuery);
    glDeleteQueries(QUERY_BUFFER_SIZE, fullFrameEndQuery);

    return 0;
}
catch (std::exception const& eErr)
{
    std::print(stderr, "Top-level Exception ({}):\n", typeid(eErr).name());
    std::print(stderr, "{}\n", eErr.what());
    std::print(stderr, "Bye.\n");
    return 1;
}

namespace
{
    void glfw_callback_error_(int aErrNum, char const* aErrDesc)
    {
        std::print(stderr, "GLFW error: {} ({})\n", aErrDesc, aErrNum);
    }

    void glfw_callback_key_(GLFWwindow* aWindow, int aKey, int, int aAction, int mods)
    {
        if (GLFW_KEY_ESCAPE == aKey && GLFW_PRESS == aAction)
        {
            glfwSetWindowShouldClose(aWindow, GLFW_TRUE);
        }
        // Task 1.2: Update input state
        bool pressed = (aAction == GLFW_PRESS || aAction == GLFW_REPEAT);
        switch (aKey) {
        case GLFW_KEY_W: input.forward = pressed; break;
        case GLFW_KEY_S: input.backward = pressed; break;
        case GLFW_KEY_A: input.left = pressed; break;
        case GLFW_KEY_D: input.right = pressed; break;
        case GLFW_KEY_E: input.up = pressed; break;
        case GLFW_KEY_Q: input.down = pressed; break;
        case GLFW_KEY_LEFT_SHIFT:
        case GLFW_KEY_RIGHT_SHIFT: input.shift = pressed; break;
        case GLFW_KEY_LEFT_CONTROL:
        case GLFW_KEY_RIGHT_CONTROL: input.ctrl = pressed; break;
        // Task 1.6: Toggle directional light
        case GLFW_KEY_1: if (aAction == GLFW_PRESS) lights.enabled[0] = !lights.enabled[0]; break;
        case GLFW_KEY_2: if (aAction == GLFW_PRESS) lights.enabled[1] = !lights.enabled[1]; break;
        case GLFW_KEY_3: if (aAction == GLFW_PRESS) lights.enabled[2] = !lights.enabled[2]; break;
        case GLFW_KEY_4: if (aAction == GLFW_PRESS) dirEnabled = !dirEnabled; break;
        // Task 1.7: Toggle animation
        case GLFW_KEY_F:
            if (aAction == GLFW_PRESS) {
                if (!animRunning) { animRunning = true; animPaused = false; animTime = 0.0f; }
                else { animPaused = !animPaused; }
            }
            break;
        case GLFW_KEY_R:
            if (aAction == GLFW_PRESS) {
                animRunning = false;
                animPaused = false;
                animTime = 0.0f;
                currentSpeed = 0.5f; // reset acceleration state
                distanceTravelled = 0.0f; // reset path parameter
                shipPos = padPos1; // reset to pad
                shipVel = { 0,0,0 };
                shipModel = modelPad1; // reset transform
                travelDirection = Vec3f{ 0.0f, 0.0f, 1.0f }; // Reset travel direction to default forward
                aliveCount = 0;
                spawnAccumulator = 0.0f;
            }
            break;
        // Task 1.8: Camera modes
        case GLFW_KEY_C:
            if (aAction == GLFW_PRESS) {
                if (mods & GLFW_MOD_SHIFT) {
                    // Shift + C → cycle right camera
                    camModeRight = static_cast<CameraMode>((camModeRight + 1) % 3);
                }
                else {
                    // C → cycle left camera
                    camModeLeft = static_cast<CameraMode>((camModeLeft + 1) % 3);
                }
            }
            break;
		// Task 1.9: Split-screen toggle
        case GLFW_KEY_V: 
            if (aAction == GLFW_PRESS) {
                splitScreenEnabled = !splitScreenEnabled;
            }
            break;
        }
    }
};

namespace
{
    GLFWCleanupHelper::~GLFWCleanupHelper()
    {
        glfwTerminate();
    }

    GLFWWindowDeleter::~GLFWWindowDeleter()
    {
        if (window)
            glfwDestroyWindow(window);
    }
}
