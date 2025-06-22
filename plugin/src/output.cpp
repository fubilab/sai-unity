#include "../include/spectacularAI/unity/output.hpp"
#include "../include/spectacularAI/unity/util.hpp"

#include <cassert>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <cmath>
#include <string>
#include <vector>
#include <iostream>
#include <Unity/IUnityGraphics.h>
#include <Eigen/Geometry>

spectacularAI::TrackingStatus sai_vio_output_get_tracking_status(const VioOutputWrapper* vioOutputHandle) {
    assert(vioOutputHandle);
    return vioOutputHandle->getHandle()->status;
}

spectacularAI::Pose sai_vio_output_get_pose(const VioOutputWrapper* vioOutputHandle) {
    assert(vioOutputHandle);
    return vioOutputHandle->getHandle()->pose;
}

spectacularAI::Vector3d sai_vio_output_get_velocity(const VioOutputWrapper* vioOutputHandle) {
    assert(vioOutputHandle);
    return vioOutputHandle->getHandle()->velocity;
}

spectacularAI::Vector3d sai_vio_output_get_angular_velocity(const VioOutputWrapper* vioOutputHandle) {
    assert(vioOutputHandle);
    return vioOutputHandle->getHandle()->angularVelocity;
}

spectacularAI::Vector3d sai_vio_output_get_acceleration(const VioOutputWrapper* vioOutputHandle) {
    assert(vioOutputHandle);
    return vioOutputHandle->getHandle()->acceleration;
}

Matrix3dWrapper sai_vio_output_get_position_covariance(const VioOutputWrapper* vioOutputHandle) {
    assert(vioOutputHandle);
    return reinterpret_cast<const Matrix3dWrapper&>(vioOutputHandle->getHandle()->positionCovariance);
}

Matrix3dWrapper sai_vio_output_get_velocity_covariance(const VioOutputWrapper* vioOutputHandle) {
    assert(vioOutputHandle);
    return reinterpret_cast<const Matrix3dWrapper&>(vioOutputHandle->getHandle()->velocityCovariance);
}

spectacularAI::CameraPose* sai_vio_output_get_camera_pose(const VioOutputWrapper* vioOutputHandle, int cameraId) {
    assert(vioOutputHandle);
    spectacularAI::CameraPose* cameraPose = new spectacularAI::CameraPose();
    *cameraPose = vioOutputHandle->getHandle()->getCameraPose(cameraId);
    return cameraPose;
}

int32_t sai_vio_output_get_tag(const VioOutputWrapper* vioOutputHandle) {
    assert(vioOutputHandle);
    return vioOutputHandle->getHandle()->tag;
}

void sai_vio_output_release(const VioOutputWrapper* vioOutputHandle) {
    if (vioOutputHandle) delete vioOutputHandle;
}

spectacularAI::Pose sai_camera_pose_get_pose(spectacularAI::CameraPose* cameraPoseHandle) {
    assert(cameraPoseHandle);
    return cameraPoseHandle->pose;
}

spectacularAI::Vector3d sai_camera_pose_get_velocity(spectacularAI::CameraPose* cameraPoseHandle) {
    assert(cameraPoseHandle);
    return cameraPoseHandle->velocity;
}

const CameraWrapper* sai_camera_pose_get_camera(spectacularAI::CameraPose* cameraPoseHandle) {
    assert(cameraPoseHandle);
    return new CameraWrapper(cameraPoseHandle->camera);
}

Matrix4dWrapper sai_camera_pose_get_world_to_camera_matrix(const spectacularAI::CameraPose* cameraPoseHandle) {
    assert(cameraPoseHandle);
    return matrix_to_wrapper(cameraPoseHandle->getWorldToCameraMatrix());
}

Matrix4dWrapper sai_camera_pose_get_camera_to_world_matrix(const spectacularAI::CameraPose* cameraPoseHandle) {
    assert(cameraPoseHandle);
    return matrix_to_wrapper(cameraPoseHandle->getCameraToWorldMatrix());
}

spectacularAI::Vector3d sai_camera_pose_get_position(const spectacularAI::CameraPose* cameraPoseHandle) {
    assert(cameraPoseHandle);
    return cameraPoseHandle->getPosition();
}

void sai_camera_pose_release(spectacularAI::CameraPose* cameraPoseHandle) {
    if (cameraPoseHandle) delete cameraPoseHandle;
}

bool sai_camera_pixel_to_ray(
        const CameraWrapper* cameraHandle,
        const spectacularAI::PixelCoordinates* pixel,
        spectacularAI::Vector3d* ray) {
    assert(cameraHandle);
    return cameraHandle->getHandle()->pixelToRay(*pixel, *ray);
}

bool sai_camera_ray_to_pixel(
        const CameraWrapper* cameraHandle,
        const spectacularAI::Vector3d* ray,
        spectacularAI::PixelCoordinates *pixel) {
    assert(cameraHandle);
    return cameraHandle->getHandle()->rayToPixel(*ray, *pixel);
}

Matrix3dWrapper sai_camera_get_intrinsic_matrix(const CameraWrapper* cameraHandle) {
    assert(cameraHandle);
    return matrix_to_wrapper(cameraHandle->getHandle()->getIntrinsicMatrix());
}

Matrix4dWrapper sai_camera_get_projection_matrix_opengl(
        const CameraWrapper* cameraHandle,
        double nearClip, 
        double farClip) {
    assert(cameraHandle);
    return matrix_to_wrapper(cameraHandle->getHandle()->getProjectionMatrixOpenGL(nearClip, farClip));
}

CameraWrapper* sai_camera_build_pinhole(
        Matrix3dWrapper intrinsicMatrix, 
        int width,
        int height) {
    const spectacularAI::Matrix3d &intrinsics = reinterpret_cast<const spectacularAI::Matrix3d&>(intrinsicMatrix);
    return new CameraWrapper(spectacularAI::Camera::buildPinhole(intrinsics, width, height));
}

void sai_camera_release(const CameraWrapper* cameraHandle) {
    if (cameraHandle) delete cameraHandle;
}

#include <GL/glew.h>
#ifdef __APPLE__
#include <OpenGL/gl3.h>
#else
#include <GL/gl.h>
#endif
#include <iostream>

// Remove threading, keep global VioOutputWrapper* and texture/orientation state
static std::mutex g_vioOutputMutex;
static std::shared_ptr<const spectacularAI::VioOutput> g_vioOutput;
static int g_cameraId{0};
static std::mutex g_orientationMutex;
static std::atomic<uint32_t> g_renderedTextureId{0};
static std::atomic<double> g_orientation_rendered[4];
static std::atomic<float> g_renderedDepth{1.0f};

static std::mutex g_projectionMutex;
static float g_projectionMatrix[16];
static std::atomic<bool> g_hasProjectionMatrix{false};

struct OrientationInit {
    OrientationInit() {
        g_orientation_rendered[0] = 1.0;
        g_orientation_rendered[1] = 0.0;
        g_orientation_rendered[2] = 0.0;
        g_orientation_rendered[3] = 0.0;
    }
};
static OrientationInit orientationInit;

// --- Modern OpenGL Core profile quad rendering ---
namespace {
GLuint gQuadVAO = 0, gQuadVBO = 0, gShader = 0;
GLint gMVPUniform = -1, gTexUniform = -1;

const char* quadVert = R"(
#version 330 core
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec2 aUV;
uniform mat4 uMVP;
out vec2 vUV;
void main() {
    vUV = aUV;
    gl_Position = uMVP * vec4(aPos, 0.0, 1.0);
}
)";

const char* quadFrag = R"(
#version 330 core
in vec2 vUV;
out vec4 FragColor;
uniform sampler2D uTex;
void main() {
    FragColor = texture(uTex, vUV);
}
)";

GLuint compileShader(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(s, 512, nullptr, log);
        std::cerr << "Shader compile error: " << log << std::endl;
    }
    return s;
}

GLuint createShaderProgram(const char* vs, const char* fs) {
    GLuint v = compileShader(GL_VERTEX_SHADER, vs);
    GLuint f = compileShader(GL_FRAGMENT_SHADER, fs);
    GLuint prog = glCreateProgram();
    glAttachShader(prog, v);
    glAttachShader(prog, f);
    glLinkProgram(prog);
    glDeleteShader(v);
    glDeleteShader(f);
    GLint ok = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetProgramInfoLog(prog, 512, nullptr, log);
        std::cerr << "Program link error: " << log << std::endl;
    }
    return prog;
}

void ensureQuadResources() {
    static bool glewInitialized = false;
    if (!glewInitialized) {
        GLenum err = glewInit();
        if (err != GLEW_OK) {
            std::cerr << "GLEW init error: " << glewGetErrorString(err) << std::endl;
        }
        glewInitialized = true;
    }
    if (gQuadVAO) return;
    float quadVerts[] = {
        // pos      // uv
        -1, -1,     0, 0,
         1, -1,     1, 0,
        -1,  1,     0, 1,
         1,  1,     1, 1
    };
    glGenVertexArrays(1, &gQuadVAO);
    glGenBuffers(1, &gQuadVBO);
    glBindVertexArray(gQuadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, gQuadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVerts), quadVerts, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    gShader = createShaderProgram(quadVert, quadFrag);
    gMVPUniform = glGetUniformLocation(gShader, "uMVP");
    gTexUniform = glGetUniformLocation(gShader, "uTex");
}
}
// --- End modern OpenGL helpers ---

static IUnityInterfaces* s_UnityInterfaces = nullptr;
static IUnityGraphics* s_UnityGraphics = nullptr;

// Forward declaration for the GL render callback
static void UNITY_INTERFACE_API OnRenderEvent(int eventId);

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
UnityPluginLoad(IUnityInterfaces* unityInterfaces) {
    s_UnityInterfaces = unityInterfaces;
    s_UnityGraphics = s_UnityInterfaces->Get<IUnityGraphics>();
    if (s_UnityGraphics) {
        s_UnityGraphics->RegisterDeviceEventCallback([](UnityGfxDeviceEventType eventType) {
            if (eventType == kUnityGfxDeviceEventInitialize) {
                s_UnityGraphics = s_UnityInterfaces->Get<IUnityGraphics>();
            } else if (eventType == kUnityGfxDeviceEventShutdown) {
                s_UnityGraphics = nullptr;
            }
        });
    }
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
UnityPluginUnload() {
    // Nothing to clean up
}

// The GL render callback (all OpenGL code goes here)
static void UNITY_INTERFACE_API OnRenderEvent(int eventId) {
    if (!s_UnityGraphics) return;

    ensureQuadResources();

    // Bind default framebuffer to ensure drawing to Unity's backbuffer
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    uint32_t texId = g_renderedTextureId.load();
    if (texId == 0) {
        std::cerr << "[OnRenderEvent] No valid texture ID set, skipping render." << std::endl;
        return;
    }
    if (!glIsTexture(texId)) {
        std::cerr << "[OnRenderEvent] Texture ID " << texId << " is not a valid GL texture, skipping render." << std::endl;
        return;
    }
    int width = 0, height = 0;
    glBindTexture(GL_TEXTURE_2D, texId);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height);
    glBindTexture(GL_TEXTURE_2D, 0);
    if (width <= 0 || height <= 0) {
        std::cerr << "[OnRenderEvent] Texture has invalid size (" << width << ", " << height << "), skipping render." << std::endl;
        return;
    }

    const float aspect = (height > 0) ? (float)width / (float)height : 1.0f;

    glViewport(0, 0, width, height);
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        std::cerr << "[OnRenderEvent] OpenGL error before rendering: 0x" << std::hex << err << std::dec << std::endl;
        return;
    }
    // Get rendered orientation
    double r_x, r_y, r_z, r_w;
    {
        std::lock_guard<std::mutex> lock(g_orientationMutex);
        r_x = g_orientation_rendered[0];
        r_y = g_orientation_rendered[1];
        r_z = g_orientation_rendered[2];
        r_w = g_orientation_rendered[3];
    }

    // std::cout << "[SAI Reprojection] Rendered orientation (from Unity): "
    //           << r_x << ", " << r_y << ", " << r_z << ", " << r_w << std::endl;

    // Get latest orientation from VIO output
    std::shared_ptr<const spectacularAI::VioOutput> localVioOutput;
    int localCameraId;
    {
        std::lock_guard<std::mutex> lock(g_vioOutputMutex);
        localVioOutput = g_vioOutput;
        localCameraId = g_cameraId;
    }

    double l_x = r_x, l_y = r_y, l_z = r_z, l_w = r_w; // Default to no change
    if (localVioOutput) {
        VioOutputWrapper tempWrapper(localVioOutput);
        spectacularAI::CameraPose* cameraPose = sai_vio_output_get_camera_pose(&tempWrapper, localCameraId);
        if (cameraPose) {
            spectacularAI::Quaternion q = cameraPose->pose.orientation;
            l_x = q.x;
            l_y = q.y;
            l_z = q.z;
            l_w = q.w;
            sai_camera_pose_release(cameraPose);
        }
    }

    // std::cout << "[SAI Reprojection] Latest orientation (from VIO): "
    //           << l_x << ", " << l_y << ", " << l_z << ", " << l_w << std::endl;

    // Compute delta = latest * inverse(rendered)
    double inv_r_x = -r_x, inv_r_y = -r_y, inv_r_z = -r_z, inv_r_w = r_w;
    double d_x = l_w * inv_r_x + l_x * inv_r_w + l_y * inv_r_z - l_z * inv_r_y;
    double d_y = l_w * inv_r_y - l_x * inv_r_z + l_y * inv_r_w + l_z * inv_r_x;
    double d_z = l_w * inv_r_z + l_x * inv_r_y - l_y * inv_r_x + l_z * inv_r_w;
    double d_w = l_w * inv_r_w - l_x * inv_r_x - l_y * inv_r_y - l_z * inv_r_z;
    double norm = sqrt(d_x*d_x + d_y*d_y + d_z*d_z + d_w*d_w);
    d_x /= norm; d_y /= norm; d_z /= norm; d_w /= norm;

    // The view rotation is the inverse of the delta rotation.
    // For a unit quaternion, inverse is the conjugate.
    const double qx = -d_x;
    const double qy = -d_y;
    const double qz = -d_z;
    const double qw = d_w;

    // --- Get Projection Matrix ---
    if (!g_hasProjectionMatrix.load()) {
        // No projection matrix from Unity yet.
        return;
    }
    float projMatrix[16];
    {
        std::lock_guard<std::mutex> lock(g_projectionMutex);
        for (int i = 0; i < 16; ++i) projMatrix[i] = g_projectionMatrix[i];
    }

    // Don't modify the projection matrix - use it as-is from Unity

    // --- Convert Delta Quaternion to Euler Angles ---
    // This provides a more stable representation of rotation than raw components.
    const double yaw_angle   = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    const double pitch_angle = -asin(2.0 * (qw * qy - qz * qx));
    const double roll_angle_rad  = -atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));

    // --- Apply transformations based on Euler angles ---
    float depth = g_renderedDepth.load();

    // Apply inverse transformations. Parallax effect for translation is scaled by depth.
    float translateX = -yaw_angle * depth * 1.5f;   // Yaw -> X translation
    float translateY = -pitch_angle * depth * 1.5f; // Pitch -> Y translation
    float rollAngle  = -roll_angle_rad * 1.5f;             // Roll -> 2D rotation of the quad (not depth dependent)

    // Create a transformation matrix with translation and roll rotation
    float cosRoll = cos(rollAngle);
    float sinRoll = sin(rollAngle);

    // Column-major matrix for OpenGL, corrected for aspect ratio
    float finalMVP[16] = {
        cosRoll,          sinRoll * aspect, 0.0f, 0.0f,
       -sinRoll / aspect, cosRoll,          0.0f, 0.0f,
        0.0f,             0.0f,             1.0f, 0.0f,
        translateX,       translateY,       0.0f, 1.0f
    };
    
    // Debug output occasionally
    static int debugCounter = 0;
    if (++debugCounter % 30 == 0) { // Every ~30 frames
        std::cout << "[Reproject] Euler (deg): Yaw=" << (yaw_angle * 180.0/3.14159)
                  << ", Pitch=" << (pitch_angle * 180.0/3.14159)
                  << ", Roll=" << (roll_angle_rad * 180.0/3.14159) << std::endl;
        std::cout << "[Reproject] Transform: X=" << translateX << " Y=" << translateY << " Roll=" << (rollAngle * 180.0f / 3.14159265f) << " deg" << std::endl;
    }

    // Modern OpenGL Core profile rendering
    glUseProgram(gShader);
    glUniformMatrix4fv(gMVPUniform, 1, GL_FALSE, finalMVP);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texId);
    glUniform1i(gTexUniform, 0);
    glBindVertexArray(gQuadVAO);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glUseProgram(0);
    err = glGetError();
    if (err != GL_NO_ERROR) {
        std::cerr << "[OnRenderEvent] OpenGL error after rendering: 0x" << std::hex << err << std::dec << std::endl;
    }
}

extern "C" {

EXPORT_API void sai_set_rendered_orientation(double x, double y, double z, double w) {
    std::lock_guard<std::mutex> lock(g_orientationMutex);
    g_orientation_rendered[0] = x;
    g_orientation_rendered[1] = y;
    g_orientation_rendered[2] = z;
    g_orientation_rendered[3] = w;
}

EXPORT_API void sai_set_vio_output_handle(VioOutputWrapper* vioOutputHandle, int cameraId) {
    std::lock_guard<std::mutex> lock(g_vioOutputMutex);
    if (vioOutputHandle) {
        g_vioOutput = vioOutputHandle->getHandle();
    } else {
        g_vioOutput.reset();
    }
    g_cameraId = cameraId;
}

EXPORT_API void sai_set_rendered_texture(uint32_t textureId) {
    g_renderedTextureId = textureId;
}

EXPORT_API void sai_set_rendered_depth(float depth) {
    g_renderedDepth = depth;
}

EXPORT_API void sai_set_projection_matrix(const float* matrix) {
    std::lock_guard<std::mutex> lock(g_projectionMutex);
    for (int i = 0; i < 16; ++i) g_projectionMatrix[i] = matrix[i];
    g_hasProjectionMatrix = true;
}

// Plugin event for orientation reprojection
EXPORT_API void* GetRenderEventFunc() {
    return (void*)OnRenderEvent;
}

EXPORT_API void sai_reprojection_plugin_event(int eventId) {
    // Deprecated: do nothing, C# should use GL.IssuePluginEvent(GetRenderEventFunc(), eventId)
}

} // extern "C"

// All state is set via atomic globals from C# before issuing the plugin event.
// No need for event IDs if only one action is performed per event.
// Thread safety is ensured by std::atomic and std::mutex for orientation.
// The plugin event reads the latest state set by C#.