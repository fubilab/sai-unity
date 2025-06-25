#include "../include/spectacularAI/unity/render.hpp"
#include "../include/spectacularAI/unity/output.hpp"
#include "../include/spectacularAI/unity/util.hpp"

#include <GL/glew.h>
#ifdef __APPLE__
#include <OpenGL/gl3.h>
#else
#include <GL/gl.h>
#endif
#include <iostream>
#include <atomic>
#include <mutex>
#include <cmath>
#include <Eigen/Geometry>
#include <cstring>

// Remove threading, keep global VioOutputWrapper* and texture/orientation state
static std::mutex g_vioOutputMutex;
static std::shared_ptr<const spectacularAI::VioOutput> g_vioOutput;
static int g_cameraId{0};
static std::mutex g_orientationMutex;
static std::atomic<uint32_t> g_renderedTextureId{0};
static std::atomic<uint32_t> g_renderedDepthTextureId{0};
static std::atomic<double> g_orientation_rendered[4];

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
GLint gTexUniform = -1, gDepthTexUniform = -1;
GLint gProjMatrixUniform = -1, gInvProjMatrixUniform = -1, gReprojMatrixUniform = -1;

const char* quadVert = R"(
#version 330 core
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec2 aUV;
out vec2 vUV;
void main() {
    vUV = aUV;
    gl_Position = vec4(aPos, 0.0, 1.0);
}
)";

const char* quadFrag = R"(
#version 330 core
in vec2 vUV;
out vec4 FragColor;
uniform sampler2D uTex;
uniform sampler2D uDepthTex;

uniform mat4 uProjectionMatrix;
uniform mat4 uInvProjectionMatrix;
uniform mat4 uReprojectionMatrix;

void main() {
    float raw_depth = texture(uDepthTex, vUV).r;

    // Unproject from screen space (uv, depth) to view space
    vec4 clip_pos;
    clip_pos.xy = vUV * 2.0 - 1.0;
    clip_pos.z = raw_depth * 2.0 - 1.0; // Convert depth from [0,1] to NDC [-1,1]
    clip_pos.w = 1.0;

    vec4 view_pos = uInvProjectionMatrix * clip_pos;
    view_pos /= view_pos.w;

    // Apply delta rotation in view space
    vec4 reproj_view_pos = uReprojectionMatrix * view_pos;

    // Project back to clip space
    vec4 reproj_clip_pos = uProjectionMatrix * reproj_view_pos;
    reproj_clip_pos /= reproj_clip_pos.w;

    // Convert to UV coordinates
    vec2 reproj_uv = reproj_clip_pos.xy * 0.5 + 0.5;

    if (reproj_uv.x < 0.0 || reproj_uv.x > 1.0 || reproj_uv.y < 0.0 || reproj_uv.y > 1.0) {
        FragColor = vec4(0.0, 0.0, 0.0, 1.0); // Black for pixels outside original view
    } else {
        FragColor = texture(uTex, reproj_uv);
    }
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
    gTexUniform = glGetUniformLocation(gShader, "uTex");
    gDepthTexUniform = glGetUniformLocation(gShader, "uDepthTex");
    gProjMatrixUniform = glGetUniformLocation(gShader, "uProjectionMatrix");
    gInvProjMatrixUniform = glGetUniformLocation(gShader, "uInvProjectionMatrix");
    gReprojMatrixUniform = glGetUniformLocation(gShader, "uReprojectionMatrix");
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

// --- New: Store Unity camera projection matrix for accurate reprojection ---
static float g_renderedProjection[16] = {
    1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1
};

extern "C" EXPORT_API void sai_set_rendered_projection(const float* matrix16) {
    // Expects column-major 4x4 matrix from Unity (float[16])
    for (int i = 0; i < 16; ++i) g_renderedProjection[i] = matrix16[i];
}

// The GL render callback (all OpenGL code goes here)
static void UNITY_INTERFACE_API OnRenderEvent(int /*eventId*/) {
    if (!s_UnityGraphics) return;

    ensureQuadResources();

    // Bind default framebuffer to ensure drawing to Unity's backbuffer
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    uint32_t texId = g_renderedTextureId.load();
    uint32_t depthTexId = g_renderedDepthTextureId.load();
    if (texId == 0 || depthTexId == 0) {
        return;
    }
    if (!glIsTexture(texId) || !glIsTexture(depthTexId)) {
        return;
    }
    int width = 0, height = 0;
    glBindTexture(GL_TEXTURE_2D, texId);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height);
    glBindTexture(GL_TEXTURE_2D, 0);
    if (width <= 0 || height <= 0) {
        return;
    }

    glViewport(0, 0, width, height);
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
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

    // Create Eigen quaternions (w, x, y, z)
    Eigen::Quaterniond q_rendered(r_w, r_x, r_y, r_z);
    Eigen::Quaterniond q_latest(l_w, l_x, l_y, l_z);
    q_rendered.normalize();
    q_latest.normalize();

    // Calculate the delta rotation from the rendered to the latest orientation
    Eigen::Quaterniond q_delta_local = q_latest.inverse() * q_rendered;
    q_delta_local.normalize();

    // Convert delta quaternion to 4x4 matrix (for view-space rotation)
    Eigen::Matrix4d reprojection_matrix_d = Eigen::Matrix4d::Identity();
    reprojection_matrix_d.block<3,3>(0,0) = q_delta_local.toRotationMatrix();
    Eigen::Matrix4f reprojection_matrix_f = reprojection_matrix_d.cast<float>();

    // Get projection matrix and its inverse
    Eigen::Matrix4f projection_matrix;
    memcpy(projection_matrix.data(), g_renderedProjection, 16 * sizeof(float));
    Eigen::Matrix4f inv_projection_matrix = projection_matrix.inverse();

    // Modern OpenGL Core profile rendering
    glUseProgram(gShader);

    glUniformMatrix4fv(gProjMatrixUniform, 1, GL_FALSE, projection_matrix.data());
    glUniformMatrix4fv(gInvProjMatrixUniform, 1, GL_FALSE, inv_projection_matrix.data());
    glUniformMatrix4fv(gReprojMatrixUniform, 1, GL_FALSE, reprojection_matrix_f.data());

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texId);
    glUniform1i(gTexUniform, 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, depthTexId);
    glUniform1i(gDepthTexUniform, 1);

    glBindVertexArray(gQuadVAO);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindVertexArray(0);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);
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

EXPORT_API void sai_set_rendered_depth_texture(uint32_t textureId) {
    g_renderedDepthTextureId = textureId;
}

// Plugin event for orientation reprojection
EXPORT_API void* GetRenderEventFunc() {
    return (void*)OnRenderEvent;
}

EXPORT_API void sai_reprojection_plugin_event(int /*eventId*/) {
    // Deprecated: do nothing, C# should use GL.IssuePluginEvent(GetRenderEventFunc(), eventId)
}

} // extern "C"

// All state is set via atomic globals from C# before issuing the plugin event.
// No need for event IDs if only one action is performed per event.
// Thread safety is ensured by std::atomic and std::mutex for orientation.
// The plugin event reads the latest state set by C#.
