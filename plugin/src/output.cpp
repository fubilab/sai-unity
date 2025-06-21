#include "../include/spectacularAI/unity/output.hpp"
#include "../include/spectacularAI/unity/util.hpp"

#include <cassert>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstdint>

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

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#else
#include <GL/gl.h>
#endif

static std::atomic<double> g_orientation_rendered[4] = {1.0, 0.0, 0.0, 0.0};
static std::atomic<VioOutputWrapper*> g_vioOutputHandle{nullptr};
static std::atomic<int> g_cameraId{0};
static std::atomic<bool> g_reprojectionThreadRunning{false};
static std::thread g_reprojectionThread;
static std::mutex g_orientationMutex;
static std::atomic<uint32_t> g_renderedTextureId{0};

void sai_set_rendered_orientation(double x, double y, double z, double w) {
    std::lock_guard<std::mutex> lock(g_orientationMutex);
    g_orientation_rendered[0] = x;
    g_orientation_rendered[1] = y;
    g_orientation_rendered[2] = z;
    g_orientation_rendered[3] = w;
}

void sai_set_vio_output_handle(VioOutputWrapper* vioOutputHandle, int cameraId) {
    g_vioOutputHandle = vioOutputHandle;
    g_cameraId = cameraId;
}

void sai_stop_reprojection_thread() {
    g_reprojectionThreadRunning = false;
    if (g_reprojectionThread.joinable()) {
        g_reprojectionThread.join();
    }
}

void reprojection_thread_func(int targetFps) {
    g_reprojectionThreadRunning = true;
    const double frameInterval = 1.0 / targetFps;
    while (g_reprojectionThreadRunning) {
        auto start = std::chrono::high_resolution_clock::now();
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
        VioOutputWrapper* vioOutput = g_vioOutputHandle.load();
        int cameraId = g_cameraId.load();
        double l_x = 0, l_y = 0, l_z = 0, l_w = 1;
        if (vioOutput) {
            spectacularAI::CameraPose* cameraPose = sai_vio_output_get_camera_pose(vioOutput, cameraId);
            if (cameraPose) {
                spectacularAI::Quaternion q = cameraPose->pose.orientation;
                l_x = q.x;
                l_y = q.y;
                l_z = q.z;
                l_w = q.w;
                sai_camera_pose_release(cameraPose);
            }
        }
        // Compute delta = latest * inverse(rendered)
        double inv_r_x = -r_x, inv_r_y = -r_y, inv_r_z = -r_z, inv_r_w = r_w;
        double d_x = l_w * inv_r_x + l_x * inv_r_w + l_y * inv_r_z - l_z * inv_r_y;
        double d_y = l_w * inv_r_y - l_x * inv_r_z + l_y * inv_r_w + l_z * inv_r_x;
        double d_z = l_w * inv_r_z + l_x * inv_r_y - l_y * inv_r_x + l_z * inv_r_w;
        double d_w = l_w * inv_r_w - l_x * inv_r_x - l_y * inv_r_y - l_z * inv_r_z;
        double norm = sqrt(d_x*d_x + d_y*d_y + d_z*d_z + d_w*d_w);
        d_x /= norm; d_y /= norm; d_z /= norm; d_w /= norm;
        float xx = d_x * d_x;
        float yy = d_y * d_y;
        float zz = d_z * d_z;
        float xy = d_x * d_y;
        float xz = d_x * d_z;
        float yz = d_y * d_z;
        float wx = d_w * d_x;
        float wy = d_w * d_y;
        float wz = d_w * d_z;
        float rot[16];
        rot[0] = 1.0f - 2.0f * (yy + zz);
        rot[1] = 2.0f * (xy - wz);
        rot[2] = 2.0f * (xz + wy);
        rot[3] = 0.0f;
        rot[4] = 2.0f * (xy + wz);
        rot[5] = 1.0f - 2.0f * (xx + zz);
        rot[6] = 2.0f * (yz - wx);
        rot[7] = 0.0f;
        rot[8] = 2.0f * (xz - wy);
        rot[9] = 2.0f * (yz + wx);
        rot[10] = 1.0f - 2.0f * (xx + yy);
        rot[11] = 0.0f;
        rot[12] = 0.0f;
        rot[13] = 0.0f;
        rot[14] = 0.0f;
        rot[15] = 1.0f;
        // OpenGL output (same as before)
        glPushAttrib(GL_ALL_ATTRIB_BITS);
        glPushMatrix();
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixf(rot);
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(-1, 1, -1, 1, -1, 1);
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, g_renderedTextureId.load());
        glBegin(GL_QUADS);
        glTexCoord2f(0, 0); glVertex2f(-1, -1);
        glTexCoord2f(1, 0); glVertex2f(1, -1);
        glTexCoord2f(1, 1); glVertex2f(1, 1);
        glTexCoord2f(0, 1); glVertex2f(-1, 1);
        glEnd();
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();
        glPopAttrib();
        // Sleep to maintain target framerate
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        double sleepTime = frameInterval - elapsed.count();
        if (sleepTime > 0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(sleepTime));
        }
    }
}

extern "C" {

EXPORT_API void sai_set_rendered_orientation(double x, double y, double z, double w) {
    sai_set_rendered_orientation(x, y, z, w);
}

EXPORT_API void sai_set_vio_output_handle(VioOutputWrapper* vioOutputHandle, int cameraId) {
    sai_set_vio_output_handle(vioOutputHandle, cameraId);
}

EXPORT_API void sai_set_rendered_texture(uint32_t textureId) {
    g_renderedTextureId = textureId;
}

EXPORT_API void sai_start_reprojection_thread(int targetFps) {
    sai_stop_reprojection_thread();
    g_reprojectionThread = std::thread(reprojection_thread_func, targetFps);
}

EXPORT_API void sai_stop_reprojection_thread() {
    sai_stop_reprojection_thread();
}

} // extern "C"