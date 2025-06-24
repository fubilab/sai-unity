#pragma once

#include <cstdint>
#include <Unity/IUnityGraphics.h>
#include "../../../include/spectacularAI/unity/output.hpp"

// Initialization functions
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginLoad(IUnityInterfaces* unityInterfaces);
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginUnload();

// Rendering API functions
extern "C" {
    EXPORT_API void sai_set_rendered_orientation(double x, double y, double z, double w);
    EXPORT_API void sai_set_vio_output_handle(VioOutputWrapper* vioOutputHandle, int cameraId);
    EXPORT_API void sai_set_rendered_texture(uint32_t textureId);
    EXPORT_API void sai_set_rendered_depth(float depth);
    EXPORT_API void* GetRenderEventFunc();
    EXPORT_API void sai_reprojection_plugin_event(int eventId);
}
