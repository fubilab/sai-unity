using System;
using System.Runtime.InteropServices;
using UnityEngine;
using SpectacularAI.DepthAI;
using SpectacularAI.Native;
using UnityEngine.Rendering;

public static class NativeReprojection
{
  [DllImport(ApiConstants.saiNativeApi, CallingConvention = ApiConstants.saiCallingConvention)]
  public static extern void sai_set_rendered_orientation(double x, double y, double z, double w);

  [DllImport(ApiConstants.saiNativeApi, CallingConvention = ApiConstants.saiCallingConvention)]
  public static extern void sai_set_vio_output_handle(IntPtr vioOutputHandle, int cameraId);

  [DllImport(ApiConstants.saiNativeApi, CallingConvention = ApiConstants.saiCallingConvention)]
  public static extern void sai_set_rendered_texture(uint textureId);

  [DllImport(ApiConstants.saiNativeApi, CallingConvention = ApiConstants.saiCallingConvention)]
  public static extern void sai_reprojection_plugin_event(int eventId);

  // New: Import the function pointer for the render event
  [DllImport(ApiConstants.saiNativeApi, CallingConvention = ApiConstants.saiCallingConvention)]
  public static extern IntPtr GetRenderEventFunc();

  // Cache the function pointer
  private static IntPtr _renderEventFuncPtr = IntPtr.Zero;
  public static IntPtr RenderEventFuncPtr
  {
    get
    {
      if (_renderEventFuncPtr == IntPtr.Zero)
        _renderEventFuncPtr = GetRenderEventFunc();
      return _renderEventFuncPtr;
    }
  }

  [DllImport(ApiConstants.saiNativeApi, CallingConvention = ApiConstants.saiCallingConvention)]
  public static extern void sai_set_rendered_depth_texture(uint textureId);

  [DllImport(ApiConstants.saiNativeApi, CallingConvention = ApiConstants.saiCallingConvention)]
  public static extern void sai_set_rendered_projection([In] float[] matrix16);
}

public class NativeReprojectionDemo : MonoBehaviour
{
  public int CameraId = 0;
  private UnityEngine.Camera _camera;
  private RenderTexture _offscreenRT;

  void Awake()
  {
    #if UNITY_EDITOR
    Debug.LogWarning("NativeReprojectionDemo: Disabled in Unity Editor - native rendering not supported in editor");
    enabled = false;
    #else
    _camera = GetComponent<UnityEngine.Camera>();
    if (_camera == null)
      _camera = UnityEngine.Camera.main;
    if (_camera == null)
      Debug.LogError("NativeReprojectionDemo: No Camera found!");
    else
      _camera.depthTextureMode = DepthTextureMode.Depth;
    #endif
  }

  void OnEnable()
  {
    RenderPipelineManager.endCameraRendering += OnEndCameraRendering;
  }

  void OnDisable()
  {
    RenderPipelineManager.endCameraRendering -= OnEndCameraRendering;
  }

  void OnEndCameraRendering(ScriptableRenderContext ctx, UnityEngine.Camera cam)
  {
    if (cam.cameraType != CameraType.Game) return;
    var target = cam.activeTexture ?? (RenderTexture)cam.targetTexture;
    if (target == null) return;
    var texPtr = target.GetNativeTexturePtr();
    if (texPtr != IntPtr.Zero)
    {
      NativeReprojection.sai_set_rendered_texture((uint)texPtr.ToInt64());
    }

    var depthTexture = Shader.GetGlobalTexture("_CameraDepthTexture");
    if (depthTexture != null)
    {
        var depthTexPtr = depthTexture.GetNativeTexturePtr();
        if (depthTexPtr != IntPtr.Zero)
        {
            NativeReprojection.sai_set_rendered_depth_texture((uint)depthTexPtr.ToInt64());
        }
    }

    var vioHandle = Vio.Output?.GetNativeHandle();
    if (vioHandle.HasValue && vioHandle.Value != IntPtr.Zero)
    {
      NativeReprojection.sai_set_vio_output_handle(vioHandle.Value, CameraId);
    }

    GL.IssuePluginEvent(NativeReprojection.RenderEventFuncPtr, 0);
  }

  void Start()
  {
    if (_camera != null)
    {
      _offscreenRT = new RenderTexture(Screen.width, Screen.height, 24, RenderTextureFormat.ARGB32);
      _offscreenRT.Create();
      _camera.targetTexture = _offscreenRT;
    }

    if (_camera != null)
    {
      Matrix4x4 proj = _camera.projectionMatrix;
      float[] projColMajor = new float[16];
      for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
          projColMajor[col * 4 + row] = proj[row, col];
      NativeReprojection.sai_set_rendered_projection(projColMajor);
    }
  }

  void OnDestroy()
  {
    if (_offscreenRT != null)
    {
      if (_camera != null && _camera.targetTexture == _offscreenRT)
        _camera.targetTexture = null;
      _offscreenRT.Release();
      Destroy(_offscreenRT);
    }
  }

  void LateUpdate()
  {
    if (Vio.Output is null)
      return; // No VIO output, nothing to update
    var unityQuat = Vio.Output.Pose._orientation;
    NativeReprojection.sai_set_rendered_orientation(unityQuat.x, unityQuat.y, unityQuat.z, unityQuat.w);
  }
}
