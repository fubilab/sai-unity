using System;
using System.Runtime.InteropServices;
using UnityEngine;
using SpectacularAI;
using SpectacularAI.DepthAI;
using SpectacularAI.Native;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using Unity.VisualScripting;

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
  public static extern void sai_set_projection_matrix([MarshalAs(UnmanagedType.LPArray, SizeConst = 16)] float[] matrix);

  [DllImport(ApiConstants.saiNativeApi, CallingConvention = ApiConstants.saiCallingConvention)]
  public static extern void sai_set_rendered_depth(float depth);
}

public class NativeReprojectionDemo : MonoBehaviour
{
  public int CameraId = 0;
  public int TargetFps = 60;
  private UnityEngine.Camera _camera;
  private RenderTexture _offscreenRT;

  void Awake()
  {
    _camera = GetComponent<UnityEngine.Camera>();
    if (_camera == null)
      _camera = UnityEngine.Camera.main;
    if (_camera == null)
      Debug.LogError("NativeReprojectionDemo: No Camera found!");
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
    // Debug.Log($"NativeReprojectionDemo: OnEndCameraRendering for camera {cam.name}");
    if (cam.cameraType != CameraType.Game) return;
    var target = cam.activeTexture ?? (RenderTexture)cam.targetTexture;
    if (target == null) return;
    var texPtr = target.GetNativeTexturePtr();
    if (texPtr != IntPtr.Zero)
    {
      // Debug.Log($"NativeReprojectionDemo: Rendering to target {target.name} with id {target.GetNativeTexturePtr()}");
      NativeReprojection.sai_set_rendered_texture((uint)texPtr.ToInt64());
    }

    // Vio output handle is new each update, so we set it here
    var vioHandle = Vio.Output?.GetNativeHandle();
    if (vioHandle.HasValue && vioHandle.Value != IntPtr.Zero)
    {
      // Debug.Log($"NativeReprojectionDemo: Setting VIO output handle for camera {CameraId} with handle {vioHandle.Value}");
      NativeReprojection.sai_set_vio_output_handle(vioHandle.Value, CameraId);
    }

    // Issue the plugin event to trigger reprojection on the render thread (new pattern)
    GL.IssuePluginEvent(NativeReprojection.RenderEventFuncPtr, 0); // eventId, not used in plugin
  }

  void Start()
  {
    // Create offscreen RenderTexture
    if (_camera != null)
    {
      _offscreenRT = new RenderTexture(Screen.width, Screen.height, 24, RenderTextureFormat.ARGB32);
      _offscreenRT.Create();
      _camera.targetTexture = _offscreenRT;

      // --- Send projection matrix to plugin ONCE ---
      // Send the camera's raw projection matrix. We will convert it in the plugin.
      Matrix4x4 projMatrix = _camera.projectionMatrix;
      float[] projMatrixArray = new float[16];
      for (int i = 0; i < 16; i++)
      {
          projMatrixArray[i] = projMatrix[i];
      }
      NativeReprojection.sai_set_projection_matrix(projMatrixArray);
    }
    // Remove VIO initialization from Start()
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

  private float _lastDepth = 1.0f; // Default fallback depth
  private bool _depthTestMode = false;

  void Update()
  {
    if (Input.GetKeyDown(KeyCode.Space))
    {
      _depthTestMode = !_depthTestMode;
      Debug.Log($"[NativeReprojectionDemo] Depth test mode: {_depthTestMode}");
    }
    if (_depthTestMode)
    {
      if (Input.GetKeyDown(KeyCode.UpArrow))
      {
        _lastDepth += 1f;
        Debug.Log($"[NativeReprojectionDemo] Depth increased: {_lastDepth:F2}");
      }
      if (Input.GetKeyDown(KeyCode.DownArrow))
      {
        _lastDepth = Mathf.Max(0.1f, _lastDepth - 1f);
        Debug.Log($"[NativeReprojectionDemo] Depth decreased: {_lastDepth:F2}");
      }
    }
    NativeReprojection.sai_set_rendered_depth(_lastDepth);
  }

  void LateUpdate()
  {
    if (_depthTestMode)
      return; // Don't send orientation updates in test mode
    if (Vio.Output is null)
      return; // No VIO output, nothing to update
    var unityQuat = Vio.Output.Pose._orientation;
    NativeReprojection.sai_set_rendered_orientation(unityQuat.x, unityQuat.y, unityQuat.z, unityQuat.w);
  }
}
