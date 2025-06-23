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
  public static extern void sai_set_rendered_depth(float depth);
}

public class NativeReprojectionDemo : MonoBehaviour
{
  public int CameraId = 0;
  private UnityEngine.Camera _camera;
  private RenderTexture _offscreenRT;
  private CameraDepthDetector _depthDetector;

  void Awake()
  {
    // Disable this component when running in the Unity Editor since native rendering doesn't work there
    #if UNITY_EDITOR
    Debug.LogWarning("NativeReprojectionDemo: Disabled in Unity Editor - native rendering not supported in editor");
    enabled = false;
    return;
    #endif
    
    _camera = GetComponent<UnityEngine.Camera>();
    if (_camera == null)
      _camera = UnityEngine.Camera.main;
    if (_camera == null)
      Debug.LogError("NativeReprojectionDemo: No Camera found!");
      
    // Get or add the depth detector component
    _depthDetector = _camera.GetComponent<CameraDepthDetector>();
    if (_depthDetector == null)
    {
      _depthDetector = _camera.gameObject.AddComponent<CameraDepthDetector>();
      Debug.Log("NativeReprojectionDemo: Added CameraDepthDetector component");
    }
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

  private float _lastDepth = 1.0f; // Default fallback depth
  private bool _depthTestMode = false;

  void Update()
  {
    // Get depth from the depth detector if not in test mode
    if (!_depthTestMode && _depthDetector != null)
    {
      _lastDepth = _depthDetector.LastDetectedDepth;
    }
    
    if (Input.GetKeyDown(KeyCode.D))
    {
      _depthTestMode = !_depthTestMode;
      PoseProvider.DepthTestMode = _depthTestMode;
      Debug.Log($"[NativeReprojectionDemo] Depth test mode: {_depthTestMode}");
      
      // Enable/disable debug sphere based on test mode
      if (_depthDetector != null)
      {
        _depthDetector.enableDebugSphere = !_depthTestMode;
      }
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

  void OnGUI()
  {
    #if UNITY_EDITOR
    // Show a message in the editor that this component is disabled
    if (!enabled)
    {
      GUI.Box(new Rect(10, 10, 400, 60), "");
      GUI.Label(new Rect(15, 15, 390, 20), "NativeReprojectionDemo: DISABLED IN EDITOR");
      GUI.Label(new Rect(15, 35, 390, 20), "Native rendering only works in builds. CameraDepthDetector still active.");
      return;
    }
    #endif
    
    // Display depth value for debugging
    GUI.Label(new Rect(10, 10, 300, 20), $"Depth: {_lastDepth:F2}m");
    GUI.Label(new Rect(10, 30, 300, 20), $"Depth Test Mode: {(_depthTestMode ? "ON" : "OFF")}");
    if (_depthDetector != null)
    {
      GUI.Label(new Rect(10, 50, 300, 20), $"Closest Object: {(_depthDetector.LastClosestObject?.name ?? "None")}");
    }
    if (_depthTestMode)
    {
      GUI.Label(new Rect(10, 70, 300, 20), "Use Up/Down arrows to adjust depth");
    }
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
