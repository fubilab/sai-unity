using System;
using System.Runtime.InteropServices;
using UnityEngine;
using SpectacularAI.DepthAI;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

public static class NativeReprojection
{
  [DllImport("SpectacularAIPlugin")]
  public static extern void sai_set_rendered_orientation(double x, double y, double z, double w);

  [DllImport("SpectacularAIPlugin")]
  public static extern void sai_set_vio_output_handle(IntPtr vioOutputHandle, int cameraId);

  [DllImport("SpectacularAIPlugin")]
  public static extern void sai_start_reprojection_thread(int targetFps);

  [DllImport("SpectacularAIPlugin")]
  public static extern void sai_stop_reprojection_thread();

  [DllImport("SpectacularAIPlugin")]
  public static extern void sai_set_rendered_texture(uint textureId);
}

public class NativeReprojectionDemo : MonoBehaviour
{
  public int CameraId = 0;
  public int TargetFps = 60;
  private bool _started = false;
  private Camera _camera;
  private RenderTexture _offscreenRT;

  void Awake()
  {
    _camera = GetComponent<Camera>();
    if (_camera == null)
      _camera = Camera.main;
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
    if (_started)
      NativeReprojection.sai_stop_reprojection_thread();
  }

  void OnEndCameraRendering(ScriptableRenderContext ctx, Camera cam)
  {
    if (cam.cameraType != CameraType.Game) return;
    var target = cam.activeTexture ?? (RenderTexture)cam.targetTexture;
    if (target == null) return;
    var texPtr = target.GetNativeTexturePtr();
    if (texPtr != IntPtr.Zero)
      NativeReprojection.sai_set_rendered_texture((uint)texPtr.ToInt64());
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
    // Get VIO output handle from DepthAI session
    var vio = FindObjectOfType<VioOutputProvider>();
    if (vio == null)
    {
      Debug.LogError("No VioOutputProvider found in scene");
      return;
    }
    IntPtr vioHandle = vio.GetNativeHandle();
    NativeReprojection.sai_set_vio_output_handle(vioHandle, CameraId);
    NativeReprojection.sai_start_reprojection_thread(TargetFps);
    _started = true;
  }

  void OnDestroy()
  {
    if (_started)
      NativeReprojection.sai_stop_reprojection_thread();
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
    // Get the current orientation in SAI coordinates
    var poseProvider = FindObjectOfType<PoseProvider>();
    if (poseProvider == null) return;
    var orientationUnity = poseProvider.transform.rotation;
    // Convert Unity quaternion to SAI (assume user provides conversion if needed)
    // For this demo, assume same coordinate system
    NativeReprojection.sai_set_rendered_orientation(
        orientationUnity.x, orientationUnity.y, orientationUnity.z, orientationUnity.w);
  }
}
