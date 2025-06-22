using System;
using System.Runtime.InteropServices;
using UnityEngine;
using SpectacularAI;
using SpectacularAI.DepthAI;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

public class NativeReprojectionTwoCameraDemo : MonoBehaviour
{
  public int CameraId = 0;
  public int MainCameraFps = 30;
  public int DisplayCameraFps = 60;
  private bool _started = false;
  private UnityEngine.Camera _mainCamera;
  private UnityEngine.Camera _displayCamera;
  private RenderTexture _offscreenRT;
  private float _mainCameraTimer = 0f;
  private float _mainCameraInterval;

  void Awake()
  {
    var cameras = GetComponentsInChildren<UnityEngine.Camera>();
    if (cameras.Length < 2)
    {
      Debug.LogError("NativeReprojectionTwoCameraDemo: Please add two child cameras (main and display)");
      return;
    }
    _mainCamera = cameras[0];
    _displayCamera = cameras[1];
    _mainCameraInterval = 1.0f / MainCameraFps;
  }

  void OnEnable()
  {
    RenderPipelineManager.endCameraRendering += OnEndCameraRendering;
  }

  void OnDisable()
  {
    RenderPipelineManager.endCameraRendering -= OnEndCameraRendering;
  }

  void Start()
  {
    // Create offscreen RenderTexture
    if (_mainCamera != null)
    {
      _offscreenRT = new RenderTexture(Screen.width, Screen.height, 24, RenderTextureFormat.ARGB32);
      _offscreenRT.Create();
      _mainCamera.targetTexture = _offscreenRT;
    }
    if (_displayCamera != null)
    {
      _displayCamera.targetTexture = null; // Display camera renders to screen
    }
    // Get VIO output handle from DepthAI session
    var vio = Vio.Output;
    IntPtr vioHandle = vio.GetNativeHandle();
    NativeReprojection.sai_set_vio_output_handle(vioHandle, CameraId);
    _started = true;
  }

  void OnDestroy()
  {
    if (_offscreenRT != null)
    {
      if (_mainCamera != null && _mainCamera.targetTexture == _offscreenRT)
        _mainCamera.targetTexture = null;
      _offscreenRT.Release();
      Destroy(_offscreenRT);
    }
  }

  void Update()
  {
    // Manually render the main camera at MainCameraFps
    _mainCameraTimer += Time.deltaTime;
    if (_mainCameraTimer >= _mainCameraInterval)
    {
      _mainCamera.Render();
      _mainCameraTimer = 0f;
    }
    // Update pose for timewarp
    var poseProvider = FindObjectOfType<PoseProvider>();
    if (poseProvider == null) return;
    var orientationUnity = Utility.TransformCameraToWorldQuaternionToSpectacularAI(poseProvider.transform.rotation);
    NativeReprojection.sai_set_rendered_orientation(
        orientationUnity.x, orientationUnity.y, orientationUnity.z, orientationUnity.w);
  }

  void OnEndCameraRendering(ScriptableRenderContext ctx, UnityEngine.Camera cam)
  {
    // Only trigger plugin event for the display camera
    var texPtr = _offscreenRT.GetNativeTexturePtr();
    if (texPtr != IntPtr.Zero)
      NativeReprojection.sai_set_rendered_texture((uint)texPtr.ToInt64());
    if (cam != _displayCamera) return;
    GL.IssuePluginEvent(
        Marshal.GetFunctionPointerForDelegate(
            (Action<int>)NativeReprojection.sai_reprojection_plugin_event),
        0
    );
  }
}
