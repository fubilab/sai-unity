using System;
using System.Runtime.InteropServices;
using UnityEngine;
using SpectacularAI;
using SpectacularAI.DepthAI;
using SpectacularAI.Native;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

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
}

public class NativeReprojectionDemo : MonoBehaviour
{
  public int CameraId = 0;
  public int TargetFps = 60;
  private UnityEngine.Camera _camera;
  private RenderTexture _offscreenRT;

  // Permutation struct for axis order and sign
  private struct Permutation
  {
    public int[] order; // e.g. [0,1,2] for x,y,z
    public int[] sign;  // e.g. [1,-1,1] for x,-y,z
    public string name;
    public Permutation(int[] o, int[] s, string n) { order = o; sign = s; name = n; }
  }
  private static readonly Permutation[] Permutations = new Permutation[] {
    new Permutation(new[]{0,1,2}, new[]{1,1,1}, "x,y,z"),
    new Permutation(new[]{0,2,1}, new[]{1,1,1}, "x,z,y"),
    new Permutation(new[]{1,0,2}, new[]{1,1,1}, "y,x,z"),
    new Permutation(new[]{1,2,0}, new[]{1,1,1}, "y,z,x"),
    new Permutation(new[]{2,0,1}, new[]{1,1,1}, "z,x,y"),
    new Permutation(new[]{2,1,0}, new[]{1,1,1}, "z,y,x"),
    // All with sign flips
    new Permutation(new[]{0,1,2}, new[]{-1,1,1}, "-x,y,z"),
    new Permutation(new[]{0,1,2}, new[]{1,-1,1}, "x,-y,z"),
    new Permutation(new[]{0,1,2}, new[]{1,1,-1}, "x,y,-z"),
    new Permutation(new[]{0,1,2}, new[]{-1,-1,1}, "-x,-y,z"),
    new Permutation(new[]{0,1,2}, new[]{-1,1,-1}, "-x,y,-z"),
    new Permutation(new[]{0,1,2}, new[]{1,-1,-1}, "x,-y,-z"),
    new Permutation(new[]{0,1,2}, new[]{-1,-1,-1}, "-x,-y,-z"),
    // Add more as needed
  };
  private int _permIndex = 0;

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

  void Update()
  {
    if (Input.GetKeyDown(KeyCode.Space))
    {
      _permIndex = (_permIndex + 1) % Permutations.Length;
      Debug.Log($"[NativeReprojectionDemo] Permutation changed to {_permIndex}: {Permutations[_permIndex].name}");
    }
  }

  void LateUpdate()
  {
    // Update pose for timewarp
    var unityQuat = Vio.Output.Pose._orientation;
    // var eulerUnity = unityQuat.eulerAngles;
    // // Apply permutation to euler angles
    // var perm = Permutations[_permIndex];
    // float[] e = { eulerUnity.x, eulerUnity.y, eulerUnity.z };
    // float[] pe = new float[3];
    // for (int i = 0; i < 3; ++i) pe[i] = e[perm.order[i]] * perm.sign[i];
    // Debug.Log($"[NativeReprojectionDemo] Unity Euler sent to plugin (perm {perm.name}): {pe[0]:F3}, {pe[1]:F3}, {pe[2]:F3}");
    // // Optionally, convert back to quaternion and send to plugin
    // var permQuat = UnityEngine.Quaternion.Euler(pe[0], pe[1], pe[2]);
    NativeReprojection.sai_set_rendered_orientation(unityQuat.x, unityQuat.y, unityQuat.z, unityQuat.w);
  }
}
