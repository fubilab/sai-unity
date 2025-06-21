using UnityEngine;
using SpectacularAI.DepthAI;

public class HelloDepthAI_NativeReprojection : MonoBehaviour
{
  public GameObject CameraRig;
  public GameObject Content;
  public int CameraId = 0;
  public int TargetFps = 60;

  private NativeReprojectionDemo _nativeReprojectionDemo;
  private PoseProvider _poseProvider;

  void Awake()
  {
    if (CameraRig == null) CameraRig = Camera.main?.gameObject;
    if (Content == null) Content = GameObject.Find("Content");
    _poseProvider = CameraRig.GetComponent<PoseProvider>();
    if (_poseProvider == null)
      _poseProvider = CameraRig.AddComponent<PoseProvider>();
    _nativeReprojectionDemo = CameraRig.AddComponent<NativeReprojectionDemo>();
    _nativeReprojectionDemo.CameraId = CameraId;
    _nativeReprojectionDemo.TargetFps = TargetFps;
  }
}
