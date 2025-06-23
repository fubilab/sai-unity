using UnityEngine;

public class CameraDepthDetector : MonoBehaviour
{
    [Header("Debug Settings")]
    public bool enableDebugSphere = true;
    public bool enableDebugLogging = false;
    
    private UnityEngine.Camera _camera;
    private GameObject _debugSphere = null;
    private GameObject _lastClosestObject = null;
    private float _lastDetectedDepth = 1.0f;
    
    public float LastDetectedDepth => _lastDetectedDepth;
    public GameObject LastClosestObject => _lastClosestObject;
    
    void Awake()
    {
        _camera = GetComponent<UnityEngine.Camera>();
        if (_camera == null)
        {
            Debug.LogError("CameraDepthDetector: No Camera component found on this GameObject!");
            enabled = false;
        }
    }
    
    void Update()
    {
        DetectClosestObjectDepth();
    }
    
    void OnDestroy()
    {
        CleanupDebugSphere();
    }
    
    void OnDisable()
    {
        CleanupDebugSphere();
    }
    
    public void DetectClosestObjectDepth()
    {
        if (_camera == null) return;
        
        // Clean up previous debug sphere
        CleanupDebugSphere();
        
        float closestDistance = float.MaxValue;
        GameObject closestObject = null;
        GameObject[] allObjects = FindObjectsByType<GameObject>(FindObjectsSortMode.None);
        
        int totalObjects = 0;
        int objectsInFrustum = 0;
        
        foreach (GameObject obj in allObjects)
        {
            totalObjects++;
            
            // Skip inactive objects and objects without renderers
            if (!obj.activeInHierarchy || obj.GetComponent<Renderer>() == null)
                continue;
                
            // Skip debug spheres we created
            if (obj.name == "DepthDebugSphere")
                continue;
                
            // Skip the camera's own GameObject
            if (obj == _camera.gameObject)
                continue;
                
            Renderer renderer = obj.GetComponent<Renderer>();
            if (renderer == null) continue;
            
            // Check if object is within camera frustum
            if (!IsObjectInCameraFrustum(renderer))
            {
                if (enableDebugLogging)
                    Debug.Log($"[CameraDepthDetector] Object '{obj.name}' NOT in frustum. Bounds: {renderer.bounds}");
                continue;
            }
            
            objectsInFrustum++;
            float distance = Vector3.Distance(_camera.transform.position, obj.transform.position);
            
            if (enableDebugLogging)
                Debug.Log($"[CameraDepthDetector] Object '{obj.name}' IS in frustum. Distance: {distance:F2}m");
                
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestObject = obj;
            }
        }
        
        if (enableDebugLogging)
            Debug.Log($"[CameraDepthDetector] Scan complete: Total={totalObjects}, InFrustum={objectsInFrustum}");
        
        // Update depth and highlight closest object
        if (closestDistance != float.MaxValue)
        {
            _lastDetectedDepth = Mathf.Max(0.1f, closestDistance);
            _lastClosestObject = closestObject;
            
            if (enableDebugSphere)
                CreateDebugSphere(closestObject);
                
            if (enableDebugLogging)
                Debug.Log($"[CameraDepthDetector] Closest object: {closestObject.name} at {_lastDetectedDepth:F2}m");
        }
        else
        {
            _lastClosestObject = null;
            if (enableDebugLogging)
                Debug.Log("[CameraDepthDetector] No objects found in frustum!");
        }
    }
    
    private bool IsObjectInCameraFrustum(Renderer renderer)
    {
        if (renderer == null || _camera == null)
            return false;
            
        // Use Unity's built-in frustum culling to check if the object's bounds intersect with camera frustum
        Plane[] frustumPlanes = GeometryUtility.CalculateFrustumPlanes(_camera);
        return GeometryUtility.TestPlanesAABB(frustumPlanes, renderer.bounds);
    }
    
    private void CreateDebugSphere(GameObject targetObj)
    {
        // Create a small bright sphere
        _debugSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        _debugSphere.name = "DepthDebugSphere";
        _debugSphere.transform.localScale = Vector3.one * 0.15f; // Slightly larger for visibility
        
        // Make it bright yellow - use the default material and modify its color
        Renderer sphereRenderer = _debugSphere.GetComponent<Renderer>();
        
        // Try to find a suitable shader, fallback to the default material's shader
        Shader shader = Shader.Find("Universal Render Pipeline/Unlit") ?? 
                       Shader.Find("Unlit/Color") ?? 
                       Shader.Find("Mobile/Unlit (Supports Lightmap)") ?? 
                       sphereRenderer.material.shader;
        
        if (shader != null)
        {
            Material debugMaterial = new Material(shader);
            debugMaterial.color = Color.yellow;
            
            // Try to set the main color property if it exists
            if (debugMaterial.HasProperty("_BaseColor"))
                debugMaterial.SetColor("_BaseColor", Color.yellow);
            if (debugMaterial.HasProperty("_Color"))
                debugMaterial.SetColor("_Color", Color.yellow);
                
            sphereRenderer.material = debugMaterial;
        }
        else
        {
            // Fallback: just modify the existing material color
            sphereRenderer.material.color = Color.yellow;
            Debug.LogWarning("[CameraDepthDetector] Could not find suitable shader, using default material");
        }
        
        // Remove collider so it doesn't interfere
        Collider sphereCollider = _debugSphere.GetComponent<Collider>();
        if (sphereCollider != null)
            DestroyImmediate(sphereCollider);
        
        // Position at the closest point on the target object to the camera
        Vector3 closestPoint = FindClosestPointToCamera(targetObj);
        _debugSphere.transform.position = closestPoint;
    }
    
    private Vector3 FindClosestPointToCamera(GameObject obj)
    {
        Renderer renderer = obj.GetComponent<Renderer>();
        if (renderer == null)
            return obj.transform.position;
        
        // Get the closest point on the object's bounds to the camera
        Vector3 cameraPos = _camera.transform.position;
        Vector3 closestPoint = renderer.bounds.ClosestPoint(cameraPos);
        
        return closestPoint;
    }
    
    private void CleanupDebugSphere()
    {
        if (_debugSphere != null)
        {
            DestroyImmediate(_debugSphere);
            _debugSphere = null;
        }
    }
    
    void OnGUI()
    {
        if (!enableDebugLogging) return;
        
        // Display debug info in the corner
        GUI.Box(new Rect(Screen.width - 250, 10, 240, 80), "Depth Detector Debug");
        GUI.Label(new Rect(Screen.width - 240, 30, 200, 20), $"Depth: {_lastDetectedDepth:F2}m");
        GUI.Label(new Rect(Screen.width - 240, 50, 200, 20), $"Closest: {(_lastClosestObject?.name ?? "None")}");
        GUI.Label(new Rect(Screen.width - 240, 70, 200, 20), $"Debug Sphere: {(enableDebugSphere ? "ON" : "OFF")}");
    }
}
