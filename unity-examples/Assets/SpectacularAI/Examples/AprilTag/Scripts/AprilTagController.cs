using System.IO;
using System.Text;
using UnityEngine;
using SpectacularAI.DepthAI;
using UnityEditor;

namespace SpectacularAI.Examples.AprilTag
{
    public class AprilTagController : MonoBehaviour
    {
        [SerializeField]
        private Vio _vio;

        string SerializeAprilTags()
        {
            AprilTag[] aprilTags = FindObjectsOfType<AprilTag>();

            StringBuilder sb = new StringBuilder();
            sb.AppendLine("[");
            for (int i = 0; i < aprilTags.Length; ++i)
            {
                AprilTag tag = aprilTags[i];
                sb.Append(tag.ToJson());
                if (i < aprilTags.Length - 1)
                {
                    sb.Append(",");
                }
                sb.Append("\n");
            }
            sb.AppendLine("]");

            string json = sb.ToString();
            string filePath = Path.Combine(Application.persistentDataPath, "tags.json");
            File.WriteAllText(filePath, json);
            Debug.Log("April Tag file written to: " + filePath);

            _vio.AprilTagPath = filePath;
            _vio.enabled = true;
            return filePath;
        }

        void Start()
        {
            if (Vio.SlamConfig == null)
            {
                SerializeAprilTags();
            }

        }
    
    #if UNITY_EDITOR
        [CustomEditor(typeof(AprilTagController))]
        public class AprilTagControllerEditor : Editor
        {
            public override void OnInspectorGUI()
            {
                DrawDefaultInspector();

                AprilTagController controller = (AprilTagController)target;
                if (GUILayout.Button("Serialize April Tags"))
                {
                    string jsonPath = controller.SerializeAprilTags();
                    EditorUtility.RevealInFinder(jsonPath);
                }
            }
        }
    #endif
    }
}
