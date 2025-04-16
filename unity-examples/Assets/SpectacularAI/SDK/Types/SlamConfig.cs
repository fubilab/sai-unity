using System;
using System.IO;
using UnityEngine;

namespace SpectacularAI
{
    [Serializable]
    public class SlamConfig 
    {
        public float[] slamToUnity;

        public static SlamConfig ReadFromFile(string path)
        {
            var json = File.ReadAllText(path);
            SlamConfig config = JsonUtility.FromJson<SlamConfig>(json);
            Debug.Log(config.slamToUnity);
            return config;
        }

        public static void WriteToFile(string path, SlamConfig config)
        {
            var json = JsonUtility.ToJson(config);
            File.WriteAllText(path, json);
        }

        private Matrix4d ToMatrix4d(float[] matrix)
        {
            Matrix4d s;
            s.m00 = matrix[0];
            s.m01 = matrix[1];
            s.m02 = matrix[2];
            s.m03 = matrix[3];
            s.m10 = matrix[4];
            s.m11 = matrix[5];
            s.m12 = matrix[6];
            s.m13 = matrix[7];
            s.m20 = matrix[8];
            s.m21 = matrix[9];
            s.m22 = matrix[10];
            s.m23 = matrix[11];
            s.m30 = matrix[12];
            s.m31 = matrix[13];
            s.m32 = matrix[14];
            s.m33 = matrix[15];
            return s;
        }

        public Matrix4x4 SlamWorldToUnityWorldMatrix
        {
            get
            {
                Matrix4d matrix = ToMatrix4d(slamToUnity);
                return Utility.TransformWorldToWorldMatrixToUnity(matrix);
            }
        }

    }
}