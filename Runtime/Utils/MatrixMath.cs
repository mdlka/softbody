using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using UnityEngine;

namespace SoftbodyPhysics
{
    public static class MatrixMath
    {
        public static void PolarDecompositionStable(Matrix4x4 A, float tolerance, out Matrix4x4 R)
        {
            Matrix4x4 at = A.transpose;
            
            float aone =  OneNorm(A);
            float ainf =  InfNorm(A);
            float eone;
            
            var madjTt = new Matrix4x4();
            var et = new Matrix4x4();

            const float eps = 1.0e-15f;

            do
            {
                madjTt.SetRow(0, at.GetRow(1).Cross(at.GetRow(2)));
                madjTt.SetRow(1, at.GetRow(2).Cross(at.GetRow(0)));
                madjTt.SetRow(2, at.GetRow(0).Cross(at.GetRow(1)));

                float det = at.m00 * madjTt.m00 + at.m01 * madjTt.m01 + at.m02 * madjTt.m02;

                if (Math.Abs(det) < eps)
                {
                    int index = int.MaxValue;
                    
                    for (int i = 0; i < 3; i++)
                    {
                        if (madjTt.GetRow(i).sqrMagnitude <= eps) 
                            continue;
                        
                        index = i;
                        break;
                    }

                    if (index == int.MaxValue)
                    {
                        R = Matrix4x4.identity;
                        return;
                    }

                    at.SetRow(index, at.GetRow((index + 1) % 3).Cross(at.GetRow((index + 2) % 3)));
                    madjTt.SetRow((index + 1) % 3, at.GetRow((index + 2) % 3).Cross(at.GetRow(index)));
                    madjTt.SetRow((index + 2) % 3, at.GetRow(index).Cross(at.GetRow((index + 1) % 3)));
                    var a2 = at.transpose;

                    aone = OneNorm(a2);
                    ainf = InfNorm(a2);

                    det = at.m00 * madjTt.m00 + at.m01 * madjTt.m01 + at.m02 * madjTt.m02;
                }

                float madjTone = OneNorm(madjTt);
                float madjTinf = InfNorm(madjTt);

                float gamma = Mathf.Sqrt(Mathf.Sqrt(madjTone * madjTinf / (aone * ainf)) / Math.Abs(det));

                float g1 = gamma * 0.5f;
                float g2 = 0.5f / (gamma * det);

                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        et[i, j] = at[i, j];
                        at[i, j] = g1 * at[i, j] + g2 * madjTt[i, j];
                        et[i, j] -= at[i, j];
                    }
                }

                eone = OneNorm(et);
                aone = OneNorm(at);
                ainf = InfNorm(at);
            }
            while (eone > aone * tolerance);
            
            R = at.transpose;
        }
        
        public static float OneNorm(Matrix4x4 A)
        {
            float sum1 = Math.Abs(A.m00) + Math.Abs(A.m10) + Math.Abs(A.m20);
            float sum2 = Math.Abs(A.m01) + Math.Abs(A.m11) + Math.Abs(A.m21);
            float sum3 = Math.Abs(A.m02) + Math.Abs(A.m12) + Math.Abs(A.m22);

            return Math.Max(sum1, Math.Max(sum2, sum3));
        }
        
        public static float InfNorm(Matrix4x4 A)
        {
            float sum1 = Math.Abs(A.m00) + Math.Abs(A.m01) + Math.Abs(A.m02);
            float sum2 = Math.Abs(A.m10) + Math.Abs(A.m11) + Math.Abs(A.m12);
            float sum3 = Math.Abs(A.m20) + Math.Abs(A.m21) + Math.Abs(A.m22);

            return Math.Max(sum1, Math.Max(sum2, sum3));
        }
    }
    
    public static class Vector4Extensions 
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector4 Cross(this Vector4 vector, Vector4 other)
        {
            var cross = Vector3.Cross(vector.ToVector3(), other.ToVector3());
            return new Vector4(cross.x, cross.y, cross.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 ToVector3(this Vector4 vector)
        {
            return new Vector3(vector.x, vector.y, vector.z);
        }
    }

    public static class MeshMath
    {
        public static float ComputeVolume(Vector3[] vertices, int[] triangles)
        {
            float volume = 0f;

            for (int i = 0; i < triangles.Length; i += 3)
            {
                var v0 = vertices[triangles[i]];
                var v1 = vertices[triangles[i + 1]];
                var v2 = vertices[triangles[i + 2]];

                var normal = Vector3.Cross(v1 - v0, v2 - v0);
                float area = normal.magnitude * 0.5f;

                volume += Vector3.Dot((v0 + v1 + v2) / 3f, normal.normalized) * area / 3f;
            }

            return Mathf.Abs(volume);
        }
    }
}