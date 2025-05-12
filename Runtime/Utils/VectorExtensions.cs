using System.Runtime.CompilerServices;
using UnityEngine;

namespace SoftbodyPhysics
{
    public static class VectorExtensions
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 With(this Vector3 vector, float? x = null, float? y = null, float? z = null)
        {
            return new Vector3
            {
                x = x ?? vector.x,
                y = y ?? vector.y,
                z = z ?? vector.z
            };
        }
    }
}