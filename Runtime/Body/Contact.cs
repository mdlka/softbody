using UnityEngine;

namespace SoftbodyPhysics
{
    public record Contact(int Index, Vector3 EntryPoint, Vector3 SurfaceNormal);
}