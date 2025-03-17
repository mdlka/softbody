using UnityEngine;

namespace SoftbodyPhysics
{
    public class Particle
    {
        public Vector3 Position;
        public Vector3 Predicted;
        public Vector3 Velocity;
        public float Mass;
        public float InvMass;
    }
}