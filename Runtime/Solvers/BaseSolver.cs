using UnityEngine;

namespace SoftbodyPhysics
{
    public abstract class BaseSolver : ScriptableObject
    {
        public abstract void AddBody(ISoftbody body);
        
        public abstract void ApplyExternalForces(float deltaTime);
        public abstract void DampVelocity(float deltaTime);
        public abstract void EstimatesPositions(float deltaTime);

        public abstract void GenerateCollisionConstraints();
        public abstract void ProjectConstraints();

        public abstract void UpdatePositions(float deltaTime);
        public abstract void UpdateVelocity(float deltaTime);

        public abstract void UpdateBodies();
    }
}