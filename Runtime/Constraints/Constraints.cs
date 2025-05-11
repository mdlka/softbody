using UnityEngine;

namespace SoftbodyPhysics
{
    public static class Constraints
    {
        private const float Epsilon = 1e-6f;
        
        public static void ApplyCollisionConstraint(ISoftbody body, float restCollisionDistance, float collisionConstraintStiffness)
        {
            foreach (var contact in body.Contacts)
            {
                float delta = Vector3.Dot(body.CenterPosition + body.Particles[contact.Index].Predicted - contact.EntryPoint,
                    contact.SurfaceNormal) - restCollisionDistance - body.ParticlesRadius;

                if (delta >= 0f) 
                    continue;

                body.Particles[contact.Index].Predicted -= contact.SurfaceNormal * delta * collisionConstraintStiffness 
                                                           * body.Particles[contact.Index].InvMass;
            }
        }

        public static void PrepareBalloonsConstraint(ISoftbody body)
        {
            body.UpdateRestVolume(MeshMath.ComputeVolume(body.Mesh.vertices, body.Mesh.triangles));
        }

        public static void ApplyBalloonsConstraint(ISoftbody body, float balloonsConstraintsStiffness, float pressureStiffness)
        {
            float predictedVolume = MeshMath.ComputePredictedVolume(body.Particles, body.ParticlesTriangles);

            float targetVolume = pressureStiffness * body.RestVolume;
            float constraint = predictedVolume - targetVolume;

            if (Mathf.Abs(constraint) < Epsilon || body.Particles.Count == 0 || body.ParticlesTriangles.Count == 0)
                return;

            var totalGradients = new Vector3[body.Particles.Count];

            for (int i = 0; i < body.ParticlesTriangles.Count; i += 3)
            {
                int i0 = body.ParticlesTriangles[i];
                int i1 = body.ParticlesTriangles[i + 1];
                int i2 = body.ParticlesTriangles[i + 2];

                var v0 = body.Particles[i0].Predicted;
                var v1 = body.Particles[i1].Predicted;
                var v2 = body.Particles[i2].Predicted;

                var grad0 = Vector3.Cross(v1, v2) / 6.0f;
                var grad1 = Vector3.Cross(v2, v0) / 6.0f;
                var grad2 = Vector3.Cross(v0, v1) / 6.0f;

                totalGradients[i0] += grad0;
                totalGradients[i1] += grad1;
                totalGradients[i2] += grad2;
            }

            float sumSqGradInvMass = 0;
            
            for (int i = 0; i < body.Particles.Count; i++)
                sumSqGradInvMass += totalGradients[i].sqrMagnitude * body.Particles[i].InvMass;

            if (sumSqGradInvMass < Epsilon)
                return;

            float lambda = -constraint / sumSqGradInvMass;
            float scale = lambda * balloonsConstraintsStiffness;

            for (int i = 0; i < body.Particles.Count; i++)
                body.Particles[i].Predicted += scale * totalGradients[i] * body.Particles[i].InvMass;
        }

        public static void PrepareShapeMatchingConstraint(ISoftbody body)
        {
            float wsum = 0;
            var restCm = Vector3.zero;
            
            foreach (var particle in body.Particles)
            {
                restCm += particle.Position * particle.Mass;
                wsum += particle.Mass;
            }

            restCm /= wsum;
            var restMatrix = Matrix4x4.zero;
            var restPositions = new Vector3[body.Particles.Count];

            for (int i = 0; i < body.Particles.Count; i++)
            {
                var particle = body.Particles[i];
                var q = particle.Position - restCm;
                restPositions[i] = q;

                restMatrix[0, 0] += particle.Mass * q.x * q.x;
                restMatrix[0, 1] += particle.Mass * q.x * q.y;
                restMatrix[0, 2] += particle.Mass * q.x * q.z;

                restMatrix[1, 0] += particle.Mass * q.y * q.x;
                restMatrix[1, 1] += particle.Mass * q.y * q.y;
                restMatrix[1, 2] += particle.Mass * q.y * q.z;

                restMatrix[2, 0] += particle.Mass * q.z * q.x;
                restMatrix[2, 1] += particle.Mass * q.z * q.y;
                restMatrix[2, 2] += particle.Mass * q.z * q.z;
            }

            body.UpdateRest(restMatrix.inverse, restPositions);
        }
        
        public static void ApplyShapeMatchingConstraint(ISoftbody body, float stiffness)
        {
            var cm = Vector3.zero;
            float wsum = 0f;

            foreach (var particle in body.Particles)
            {
                cm += particle.Predicted * particle.Mass;
                wsum += particle.Mass;
            }

            cm /= wsum;
            var A = Matrix4x4.zero;

            for (int i = 0; i < body.Particles.Count; i++)
            {
                Vector3 q = body.RestParticlesPositions[i];
                Vector3 p = body.Particles[i].Position - cm;

                A[0, 0] += body.Particles[i].Mass * p.x * q.x;
                A[0, 1] += body.Particles[i].Mass * p.x * q.y;
                A[0, 2] += body.Particles[i].Mass * p.x * q.z;

                A[1, 0] += body.Particles[i].Mass * p.y * q.x;
                A[1, 1] += body.Particles[i].Mass * p.y * q.y;
                A[1, 2] += body.Particles[i].Mass * p.y * q.z;

                A[2, 0] += body.Particles[i].Mass * p.z * q.x;
                A[2, 1] += body.Particles[i].Mass * p.z * q.y;
                A[2, 2] += body.Particles[i].Mass * p.z * q.z;
            }

            A *= body.InvRestMatrix;
            
            MatrixMath.PolarDecompositionStable(A, Epsilon, out Matrix4x4 R);

            for (int i = 0; i < body.Particles.Count; i++)
            {
                var goal = cm + (R * body.RestParticlesPositions[i]).ToVector3();
                body.Particles[i].Predicted += (goal - body.Particles[i].Predicted) * stiffness;
            }
        }
    }
}