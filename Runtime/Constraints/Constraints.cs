﻿using UnityEngine;

namespace SoftbodyPhysics
{
    public static class Constraints
    {
        public static void ApplyCollisionConstraint(ISoftbody body, float restCollisionDistance, float collisionConstraintStiffness)
        {
            foreach (var contact in body.Contacts)
            {
                float delta = Vector3.Dot(body.CenterPosition + body.Particles[contact.Index].Predicted - contact.EntryPoint,
                    contact.SurfaceNormal) - restCollisionDistance - body.ParticlesRadius;

                if (delta > 0f) 
                    return;

                body.Particles[contact.Index].Predicted -= contact.SurfaceNormal * delta * collisionConstraintStiffness;
            }
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
            
            MatrixMath.PolarDecompositionStable(A, 1e-6f, out Matrix4x4 R);

            for (int i = 0; i < body.Particles.Count; i++)
            {
                var goal = cm + (R * body.RestParticlesPositions[i]).ToVector3();
                body.Particles[i].Predicted += (goal - body.Particles[i].Predicted) * stiffness;
            }
        }
    }
}