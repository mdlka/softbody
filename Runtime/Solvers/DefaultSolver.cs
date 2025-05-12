using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace SoftbodyPhysics
{
    public class DefaultSolver : BaseSolver
    {
        private readonly HashSet<ISoftbody> _bodies = new();
        
        [Header("External Forces")]
        [SerializeField] private Vector3 _gravity;

        [Header("Velocity")] 
        [SerializeField] private float _damping;
        [SerializeField] private float _restitution;
        [SerializeField] private float _friction;
        
        [Header("Collision Constraint")]
        [SerializeField] private float _restCollisionDistance;
        [SerializeField, Range(0, 1f)] private float _collisionConstraintStiffness;
        
        [Header("Shape Matching Constraint")]
        [SerializeField, Range(0, 1f)] private float _shapeMatchingConstraintStiffness;

        [Header("Balloons Constraint")] 
        [SerializeField, Range(0, 1f)] private float _balloonsConstraintsStiffness;
        [SerializeField, Range(0, 1f)] private float _pressureStiffness;
        
        public override void AddBody(ISoftbody body)
        {
            if (!_bodies.Add(body))
                throw new InvalidOperationException();

            body.Initialize();
            Constraints.PrepareShapeMatchingConstraint(body);
            Constraints.PrepareBalloonsConstraint(body);
        }

        public override void ApplyExternalForces(float deltaTime)
        {
            foreach (var particle in _bodies.SelectMany(body => body.Particles))
                particle.Velocity += deltaTime * particle.InvMass * _gravity;
        }

        public override void DampVelocity(float deltaTime)
        {
            foreach (var particle in _bodies.SelectMany(body => body.Particles))
                particle.Velocity -= particle.Velocity * _damping * deltaTime;
        }

        public override void EstimatesPositions(float deltaTime)
        {
            foreach (var particle in _bodies.SelectMany(body => body.Particles))
                particle.Predicted = particle.Position + deltaTime * particle.Velocity;
        }

        public override void GenerateCollisionConstraints()
        {
            foreach (var body in _bodies)
            {
                body.ClearContacts();
                var colliders = new Collider[1];

                for (int i = 0; i < body.Particles.Count; i++)
                {
                    var path = body.Particles[i].Predicted - body.Particles[i].Position;
                    var ray = new Ray(body.CenterPosition + body.Particles[i].Position, path.normalized);
                    var predictedPosition = body.CenterPosition + body.Particles[i].Predicted;
                    
                    if (Physics.SphereCast(ray, body.ParticlesRadius, out var hitInfo, path.magnitude))
                    {
                        body.AddContact(new Contact(i, hitInfo.point, hitInfo.normal));
                    }
                    else if (Physics.OverlapSphereNonAlloc(predictedPosition, body.ParticlesRadius, colliders) > 0)
                    {
                        var hitPredictedPosition = body.CenterPosition + new Vector3(body.Particles[i].Predicted.x, 0f, 
                            body.Particles[i].Predicted.z);
                        var hitPosition = colliders[0].ClosestPointOnBounds(hitPredictedPosition);
                        var hitNormal = (hitPredictedPosition - hitPosition).normalized;
                        
                        body.AddContact(new Contact(i, hitPosition, hitNormal));
                    }
                }
            }
        }

        public override void ProjectConstraints()
        {
            foreach (var body in _bodies)
            {
                Constraints.ApplyCollisionConstraint(body, _restCollisionDistance, _collisionConstraintStiffness);
                Constraints.ApplyShapeMatchingConstraint(body, _shapeMatchingConstraintStiffness);
                Constraints.ApplyBalloonsConstraint(body, _balloonsConstraintsStiffness, _pressureStiffness);
            }
        }

        public override void UpdatePositions(float deltaTime)
        {
            foreach (var particle in _bodies.SelectMany(body => body.Particles))
            {
                particle.Velocity = (particle.Predicted - particle.Position) / deltaTime;
                particle.Position = particle.Predicted;
            }
        }

        public override void UpdateVelocity(float deltaTime)
        {
            foreach (var body in _bodies)
            {
                foreach (var contact in body.Contacts)
                {
                    float velocityNormal = Vector3.Dot(body.Particles[contact.Index].Velocity, contact.SurfaceNormal);
            
                    if (velocityNormal > 0f)
                        body.Particles[contact.Index].Velocity -= (1 + _restitution) * velocityNormal * contact.SurfaceNormal;
                
                    var velocityTangent = body.Particles[contact.Index].Velocity - contact.SurfaceNormal * velocityNormal;
                    body.Particles[contact.Index].Velocity -= velocityTangent * _friction;
                }
            }
        }

        public override void UpdateBodies()
        {
            foreach (var body in _bodies)
            {
                body.UpdateCenterPosition();
                body.UpdateVertices();
            }
        }
    }
}