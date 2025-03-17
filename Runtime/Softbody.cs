using System;
using System.Collections.Generic;
using UnityEngine;

namespace SoftbodyPhysics
{
    public class Softbody : MonoBehaviour
    {
        private enum ConstraintType
        {
            Distance,
            ShapeMatching
        }
        
        private readonly Dictionary<(int, int), float> _distance = new();
        private readonly List<Contact> _contacts = new();

        [SerializeField] private bool _needCreateObject;
        [SerializeField] private PrimitiveType _type;
        [SerializeField] private MeshRenderer _meshRenderer;
        [SerializeField] private MeshFilter _meshFilter;
        [SerializeField] private Vector3 _gravity;
        [SerializeField] private int _solverIterations;
        [Space, SerializeField] private ConstraintType _currentConstraint;
        [SerializeField] private bool _applyVolumeConstraint;
        [SerializeField, Range(0, 1f)] private float _distanceConstraintStiffness;
        [SerializeField, Range(0, 1f)] private float _shapeMatchingConstraintStiffness;
        [SerializeField, Range(0, 1f)] private float _volumeConstraintStiffness;
        [SerializeField, Range(0, 1f)] private float _collisionConstraintStiffness;
        [SerializeField] private float _damping;
        [SerializeField] private float _restCollisionDistance;
        [SerializeField] private float _restitution;
        [SerializeField] private float _friction;
        [SerializeField] private float _particleRadius;
        [SerializeField] private bool _needDrawParticleRadius;

        private Body _body;
        
        private Matrix4x4 _invRestMatrix;
        private Vector3[] _restPositions;

        private record Contact(int Index, Vector3 EntryPoint, Vector3 SurfaceNormal)
        {
            public int Index { get; } = Index;
            public Vector3 EntryPoint { get; } = EntryPoint;
            public Vector3 SurfaceNormal { get; } = SurfaceNormal;
        }

        private void Awake()
        {
            if (_needCreateObject)
            {
                var instance = GameObject.CreatePrimitive(_type);
                _meshFilter.mesh = instance.GetComponent<MeshFilter>().mesh;
                Destroy(instance);
            }

            _body = new Body(_meshFilter.mesh);
            _body.UpdateRotation(transform);
            
            Debug.Log(_body.Particles.Count);

            for (int i = 0; i < _body.Particles.Count; i++)
                for (int j = i+1; j < _body.Particles.Count; j++)
                    _distance[(i, j)] = (_body.Particles[i].Position - _body.Particles[j].Position).magnitude;
            
            // ShapeMatchingConstraint initialization
            _invRestMatrix = Matrix4x4.identity;
            _restPositions = new Vector3[_body.Particles.Count];

            float wsum = 0;
            var restCm = Vector3.zero;
            
            foreach (var particle in _body.Particles)
            {
                restCm += particle.Position * particle.Mass;
                wsum += particle.Mass;
            }

            restCm /= wsum;
            var A = Matrix4x4.zero;

            for (int i = 0; i < _body.Particles.Count; i++)
            {
                var particle = _body.Particles[i];
                var q = particle.Position - restCm;

                A[0, 0] += particle.Mass * q.x * q.x;
                A[0, 1] += particle.Mass * q.x * q.y;
                A[0, 2] += particle.Mass * q.x * q.z;

                A[1, 0] += particle.Mass * q.y * q.x;
                A[1, 1] += particle.Mass * q.y * q.y;
                A[1, 2] += particle.Mass * q.y * q.z;

                A[2, 0] += particle.Mass * q.z * q.x;
                A[2, 1] += particle.Mass * q.z * q.y;
                A[2, 2] += particle.Mass * q.z * q.z;

                _restPositions[i] = q;
            }

            _invRestMatrix = A.inverse;
        }

        private void FixedUpdate()
        {
            float deltaTime = Time.fixedDeltaTime;
            
            // Hook up external forces to the system (e.g. gravity)
            foreach (var particle in _body.Particles)
                particle.Velocity += deltaTime * particle.InvMass * _gravity;

            DampVelocity(deltaTime);

            // Estimates for new locations of the vertices
            foreach (var particle in _body.Particles)
                particle.Predicted = particle.Position + deltaTime * particle.Velocity;

            GenerateCollisionConstraints();
            
            for (int i = 0; i < _solverIterations; i++)
                ProjectConstraints(i + 1);
            
            foreach (var particle in _body.Particles)
            {
                particle.Velocity = (particle.Predicted - particle.Position) / deltaTime;
                particle.Position = particle.Predicted;
            }

            UpdateVelocity(deltaTime);
            _body.UpdateCenter(transform);
            _body.UpdateVertices();
        }

        private void ProjectConstraints(int iteration)
        {
            foreach (var contact in _contacts)
                ApplyCollisionConstraint(contact);

            if (_currentConstraint == ConstraintType.ShapeMatching)
            {
                ApplyShapeMatchingConstraint(_shapeMatchingConstraintStiffness);
            }
            else if (_currentConstraint == ConstraintType.Distance)
            {
                foreach (var pair in _distance) 
                    ApplyDistanceConstraint(pair.Key.Item1, pair.Key.Item2, pair.Value, _distanceConstraintStiffness);
            }

            if (_applyVolumeConstraint)
                ApplyVolumeConstraint(_volumeConstraintStiffness);
        }
        
        private void GenerateCollisionConstraints()
        {
            _contacts.Clear();
            var colliders = new Collider[1];

            for (int i = 0; i < _body.Particles.Count; i++)
            {
                var path = _body.Particles[i].Predicted - _body.Particles[i].Position;
                var ray = new Ray(transform.position + _body.Particles[i].Position, path.normalized);
                var predictedPosition = transform.position + _body.Particles[i].Predicted;
                
                Debug.DrawRay(ray.origin, ray.direction * path.magnitude, Color.green, Time.deltaTime);

                if (Physics.Raycast(ray, out var hitInfo, path.magnitude))
                {
                    _contacts.Add(new Contact(i, hitInfo.point, hitInfo.normal));
                }
                else if (Physics.OverlapSphereNonAlloc(predictedPosition, _particleRadius, colliders) > 0)
                {
                    var hitPosition = colliders[0].ClosestPoint(predictedPosition);
                    var hitNormal = (predictedPosition - hitPosition).normalized;
                    _contacts.Add(new Contact(i, hitPosition, hitNormal));
                }
            }
        }

        private void DampVelocity(float deltaTime)
        {
            foreach (var particle in _body.Particles)
                particle.Velocity -= particle.Velocity * _damping * deltaTime;
        }
        
        private void UpdateVelocity(float deltaTime)
        {
            // foreach (var contact in _contacts)
            // {
            //     float velocityNormal = Vector3.Dot(_body.Particles[contact.Index].Velocity, contact.SurfaceNormal);
            //
            //     Debug.DrawRay(transform.position + _body.Particles[contact.Index].Position, 
            //         transform.position + _body.Particles[contact.Index].Position + _body.Particles[contact.Index].Velocity, Color.magenta, Time.deltaTime);
            //     
            //     if (velocityNormal > 0f)
            //         _body.Particles[contact.Index].Velocity -= (1 + _restitution) * velocityNormal * contact.SurfaceNormal;
            //     
            //     var velocityTangent = _body.Particles[contact.Index].Velocity - contact.SurfaceNormal * velocityNormal;
            //     _body.Particles[contact.Index].Velocity -= velocityTangent * _friction;
            //     
            //     Debug.DrawRay(transform.position + _body.Particles[contact.Index].Position, 
            //         transform.position + _body.Particles[contact.Index].Position + _body.Particles[contact.Index].Velocity, Color.yellow, Time.deltaTime);
            // }
        }

        private void ApplyDistanceConstraint(int i1, int i2, float restLength, float stiffness)
        {
            float wSum = _body.Particles[i1].InvMass + _body.Particles[i2].InvMass;

            if (wSum == 0)
                return;
            
            var diff = _body.Particles[i1].Predicted - _body.Particles[i2].Predicted;
            var correction = stiffness * diff.normalized * (diff.magnitude - restLength) / wSum;

            _body.Particles[i1].Predicted -= _body.Particles[i1].InvMass * correction;
            _body.Particles[i2].Predicted += _body.Particles[i2].InvMass * correction;
        }

        private void ApplyShapeMatchingConstraint(float stiffness)
        {
            var cm = Vector3.zero;
            float wsum = 0f;

            foreach (var particle in _body.Particles)
            {
                cm += particle.Predicted * particle.Mass;
                wsum += particle.Mass;
            }

            cm /= wsum;

            var A = Matrix4x4.zero;

            for (int i = 0; i < _body.Particles.Count; i++)
            {
                Vector3 q = _restPositions[i];
                Vector3 p = _body.Particles[i].Position - cm;

                A[0, 0] += _body.Particles[i].Mass * p.x * q.x;
                A[0, 1] += _body.Particles[i].Mass * p.x * q.y;
                A[0, 2] += _body.Particles[i].Mass * p.x * q.z;

                A[1, 0] += _body.Particles[i].Mass * p.y * q.x;
                A[1, 1] += _body.Particles[i].Mass * p.y * q.y;
                A[1, 2] += _body.Particles[i].Mass * p.y * q.z;

                A[2, 0] += _body.Particles[i].Mass * p.z * q.x;
                A[2, 1] += _body.Particles[i].Mass * p.z * q.y;
                A[2, 2] += _body.Particles[i].Mass * p.z * q.z;
            }

            A *= _invRestMatrix;
            
            MatrixMath.PolarDecompositionStable(A, 1e-6f, out Matrix4x4 R);

            for (int i = 0; i < _body.Particles.Count; i++)
            {
                var goal = cm + (R * _restPositions[i]).ToVector3();
                _body.Particles[i].Predicted += (goal - _body.Particles[i].Predicted) * stiffness;
            }
        }
        
        private void ApplyVolumeConstraint(float stiffness)
        {
            // var triangles = _meshFilter.mesh.triangles;
            // var restVolume = MeshMath.ComputeVolume(_restPositions, triangles);
            // var predictedVolume = MeshMath.ComputeVolume(_predicted, triangles);
            //
            // float s = 0f;
            //
            // for (int i = 0; i < triangles.Length; i += 3)
            // {
            //     var v0 = _predicted[triangles[i]];
            //     var v1 = _predicted[triangles[i + 1]];
            //     var v2 = _predicted[triangles[i + 2]];
            //
            //     var normal = Vector3.Cross(v1 - v0, v2 - v0);
            //     float area = normal.magnitude * 0.5f;
            //
            //     s += Vector3.Dot(normal.normalized, normal.normalized);
            // }
            //
            // float c = predictedVolume - restVolume;
            //
            // for (int i = 0; i < triangles.Length; i += 3)
            // {
            //     var v0 = _predicted[triangles[i]];
            //     var v1 = _predicted[triangles[i + 1]];
            //     var v2 = _predicted[triangles[i + 2]];
            //
            //     var normal = Vector3.Cross(v1 - v0, v2 - v0).normalized;
            //
            //     _predicted[triangles[i]] -= c / s * normal * stiffness;
            // }
        }

        private void ApplyCollisionConstraint(Contact contact)
        {
            var delta = Vector3.Dot(transform.position + _body.Particles[contact.Index].Predicted - contact.EntryPoint,
                contact.SurfaceNormal) - _restCollisionDistance - _particleRadius;

            if (delta > 0f) 
                return;

            _body.Particles[contact.Index].Predicted -= contact.SurfaceNormal * delta * _collisionConstraintStiffness;
        }
        
        private void OnDrawGizmos()
        {
            Gizmos.color = Color.blue;

            if (_needDrawParticleRadius)
                foreach (var particle in _body?.Particles ?? Array.Empty<Particle>())
                    Gizmos.DrawWireSphere(transform.position + particle.Position, _particleRadius);

            Gizmos.color = Color.red;

            foreach (var contact in _contacts)
            {
                Gizmos.DrawSphere(contact.EntryPoint, 0.02f);
                Gizmos.DrawLine(contact.EntryPoint, contact.EntryPoint + contact.SurfaceNormal * 0.2f);
            }
        }
    }
}
