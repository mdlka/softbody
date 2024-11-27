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
        [SerializeField] private float _damping;
        [SerializeField] private float _restCollisionDistance;
        [SerializeField] private float _restitution;
        [SerializeField] private float _friction;

        private Vector3[] _positions;
        private Vector3[] _predicted;
        private Vector3[] _velocities;
        private float[] _masses;
        private float[] _invMasses;
        
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
            
            _positions = _meshFilter.mesh.vertices;
            _predicted = new Vector3[_positions.Length];
            _velocities = new Vector3[_positions.Length];
            _masses = new float[_positions.Length];
            _invMasses = new float[_positions.Length];
            
            Array.Fill(_masses, 1f);

            for (int i = 0; i < _masses.Length; i++)
                _invMasses[i] = 1f / _masses[i];

            for (int i = 0; i < _positions.Length; i++)
                for (int j = i+1; j < _positions.Length; j++)
                    _distance[(i, j)] = (_positions[i] - _positions[j]).magnitude;
            
            // ShapeMatchingConstraint initialization
            _invRestMatrix = Matrix4x4.identity;
            _restPositions = new Vector3[_positions.Length];

            float wsum = 0;
            var restCm = Vector3.zero;
            
            for (int i = 0; i < _positions.Length; i++)
            {
                restCm += _positions[i] * _masses[i];
                wsum += _masses[i];
            }

            restCm /= wsum;
            var A = Matrix4x4.zero;
            
            for (int i = 0; i < _positions.Length; i++)
            {
                var q = _positions[i] - restCm;

                A[0, 0] += _masses[i] * q.x * q.x;
                A[0, 1] += _masses[i] * q.x * q.y;
                A[0, 2] += _masses[i] * q.x * q.z;

                A[1, 0] += _masses[i] * q.y * q.x;
                A[1, 1] += _masses[i] * q.y * q.y;
                A[1, 2] += _masses[i] * q.y * q.z;

                A[2, 0] += _masses[i] * q.z * q.x;
                A[2, 1] += _masses[i] * q.z * q.y;
                A[2, 2] += _masses[i] * q.z * q.z;

                _restPositions[i] = q;
            }

            _invRestMatrix = A.inverse;
        }

        private void FixedUpdate()
        {
            float deltaTime = Time.fixedDeltaTime;
            
            // Hook up external forces to the system (e.g. gravity)
            for (int i = 0; i < _positions.Length; i++)
                _velocities[i] += deltaTime * _invMasses[i] * _gravity;

            DampVelocity(deltaTime);

            // Estimates for new locations of the vertices
            for (int i = 0; i < _positions.Length; i++)
                _predicted[i] = _positions[i] + deltaTime * _velocities[i];

            GenerateCollisionConstraints();
            
            for (int i = 0; i < _solverIterations; i++)
                ProjectConstraints(i + 1);
            
            for (int i = 0; i < _positions.Length; i++)
            {
                _velocities[i] = (_predicted[i] - _positions[i]) / deltaTime;
                _positions[i] = _predicted[i];
            }

            UpdateVelocity(deltaTime);
            UpdateCenter();
            
            _meshFilter.mesh.SetVertices(_positions);
        }
        
        private void UpdateCenter()
        {
            var center = Vector3.zero;
            float totalMass = 0;

            for (int i = 0; i < _positions.Length; i++)
            {
                center += (transform.position + _positions[i]) * _masses[i];
                totalMass += _masses[i];
            }

            var offset = center / totalMass - transform.position;
            transform.position += offset;

            for (int i = 0; i < _positions.Length; i++)
                _positions[i] -= offset;
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

            for (int i = 0; i < _positions.Length; i++)
            {
                var path = _predicted[i] - _positions[i];
                var ray = new Ray(transform.position + _positions[i], path.normalized);
                
                Debug.DrawRay(ray.origin, ray.direction * path.magnitude, Color.green, Time.deltaTime);

                if (Physics.Raycast(ray, out var hitInfo, path.magnitude))
                    _contacts.Add(new Contact(i, hitInfo.point, hitInfo.normal));
            }
        }

        private void DampVelocity(float deltaTime)
        {
            for (int i = 0; i < _velocities.Length; i++)
                _velocities[i] -= _velocities[i] * _damping * deltaTime;
        }
        
        private void UpdateVelocity(float deltaTime)
        {
            foreach (var contact in _contacts)
            {
                float velocityNormal = Vector3.Dot(_velocities[contact.Index], contact.SurfaceNormal);

                Debug.DrawRay(transform.position + _positions[contact.Index], 
                    transform.position + _positions[contact.Index] + _velocities[contact.Index], Color.magenta, Time.deltaTime);
                
                if (velocityNormal > 0f)
                    _velocities[contact.Index] -= (1 + _restitution) * velocityNormal * contact.SurfaceNormal;
                
                var velocityTangent = _velocities[contact.Index] - contact.SurfaceNormal * velocityNormal;
                _velocities[contact.Index] -= velocityTangent * _friction;
                
                Debug.DrawRay(transform.position + _positions[contact.Index], 
                    transform.position + _positions[contact.Index] + _velocities[contact.Index], Color.yellow, Time.deltaTime);
            }
        }

        private void ApplyDistanceConstraint(int i1, int i2, float restLength, float stiffness)
        {
            float wSum = _invMasses[i1] + _invMasses[i2];

            if (wSum == 0)
                return;
            
            var diff = _predicted[i1] - _predicted[i2];
            var correction = stiffness * diff.normalized * (diff.magnitude - restLength) / wSum;

            _predicted[i1] -= _invMasses[i1] * correction;
            _predicted[i2] += _invMasses[i2] * correction;
        }

        private void ApplyShapeMatchingConstraint(float stiffness)
        {
            var cm = Vector3.zero;
            float wsum = 0f;

            for (int i = 0; i < _predicted.Length; i++)
            {
                cm += _predicted[i] * _masses[i];
                wsum += _masses[i];
            }

            cm /= wsum;

            var A = Matrix4x4.zero;

            for (int i = 0; i < _positions.Length; i++)
            {
                Vector3 q = _restPositions[i];
                Vector3 p = _positions[i] - cm;

                A[0, 0] += _masses[i] * p.x * q.x;
                A[0, 1] += _masses[i] * p.x * q.y;
                A[0, 2] += _masses[i] * p.x * q.z;

                A[1, 0] += _masses[i] * p.y * q.x;
                A[1, 1] += _masses[i] * p.y * q.y;
                A[1, 2] += _masses[i] * p.y * q.z;

                A[2, 0] += _masses[i] * p.z * q.x;
                A[2, 1] += _masses[i] * p.z * q.y;
                A[2, 2] += _masses[i] * p.z * q.z;
            }

            A *= _invRestMatrix;
            
            MatrixMath.PolarDecompositionStable(A, 1e-6f, out Matrix4x4 R);

            for (int i = 0; i < _predicted.Length; i++)
            {
                var goal = cm + (R * _restPositions[i]).ToVector3();
                _predicted[i] += (goal - _predicted[i]) * stiffness;
            }
        }
        
        private void ApplyVolumeConstraint(float stiffness)
        {
            var triangles = _meshFilter.mesh.triangles;
            var restVolume = MeshMath.ComputeVolume(_restPositions, triangles);
            var predictedVolume = MeshMath.ComputeVolume(_predicted, triangles);

            float s = 0f;
            
            for (int i = 0; i < triangles.Length; i += 3)
            {
                var v0 = _predicted[triangles[i]];
                var v1 = _predicted[triangles[i + 1]];
                var v2 = _predicted[triangles[i + 2]];

                var normal = Vector3.Cross(v1 - v0, v2 - v0);
                float area = normal.magnitude * 0.5f;

                s += Vector3.Dot(normal.normalized, normal.normalized);
            }

            float c = predictedVolume - restVolume;
            
            for (int i = 0; i < triangles.Length; i += 3)
            {
                var v0 = _predicted[triangles[i]];
                var v1 = _predicted[triangles[i + 1]];
                var v2 = _predicted[triangles[i + 2]];

                var normal = Vector3.Cross(v1 - v0, v2 - v0).normalized;

                _predicted[triangles[i]] -= c / s * normal * stiffness;
            }
        }

        private void ApplyCollisionConstraint(Contact contact)
        {
            var delta = Vector3.Dot(transform.position + _predicted[contact.Index] - contact.EntryPoint, contact.SurfaceNormal) - _restCollisionDistance;

            if (delta > 0f) 
                return;
            
            _predicted[contact.Index] += contact.SurfaceNormal * -delta;
        }
        
        private void OnDrawGizmos()
        {
            Gizmos.color = Color.blue;

            foreach (var position in _positions ?? Array.Empty<Vector3>())
                Gizmos.DrawSphere(transform.position + position, 0.02f);

            Gizmos.color = Color.red;
            
            foreach (var contact in _contacts)
                Gizmos.DrawSphere(contact.EntryPoint, 0.02f);
        }
    }
}
