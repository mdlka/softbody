using System;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

namespace SoftbodyPhysics
{
    public class Softbody : MonoBehaviour
    {
        private readonly Dictionary<(int, int), float> _distance = new();
        private readonly List<Contact> _contacts = new();

        [SerializeField] private bool _needCreateObject;
        [SerializeField] private PrimitiveType _type;
        [SerializeField] private MeshRenderer _meshRenderer;
        [SerializeField] private MeshFilter _meshFilter;
        [SerializeField] private Vector3 _gravity;
        [SerializeField] private int _solverIterations;
        [SerializeField, Range(0, 1f)] private float _distanceConstraintStiffness;
        [SerializeField] private float _pushForce;
        [SerializeField] private float _damping;
        [SerializeField] private float _restCollisionDistance;

        private Vector3[] _positions;
        private Vector3[] _predicted;
        private Vector3[] _velocities;
        private float[] _masses;
        private float[] _invMasses;

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
            
            Debug.Log(_positions.Length);
            
            Array.Fill(_masses, 1f);

            for (int i = 0; i < _masses.Length; i++)
                _invMasses[i] = 1f / _masses[i];

            for (int i = 0; i < _positions.Length; i++)
                for (int j = i+1; j < _positions.Length; j++)
                    _distance[(i, j)] = (_positions[i] - _positions[j]).magnitude;
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
            
            _meshFilter.mesh.SetVertices(_positions);
        }

        [ContextMenu("Test")]
        private void Test()
        {
            Vector3 offset = new Vector3(Random.Range(-1f, 1f), 0, Random.Range(-1f, 1f)).normalized * _pushForce;
            _positions[Random.Range(0, _positions.Length)] += offset;
            _meshFilter.mesh.SetVertices(_positions);
        }

        private void ProjectConstraints(int iteration)
        {
            foreach (var contact in _contacts)
                ApplyCollisionConstraint(contact);
            
            foreach (var pair in _distance)
                ApplyDistanceConstraint(pair.Key.Item1, pair.Key.Item2, pair.Value, _distanceConstraintStiffness);
        }
        
        private void GenerateCollisionConstraints()
        {
            _contacts.Clear();

            for (int i = 0; i < _positions.Length; i++)
            {
                var path = _predicted[i] - _positions[i];
                var ray = new Ray(_positions[i], path.normalized);

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
                float normalVelocity = Vector3.Dot(_velocities[contact.Index], contact.SurfaceNormal);
    
                if (normalVelocity < 0f)
                    _velocities[contact.Index] -= (1 + _damping) * normalVelocity * contact.SurfaceNormal;
            }
        }

        private void ApplyDistanceConstraint(int i1, int i2, float restLength, float stiffness)
        {
            float wSum = _invMasses[i1] + _invMasses[i2];

            if (wSum == 0)
                return;
            
            var n = _predicted[i2] - _predicted[i1];
            float d = n.magnitude;
            n.Normalize();
            
            var correction = stiffness * n * (d - restLength) / wSum;

            _predicted[i1] += _invMasses[i1] * correction;
            _predicted[i2] -= _invMasses[i2] * correction;
        }

        private void ApplyCollisionConstraint(Contact contact)
        {
            float distance = Vector3.Dot(_predicted[contact.Index] - contact.EntryPoint, contact.SurfaceNormal) - _restCollisionDistance;

            if (distance < 0f) 
                _predicted[contact.Index] += contact.SurfaceNormal * -distance;
        }
    }
}
