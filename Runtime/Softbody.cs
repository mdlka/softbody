using System;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

namespace SoftbodyPhysics
{
    public class Softbody : MonoBehaviour
    {
        private readonly Dictionary<(int, int), float> _distance = new();

        [SerializeField] private MeshRenderer _meshRenderer;
        [SerializeField] private MeshFilter _meshFilter;
        [SerializeField] private Vector3 _gravity;
        [SerializeField] private int _solverIterations;
        [SerializeField, Range(0, 1f)] private float _distanceConstraintStiffness;

        private Vector3[] _verticesPositions;
        private Vector3[] _verticesNewPositions;
        private Vector3[] _verticesVelocities;
        private float[] _verticesMasses;
        private float[] _verticesInverseMasses;
        
        private void Awake()
        {
            var instance = GameObject.CreatePrimitive(PrimitiveType.Quad);
            _meshFilter.mesh = instance.GetComponent<MeshFilter>().mesh;
            Destroy(instance);

            _verticesPositions = _meshFilter.mesh.vertices;
            _verticesNewPositions = new Vector3[_verticesPositions.Length];
            _verticesVelocities = new Vector3[_verticesPositions.Length];
            _verticesMasses = new float[_verticesPositions.Length];
            _verticesInverseMasses = new float[_verticesPositions.Length];
            
            Array.Fill(_verticesMasses, 1f);

            for (int i = 0; i < _verticesMasses.Length; i++)
                _verticesInverseMasses[i] = 1f / _verticesMasses[i];
            
            for (int i = 0; i < _verticesPositions.Length; i++)
                for (int j = 0; j < _verticesPositions.Length; j++)
                    _distance[(i, j)] = (_verticesPositions[i] - _verticesPositions[j]).magnitude;
        }

        private void FixedUpdate()
        {
            float deltaTime = Time.fixedDeltaTime;
            
            // Hook up external forces to the system (e.g. gravity)
            for (int i = 0; i < _verticesPositions.Length; i++)
                _verticesVelocities[i] += deltaTime * _verticesInverseMasses[i] * _gravity;

            DampVelocity();

            // Estimates for new locations of the vertices
            for (int i = 0; i < _verticesPositions.Length; i++)
                _verticesNewPositions[i] = _verticesPositions[i] + deltaTime * _verticesVelocities[i];
            
            // TODO: I skip a step (8) in paper because I don't need collisions for now
            
            for (int i = 0; i < _solverIterations; i++)
                ProjectConstraints(i + 1);
            
            for (int i = 0; i < _verticesPositions.Length; i++)
            {
                _verticesVelocities[i] = (_verticesNewPositions[i] - _verticesPositions[i]) / deltaTime;
                _verticesPositions[i] = _verticesNewPositions[i];
            }

            UpdateVelocity();
            
            _meshFilter.mesh.SetVertices(_verticesPositions);
        }

        [ContextMenu("Test")]
        private void Test()
        {
            Vector3 offset = new Vector3(Random.Range(-1f, 1f), 0, Random.Range(-1f, 1f)).normalized * 0.2f;
            _verticesPositions[Random.Range(0, _verticesPositions.Length)] += offset;
            _meshFilter.mesh.SetVertices(_verticesPositions);
        }

        private void ProjectConstraints(int iteration)
        {
            for (int i = 0; i < _verticesNewPositions.Length; i++)
            {
                for (int j = 0; j < _verticesNewPositions.Length; j++)
                {
                    if (SolveDistanceConstraints(i, j, _distance[(i, j)], _distanceConstraintStiffness, iteration, out var correction1, out var correction2))
                    {
                        if (_verticesInverseMasses[i] != 0)
                            _verticesNewPositions[i] += correction1;
                
                        if (_verticesInverseMasses[j] != 0)
                            _verticesNewPositions[j] += correction2;
                    }
                }
            }
        }

        private void DampVelocity()
        {
            //
        }
        
        private void UpdateVelocity()
        {
            //
        }

        private bool SolveDistanceConstraints(int vertex1, int vertex2, float distance, float stiffness, int iteration, out Vector3 correction1, out Vector3 correction2)
        {
            correction1 = correction2 = default;
            float wSum = _verticesInverseMasses[vertex1] + _verticesInverseMasses[vertex2];

            if (wSum == 0)
                return false;
            
            var n = _verticesPositions[vertex2] - _verticesPositions[vertex1];
            float d = n.magnitude;
            n = n.normalized;
            
            var correction = (1f - Mathf.Pow(1f - stiffness, 1f / iteration)) * n * (d - distance) / wSum;

            correction1 = _verticesInverseMasses[vertex1] * correction;
            correction2 = -_verticesInverseMasses[vertex2] * correction;
            
            return true;
        }
    }
}
