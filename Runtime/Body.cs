using System.Collections.Generic;
using UnityEngine;

namespace SoftbodyPhysics
{
    internal class Body
    {
        private readonly Dictionary<int, List<int>> _verticesByParticle = new();
        private readonly List<Particle> _particles = new();
        private readonly Vector3[] _vertices;
        private readonly Mesh _mesh;

        public Body(Mesh mesh)
        {
            _mesh = mesh;
            _vertices = mesh.vertices;

            var particleByPosition = new Dictionary<Vector3, int>();

            for (int i = 0; i < _vertices.Length; i++)
            {
                if (particleByPosition.ContainsKey(_vertices[i]))
                {
                    _verticesByParticle[particleByPosition[_vertices[i]]].Add(i);
                }
                else
                {
                    var particle = new Particle
                    {
                        Position = _vertices[i],
                        Predicted = Vector3.zero,
                        Velocity = Vector3.zero,
                        Mass = 1f,
                        InvMass = 1f
                    };
                    
                    _particles.Add(particle);
                    _verticesByParticle[_particles.Count - 1] = new List<int> { i };
                    particleByPosition[_vertices[i]] = _particles.Count - 1;
                }
            }
        }

        public IReadOnlyList<Particle> Particles => _particles;

        public void UpdateVertices()
        {
            foreach (var pair in _verticesByParticle)
                foreach (int vertexIndex in pair.Value)
                    _vertices[vertexIndex] = _particles[pair.Key].Position;

            _mesh.vertices = _vertices;
        }
        
        public void UpdateCenter(Transform transform)
        {
            var center = Vector3.zero;
            float totalMass = 0;

            foreach (var particle in _particles)
            {
                center += (transform.position + particle.Position) * particle.Mass;
                totalMass += particle.Mass;
            }

            var offset = center / totalMass - transform.position;
            transform.position += offset;

            foreach (var particle in _particles)
                particle.Position -= offset;
        }

        public void UpdateRotation(Transform transform)
        {
            foreach (var particle in _particles)
                particle.Position = transform.rotation * particle.Position;
            
            UpdateVertices();
            _mesh.RecalculateNormals();
            
            transform.rotation = Quaternion.identity;
        }
    }

    internal class Particle
    {
        public Vector3 Position;
        public Vector3 Predicted;
        public Vector3 Velocity;
        public float Mass;
        public float InvMass;
    }
}