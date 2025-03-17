using System.Collections.Generic;
using UnityEngine;

namespace SoftbodyPhysics
{
    public class Softbody : MonoBehaviour, ISoftbody
    {
        private readonly List<Contact> _contacts = new();
        private readonly Dictionary<int, List<int>> _verticesByParticle = new();
        private readonly List<Particle> _particles = new();
        
        [SerializeField, Min(0.00001f)] private float _particlesRadius;
        [SerializeField] private MeshFilter _meshFilter;
        
        [Header("Editor")]
        [SerializeField] private bool _needDrawParticleRadius;

        private Vector3[] _vertices;
        private Vector3[] _restPositions;

        public float ParticlesRadius => _particlesRadius;
        public Vector3 CenterPosition => transform.position;
        
        public IReadOnlyList<Particle> Particles => _particles;
        public IReadOnlyList<Contact> Contacts => _contacts;
        public IReadOnlyList<Vector3> RestParticlesPositions => _restPositions;
        
        public Matrix4x4 InvRestMatrix { get; private set; }

        public void Initialize()
        {
            _vertices = _meshFilter.mesh.vertices;
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
            
            UpdateRotation();
        }

        public void UpdateCenterPosition()
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

        public void UpdateVertices()
        {
            foreach (var pair in _verticesByParticle)
                foreach (int vertexIndex in pair.Value)
                    _vertices[vertexIndex] = _particles[pair.Key].Position;

            _meshFilter.mesh.vertices = _vertices;
        }

        public void AddContact(Contact contact)
        {
            _contacts.Add(contact);
        }

        public void ClearContacts()
        {
            _contacts.Clear();
        }

        public void UpdateRest(Matrix4x4 invRestMatrix, Vector3[] restPositions)
        {
            InvRestMatrix = invRestMatrix;
            _restPositions = restPositions;
        }
        
        private void UpdateRotation()
        {
            foreach (var particle in _particles)
                particle.Position = transform.rotation * particle.Position;
            
            UpdateVertices();
            _meshFilter.mesh.RecalculateNormals();
            
            transform.rotation = Quaternion.identity;
        }
        
        private void OnDrawGizmos()
        {
            Gizmos.color = Color.blue;

            if (_needDrawParticleRadius)
                foreach (var particle in _particles)
                    Gizmos.DrawWireSphere(transform.position + particle.Position, _particlesRadius);

            Gizmos.color = Color.red;

            foreach (var contact in _contacts)
            {
                Gizmos.DrawSphere(contact.EntryPoint, 0.02f);
                Gizmos.DrawLine(contact.EntryPoint, contact.EntryPoint + contact.SurfaceNormal * 0.2f);
            }
        }
    }
}
