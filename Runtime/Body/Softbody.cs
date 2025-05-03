using System.Collections.Generic;
using UnityEngine;

namespace SoftbodyPhysics
{
    public class Softbody : MonoBehaviour, ISoftbody
    {
        private readonly List<Contact> _contacts = new();
        private readonly Dictionary<int, List<int>> _verticesByParticle = new();
        private readonly List<Particle> _particles = new();
        private readonly List<int> _particlesTriangles = new();
        
        [SerializeField, Min(0.00001f)] private float _particlesRadius;
        [SerializeField] private MeshFilter _meshFilter;
        
        [Header("Editor")]
        [SerializeField] private bool _needDrawParticleRadius;
        [SerializeField] private bool _needDrawParticleContact;
        [SerializeField] private bool _needDrawTriangles;

        private Vector3[] _vertices;
        private Vector3[] _restPositions;

        public float ParticlesRadius => _particlesRadius;
        public Vector3 CenterPosition => transform.position;
        public Mesh Mesh => _meshFilter.mesh;

        public IReadOnlyList<Particle> Particles => _particles;
        public IReadOnlyList<Contact> Contacts => _contacts;
        public IReadOnlyList<int> ParticlesTriangles => _particlesTriangles;
        public IReadOnlyList<Vector3> RestParticlesPositions => _restPositions;
        
        public Matrix4x4 InvRestMatrix { get; private set; }
        public float RestVolume { get; private set; }

        public void Initialize()
        {
            _vertices = _meshFilter.mesh.vertices;
            var particleByPosition = new Dictionary<Vector3, int>();
            int[] oldToNewIndices = new int[_vertices.Length];
            
            for (int i = 0; i < _vertices.Length; i++)
            {
                if (particleByPosition.ContainsKey(_vertices[i]))
                {
                    _verticesByParticle[particleByPosition[_vertices[i]]].Add(i);
                    oldToNewIndices[i] = particleByPosition[_vertices[i]];
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

                    int particleIndex = _particles.Count - 1;
                    
                    _verticesByParticle[particleIndex] = new List<int> { i };
                    particleByPosition[_vertices[i]] = particleIndex;
                    oldToNewIndices[i] = particleIndex;
                }
            }

            var triangles = Mesh.triangles;

            for (int i = 0; i < triangles.Length; i += 3)
            {
                _particlesTriangles.Add(oldToNewIndices[triangles[i]]);
                _particlesTriangles.Add(oldToNewIndices[triangles[i + 1]]);
                _particlesTriangles.Add(oldToNewIndices[triangles[i + 2]]);
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

        public void UpdateRestVolume(float volume)
        {
            RestVolume = volume;
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
            if (_needDrawParticleRadius)
                DrawParticleRadius();

            if (_needDrawParticleContact)
                DrawParticleContact();

            if (_needDrawTriangles)
                DrawTriangles();
        }

        private void DrawParticleRadius()
        {
            Gizmos.color = Color.blue;
                
            foreach (var particle in _particles)
                Gizmos.DrawWireSphere(transform.position + particle.Position, _particlesRadius);
        }

        private void DrawParticleContact()
        {
            Gizmos.color = Color.red;

            foreach (var contact in _contacts)
            {
                Gizmos.DrawSphere(contact.EntryPoint, 0.02f);
                Gizmos.DrawLine(contact.EntryPoint, contact.EntryPoint + contact.SurfaceNormal * 0.2f);
            }
        }

        private void DrawTriangles()
        {
            if (_particlesTriangles.Count < 2)
                return;
            
            Gizmos.color = Color.blue;
            
            for (int i = 0; i < _particlesTriangles.Count; i += 3)
                DrawTriangle(CenterPosition, 
                    _particles[_particlesTriangles[i]].Predicted, 
                    _particles[_particlesTriangles[i + 1]].Predicted, 
                    _particles[_particlesTriangles[i + 2]].Predicted);
        }

        private static void DrawTriangle(Vector3 center, Vector3 a, Vector3 b, Vector3 c)
        {
            Gizmos.DrawLine(center + a, center + b);
            Gizmos.DrawLine(center + b, center + c);
            Gizmos.DrawLine(center + c, center + a);
        }
    }
}
