using System.Collections.Generic;
using UnityEngine;

namespace SoftbodyPhysics
{
    public interface ISoftbody
    {
        float ParticlesRadius { get; }
        Vector3 CenterPosition { get; }
        
        Mesh Mesh { get; }
        
        IReadOnlyList<Particle> Particles { get; }
        IReadOnlyList<Contact> Contacts { get; }
        IReadOnlyList<int> ParticlesTriangles { get; }
        
        IReadOnlyList<Vector3> RestParticlesPositions { get; }
        Matrix4x4 InvRestMatrix { get; }
        float RestVolume { get; }

        void Initialize();

        void UpdateCenterPosition();
        void UpdateVertices();
        
        void AddContact(Contact contact);
        void ClearContacts();

        void UpdateRest(Matrix4x4 invRestMatrix, Vector3[] restPositions);
        void UpdateRestVolume(float volume);
    }
}