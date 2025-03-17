using System.Collections.Generic;
using UnityEngine;

namespace SoftbodyPhysics
{
    public interface ISoftbody
    {
        float ParticlesRadius { get; }
        Vector3 CenterPosition { get; }
        
        IReadOnlyList<Particle> Particles { get; }
        IReadOnlyList<Contact> Contacts { get; }
        
        IReadOnlyList<Vector3> RestParticlesPositions { get; }
        Matrix4x4 InvRestMatrix { get; }

        void Initialize();

        void UpdateCenterPosition();
        void UpdateVertices();
        
        void AddContact(Contact contact);
        void ClearContacts();

        void UpdateRest(Matrix4x4 invRestMatrix, Vector3[] restPositions);
    }
}