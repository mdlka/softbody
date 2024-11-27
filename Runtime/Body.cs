using System.Collections.Generic;
using UnityEngine;

namespace SoftbodyPhysics
{
    internal class Body
    {
        private readonly Dictionary<int, List<int>> _verticesByNodes = new();
        private readonly List<Node> _nodes = new();
        private readonly Vector3[] _vertices;
        private readonly Mesh _mesh;

        public Body(Mesh mesh)
        {
            _mesh = mesh;
            _vertices = mesh.vertices;

            var nodeByPosition = new Dictionary<Vector3, int>();

            for (int i = 0; i < _vertices.Length; i++)
            {
                if (nodeByPosition.ContainsKey(_vertices[i]))
                {
                    _verticesByNodes[nodeByPosition[_vertices[i]]].Add(i);
                }
                else
                {
                    var node = new Node
                    {
                        Position = _vertices[i],
                        Predicted = Vector3.zero,
                        Velocity = Vector3.zero,
                        Mass = 1f,
                        InvMass = 1f
                    };
                    
                    _nodes.Add(node);
                    _verticesByNodes[_nodes.Count - 1] = new List<int> { i };
                    nodeByPosition[_vertices[i]] = _nodes.Count - 1;
                }
            }
        }

        public IReadOnlyList<Node> Nodes => _nodes;

        public void UpdateVertices()
        {
            foreach (var pair in _verticesByNodes)
                foreach (int vertexIndex in pair.Value)
                    _vertices[vertexIndex] = _nodes[pair.Key].Position;

            _mesh.vertices = _vertices;
        }
        
        public void UpdateCenter(Transform transform)
        {
            var center = Vector3.zero;
            float totalMass = 0;

            foreach (var node in _nodes)
            {
                center += (transform.position + node.Position) * node.Mass;
                totalMass += node.Mass;
            }

            var offset = center / totalMass - transform.position;
            transform.position += offset;

            foreach (var node in _nodes)
                node.Position -= offset;
        }
    }

    internal class Node
    {
        public Vector3 Position;
        public Vector3 Predicted;
        public Vector3 Velocity;
        public float Mass;
        public float InvMass;
    }
}