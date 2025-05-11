using System.Collections;
using UnityEngine;

namespace SoftbodyPhysics
{
    public class SoftPhysicsSimulation : MonoBehaviour
    {
        [SerializeField, Min(1e-6f)] private float _timeStep = 0.02f;
        [SerializeField, Min(1)] private int _solverIterations;
        [SerializeField] private BaseSolver _solver;

        private void Awake()
        {
            foreach (var body in GetComponentsInChildren<ISoftbody>())
                _solver.AddBody(body);
        }

        private void Start()
        {
            StartCoroutine(Loop());
        }

        private IEnumerator Loop()
        {
            while (true)
            {
                float deltaTime = _timeStep;
            
                _solver.ApplyExternalForces(deltaTime);
                _solver.DampVelocity(deltaTime);
                _solver.EstimatesPositions(deltaTime);
            
                _solver.GenerateCollisionConstraints();
            
                for (int i = 0; i < _solverIterations; i++)
                    _solver.ProjectConstraints();
            
                _solver.UpdatePositions(deltaTime);
                _solver.UpdateVelocity(deltaTime);
            
                _solver.UpdateBodies();
                
                yield return new WaitForSeconds(_timeStep);
            }
        }
    }
}