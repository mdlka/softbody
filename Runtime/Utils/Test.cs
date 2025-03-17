using UnityEngine;

namespace SoftbodyPhysics
{
    public class Test : MonoBehaviour
    {
        [SerializeField] private Rigidbody _template;
        [SerializeField] private float _pushForce;
        
        private Camera _camera;

        private void Awake()
        {
            _camera = Camera.main;
        }

        private void Update()
        {
            if (Input.GetMouseButtonDown(0) == false)
                return;

            var ray = _camera.ScreenPointToRay(Input.mousePosition);
            var instance = Instantiate(_template, _camera.transform.position, Quaternion.identity, transform);
            instance.AddForce(ray.direction * _pushForce, ForceMode.VelocityChange);
            
            Destroy(instance.gameObject, 5f);
        }
    }
}