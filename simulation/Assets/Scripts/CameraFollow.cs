using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace DefaultNamespace
{
    public class CameraFollow : MonoBehaviour
    {
        [SerializeField] private Camera camera;
        [SerializeField] private Transform target;
        [SerializeField] private Vector3 positionOffset;


        private void Start()
        {
            if (!camera) camera = Camera.main;
            if (!target) target = gameObject.transform;
        }

        private void LateUpdate()
        {
            if (!target || !camera) return;
            // camera.transform.position = target.position + positionOffset;
            // camera.transform.rotation = target.transform.rotation * Quaternion.Euler(rotationOffset);

            var newPos = target.position - target.forward * -positionOffset.z + target.up * positionOffset.y +
                         target.right * positionOffset.x;
            camera.transform.position = Vector3.Lerp(camera.transform.position, newPos, 0.1f);
            camera.transform.LookAt(target.position + target.up * 2);
        }
    }
}