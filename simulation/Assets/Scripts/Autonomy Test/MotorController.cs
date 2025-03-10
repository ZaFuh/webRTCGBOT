using UnityEngine;

namespace Autonomy_Test
{
    public class MotorController : MonoBehaviour
    {
        [Header("Thrusters")] [SerializeField] private ConstantForce leftThruster;
        [SerializeField] private ConstantForce rightThruster;
        [SerializeField] private Transform leftThrusterMesh;
        [SerializeField] private Transform rightThrusterMesh;
        [SerializeField] private float thrusterDistanceFromCenter = 1.031788f / 2f;

        [Header("Motor Controls")] [SerializeField] [Range(-90f, 90f)]
        private float leftThrusterAngle;

        [SerializeField] [Range(-90f, 90f)] private float rightThrusterAngle;
        [SerializeField] [Range(0f, 10f)] private float leftThrusterSpeed;
        [SerializeField] [Range(0f, 10f)] private float rightThrusterSpeed;

        public float LeftThrusterAngle
        {
            get => leftThrusterAngle;
            set => leftThrusterAngle = Mathf.Clamp(value, -90f, 90f);
        }

        public float RightThrusterAngle
        {
            get => rightThrusterAngle;
            set => rightThrusterAngle = Mathf.Clamp(value, -90f, 90f);
        }

        public float LeftThrusterSpeed
        {
            get => leftThrusterSpeed;
            set => leftThrusterSpeed = Mathf.Clamp(value, 0f, 10f);
        }

        public float RightThrusterSpeed
        {
            get => rightThrusterSpeed;
            set => rightThrusterSpeed = Mathf.Clamp(value, 0f, 10f);
        }

        // Start is called before the first frame update
        private void Start()
        {
            leftThrusterAngle = 0f;
            rightThrusterAngle = 0f;
            leftThrusterSpeed = 0f;
            rightThrusterSpeed = 0f;
        }

        private void Update()
        {
            // update the mesh positions to reflect the motor angles
            leftThrusterMesh.localRotation = Quaternion.Euler(0f, leftThrusterAngle, 0f);
            rightThrusterMesh.localRotation = Quaternion.Euler(0f, rightThrusterAngle, 0f);
        }

        private void FixedUpdate()
        {
            // calculate the force vector for each thruster based on the thruster angles
            var leftForce = CalculateForceVector(leftThrusterAngle, leftThrusterSpeed);
            var rightForce = CalculateForceVector(rightThrusterAngle, rightThrusterSpeed);

            // apply the force to the thrusters
            leftThruster.relativeForce = leftForce;
            rightThruster.relativeForce = rightForce;
        }

        private Vector3 CalculateForceVector(float angle, float speed)
        {
            var force = new Vector3(0f, 0f, speed * 500);
            return Quaternion.Euler(0f, angle, 0f) * force;
        }
    }
}