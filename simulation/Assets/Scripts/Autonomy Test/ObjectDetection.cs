using System.Collections;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Autonomy_Test
{
    [RequireComponent(typeof(BoxCollider))]
    public class ObjectDetection : MonoBehaviour
    {
        // The field of view in degrees
        [SerializeField] private float fov = 70f;

        // The maximum distance the object can be detected
        [SerializeField] private float maxDistance = 8f;

        // The rate the detection will run in number of times per sescond
        [SerializeField] private float detectionFrequency = 2f;

        // The rate the tracking will run in number of times per sescond
        [SerializeField] private float trackingFrequency = 60f;

        [SerializeField] private LayerMask layerMask = 1 << 9;

        [SerializeField] private BoxCollider trigger;

        [SerializeField] private string topicName = "object_detection";

        private List<Transform> detectedPositions;

        private float detectionDuration;

        private float detectionTime;


        private ROSConnection ros;

        private List<Transform> trackedPositions;
        private float trackingDuration;
        private float trackingTime;

        private void Awake()
        {
            trigger ??= gameObject.GetComponent<BoxCollider>();
            trigger.isTrigger = true;

            // size box collider
            trigger.center = transform.position + new Vector3(0f, 0f, 1f + maxDistance / 2f);
            trigger.size = new Vector3(2f * Mathf.Tan(fov / 2f) * maxDistance, 4f, maxDistance);

            detectionDuration = 0.8f * 1f / detectionFrequency;
            trackingDuration = 0.8f * 1f / trackingFrequency;
        }

        private void Start()
        {
            detectedPositions = new List<Transform>();

            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<Float32MultiArrayMsg>(topicName);

            // start the coroutine to detect objects
            StartCoroutine(DetectObjects());
            StartCoroutine(TrackObjects());
        }

        private void OnDrawGizmos()
        {
            // draw lines for the fov
            var halfFov = fov / 2;
            var leftRayRotation = Quaternion.AngleAxis(-halfFov, Vector3.up);
            var rightRayRotation = Quaternion.AngleAxis(halfFov, Vector3.up);
            var leftRayDirection = leftRayRotation * transform.forward;
            var rightRayDirection = rightRayRotation * transform.forward;
            Gizmos.color = Color.green;
            Gizmos.DrawRay(transform.position, leftRayDirection * maxDistance);
            Gizmos.DrawRay(transform.position, rightRayDirection * maxDistance);
            // DrawBoxCast();

            // detection
            if (Mathf.Max(0.2f, detectionTime + detectionDuration) < Time.time)
                if (detectedPositions != null)
                    foreach (var pos in detectedPositions)
                    {
                        if (!IsInFOV(pos)) continue;
                        Gizmos.color = Color.red;
                        Gizmos.DrawWireSphere(pos.position, 1.2f);
                    }

            // tracking
            if (trackingTime + trackingDuration < Time.time)
                if (trackedPositions != null)
                    foreach (var pos in trackedPositions)
                    {
                        if (!IsInFOV(pos)) continue;
                        Gizmos.color = Color.yellow;
                        Gizmos.DrawWireSphere(pos.position, 1f);
                    }
        }

        private void OnTriggerEnter(Collider other)
        {
            Debug.Log("Detected: " + other.name);
            detectedPositions.Add(other.transform);
        }

        private void OnTriggerExit(Collider other)
        {
            Debug.Log("Lost: " + other.name);
            detectedPositions.Remove(other.transform);
            // remove from tracked if it exists
            if (trackedPositions != null && trackedPositions.Contains(other.transform))
                trackedPositions.Remove(other.transform);
        }

        private bool IsInFOV(Transform pos)
        {
            // get the angle between the object and the forward vector of the object
            var direction = pos.position - transform.position;
            var angle = Vector3.Angle(transform.forward, direction);

            return angle <= fov / 2;
        }

        // coroutine to detect and track objects
        private IEnumerator DetectObjects()
        {
            while (true)
            {
                if (detectedPositions != null)
                    trackedPositions = detectedPositions.Where(IsInFOV).ToList();
                detectionTime = Time.time;

                yield return new WaitForSeconds(1f / detectionFrequency);
            }
        }

        private IEnumerator TrackObjects()
        {
            while (true)
            {
                if (trackedPositions != null)
                {
                    trackingTime = Time.time;

                    // publish on ros
                    var validPositions = trackedPositions.Where(IsInFOV).ToList();
                    if (validPositions.Count > 0)
                        ros.Publish(topicName, GenerateMessage(validPositions));

                    // remove any objects which are out of the fov from the tracked list
                    trackedPositions = validPositions;
                }

                yield return new WaitForSeconds(1f / trackingFrequency);
            }
        }

        private Float32MultiArrayMsg GenerateMessage(List<Transform> positions)
        {
            // var data = new List<float>();
            var data = new float[positions.Count * 2];
            // foreach (var pos in positions)
            // {
            //     // data.Add(pos.position.x);
            //     // data.Add(pos.position.z);
            // }

            for (var i = 0; i < positions.Count; i++)
            {
                var relativePos = transform.InverseTransformPoint(positions[i].position);
                data[i * 2] = relativePos.x;
                data[i * 2 + 1] = relativePos.z;
            }

            var msg = new Float32MultiArrayMsg
            {
                data = data
            };

            return msg;
        }
    }
}