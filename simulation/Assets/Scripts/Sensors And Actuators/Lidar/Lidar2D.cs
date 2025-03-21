using System;
using RosMessageTypes.Sensor;
using Unity.Collections;
using Unity.Jobs;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;


public class Lidar2D : MonoBehaviour
{
    public float minAngleDegrees = -45.0f;
    public float maxAngleDegrees = 45.0f;
    public float angleIncrementDegrees = 1.0f;
    public float minRange = 0.1f;
    public float maxRange = 50.0f;
    public float Hz = 5.0f;
    public string topicName = "scan";
    public string frameId = "lidar_link";
    public bool publishData = true;
    public bool drawRays = false;
    private Vector3 transformScale;
    private float scanTime;
    private float timeSinceScan = 0.0f;
    private Vector3[] scanDirVectors;
    private LaserScanMsg msg = new LaserScanMsg();
    private ROSConnection ros;


    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topicName);
        scanDirVectors = GenerateScanVectors();
    }

    void FixedUpdate(){
            timeSinceScan += Time.deltaTime;
            if (timeSinceScan < 1.0f/Hz){
                return;
            }

            transformScale = transform.lossyScale;
            scanDirVectors = GenerateScanVectors();
            float[] dists  =  PerformScan(scanDirVectors);
            if (publishData){
                msg = DistancesToLaserscan(dists);
                ros.Publish(topicName, msg);
            }
            timeSinceScan = 0.0f;
        }




        private Vector3[] GenerateScanVectors()
        {
            int numBeams = (int)((maxAngleDegrees-minAngleDegrees)/(angleIncrementDegrees));
            Debug.Assert(numBeams >= 0, "Number of beams is negative. Check min/max angle and angle increment.");
            Vector3[] scanVectors = new Vector3[numBeams];
            float minAngleRad = Mathf.Deg2Rad*minAngleDegrees;
            float angleIncrementRad = Mathf.Deg2Rad*angleIncrementDegrees;
            for (int i = 0; i < numBeams; i++)
            {
                float hRot = minAngleRad + angleIncrementRad*i;
                float x = -Mathf.Sin(hRot);
                float y = 0;
                float z = Mathf.Cos(hRot);
                scanVectors[i].x = x;
                scanVectors[i].y = y;
                scanVectors[i].z = z;
            }
            return scanVectors;
        }


        private float[] PerformScan(Vector3[] dirs)
        {
            int numPoints = dirs.Length;
            var commands = new NativeArray<RaycastCommand>(numPoints, Allocator.TempJob);
            var results = new NativeArray<RaycastHit>(numPoints, Allocator.TempJob);

            for (int i = 0; i < numPoints; i++)
            {
                Vector3 origin = transform.position;
                Vector3 direction = transform.rotation * dirs[i];
                commands[i] = new RaycastCommand(origin, direction, QueryParameters.Default, maxRange);
            }

            int batchSize = 500;
            JobHandle handle = RaycastCommand.ScheduleBatch(commands, results, batchSize, 1);
            handle.Complete();

            float[] dists = new float[numPoints+1];
            for (int i = 0; i < numPoints; i++)
            {
                var hit = results[i];
                if (hit.collider != null && (transform.position - hit.point).sqrMagnitude > minRange * minRange)
                {
                    Vector3 beam = transform.InverseTransformPoint(hit.point);
                    dists[i] = hit.distance;
                    if (drawRays)
                    {
                        Debug.DrawLine(transform.position, transform.TransformPoint(beam), Color.red);
                    }
                }
                else{
                    dists[i] = float.NaN;
                }
            }

            results.Dispose();
            commands.Dispose();
            return dists;
        }

        private LaserScanMsg DistancesToLaserscan(float[] dists){
            LaserScanMsg msg = new LaserScanMsg();
            msg.header.frame_id = frameId;

            var publishTime = Clock.Now;
            var sec = publishTime;
            var nanosec = ((publishTime - Math.Floor(publishTime)) * Clock.k_NanoSecondsInSeconds);
            msg.header.stamp.sec = (int)sec;
            msg.header.stamp.nanosec = (uint)nanosec;

            msg.angle_min = minAngleDegrees*Mathf.Deg2Rad;
            msg.angle_max = maxAngleDegrees*Mathf.Deg2Rad;
            msg.angle_increment = angleIncrementDegrees*Mathf.Deg2Rad;
            msg.scan_time = 1f/Hz;
            msg.range_min = minRange;
            msg.range_max = maxRange;
            msg.ranges = dists;
            return msg;
        }
}
