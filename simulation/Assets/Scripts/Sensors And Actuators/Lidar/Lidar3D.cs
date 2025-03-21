using System;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using Unity.Collections;
using Unity.Jobs;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace sensors_actuators
{
    public class Lidar3D : MonoBehaviour
    {
        private ROSConnection ros;

        [Range(0.1f, 200.0f)]
        public float maxRange = 100.0f;
        public bool publishData = true;
        public string topicName = "points";
        public string frameId = "lidar_link";

        [Range(0, 5000)]
        public int numHorizontalBeams = 500; 

        [Range(0.1f, 60.0f)]
        public float hz = 10.0f;

        [Range(0, 16)]
        public int numVerticalBeams = 16; 
        [Range(0.1f, 5.0f)]
        public float minDistance = 0.3f;

        [Range(0.0f, 2.0f * Mathf.PI)]
        public float horizontalFOV = 60.0f * Mathf.PI; 

        [Range(0.0f, Mathf.PI)]
        public float verticalFOV = 0.5f * Mathf.PI;

    
        [Range(0.0f, 20.0f)]
        public float noisePercentage = 5.0f;
        public bool drawRays = true;
        private float timeSinceScan;

        private Vector3[] scanDirVectors;
        private Transform transform1;
        private Vector3 transformScale;


        private float[] scanPatternParams;
        private float[] scanPatternParamsPrev;

        void Start(){
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<PointCloud2Msg>(topicName);
            scanDirVectors = GenerateScanVectors();
            scanPatternParamsPrev = new float[4];
        }

        void FixedUpdate(){
            scanPatternParams = new []{numHorizontalBeams, numVerticalBeams, horizontalFOV, verticalFOV};
            if (scanPatternParams != scanPatternParamsPrev){ //dont re-calculate lidar scan vectors if parameters unchanged
                scanDirVectors = GenerateScanVectors();
            }
            transform1 = transform;
            timeSinceScan += Time.deltaTime;
            if (timeSinceScan < 1.0f/hz){
                return;
            }

            transformScale = transform.lossyScale;
            Vector3[] points =  PerformScan(scanDirVectors);
            if (publishData){
                PointCloud2Msg msg = PointsToPointCloud2(points);
                ros.Publish(topicName, msg);
            }
            timeSinceScan = 0.0f;
            scanPatternParamsPrev = scanPatternParams;
        }




        private Vector3[] GenerateScanVectors()
        {

            float fidelityHorizontal = horizontalFOV / numHorizontalBeams;
            float fidelityVertical = verticalFOV / numVerticalBeams;

            Vector3[] scanVectors = new Vector3[numHorizontalBeams * numVerticalBeams];
            int index = 0;

            for (int i = 0; i < numHorizontalBeams; i++)
            {
                float hRot = 0.5f*(Mathf.PI - horizontalFOV) + fidelityHorizontal * i;

                for (int j = 0; j < numVerticalBeams; j++)
                {
                    float vRot = 0.5f*(Mathf.PI-verticalFOV)+(fidelityVertical * j);

                    float x = Mathf.Sin(vRot) * Mathf.Cos(hRot);
                    float y = Mathf.Cos(vRot);
                    float z = Mathf.Sin(vRot) * Mathf.Sin(hRot);

                    scanVectors[index] = new Vector3(x, y, z);
                    index++;
                }
            }
            return scanVectors;
        }


        private Vector3[] PerformScan(Vector3[] dirs)
        {
            int numPoints = dirs.Length;
            Vector3 nanVec = new Vector3(float.NaN, float.NaN, float.NaN);
            var commands = new NativeArray<RaycastCommand>(numPoints, Allocator.TempJob);
            var results = new NativeArray<RaycastHit>(numPoints, Allocator.TempJob);

            for (int i = 0; i < numPoints; i++)
            {
                Vector3 origin = transform1.position;
                Vector3 direction = transform1.rotation * dirs[i];
                commands[i] = new RaycastCommand(origin, direction, QueryParameters.Default, maxRange);
            }

            int batchSize = 500;
            JobHandle handle = RaycastCommand.ScheduleBatch(commands, results, batchSize, 1);
            handle.Complete();

            Vector3[] points = new Vector3[numPoints];
            for (int i = 0; i < numPoints; i++)
            {
                var hit = results[i];
                if (hit.collider != null && (transform1.position - hit.point).sqrMagnitude > minDistance * minDistance)
                {
                    Vector3 beam = transform1.InverseTransformPoint(hit.point);
                    points[i] = beam;
                    
                    if (drawRays)
                    {
                        Debug.DrawLine(transform1.position, transform.TransformPoint(beam), Color.red);
                    }
                }
                else{
                    points[i] = nanVec;
                }
            }

            results.Dispose();
            commands.Dispose();
            return points;
        }

        private PointCloud2Msg PointsToPointCloud2(Vector3[] points){
            PointCloud2Msg msg = new PointCloud2Msg();
            msg.header.frame_id = frameId;

            var publishTime = Clock.Now;
            var sec = publishTime;
            var nanosec = ((publishTime - Math.Floor(publishTime)) * Clock.k_NanoSecondsInSeconds);
            msg.header.stamp.sec = (int)sec;
            msg.header.stamp.nanosec = (uint)nanosec;

            // publishing as unordered cloud (height = 1). might reconsider later, idk.
            msg.height = 1;
            // currently the size of each message is non-constant, as the number of scan returns varies.
            // could consider having a constant size pointcloud and filling non-hits with NaN values.
            msg.width = (uint)points.Length;
        
            PointFieldMsg[] fields = new PointFieldMsg[3];
            fields[0] = new PointFieldMsg("x", 0, 7, 1); // "name", offset, datatype (7 = float), number of elements in field
            fields[1] = new PointFieldMsg("y", 4, 7, 1); // 4 byte offset, since float32 uses 4 bytes
            fields[2] = new PointFieldMsg("z", 8, 7, 1); // another 4 bytes as offset
            // theres an option for this field too, but i dont see a use for it currently
            // fields[3] = new PointFieldMsg("intensity", 12, 7, msg.width);
            msg.fields = fields;
        
            msg.point_step = (uint)fields.Length*4; // each point needs 12 bytes (3 float32's, when using fields x, y, z)
            msg.row_step = msg.point_step*msg.width;
            msg.is_dense = true;

            // finally, populate the data field, containing the actual points in bytes
            List<byte> dataList = new List<byte>();
            foreach(Vector3 point in points) {
                dataList.AddRange(BitConverter.GetBytes(point.z*transformScale.z));
                dataList.AddRange(BitConverter.GetBytes(-point.x*transformScale.x));
                dataList.AddRange(BitConverter.GetBytes(point.y*transformScale.y));
            }
            msg.data = dataList.ToArray();
            return msg;

        }
    }
}