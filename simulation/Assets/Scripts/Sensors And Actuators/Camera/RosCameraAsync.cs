using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using System;
using UnityEngine.Rendering;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

public class RosCameraAsync : MonoBehaviour
{
    public  RenderTexture rgbRenderTexture;
    public string topicName = "camera/image_raw";
    public string frameId = "camera_link_optical_frame";
    public bool publish = true;
    [Range(3.0f, 40.0f)]
    public float Hz = 15.0f;
    private ROSConnection ros;
    private Camera sensorCamera;
    private Texture2D rgbTexture2D;
    private float timeSincePublish;
    private HeaderMsg headerMsg = new HeaderMsg();
    private ImageMsg msg;

    void Start()
    {
        timeSincePublish = 0.0f;
        sensorCamera = gameObject.GetComponent<Camera>();
        sensorCamera.targetTexture = rgbRenderTexture;
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);
        headerMsg.frame_id = frameId;
    }

    void Update()
    {
        timeSincePublish += Time.deltaTime;
        //if (!publish) return;
        if (timeSincePublish > 1.0f / Hz)
        {
            RequestReadback(rgbRenderTexture);
            timeSincePublish = 0.0f;
        }
    }

    void RequestReadback(RenderTexture targetTexture)
    {
        AsyncGPUReadback.Request(targetTexture, 0, TextureFormat.RGB24, OnReadbackComplete);
    }

    void OnReadbackComplete(AsyncGPUReadbackRequest request)
    {
        if (request.hasError)
        {
            Debug.LogWarning("Failed to read back texture once");
            return;
        }

        if (rgbTexture2D == null || rgbTexture2D.width != sensorCamera.targetTexture.width || rgbTexture2D.height != sensorCamera.targetTexture.height)
        {
            rgbTexture2D = new Texture2D(sensorCamera.targetTexture.width, sensorCamera.targetTexture.height, TextureFormat.RGB24, false);
        }

        rgbTexture2D.LoadRawTextureData(request.GetData<byte>());
        rgbTexture2D.Apply();
        SendImage();
    }

    void SendImage()
    {
        var publishTime = Clock.Now;
        var sec = publishTime;
        var nanosec = (publishTime - Math.Floor(publishTime)) * Clock.k_NanoSecondsInSeconds;
        headerMsg.stamp.sec = (int)sec;
        headerMsg.stamp.nanosec = (uint)nanosec;

        msg = rgbTexture2D.ToImageMsg(headerMsg);
        ros.Publish(topicName, msg);
    }
}
