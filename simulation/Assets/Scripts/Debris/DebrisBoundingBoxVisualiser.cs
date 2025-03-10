using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Serialization;

namespace DefaultNamespace
{
    // TODO: make this a singleton
    // fakes object detectoin (e.g. yolo) and draws a 2d bounding box in screen space
    public class DebrisBoundingBoxVisualiser : MonoBehaviour
    {
        [FormerlySerializedAs("camera")] [SerializeField]
        private Camera mainCamera;

        [SerializeField] private Color color = Color.red;

        [HideInInspector] public DebrisData[] Targets;

        // returns the transforms of objects with the debris tag in the camera view
        private static GameObject[] GetTargets()
        {
            var targets = new List<Transform>();
            return GameObject.FindGameObjectsWithTag("Debris");
        }

        private bool IsTargetVisible(Transform target)
        {
            if (!target) return false;

            var screenPos = mainCamera.WorldToScreenPoint(target.position);
            return screenPos.x > 0 && screenPos.x < mainCamera.pixelWidth && screenPos.y > 0 &&
                   screenPos.y < mainCamera.pixelHeight;
        }

        private void UpdateTargetVisibility()
        {
            for (var i = 0; i < Targets.Length; i++)
            {
                if (!Targets[i].transform) continue;

                Targets[i].isVisible = IsTargetVisible(Targets[i].transform);
            }
        }

        private void Start()
        {
            if (!mainCamera) mainCamera = Camera.main;

            var objects = GetTargets();
            Targets = objects.Select(obj => new DebrisData
            {
                isVisible = IsTargetVisible(obj.transform), meshFilter = obj.GetComponent<MeshFilter>(),
                transform = obj.transform
            }).ToArray();
        }


        private void Update()
        {
            UpdateTargetVisibility();
        }

        private void OnGUI()
        {
            // Set GUI color to red for the bounding box lines
            GUI.color = Color.red;

            foreach (var t in Targets)
            {
                if (!t.transform || !t.meshFilter || !t.isVisible) continue;

                var mesh = t.meshFilter.sharedMesh;
                if (!mesh) continue;

                // Initialize min and max screen points
                var minScreenPoint = new Vector3(Screen.width, Screen.height, 0);
                var maxScreenPoint = new Vector3(0, 0, 0);

                // Loop through each vertex in the mesh
                foreach (var vertex in mesh.vertices)
                {
                    // Convert the vertex to world space and then to screen space
                    var worldPoint = t.transform.TransformPoint(vertex);
                    var screenPoint = mainCamera.WorldToScreenPoint(worldPoint);

                    // Update min and max screen points
                    minScreenPoint.x = Mathf.Min(minScreenPoint.x, screenPoint.x);
                    minScreenPoint.y = Mathf.Min(minScreenPoint.y, screenPoint.y);
                    maxScreenPoint.x = Mathf.Max(maxScreenPoint.x, screenPoint.x);
                    maxScreenPoint.y = Mathf.Max(maxScreenPoint.y, screenPoint.y);
                }

                // Convert coordinates for GUI (Y is flipped in screen space)
                minScreenPoint.y = Screen.height - minScreenPoint.y;
                maxScreenPoint.y = Screen.height - maxScreenPoint.y;

                // Draw the 4 lines to form a bounding box
                DrawLine(new Vector2(minScreenPoint.x, minScreenPoint.y),
                    new Vector2(maxScreenPoint.x, minScreenPoint.y)); // Top
                DrawLine(new Vector2(minScreenPoint.x, maxScreenPoint.y),
                    new Vector2(maxScreenPoint.x, maxScreenPoint.y)); // Bottom
                DrawLine(new Vector2(minScreenPoint.x, minScreenPoint.y),
                    new Vector2(minScreenPoint.x, maxScreenPoint.y)); // Left
                DrawLine(new Vector2(maxScreenPoint.x, minScreenPoint.y),
                    new Vector2(maxScreenPoint.x, maxScreenPoint.y)); // Right

                // Draw the label above the box
                var labelPosition = new Vector2((minScreenPoint.x + maxScreenPoint.x) / 2, minScreenPoint.y - 20);
                GUI.Label(new Rect(labelPosition.x - 25, labelPosition.y, 50, 20), "debris",
                    new GUIStyle { alignment = TextAnchor.MiddleCenter, normal = { textColor = Color.red } });
            }

            // Reset GUI color to default
            GUI.color = Color.white;
        }

        // Helper function to draw a line in GUI
        private void DrawLine(Vector2 pointA, Vector2 pointB)
        {
            var d = pointB - pointA;
            var angle = Mathf.Rad2Deg * Mathf.Atan2(d.y, d.x);
            var width = 2; // Width of the line

            GUIUtility.RotateAroundPivot(angle, pointA);
            GUI.DrawTexture(new Rect(pointA.x, pointA.y, d.magnitude, width), Texture2D.whiteTexture);
            GUIUtility.RotateAroundPivot(-angle, pointA);
        }

        public struct DebrisData
        {
            public Transform transform;
            public MeshFilter meshFilter;
            public bool isVisible;
        }
    }
}