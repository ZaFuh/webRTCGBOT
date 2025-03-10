using System;
using UnityEngine;

namespace DefaultNamespace
{
    // temp class to delete debris once it touches the trigger collider at the front of the uuv

    public class DebrisCollector : MonoBehaviour
    {
        [SerializeField] private Collider trigger;

        private DebrisBoundingBoxVisualiser boundingBoxVisualiser;

        private static int debrisCollected = 0;

        private void Awake()
        {
            debrisCollected = 0;
        }

        private void Start()
        {
            if (trigger == null)
            {
                Debug.Log("No trigger collider found, looking for one");
                // get the trigger collider
                var colliders = GetComponents<Collider>();
                foreach (var collider in colliders)
                    if (collider.isTrigger)
                    {
                        trigger = collider;
                        Debug.Log("Found trigger collider: " + trigger.name);
                        break;
                    }

                if (trigger == null)
                {
                    Debug.LogError("No trigger collider found, please add one to the object");
                    enabled = false;
                }
            }

            boundingBoxVisualiser = FindObjectOfType<DebrisBoundingBoxVisualiser>();
        }

        private void OnTriggerEnter(Collider other)
        {
            if (other.CompareTag("Debris"))
            {
                Destroy(other.gameObject);
                Debug.Log($"Debris collected! Total: {++debrisCollected}");
            }
        }
    }
}