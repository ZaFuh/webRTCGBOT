using UnityEngine;
using System.Collections;
using UnityEngine.Rendering.HighDefinition;
using Random = UnityEngine.Random;

public class DebrisSpawner : MonoBehaviour
{
    [Header("Bounds")] [SerializeField] private Vector2 topLeftBound = new(-5, -10);
    [SerializeField] private Vector2 bottomRightBound = new(5, 10);
    [SerializeField] private float height = 1;
    [SerializeField] private bool showBounds = true;

    [Header("Debris")] [SerializeField] private GameObject debrisPrefab;
    [SerializeField] private WaterSurface waterSurface;
    [SerializeField] private int initialSpawnCountMin = 10;
    [SerializeField] private int initialSpawnCountMax = 20;

    // section
    [Header("Continuous Spawning")] [SerializeField]
    private bool keepSpawning = false;

    [SerializeField] private float spawnIntervalSeconds = 15f;
    [SerializeField] private int minSpawnCount = 1;
    [SerializeField] private int maxSpawnCount = 3;

    private void SpawnDebris(int minCount, int maxCount)
    {
        // get count
        var count = Random.Range(minCount, maxCount);
        Debug.Log($"Spawning {count} debris");

        // spawn debris
        for (var i = 0; i < count; i++)
        {
            var x = Random.Range(topLeftBound.x, bottomRightBound.x);
            var z = Random.Range(topLeftBound.y, bottomRightBound.y);
            var pos = new Vector3(x, height, z);
            var go = Instantiate(debrisPrefab, pos, Quaternion.identity);
            go.GetComponent<Submersion>().waterSurface = waterSurface;
        }
    }

    // Start is called before the first frame update
    private void Start()
    {
        SpawnDebris(initialSpawnCountMin, initialSpawnCountMax);
        if (keepSpawning) StartCoroutine(SpawnDebrisCoroutine());
    }

    private IEnumerator SpawnDebrisCoroutine()
    {
        Debug.Log("Continuous debris spawning enabled");
        while (keepSpawning)
        {
            SpawnDebris(minSpawnCount, maxSpawnCount);
            yield return new WaitForSeconds(spawnIntervalSeconds);
        }
    }

    private void OnDrawGizmos()
    {
        if (!showBounds) return;

        Gizmos.color = Color.yellow;

        var topLeft = new Vector3(topLeftBound.x, height, topLeftBound.y);
        var topRight = new Vector3(bottomRightBound.x, height, topLeftBound.y);
        var bottomLeft = new Vector3(topLeftBound.x, height, bottomRightBound.y);
        var bottomRight = new Vector3(bottomRightBound.x, height, bottomRightBound.y);

        Gizmos.DrawLine(topLeft, topRight);
        Gizmos.DrawLine(topRight, bottomRight);
        Gizmos.DrawLine(bottomRight, bottomLeft);
        Gizmos.DrawLine(bottomLeft, topLeft);

        // draw squares at corners
        Gizmos.DrawCube(topLeft, Vector3.one);
        Gizmos.DrawCube(topRight, Vector3.one);
        Gizmos.DrawCube(bottomLeft, Vector3.one);
        Gizmos.DrawCube(bottomRight, Vector3.one);
    }
}