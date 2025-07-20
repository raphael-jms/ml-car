using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Random = System.Random;

public class GridTrajectoryGenerator
{
    private readonly int width;
    private readonly int height;
    private readonly Random random;
    private readonly HashSet<(int x, int y)> visited;
    private readonly List<(int x, int y)> trajectory;
    private readonly List<(float x, float y)> physical_trajectory;
    private readonly int LayerWall;
    private List<GameObject> cubeList;
    private GameObject lastCubeParent;
    private GameObject lastTriggerParent;

    public GridTrajectoryGenerator(int width, int height, int? seed = null)
    {
        this.width = width;
        this.height = height;
        this.random = seed.HasValue ? new Random(seed.Value) : new Random();
        this.visited = new HashSet<(int x, int y)>();
        this.trajectory = new List<(int x, int y)>();
        this.physical_trajectory = new List<(float x, float y)>();
        this.LayerWall = LayerMask.NameToLayer("Wall");
        this.cubeList = new List<GameObject>();
        this.lastCubeParent = null;
        this.lastTriggerParent = null;
    }

    public List<(float x, float y)> GenerateTrajectory(int maxLength = -1)
    {
        visited.Clear();
        trajectory.Clear();
        physical_trajectory.Clear();

        if (maxLength == -1)
            maxLength = width * height;

        // Start at a random position
        var startX = random.Next(0, width);
        var startY = random.Next(0, height);
        var current = (startX, startY);

        trajectory.Add(current);
        visited.Add(current);

        while (trajectory.Count < maxLength)
        {
            var validMoves = GetValidMoves(current);
            
            if (validMoves.Count == 0)
                break; // No valid moves, trajectory complete

            // Choose a random valid move
            var nextMove = validMoves[random.Next(validMoves.Count)];
            current = nextMove;
            
            trajectory.Add(current);
            visited.Add(current);
        }

        foreach(var pos in trajectory)
        {
            physical_trajectory.Add(new(10f * pos.x, 10f * pos.y));
        }

        return new List<(float x, float y)>(physical_trajectory);
    }

    private List<(int x, int y)> GetValidMoves((int x, int y) current)
    {
        var validMoves = new List<(int x, int y)>();
        
        // Four cardinal directions
        var directions = new[]
        {
            (0, 1),   // Up
            (0, -1),  // Down
            (1, 0),   // Right
            (-1, 0)   // Left
        };

        foreach (var (dx, dy) in directions)
        {
            var newX = current.x + dx;
            var newY = current.y + dy;
            var newPos = (newX, newY);

            // Check bounds
            if (newX < 0 || newX >= width || newY < 0 || newY >= height)
                continue;

            // Check if already visited (prevents self-intersection)
            if (visited.Contains(newPos))
                continue;

            // Check for parallel movement prevention
            if (WouldCreateAdjacentPath(newPos))
                continue;

            validMoves.Add(newPos);
        }

        return validMoves;
    }

    private bool WouldCreateAdjacentPath((int x, int y) next)
    {
        // manually iterate through the list as the last step is always adjacent -> hash list does not work 
        var directions = new[]
        {
            (0, 1),   // Up
            (0, -1),  // Down
            (1, 0),   // Right
            (-1, 0)   // Left
        };

        foreach (var dir in directions)
        {
            (int, int) candidate = (next.x + dir.Item1, next.y + dir.Item2);
            for(int i = 0; i<trajectory.Count - 2; i++)
            {
                if (trajectory[i].x == candidate.Item1 && trajectory[i].y == candidate.Item2)
                {
                    return true;
                }
            }
        }

        return false;
    }

    public void PlotTrajectory()
    {
        if (trajectory.Count == 0)
        {
            Debug.Log("Trajectory is zero, cannot print");
            return;
        }

        Vector3 start = new Vector3(trajectory[0].x,29f,trajectory[0].y);
        Vector3 end = new Vector3(0,29f,0);

        for(int i = 1; i<trajectory.Count - 1; i++)
        {
            end[0] = trajectory[i].x;
            end[2] = trajectory[i].y;

            Debug.DrawLine(start, end, Color.blue, 10f);
            start = end;
        }
    }

    public void CreatePlate((float x, float z) pos)
    {
        GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.transform.position = new Vector3(pos.x, 0f, pos.z); // Scale to physical coordinates
        cube.transform.localScale = new Vector3(9f, 0.1f, 9f);
    }

    public void PlotPhysical(Vector3 localOffset = default)
    {
        if (localOffset == default) localOffset = Vector3.zero;

        if (physical_trajectory.Count == 0)
        {
            Debug.Log("Trajectory is zero, cannot print");
            return;
        }

        Vector3 start = new Vector3(physical_trajectory[0].x,1f,physical_trajectory[0].y);
        Vector3 end = new Vector3(0,1f,0);

        for(int i = 1; i<physical_trajectory.Count - 1; i++)
        {
            end[0] = physical_trajectory[i].x;
            end[2] = physical_trajectory[i].y;

            Debug.DrawLine(start, end, Color.blue, 10f);
            CreatePlate(physical_trajectory[i]);
            start = end;
        }
    }
    public string PrintElements()
    {
        string traj = "";
        foreach(var el in trajectory)
        {
            traj += ", (" + el.x + ", " + el.y + ")";
        }
        return traj;
    }

    public void CreateCorridorWalls(Transform parent = null)
    {
        if (lastCubeParent != null)
        {
            UnityEngine.Object.Destroy(lastCubeParent);
            cubeList.Clear();
        }

        if (trajectory.Count == 0)
        {
            Debug.Log("No trajectory to create corridor for");
            return;
        }

        // Step 1: Iterate through trajectory and add adjacent grid elements to potential walls
        HashSet<(int x, int y)> potentialWalls = new HashSet<(int x, int y)>();
        
        var directions = new[]
        {
            (0, 1),   // Up
            (0, -1),  // Down
            (1, 0),   // Right
            (-1, 0)   // Left
        };

        foreach (var point in trajectory)
        {
            foreach (var (dx, dy) in directions)
            {
                var newX = point.x + dx;
                var newY = point.y + dy;
                var adjacentPos = (newX, newY);

                // Check bounds
                if (newX >= 0 && newX < width && newY >= 0 && newY < height)
                {
                    potentialWalls.Add(adjacentPos);
                }
            }
        }

        // Step 2: Remove elements that are in visited
        potentialWalls.ExceptWith(visited);

        // Step 3: Iterate through the remaining potential walls and create cubes
        GameObject cubeParent = new GameObject("CorridorWalls");
        lastCubeParent = cubeParent;
        if (parent != null)
            cubeParent.transform.SetParent(parent);
            cubeParent.transform.localPosition = Vector3.zero;

        foreach (var (x, y) in potentialWalls)
        {
            GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.transform.SetParent(cubeParent.transform);
            cube.transform.localPosition = new Vector3(10f * x, 5f, 10f * y);
            cube.transform.localScale = Vector3.one * 10f;
            cube.name = $"Wall_{x}_{y}";
            cube.layer = LayerWall;

            // Set wall material
            Material material = new Material(Shader.Find("Standard"));
            material.color = Color.gray;
            cube.GetComponent<Renderer>().material = material;

            cubeList.Add(cube);
        }

        Debug.Log($"Created {potentialWalls.Count} wall cubes");
    }

    private void SetCubeMaterial(GameObject cube, Color color)
    {
        Material material = new Material(Shader.Find("Standard"));
        material.color = color;
        cube.GetComponent<Renderer>().material = material;
    }

    // public void CreateColliders(Transform parent = null)
    // {
    //     GameObject triggerParent = new GameObject("Triggers");
    //     if (parent != null)
    //         triggerParent.transform.SetParent(parent);
    //         triggerParent.transform.localPosition = Vector3.zero;

    //     var triggerSize = new Vector3(9f,1f,9f);
    //     LayerMask goalMarker = LayerMask.NameToLayer("Trigger");

    //     foreach(var pos in physical_trajectory)
    //     {
    //         GameObject triggerObj = new GameObject("BoxTrigger");
    //         triggerObj.transform.SetParent(triggerParent.transform);
    //         var position = new Vector3(pos.x, 1f, pos.y);
    //         triggerObj.transform.localPosition = position;
    //         Collider collider = CreateTrigger(position, triggerSize, TriggerShape.Box, onEnter, null, null, -1, null, null, "triggerCollider");
    //         collider.isTrigger = true;
    //         CollisionDetector detector = triggerObj.AddComponent<CollisionDetector>();
    //         detector.detectionLayers = goalMarker;
    //     }
    // }
    public void CreateColliders(Transform parent = null)
    {
        if (lastTriggerParent != null)
        {
            UnityEngine.Object.Destroy(lastTriggerParent);
        }

        GameObject triggerParent = new GameObject("Triggers");
        lastTriggerParent = triggerParent;
        if (parent != null)
        {
            triggerParent.transform.SetParent(parent);
            triggerParent.transform.localPosition = Vector3.zero;
        }

        var triggerSize = new Vector3(9f, 1f, 9f);
        int triggerLayer = LayerMask.NameToLayer("Trigger");

        // foreach(var pos in physical_trajectory)
        for (int i = 1; i < physical_trajectory.Count-1; i++)
        {
            var pos = physical_trajectory[i];
            GameObject triggerObj = new GameObject("BoxTrigger");
            triggerObj.transform.SetParent(triggerParent.transform);
            triggerObj.transform.localPosition = new Vector3(pos.x, 1f, pos.y);
            
            // Create the trigger collider
            BoxCollider collider = triggerObj.AddComponent<BoxCollider>();
            collider.size = triggerSize;
            collider.isTrigger = true;
            
            // Set the layer
            triggerObj.layer = triggerLayer;
        }
    }
}

// public class CreateTrajectory : MonoBehaviour
// {
//     void Start()
//     {
//         var generator = new GridTrajectoryGenerator(10, 10);
//         var traj = generator.GenerateTrajectory();
//         Debug.Log(generator.PrintElements());
//         // generator.PlotTrajectory();
//         generator.PlotPhysical();
//         generator.CreateCorridorWalls();
//     }
// }