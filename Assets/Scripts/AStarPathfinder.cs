using System;
using System.Collections.Generic;
using UnityEngine;

public class AStarPathfinder
{
    private class Node
    {
        public int x, z;
        public float gCost, hCost;
        public float FCost => gCost + hCost;
        public Node parent;

        public Node(int x, int z)
        {
            this.x = x;
            this.z = z;
        }
    }

    public static PathResult FindPath(float[,] traversabilityMatrix, int startX, int startZ, int endX, int endZ)
    {
        int width = traversabilityMatrix.GetLength(0);
        int height = traversabilityMatrix.GetLength(1);

        // Check bounds and if start/end are walkable
        if (startX < 0 || startX >= width || startZ < 0 || startZ >= height ||
            endX < 0 || endX >= width || endZ < 0 || endZ >= height ||
            traversabilityMatrix[startX, startZ] == 1f || traversabilityMatrix[endX, endZ] == 1f)
        {
            return new PathResult(false, new List<(int, int)>(), new List<Vector3>());
        }

        List<Node> openList = new List<Node>();
        HashSet<Vector2Int> closedSet = new HashSet<Vector2Int>();

        Node startNode = new Node(startX, startZ);
        startNode.hCost = GetDistance(startX, startZ, endX, endZ);
        openList.Add(startNode);

        while (openList.Count > 0)
        {
            // Find node with lowest F cost
            Node currentNode = openList[0];
            for (int i = 1; i < openList.Count; i++)
            {
                if (openList[i].FCost < currentNode.FCost)
                    currentNode = openList[i];
            }

            openList.Remove(currentNode);
            closedSet.Add(new Vector2Int(currentNode.x, currentNode.z));

            // Check if we reached the goal
            if (currentNode.x == endX && currentNode.z == endZ)
            {
                var path = ReconstructPath(currentNode);
                var worldPath = ConvertToWorldCoordinates(path);
                return new PathResult(true, path, worldPath);
            }

            // Check 8 neighbors (including diagonals)
            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dz == 0) continue; // Skip center
                    if (!(dx == 0 || dz == 0)) continue; // Skip center

                    int neighborX = currentNode.x + dx;
                    int neighborZ = currentNode.z + dz;

                    // Check bounds and walkability
                    if (neighborX < 0 || neighborX >= width || neighborZ < 0 || neighborZ >= height ||
                        traversabilityMatrix[neighborX, neighborZ] == 1f ||
                        closedSet.Contains(new Vector2Int(neighborX, neighborZ)))
                        continue;

                    float moveCost = (dx != 0 && dz != 0) ? 1.414f : 1f; // Diagonal cost
                    float newGCost = currentNode.gCost + moveCost;

                    // Check if this neighbor is already in open list
                    Node existingNode = openList.Find(n => n.x == neighborX && n.z == neighborZ);
                    
                    if (existingNode == null)
                    {
                        Node newNode = new Node(neighborX, neighborZ);
                        newNode.gCost = newGCost;
                        newNode.hCost = GetDistance(neighborX, neighborZ, endX, endZ);
                        newNode.parent = currentNode;
                        openList.Add(newNode);
                    }
                    else if (newGCost < existingNode.gCost)
                    {
                        existingNode.gCost = newGCost;
                        existingNode.parent = currentNode;
                    }
                }
            }
        }

        // No path found
        return new PathResult(false, new List<(int, int)>(), new List<Vector3>());
    }

    private static float GetDistance(int x1, int z1, int x2, int z2)
    {
        int dx = Mathf.Abs(x1 - x2);
        int dz = Mathf.Abs(z1 - z2);
        return Mathf.Sqrt(dx * dx + dz * dz);
    }

    private static List<(int, int)> ReconstructPath(Node endNode)
    {
        List<(int, int)> path = new List<(int, int)>();
        Node current = endNode;

        while (current != null)
        {
            path.Add((current.x, current.z));
            current = current.parent;
        }

        path.Reverse();
        return path;
    }

    private static List<Vector3> ConvertToWorldCoordinates(List<(int, int)> gridPath)
    {
        List<Vector3> worldPath = new List<Vector3>();
        const float scalingFactor = 10f;

        foreach (var (x, z) in gridPath)
        {
            worldPath.Add(new Vector3(x * scalingFactor, 0f, z * scalingFactor));
        }

        return worldPath;
    }

    public static void Plot(PathResult path, Vector3 localOffset = default)
    {
        if (localOffset == default) localOffset = Vector3.zero;

        var physical_trajectory = path.WorldPath;

        if (physical_trajectory.Count == 0)
        {
            Debug.Log("Trajectory is zero, cannot print");
            return;
        }

        Vector3 start = new Vector3(physical_trajectory[0].x,1f,physical_trajectory[0].z);
        Vector3 end = new Vector3(0,1f,0);

        for(int i = 1; i<physical_trajectory.Count - 1; i++)
        {
            end[0] = physical_trajectory[i].x;
            end[2] = physical_trajectory[i].z;

            Debug.DrawLine(start, end, Color.blue, 10f);
            // CreatePlate(physical_trajectory[i]);
            start = end;
        }
    }

    public static void CreateColliders(List<Vector3> physical_trajectory)
    {
        GameObject triggerParent = new GameObject("Triggers");

        var triggerSize = new Vector3(9f, 1f, 9f);
        int triggerLayer = LayerMask.NameToLayer("Trigger");

        // foreach(var pos in physical_trajectory)
        for (int i = 1; i < physical_trajectory.Count-1; i++)
        {
            var pos = physical_trajectory[i];
            GameObject triggerObj = new GameObject("BoxTrigger");
            triggerObj.transform.SetParent(triggerParent.transform);
            triggerObj.transform.localPosition = new Vector3(pos.x, 1f, pos.z);
            
            // Create the trigger collider
            BoxCollider collider = triggerObj.AddComponent<BoxCollider>();
            collider.size = triggerSize;
            collider.isTrigger = true;
            
            // Set the layer
            triggerObj.layer = triggerLayer;
        }
    }
}

public class PathResult
{
    public bool PathFound { get; }
    public List<(int x, int z)> GridPath { get; }
    public List<Vector3> WorldPath { get; }

    public PathResult(bool pathFound, List<(int x, int z)> gridPath, List<Vector3> worldPath)
    {
        PathFound = pathFound;
        GridPath = gridPath;
        WorldPath = worldPath;
    }
}

