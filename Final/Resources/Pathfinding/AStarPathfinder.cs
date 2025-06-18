using System;
using System.Collections.Generic;

public class AStarPathfinder
{
    private int width, height;
    private Node1[,] grid;

    public AStarPathfinder(int width, int height, bool[,] walkableMap)
    {
        this.width = width;
        this.height = height;
        grid = new Node1[width, height];
        for (int x = 0; x < width; x++)
            for (int y = 0; y < height; y++)
                grid[x, y] = new Node1(x, y, walkableMap[x, y]);
    }

    public List<Node1> FindPath((int x, int y) start, (int x, int y) goal)
    {
        var startNode1 = grid[start.x, start.y];
        var goalNode1 = grid[goal.x, goal.y];
        var open = new List<Node1> { startNode1 };
        var closed = new HashSet<Node1>();

        foreach (var Node1 in grid)
        {
            Node1.G = Node1.H = 0;
            Node1.Parent = null;
        }

        while (open.Count > 0)
        {
            open.Sort((a, b) => a.F.CompareTo(b.F));
            var current = open[0];
            if (current.Equals(goalNode1))
                return Reconstruct(current);

            open.Remove(current);
            closed.Add(current);

            foreach (var neighbor in GetNeighbors(current))
            {
                if (!neighbor.Walkable || closed.Contains(neighbor)) continue;
                int tentativeG = current.G + 1;
                if (tentativeG < neighbor.G || !open.Contains(neighbor))
                {
                    neighbor.G = tentativeG;
                    neighbor.H = Math.Abs(neighbor.X - goalNode1.X) + Math.Abs(neighbor.Y - goalNode1.Y);
                    neighbor.Parent = current;
                    if (!open.Contains(neighbor)) open.Add(neighbor);
                }
            }
        }

        return null;
    }

    private List<Node1> Reconstruct(Node1 end)
    {
        var path = new List<Node1>();
        for (var n = end; n != null; n = n.Parent)
            path.Add(n);
        path.Reverse();
        return path;
    }

    private IEnumerable<Node1> GetNeighbors(Node1 Node1)
    {
        int[] dx = { 0, 1, 0, -1 };
        int[] dy = { 1, 0, -1, 0 };

        for (int i = 0; i < 4; i++)
        {
            int nx = Node1.X + dx[i];
            int ny = Node1.Y + dy[i];
            if (nx >= 0 && nx < width && ny >= 0 && ny < height)
                yield return grid[nx, ny];
        }
    }
}
