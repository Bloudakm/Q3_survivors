using System; 
using System.Collections.Generic; 
using System.Linq;


// graph representation of points (source, destination)
public class Graph
{

    public Dictionary<string, List<(string dest, int weight)>> AdjacencyList = new();

    // method to add an edge to the graph (euclidian distance from one point in graph to another)
    public void AddEdge(string source, string destination, int weight)
    {

        if (!AdjacencyList.ContainsKey(source))
            AdjacencyList[source] = new List<(string, int)>();
        AdjacencyList[source].Add((destination, weight));

        if (!AdjacencyList.ContainsKey(destination))
            AdjacencyList[destination] = new List<(string, int)>();
        AdjacencyList[destination].Add((source, weight)); // if roads are bidirectional 

    }

    // Dijkstra's algorithm to calculate route
    public int Dijkstra(string start, string end)
    {

        var dist = new Dictionary<string, int>();
        var pq = new PriorityQueue<(string node, int cost), int>();

        foreach (var node in AdjacencyList.Keys)
            dist[node] = int.MaxValue;

        dist[start] = 0;
        pq.Enqueue((start, 0), 0);

        while (pq.Count > 0)
        {

            var (current, cost) = pq.Dequeue();

            foreach (var (neighbor, weight) in AdjacencyList[current])
            {

                int newDist = dist[current] + weight;
                if (newDist < dist[neighbor])
                {
                    dist[neighbor] = newDist;
                    pq.Enqueue((neighbor, newDist), newDist);
                }
            }
        }
        return dist[end];
    }

    public class RideRequest
    {
        public string Pickup { get; set; }
        public string Dropoff { get; set; }
    }
    public interface IDynamicRouter
    {
        (List<global::System.String> route, global::System.Int32 cost) FindOptimalRoute(global::System.String busLocation, List<RideRequest> requests);
    }

    public class DynamicRouter : IDynamicRouter
    {

        private Graph _graph;
        public DynamicRouter(Graph graph)
        {
            _graph = graph;
        }
        public (List<string> route, int cost) FindOptimalRoute(string busLocation, List<RideRequest> requests)
        {

            var points = new List<string> { busLocation };
            foreach (var req in requests)
            {

                points.Add(req.Pickup);
                points.Add(req.Dropoff);
            }

            // Generate all valid routes with pickup before dropoff 
            var validRoutes = GenerateValidRoutes(busLocation, requests);
            int minCost = int.MaxValue;
            List<string> bestRoute = null;

            foreach (var route in validRoutes)
            {
                int totalCost = 0;
                for (int i = 0; i < route.Count - 1; i++)
                {

                    totalCost += _graph.Dijkstra(route[i], route[i + 1]);
                }
                if (totalCost < minCost)
                {

                    minCost = totalCost;
                    bestRoute = route;
                }
            }
            return (bestRoute, minCost);
        }
        private List<List<string>> GenerateValidRoutes(string start, List<RideRequest> requests)
        {

            var points = new List<string>();
            var constraints = new Dictionary<string, string>(); // dropoff -> pickup 
            foreach (var req in requests)
            {
                points.Add(req.Pickup);
                points.Add(req.Dropoff);
                constraints[req.Dropoff] = req.Pickup;
            }
            var result = new List<List<string>>();
            Permute(new List<string>(), points, new HashSet<string>(), constraints, start, result);
            return result;
        }

        private void Permute(List<string> path, List<string> points, HashSet<string> visited,
                             Dictionary<string, string> constraints, string current, List<List<string>> result)

        {
            if (visited.Count == points.Count)
            {
                result.Add(new List<string> { current }.Concat(path).ToList());
                return;
            }

            foreach (var point in points)
            {
                if (visited.Contains(point)) continue;
                if (constraints.ContainsKey(point) && !visited.Contains(constraints[point])) continue; // dropoff before pickup not allowed 
                visited.Add(point);
                path.Add(point);
                Permute(path, points, visited, constraints, current, result);
                path.RemoveAt(path.Count - 1);
                visited.Remove(point);
            }
        }
    }
}

var graph = new Graph(); 

// Define roads 

graph.AddEdge("A", "B", 4); 
graph.AddEdge("B", "C", 3); 
graph.AddEdge("C", "D", 5); 
graph.AddEdge("A", "D", 10); 
graph.AddEdge("B", "D", 2); 

var requests = new List<RideRequest> 
{ 
    new RideRequest { Pickup = "A", Dropoff = "C" }, 
    new RideRequest { Pickup = "B", Dropoff = "D" } 
}; 

var router = new DynamicRouter(graph); 
var (bestRoute, cost) = router.FindOptimalRoute("A", requests); 
Console.WriteLine("Best Route: " + string.Join(" → ", bestRoute)); 
Console.WriteLine("Total Cost: " + cost); 