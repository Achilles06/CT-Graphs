import heapq

# Graph Class Definition
class Graph:
    def __init__(self):
        self.vertices = {}

    def add_vertex(self, vertex):
        if vertex not in self.vertices:
            self.vertices[vertex] = {}

    def add_edge(self, source, destination, weight):
        if source in self.vertices and destination in self.vertices:
            self.vertices[source][destination] = weight
            self.vertices[destination][source] = weight  # For undirected graph

    def get_neighbors(self, vertex):
        return self.vertices.get(vertex, {})

# Dijkstra's Algorithm
def dijkstra(graph, start):
    # Initialize dictionaries for shortest paths and previous nodes
    shortest_paths = {vertex: float('inf') for vertex in graph.vertices}
    shortest_paths[start] = 0
    previous_nodes = {vertex: None for vertex in graph.vertices}

    # Priority queue to explore nodes
    priority_queue = [(0, start)]
    
    while priority_queue:
        current_distance, current_vertex = heapq.heappop(priority_queue)

        # Skip processing if the node has been visited with a shorter path
        if current_distance > shortest_paths[current_vertex]:
            continue

        # Explore neighbors
        for neighbor, weight in graph.get_neighbors(current_vertex).items():
            distance = current_distance + weight

            # Only update the path if the new path is shorter
            if distance < shortest_paths[neighbor]:
                shortest_paths[neighbor] = distance
                previous_nodes[neighbor] = current_vertex
                heapq.heappush(priority_queue, (distance, neighbor))

    return shortest_paths, previous_nodes

# Test the Algorithm with an Example Graph
def test_dijkstra():
    graph = Graph()
    graph.add_vertex('A')
    graph.add_vertex('B')
    graph.add_vertex('C')
    graph.add_vertex('D')
    graph.add_vertex('E')

    graph.add_edge('A', 'B', 2)
    graph.add_edge('A', 'C', 4)
    graph.add_edge('B', 'C', 1)
    graph.add_edge('B', 'D', 7)
    graph.add_edge('C', 'E', 3)
    graph.add_edge('D', 'E', 1)

    shortest_paths, previous_nodes = dijkstra(graph, 'A')
    print("Shortest Paths from 'A':", shortest_paths)
    print("Previous Nodes for paths from 'A':", previous_nodes)

# Run the test
if __name__ == "__main__":
    test_dijkstra()
