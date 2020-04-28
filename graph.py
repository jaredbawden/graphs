'''
Jared Bawden
Project 8 - Graphs
Graph project using adjacency lists
'''

class Graph:
    ''' Graph class '''

    def __init__(self):
        ''' initializer for Graph class '''
        self.vertices = {}

    def add_vertex(self, label):
        ''' adds a vertex witha  specified label. Returns the graph. Must be a string. '''
        if not isinstance(label, str):
            raise ValueError('vertex label must be a string')
        if label not in self.vertices:
            self.vertices[label] = []
        return self

    def add_edge(self, src, dest, w):
        ''' adds an edge from the vertex src to vertex dest with weight w. return the graph. '''
        if src not in self.vertices:
            raise ValueError('src not defined vertex')
        if dest not in self.vertices:
            raise ValueError('dest not defined vertex')
        if not isinstance(w, (float, int)):
            raise ValueError('w must be an int or float')
        if w < 0:
            raise ValueError('w must not be a negative')
        self.vertices[src].append((dest, float(w)))
        return self

    def get_weight(self, src, dest):
        ''' returns the weight on edge src-dest '''
        if src not in self.vertices:
            raise ValueError('src not defined vertex')
        if dest not in self.vertices:
            raise ValueError('dest not defined vertex')
        for key, value in self.vertices[src]:
            if key == dest:
                return value
        return float('Inf')

    def dfs(self, starting_vertex):
        ''' returns a generator for traversing the graph in depth-first order '''
        if starting_vertex not in self.vertices:
            raise ValueError('starting_vertex must be defined')
        frontier_queue = [starting_vertex]
        discovered_set = set(starting_vertex)
        while len(frontier_queue) != 0:
            currentV = frontier_queue.pop()
            yield currentV
            for adjV, _ in self.vertices[currentV]:
                if adjV not in discovered_set:
                    frontier_queue.append(adjV)
                    discovered_set.add(adjV)

    def bfs(self, starting_vertex):
        ''' returns a generator for traversing the graph in breadth-first order '''
        if starting_vertex not in self.vertices:
            raise ValueError('starting_vertex must be defined')
        frontier_queue = [starting_vertex]
        discovered_set = set(starting_vertex)
        while len(frontier_queue) != 0:
            currentV = frontier_queue.pop(0)
            yield currentV
            for adjV, _ in self.vertices[currentV]:
                if adjV not in discovered_set:
                    frontier_queue.append(adjV)
                    discovered_set.add(adjV)

    def dijkstra_shortest_path(self, src, dest=None):
        ''' returns a dictionary of the shortest weighted path between src and other vertices
            the key is the vertex, the value is a tuple (path length, vertices from key to src) '''
        if src not in self.vertices:
            raise ValueError('vertex not in graph')
        unvisited_set = set(self.vertices)
        visited_set = set()
        distance = {}
        previous = {}
        for vertex in unvisited_set:
            distance[vertex] = float('Inf')
            previous[vertex] = []
        distance[src] = float(0)
        previous[src].append(src)
        currentV = src
        while len(unvisited_set) != 0:
            self.vertices[currentV].sort(key=lambda x: x[1])
            unvisited_set.remove(currentV)
            visited_set.add(currentV)
            for adjV in self.vertices[currentV]:
                total_distance = distance[currentV] + adjV[1]
                if total_distance < distance[adjV[0]]:
                    distance[adjV[0]] = total_distance
                    previous[adjV[0]] = []
                    for vertex in previous[currentV]:
                        previous[adjV[0]].append(vertex)
                    previous[adjV[0]].insert(0, adjV[0])
            if self.vertices[currentV] == [] and currentV != src:
                currentV = previous[currentV][1]
            for adjV, _ in self.vertices[currentV]:
                if adjV in unvisited_set:
                    currentV = adjV
                    break
            if currentV in visited_set:
                break
        return_dict = {}
        for vertex in self.vertices:
            return_dict[vertex] = (distance[vertex], previous[vertex])
        if dest is None:
            return return_dict
        return return_dict[dest]

    def __str__(self):
        ''' produces a string representation of the graph '''
        return_string = f'numVertices: {len(self.vertices)}\n'
        return_string += 'Vertex\tAdjacency List\n'
        for vertex in self.vertices:
            return_string += f'{vertex}\t{self.vertices[vertex]}\n'
        return return_string

'''
def main():
    g = Graph()

    g.add_vertex('A')
    g.add_vertex('B')
    g.add_vertex('C')
    g.add_vertex('D')
    g.add_vertex('E')
    g.add_vertex('F')

    g.add_edge('A', 'B', 2)
    g.add_edge('A', 'F', 9)

    g.add_edge('B', 'C', 8)
    g.add_edge('B', 'D', 15)
    g.add_edge('B', 'F', 6)

    g.add_edge('C', 'D', 1)

    g.add_edge('E', 'C', 7)
    g.add_edge('E', 'D', 3)

    g.add_edge('F', 'E', 3)

    print('A')
    print(g.dijkstra_shortest_path('A'))
    print('B')
    print(g.dijkstra_shortest_path('B'))
    print('C')
    print(g.dijkstra_shortest_path('C'))
    print('D')
    print(g.dijkstra_shortest_path('D'))
    print('E')
    print(g.dijkstra_shortest_path('E'))
    print('F')
    print(g.dijkstra_shortest_path('F'))
    #print(g.dijkstra_shortest_path('F', 'D'))

if __name__ == "__main__":
    main()
'''
