class Vertex:
    def __init__(self,key):
        self.id = key
        self.connectedTo = {}
        self.distance = float('inf')  # initialize distance to infinity
        self.previous = None         # initialize previous to None

    def addNeighbor(self,nbr,weight=0):
        self.connectedTo[nbr] = weight

    def __str__(self):
        return str(self.id) + ' connectedTo: ' + str([x.id for x in self.connectedTo])

    def getConnections(self):
        return self.connectedTo.keys()

    def getId(self):
        return self.id

    def getWeight(self,nbr):
        return self.connectedTo[nbr]

    def getDistance(self):
        return self.distance

    def getPrevious(self):
        return self.previous

class Graph:
    def __init__(self):
        self.vertList = {}
        self.numVertices = 0

    def addVertex(self,key):
        self.numVertices = self.numVertices + 1
        newVertex = Vertex(key)
        self.vertList[key] = newVertex
        return newVertex

    def getVertex(self,n):
        if n in self.vertList:
            return self.vertList[n]
        else:
            return None

    def __contains__(self,n):
        return n in self.vertList
        
    def addEdge(self,f,t,weight=0):
        if f not in self.vertList:
            nv = self.addVertex(f)
        if t not in self.vertList:
            nv = self.addVertex(t)
        self.vertList[f].addNeighbor(self.vertList[t], weight)

    def getVertices(self):
        return self.vertList.keys()

    def __iter__(self):
        return iter(self.vertList.values())

    def dfs(self, start):
        """
        Performs a depth-first search starting at the specified vertex and returns a list of visited vertices
        with their distance from the start and their previous vertex in the DFS traversal.
        """
        visited = {}
        stack = [(self.vertList[start], 0, None)]
        while stack:
            vertex, dist, prev = stack.pop()
            if vertex not in visited:
                visited[vertex] = {'distance': dist, 'previous': prev}
                stack.extend((self.vertList[nbr.id], dist+1, vertex) for nbr in vertex.getConnections() if self.vertList[nbr.id] not in visited)
        return visited

    def bfs(self, start):
        """
        Performs a breadth-first search starting at the specified vertex and returns a list of visited vertices
        with their distance from the start and their previous vertex in the BFS traversal.
        """
        visited = {}
        queue = [(self.vertList[start], 0, None)]
        while queue:
            vertex, dist, prev = queue.pop(0)
            if vertex not in visited:
                visited[vertex] = {'distance': dist, 'previous': prev}
                queue.extend((self.vertList[nbr.id], dist+1, vertex) for nbr in vertex.getConnections() if self.vertList[nbr.id] not in visited)
        return visited

class DFSGraph(Graph):
    def __init__(self, graph, start):
        super().__init__()
        self.dfs_start = self.getVertex(start)
        self.visited = {}
        for vertex in graph:
            self.addVertex(vertex.getId())
        for vertex in graph:
            for nbr in vertex.getConnections():
                self.addEdge(vertex.getId(), nbr.getId())
    
    def __iter__(self):
        self.visited = {}
        for vertex in self.vertList.values():
            if vertex not in self.visited:
                yield from self.dfs(vertex)
                
    def dfs(self, start):
        stack = [start]
        self.visited[start] = True
        start.distance = 0
        while stack:
            current = stack.pop()
            for nbr in current.getConnections():
                if nbr not in self.visited:
                    nbr.previous = current
                    nbr.distance = current.distance + 1
                    self.visited[nbr] = True
                    stack.append(nbr)
            yield current
        
class BFSGraph(Graph):
    def __init__(self, graph, start):
        super().__init__()
        self.bfs_start = self.getVertex(start)
        self.visited = {}
        for vertex in graph:
            self.addVertex(vertex.getId())
        for vertex in graph:
            for nbr in vertex.getConnections():
                self.addEdge(vertex.getId(), nbr.getId())
    
    def __iter__(self):
        self.visited = {}
        for vertex in self.vertList.values():
            if vertex not in self.visited:
                yield from self.bfs(vertex)
                
    def bfs(self, start):
        queue = [start]
        self.visited[start] = True
        start.distance = 0
        while queue:
            current = queue.pop(0)
            for nbr in current.getConnections():
                if nbr not in self.visited:
                    nbr.previous = current
                    nbr.distance = current.distance + 1
                    self.visited[nbr] = True
                    queue.append(nbr)
            yield current

# Create a graph
g = Graph()
g.addEdge('A', 'B')
g.addEdge('A', 'C')
g.addEdge('B', 'D')
g.addEdge('B', 'E')
g.addEdge('C', 'F')
g.addEdge('C', 'G')
g.addEdge('D', 'H')
g.addEdge('E', 'I')
g.addEdge('F', 'J')
g.addEdge('G', 'K')

# Perform a DFS starting at vertex 'A'
dfs_result = g.dfs('A')
for vertex, info in dfs_result.items():
    print(vertex.getId(), info['distance'], info['previous'].getId() if info['previous'] else None)

# Perform a BFS starting at vertex 'A'
bfs_result = g.bfs('A')
for vertex, info in bfs_result.items():
    print(vertex.getId(), info['distance'], info['previous'].getId() if info['previous'] else None)

# Create a DFS graph and iterate over vertices
dfs_g = DFSGraph(g, 'A')
for vertex in dfs_g:
    print(vertex.getId(), vertex.distance, vertex.previous.getId() if vertex.previous else None)

# Create a BFS graph and iterate over vertices
bfs_g = BFSGraph(g, 'A')
for vertex in bfs_g:
    print(vertex.getId(), vertex.distance, vertex.previous.getId() if vertex.previous else None)
