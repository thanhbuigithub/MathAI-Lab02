class Vertex:
    def __init__(self, key):
        self.id = key
        self.connectedTo = {}
        self.distance = float('inf')  # initialize distance to infinity
        self.previous = None  # initialize previous to None

    def addNeighbor(self, nbr, weight=0):
        self.connectedTo[nbr] = weight

    def __str__(self):
        return str(self.id) + ' connectedTo: ' + str([x.id for x in self.connectedTo])

    def getConnections(self):
        return self.connectedTo.keys()

    def getId(self):
        return self.id

    def getWeight(self, nbr):
        return self.connectedTo[nbr]

    def getDistance(self):
        return self.distance

    def getPrevious(self):
        return self.previous


class Graph:
    def __init__(self):
        self.vertList = {}
        self.numVertices = 0

    def addVertex(self, key):
        self.numVertices = self.numVertices + 1
        newVertex = Vertex(key)
        self.vertList[key] = newVertex
        return newVertex

    def getVertex(self, n):
        if n in self.vertList:
            return self.vertList[n]
        else:
            return None

    def __contains__(self, n):
        return n in self.vertList

    def addEdge(self, f, t, weight=0):
        if f not in self.vertList:
            nv = self.addVertex(f)
        if t not in self.vertList:
            nv = self.addVertex(t)
        self.vertList[f].addNeighbor(self.vertList[t], weight)

    def getVertices(self):
        return self.vertList.keys()

    def __iter__(self):
        return iter(self.vertList.values())

    def dfs(self, start, visited=None):
        """
        Performs a depth-first search starting at the specified vertex and returns a list of visited vertices
        with their distance from the start and their previous vertex in the DFS traversal.
        """
        if visited is None:
            visited = {}
        stack = [(self.vertList[start], 0, None)]
        while stack:
            vertex, dist, prev = stack.pop()
            if vertex.getId() not in visited:
                visited[vertex.getId()] = {'distance': dist, 'previous': prev}
                stack.extend((self.vertList[nbr.id], dist + 1, vertex) for nbr in vertex.getConnections() if
                             self.vertList[nbr.id] not in visited)
        return visited

    def bfs(self, start, visited=None):
        """
        Performs a breadth-first search starting at the specified vertex and returns a list of visited vertices
        with their distance from the start and their previous vertex in the BFS traversal.
        """
        if visited is None:
            visited = {}
        queue = [(self.vertList[start], 0, None)]
        while queue:
            vertex, dist, prev = queue.pop(0)
            if vertex not in visited:
                visited[vertex] = {'distance': dist, 'previous': prev}
                queue.extend((self.vertList[nbr.id], dist + 1, vertex) for nbr in vertex.getConnections() if
                             self.vertList[nbr.id] not in visited)
        return visited

    def fillOrder(self, vertex, visited, stack):
        visited[vertex] = True
        for nbr in vertex.getConnections():
            if nbr not in visited:
                self.fillOrder(nbr, visited, stack)
        stack = stack.append(vertex)

    # transpose the graph
    def transpose(self):
        g = Graph()

        for vertex in self:
            for nbr in vertex.getConnections():
                g.addEdge(nbr.getId(), vertex.getId())
        return g

    # Print strongly connected components
    def printScc(self):
        stack = []
        visited = {}

        for vertex in self:
            if vertex not in visited:
                self.fillOrder(vertex, visited, stack)

        gr = self.transpose()

        visited = {}
        scc_visited = []
        scc = []

        while stack:
            vertex = stack.pop()
            if vertex.getId() not in visited:
                visited = gr.dfs(vertex.getId(), visited)
                scc.append([vertex for vertex in visited if vertex not in scc_visited])
                scc_visited = [vertex for vertex in visited]
        print(scc)

INF = 1e9

class MaxFlowDFSGraph(Graph):
    def __init__(self, c, f, source, sink):
        super().__init__()
        self.source = self.getVertex(source)
        self.sink = self.getVertex(sink)
        
        self.c = c
        self.f = f
        
        self.visited = {}
        for vertex in c:
            self.addVertex(vertex.getId())
        for vertex in c:
            for nbr in vertex.getConnections():
                self.addEdge(vertex.getId(), nbr.getId())

    def __iter__(self):
        self.visited = {}
        for vertex in self.vertList.values():
            if vertex not in self.visited:
                yield from self.dfs(vertex)

    def dfs(self, u):
        self.visited.add(u)
        for nbr in u.getConnections():
            if nbr not in self.visited:
                nbr.previous = u
                
    def find_augment_from_to(self, source, sink):
        self.visited = {}
        # dùng thuật toán dfs tìm đường tăng luồng
        self.dfs(source)
        return sink in self.visited

    def increase_flow(self, source, sink):
        min_capacity = INF
        u = sink
        while u != source:
            prev = u.previous
            min_capacity = min(min_capacity, self.c.getVertex(prev).getWeight(u) - self.f.getVertex(prev).getWeight(u))
            u = prev
        
        while sink != source:
            prev = sink.previous
            f_prev_sink = self.f.getVertex(prev).getWeight(sink)
            f_sink_prev = self.f.getVertex(sink).getWeight(prev)
            self.f.getVertex(prev).addNeighbor(sink, f_prev_sink + min_capacity)
            self.f.getVertex(sink).addNeighbor(prev, f_sink_prev - min_capacity)
            sink = prev
    
    def max_flow(self):
        while self.find_augment_from_to(self.source, self.sink):
            self.increase_flow(self.source, self.sink)