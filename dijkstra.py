import arcpy

class DijkstrasBestPath:
    def __init__(self, graph, start_point, end_point):
        self.graf = graph
        self.end = end_point
        self.start = start_point
        self.path = []

    def dijkstra(self):
        S = set()
        Q = list(self.graf.nodes.keys())
        
        d={}
        p={}
            
        for node in self.graf.nodes.values():
            n = node.id
            d[n] = float('inf')
            p[n] = None
            Q.append(n)
        
        d[self.start] = 0
        
        while Q:
            u = min(Q, key=lambda w: d[w]) #wybiera najmniejsza wartosc w d
            Q.remove(u)
            S.add(u)
    
            u_node = self.graf.nodes[u]
            
            for edge, v_node in u_node.edges:
                v = v_node.id
                koszt = edge.cost
    
                if v in S:
                    continue
    
                if d[v] > d[u] + koszt:
                    d[v] = d[u] + koszt
                    p[v] = u

        return d, p

    def getPath(self):
        v = self.end
        d, p = self.dijkstra()
        
        while v is not None:
            self.path.append(v)
            v = p[v]

        self.path.reverse()

        if self.path[0] == self.start:
            return self.path
        else:
            return None

class Path:
    def __init__(self, graph, nodes): 
        self.nodes = nodes
        self.graph = graph
        self.path_edges = []
        
    def getEdges(self):
        for i in range(len(self.nodes)-1):
            n1 = self.nodes[i]
            n2 = self.nodes[i+1]

            for edge, neighbor in self.graph.nodes[n1].edges:
                if neighbor.id == n2:
                    self.path_edges.append(edge.id)
                    
        return self.path_edges
        
    
'''
p = DijkstrasBestPath(g, 1, 10)
path_nodes = p.getPath()
#print(path_nodes)

p_e = Path(g, path_nodes)
edgs = p_e.getEdges()
print(edgs)
'''
