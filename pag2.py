import arcpy

class Node:
    def __init__(self, node_id, x, y):
        self.id = node_id   # id
        self.x = x          # x-coordinate
        self.y = y          # x-coordinate
        self.edges = []     # incident edges, set later: list of (edge,vertex)
        
    def __repr__(self):
        eids = ",".join([str(e.id) for e, v in self.edges])
        return f"N({self.id},({self.x},{self.y}),[{eids}])"
        
class Edge:
    def __init__(self, edge_id, cost, start, end):
        self.id = edge_id   # id
        self.cost = cost    # cost of the edge        
        self.start = start  # from Node
        self.end = end      # to Node
        
    def __repr__(self):
        sid = self.start.id if self.start is not None else "None"
        eid = self.end.id if self.end is not None else "None"
        return f"E({self.id},{sid},{eid})"
    
class Graph:
    def __init__(self):
        self.edges = {}  # edge_id -> Edge
        self.nodes = {}  # node_id -> Node
        
    def __repr__(self):
        ns = ",".join([str(n) for n in self.nodes.values()]) # all nodes
        es = ",".join([str(e) for e in self.edges.values()]) # all edges
        return f"Graph:\n  Nodes: [{ns}]\n  Edges: [{es}]"

class GraphCreator:
    def __init__(self):
        self.graph = Graph() # new graph
        self.new_id = 0      # to assign node ids
        self.index = {}      # index: (x, y) -> Node
        
    def getNewId(self):
        self.new_id = self.new_id + 1
        return self.new_id        
    
    def newNode(self, p):
        if p not in self.index:
            n = Node(self.getNewId(), p[0], p[1])  # create a node
            self.graph.nodes[n.id] = n             # add the node to collection
            self.index[p] = n                      # add the node to the index
        return self.index[p]
        
    def newEdge(self, id, length, p1, p2):
        # create nodes
        n1 = self.newNode(p1)
        n2 = self.newNode(p2)

        # create a new edge
        e = Edge(id, length, n1, n2)          
        self.graph.edges[id] = e

        # connect edges and nodes
        n1.edges.append((e, n2))
        n2.edges.append((e, n1))
              
    
def create_graph(workspace, layer):
    gc = GraphCreator()

    # Read data
    arcpy.env.workspace = workspace
    cursor = arcpy.SearchCursor(layer)
    for row in cursor:
        length = round(row.Shape.length, 2)
        p1 = (round(row.Shape.firstPoint.X, 2), round(row.Shape.firstPoint.Y, 2))
        p2 = (round(row.Shape.lastPoint.X, 2), round(row.Shape.lastPoint.Y, 2))   
        gc.newEdge(row.FID, length, p1, p2)
        
    return gc.graph

g = create_graph(r'C:\zajecia_3', 'jezdnie')
#print(g)


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