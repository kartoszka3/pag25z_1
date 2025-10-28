import dotenv
import os
import math
import arcpy
try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None

dotenv.load_dotenv()

class Node:
    def __init__(self, node_id, x, y):
        self.id = node_id   # id węzła
        self.x = x          # współrzędna x
        self.y = y          # współrzędna y
        self.edges = []     # lista krawędzi (krawędź, wierzchołek)
        
    def __repr__(self):
        eids = ",".join([str(e.id) for e, v in self.edges])
        return f"N({self.id},({self.x},{self.y}),[{eids}])"
        
class Edge:
    def __init__(self, edge_id, cost, start, end):
        self.id = edge_id   # nr krawędzi
        self.cost = cost    # waga/długość
        self.start = start  # początek
        self.end = end      # koniec
        
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
        self.graph = Graph() # graf
        self.new_id = 0      # licznik id
        self.index = {}      # indeks punktów
        
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
              
def dijkstra(graph, start_id, end_id):
    # init
    S = set()                         # odwiedzone
    Q = set()                         # do sprawdzenia
    d = {}                            # dystanse
    p = {}                            # poprzednicy
    pe = {}                           # poprzednie krawędzie
    neighbor_count = 0                # licznik sąsiadów
    
    # ustaw odległości początkowe
    for node_id in graph.nodes:
        d[node_id] = math.inf
        p[node_id] = None
        pe[node_id] = None
    
    d[start_id] = 0
    Q.add(start_id)
    
    # --- Pętla główna ---
    while Q:
        # wybierz wierzchołek o najmniejszym d[v]
        v = min(Q, key=lambda node_id: d[node_id])
        Q.remove(v)
        
        if v == end_id:
            break
        
        v_node = graph.nodes[v]
        
        # przejrzyj wszystkich sąsiadów
        for edge, u_node in v_node.edges:
            neighbor_count += 1
            u = u_node.id
            if u in S:
                continue
                
            new_dist = d[v] + edge.cost
            if new_dist < d[u]:
                d[u] = new_dist
                p[u] = v
                pe[u] = edge
                Q.add(u)
        
        S.add(v)
    
    # --- Odtworzenie ścieżki ---
    path_nodes = []
    path_edges = []
    
    if d[end_id] < math.inf:
        node = end_id
        while node is not None:
            path_nodes.insert(0, node)
            edge = pe[node]
            if edge is not None:
                path_edges.insert(0, edge.id)
            node = p[node]
    else:
        print("Brak połączenia między wierzchołkami.")
        return None, None, None, None
    
    # --- Wyniki ---
    print(f"Najkrotsza odleglosc od {start_id} do {end_id}: {d[end_id]:.2f}")
    print(f"Sciezka po wierzcholkach: {path_nodes}")
    print(f"Krawedzie trasy: {path_edges}")
    print(f"Liczba odwiedzonych wierzcholkow: {len(S)}")
    print(f"Liczba przejrzanych sasiadow: {neighbor_count}")
    
    return path_nodes, path_edges, d[end_id], len(S), neighbor_count




def create_graph(workspace, layer):
    gc = GraphCreator()

    # wczytaj shapefile
    arcpy.env.workspace = workspace
    fields = ['FID', 'SHAPE@']
    with arcpy.da.SearchCursor(layer, fields) as cursor:
        for row in cursor:
            fid = row[0]
            shape = row[1]
            length = round(shape.length, 2)
            p1 = (round(shape.firstPoint.X, 2), round(shape.firstPoint.Y, 2))
            p2 = (round(shape.lastPoint.X, 2), round(shape.lastPoint.Y, 2))   
            gc.newEdge(fid, length, p1, p2)
        
    return gc.graph

g = create_graph(os.getenv('WORKSPACE_PATH'), os.getenv('WORKSPACE_NAME'))
print(g)
start_node = 1   # start
end_node = 15   # koniec
path_nodes, path_edges, dist, visited, neighbors = dijkstra(g, start_node, end_node)

# --- wizualizacja ---
def visualize(graph, path_nodes, path_edges, dist, out_png='shortest_path.png'):
    if plt is None:
        print("matplotlib not available — pomiń wizualizację.")
        return

    fig, ax = plt.subplots(figsize=(8, 8))

    for e in graph.edges.values():
        x1, y1 = e.start.x, e.start.y
        x2, y2 = e.end.x, e.end.y
        ax.plot([x1, x2], [y1, y2], color='#cccccc', linewidth=1, zorder=1)

        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2
        ax.text(mid_x, mid_y, str(e.id), color='gray', fontsize=8, ha='center', va='center', zorder=5)

    for n in graph.nodes.values():
        ax.scatter(n.x, n.y, color='black', s=15, zorder=2)

        ax.text(n.x, n.y + 0.3, str(n.id), color='blue', fontsize=9, ha='center', va='bottom', zorder=6)

    if path_edges and path_nodes:
        for eid in path_edges:
            e = graph.edges.get(eid)
            if e:
                x1, y1 = e.start.x, e.start.y
                x2, y2 = e.end.x, e.end.y
                ax.plot([x1, x2], [y1, y2], color='red', linewidth=2.5, zorder=3)
        path_x = [graph.nodes[nid].x for nid in path_nodes]
        path_y = [graph.nodes[nid].y for nid in path_nodes]
        ax.scatter(path_x, path_y, color='red', s=40, zorder=4)

    ax.set_aspect('equal', 'box')
    ax.set_title(f"Najkrótsza trasa {path_nodes}  dystans={dist:.2f}")
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.tight_layout()
    plt.savefig(out_png, dpi=150)
    plt.close(fig)
    print(f"Wizualizacja zapisana jako {out_png}")

# call visualization
visualize(g, path_nodes, path_edges, dist)