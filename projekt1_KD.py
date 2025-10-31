import dotenv
import os
import math
import arcpy
from arcpy import SpatialReference
from arcpy.management import Project

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

    def dijkstra(self, start_id, end_id):
        # init
        S = set()                         # odwiedzone
        Q = set()                         # do sprawdzenia
        d = {}                            # dystanse
        p = {}                            # poprzednicy
        pe = {}                           # poprzednie krawędzie
        neighbor_count = 0                # licznik sąsiadów
        
        # ustaw odległości początkowe
        for node_id in self.graph.nodes:
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
            
            v_node = self.graph.nodes[v]
            
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
            return None, None, None, None, None
        
        # --- Wyniki ---
        print(f"Najkrotsza odleglosc od {start_id} do {end_id}: {d[end_id]:.2f}")
        print(f"Sciezka po wierzcholkach: {path_nodes}")
        print(f"Krawedzie trasy: {path_edges}")
        print(f"Liczba odwiedzonych wierzcholkow: {len(S)}")
        print(f"Liczba przejrzanych sasiadow: {neighbor_count}")
        
        return path_nodes, path_edges, d[end_id], len(S), neighbor_count
    
    def find_closest_nodes(self, point1, point2):
        """
        Finds closest nodes in graph for two given points
        Args:
            point1: tuple (lng,lat) - coordinates in WGS84
            point2: tuple (lng,lat) - coordinates in WGS84
        Returns:
            tuple (node_id1, node_id2) - IDs of closest nodes
        """
        def transform_point(point):
            # Create WGS84 spatial reference
            wgs84 = SpatialReference(4326)
            # Create target spatial reference (Poland CS92)
            target_sr = SpatialReference(2180)
            
            # Create point geometry
            point_geom = arcpy.PointGeometry(
                arcpy.Point(point[0], point[1]), 
                wgs84
            )
            
            # Project point
            projected_point = point_geom.projectAs(target_sr)
            return (projected_point.firstPoint.X, projected_point.firstPoint.Y)
            
        def distance(p1, p2):
            return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5
        
        # Transform input points to match graph coordinate system
        transformed_p1 = transform_point(point1)
        transformed_p2 = transform_point(point2)
        
        print(f"\nTransformed coordinates:")
        print(f"Point 1: WGS84 {point1} -> CS92 {transformed_p1}")
        print(f"Point 2: WGS84 {point2} -> CS92 {transformed_p2}")
        
        closest_to_p1 = None
        closest_to_p2 = None
        min_dist1 = float('inf')
        min_dist2 = float('inf')
        
        # Iterate through all nodes to find closest ones
        for node_id, node in self.graph.nodes.items():
            node_point = (node.x, node.y)
            
            # Check distance to point1
            dist1 = distance(transformed_p1, node_point)
            if dist1 < min_dist1:
                min_dist1 = dist1
                closest_to_p1 = node_id
                
            # Check distance to point2
            dist2 = distance(transformed_p2, node_point)
            if dist2 < min_dist2:
                min_dist2 = dist2
                closest_to_p2 = node_id
        
        print(f"\nFound closest nodes:")
        if closest_to_p1:
            node1 = self.graph.nodes[closest_to_p1]
            print(f"For point 1: Node {closest_to_p1} at ({node1.x}, {node1.y})")
        if closest_to_p2:
            node2 = self.graph.nodes[closest_to_p2]
            print(f"For point 2: Node {closest_to_p2} at ({node2.x}, {node2.y})")
        
        return closest_to_p1, closest_to_p2


def create_graph(workspace, layer):
    gc = GraphCreator()
    
    print(f"Creating graph from {workspace}/{layer}")
    
    # Set workspace
    arcpy.env.workspace = workspace
    fields = ['FID', 'SHAPE@']
    
    # Debug counter
    edge_count = 0
    
    # Read shapefile
    with arcpy.da.SearchCursor(layer, fields) as cursor:
        for row in cursor:
            fid = row[0]
            geometry = row[1]
            
            # Get start and end points
            start_point = geometry.firstPoint
            end_point = geometry.lastPoint
            
            # Create edge
            gc.newEdge(
                fid,
                geometry.length,
                (start_point.X, start_point.Y),
                (end_point.X, end_point.Y)
            )
            edge_count += 1
    
    print(f"Created graph with {len(gc.graph.nodes)} nodes and {edge_count} edges")
    return gc.graph

def save_to_geojson(workspace, layer, path_edges, output_path):
    """
    Save path nodes and edges to GeoJSON file
    Args:
        workspace: workspace path
        layer: layer name
        path_edges: list of edge IDs
        output_path: output GeoJSON file path
    """
    # Check if path_edges is empty
    if not path_edges:
        return None
        
    try:
        # Create a temporary geodatabase
        temp_gdb = "in_memory"
        temp_layer = "temp_layer"
        wgs84_layer = "wgs84_layer"
        
        # Delete existing file if it exists
        if os.path.exists(output_path):
            os.remove(output_path)
            
        # Make feature layer with selected edges
        where_clause = f"FID IN ({','.join(map(str, path_edges))})"
        arcpy.MakeFeatureLayer_management(
            in_features=os.path.join(workspace, layer),
            out_layer=temp_layer,
            where_clause=where_clause
        )
        
        # Create WGS84 spatial reference
        wgs84_sr = arcpy.SpatialReference(4326)  # WGS84
        
        # Project to WGS84 - save to in_memory workspace
        arcpy.Project_management(
            in_dataset=temp_layer,
            out_dataset=os.path.join(temp_gdb, wgs84_layer),
            out_coor_system=wgs84_sr
        )
        
        # Export to GeoJSON
        arcpy.conversion.FeaturesToJSON(
            in_features=os.path.join(temp_gdb, wgs84_layer),
            out_json_file=output_path,
            format_json="FORMATTED",
            include_z_values="NO_Z_VALUES",
            include_m_values="NO_M_VALUES",
            geoJSON="GEOJSON"
        )
        
        # Clean up
        arcpy.Delete_management(temp_layer)
        arcpy.Delete_management(os.path.join(temp_gdb, wgs84_layer))
        
        return output_path
        
    except arcpy.ExecuteError:
        print(f"ArcPy error: {arcpy.GetMessages()}")
        raise
    except Exception as e:
        print(f"Error: {str(e)}")
        raise
