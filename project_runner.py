import os
import dotenv
from projekt1_KD import GraphCreator, create_graph, save_to_geojson

class Runner:
    def __init__(self):
        dotenv.load_dotenv()
        self.gc = GraphCreator()
        self.gc.graph = create_graph(os.getenv('WORKSPACE_PATH'), os.getenv('WORKSPACE_NAME'))

    def find_closest_points(self, point1, point2):
        """
        Finds closest nodes in graph for given points
        Args:
            point1: tuple (x,y) - coordinates of first point
            point2: tuple (x,y) - coordinates of second point
        Returns:
            tuple (node_id1, node_id2, graph) - IDs of closest nodes and graph object
        """
        start_node_id, end_node_id = self.gc.find_closest_nodes(point1, point2)
        return start_node_id, end_node_id, self.gc.graph

    def calculate_path(self, start_id, end_id):
        """
        Calculates shortest path between nodes and saves to GeoJSON
        Args:
            start_id: ID of starting node
            end_id: ID of ending node
        Returns:
            str: path to generated GeoJSON file or None if no path found
        """
        output_path = "c:/uni/5sem/pag/proj1/static/shortest_path.geojson"
        
        # Calculate path
        path_nodes, path_edges, dist, visited, neighbors = self.gc.dijkstra(start_id, end_id)
        
        # Check if path was found
        if not path_edges:
            return None
            
        # Save to GeoJSON
        result_path = save_to_geojson(
            os.getenv('WORKSPACE_PATH'),
            os.getenv('WORKSPACE_NAME'),
            path_edges,
            output_path
        )
        
        return result_path