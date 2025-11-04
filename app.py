from flask import Flask, request, jsonify, send_from_directory
import os
from shapely.geometry import LineString, mapping
from flask_cors import CORS
from project_runner import Runner
import arcpy

app = Flask(__name__, static_url_path='', static_folder='static')
CORS(app)

def init_roads_geojson():
    """Initialize roads GeoJSON file if it doesn't exist"""
    try:
        workspace = os.getenv('WORKSPACE_PATH')
        layer = os.getenv('WORKSPACE_NAME')
        
        # Use absolute paths
        base_dir = os.path.dirname(os.path.abspath(__file__))
        output_path = os.path.join(base_dir, 'static', 'roads.geojson')
        input_path = os.path.join(workspace, layer)
        
        print(f"Input shapefile path: {input_path}")
        print(f"Output GeoJSON path: {output_path}")
        
        # Check if input exists
        if not arcpy.Exists(input_path):
            raise FileNotFoundError(f"Input shapefile not found: {input_path}")
        
        # Remove existing file if it exists
        if os.path.exists(output_path):
            os.remove(output_path)
            print(f"Removed existing GeoJSON file: {output_path}")
            
        # Convert to GeoJSON
        arcpy.conversion.FeaturesToJSON(
            in_features=input_path,
            out_json_file=output_path,
            format_json="FORMATTED",
            include_z_values="NO_Z_VALUES",
            include_m_values="NO_M_VALUES",
            geoJSON="GEOJSON",
            outputToWGS84="WGS84"
        )
        print(f"Successfully created GeoJSON at {output_path}")
        
    except arcpy.ExecuteError:
        print(f"ArcPy error: {arcpy.GetMessages()}")
        raise
    except Exception as e:
        print(f"Error creating GeoJSON: {str(e)}")
        raise

# Create Runner instance and initialize GeoJSON
runner = Runner()
init_roads_geojson()

@app.route('/')
def serve_index():
    return send_from_directory('static', 'index.html')

@app.route('/process', methods=['POST'])
def process():
    data = request.get_json()
    p1 = data['p1']
    p2 = data['p2']

    # Convert coordinates to format expected by Runner (x,y)
    # Note: incoming coordinates are in (lng,lat) format from Leaflet
    point1 = (float(p1['lng']), float(p1['lat']))
    point2 = (float(p2['lng']), float(p2['lat']))

    try:
        print(f"Processing points:")
        print(f"Point 1 (lng,lat): {point1}")
        print(f"Point 2 (lng,lat): {point2}")
        
        # Find closest nodes
        start_node, end_node, graph = runner.find_closest_points(point1, point2)
        print(f"Found closest nodes: {start_node}, {end_node}")
        print(f"Graph has {len(graph.nodes)} nodes and {len(graph.edges)} edges")
        
        # Calculate path and save to GeoJSON
        geojson_path = runner.calculate_path(start_node, end_node)
        
        if geojson_path is None:
            print("No path was found - check if nodes are connected in graph")
            return jsonify({
                "status": "error",
                "message": "No path found between selected points"
            }), 404
        
        # Return success response with path to GeoJSON
        response = {
            "status": "success",
            "geojson_url": "/shortest_path.geojson",
            "start_node": start_node,
            "end_node": end_node
        }
        return jsonify(response)
    
    except Exception as e:
        print(f"Error processing request: {str(e)}")
        import traceback
        traceback.print_exc()
        return jsonify({
            "status": "error",
            "message": str(e)
        }), 500

if __name__ == '__main__':
    app.run(debug=True)
