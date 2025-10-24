import arcpy

# Collections to store points and polylines
points_collection = {}  # {point_id: {'x': x_coord, 'y': y_coord, 'edges_out': [list of polyline_ids]}}
polylines_collection = {}  # {polyline_id: {'first_point_id': id, 'last_point_id': id, 'length': length}}

# Counter for unique point IDs
point_id_counter = 0

def get_or_create_point_id(x, y, tolerance=0.01):
    """
    Get existing point ID if point exists within tolerance, otherwise create new point
    """
    global point_id_counter
    
    # Check if point already exists (within tolerance)
    for pid, point in points_collection.items():
        if abs(point['x'] - x) < tolerance and abs(point['y'] - y) < tolerance:
            return pid
    
    # Create new point
    new_id = point_id_counter
    points_collection[new_id] = {'x': x, 'y': y, 'edges_out': []}
    point_id_counter += 1
    return new_id

def dijkstra_shortest_path(points, start_point_id, end_point_id):
    S = set()
    Q = set()
    p = []
    d = []
    for point in points:
        d[point] = float('inf')
        p[point] = None
        Q.add(point)
    
    d[end_point_id] = 0

    while Q:
        # Get point in Q with smallest d[]
        u = min(Q, key=lambda point: d[point])
        Q.remove(u)
        S.add(u)
        

        for edge in points[u]['edges_out']:
            v = edge['last_point_id'] if edge['first_point_id'] == u else edge['first_point_id']
            if v in Q:
                alt = d[u] + edge['length']
                if alt < d[v]:
                    d[v] = alt
                    p[v] = u

arcpy.env.workspace = r"C:\uni\5sem\pag\proj1\drogi_testowe.shp"
cursor = arcpy.SearchCursor('drogi_testowe')  # No WHERE clause = get all features

for row in cursor:
    polyline_id = row.FID
    
    if row.Shape.type == 'polyline' and row.Shape.firstPoint and row.Shape.lastPoint:
        # Get or create point IDs for first and last points
        first_point_id = get_or_create_point_id(
            row.Shape.firstPoint.X, 
            row.Shape.firstPoint.Y
        )
        last_point_id = get_or_create_point_id(
            row.Shape.lastPoint.X, 
            row.Shape.lastPoint.Y
        )
        
        # Calculate polyline length
        polyline_length = row.Shape.length
        
        # Store polyline with references to point IDs and length
        polylines_collection[polyline_id] = {
            'first_point_id': first_point_id,
            'last_point_id': last_point_id,
            'length': polyline_length
        }
        
        # Add this polyline to the edges_out list of BOTH connected points
        points_collection[first_point_id]['edges_out'].append(polyline_id)
        points_collection[last_point_id]['edges_out'].append(polyline_id)

# Save the collections to a text file
output_file = r"C:\uni\5sem\pag\proj1\road_network_data.txt"




with open(output_file, 'w') as f:
    f.write("=== POINTS COLLECTION ===\n")
    for point_id, point in points_collection.items():
        edges_out_str = ', '.join(map(str, point['edges_out'])) if point['edges_out'] else 'None'
        f.write(f"Point {point_id}: X={point['x']:.2f}, Y={point['y']:.2f}, Edges_out=[{edges_out_str}]\n")
    
    f.write("\n=== POLYLINES COLLECTION ===\n")
    total_length = 0
    for polyline_id, polyline in polylines_collection.items():
        first_pt = points_collection[polyline['first_point_id']]
        last_pt = points_collection[polyline['last_point_id']]
        length = polyline['length']
        total_length += length
        f.write(f"Polyline {polyline_id}: Length={length:.2f}m, "
                f"First Point ID={polyline['first_point_id']} "
                f"({first_pt['x']:.2f}, {first_pt['y']:.2f}), "
                f"Last Point ID={polyline['last_point_id']} "
                f"({last_pt['x']:.2f}, {last_pt['y']:.2f})\n")
    
    f.write(f"\n=== SUMMARY ===\n")
    f.write(f"Total unique points: {len(points_collection)}\n")
    f.write(f"Total polylines: {len(polylines_collection)}\n")
    f.write(f"Total network length: {total_length:.2f}m ({total_length/1000:.3f}km)\n")

print(f"Data saved to: {output_file}")

#boombaya dla testu branchy
