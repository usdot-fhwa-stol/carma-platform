"""
This script creates a vector map with two parallel lanes using the Lanelet2 library and saves it as an OSM file.
The script can be run from the command line with the following arguments:
- filename: The output filename for the vector map.
- total_length: The length of the lanes (default is 50.0).
- lane_width: The width of the lanes (default is 3.7).
- points_per_meter: The number of points per meter (default is 5).

Dependencies:
- lanelet2: The Lanelet2 library for handling lanelet maps.
- argparse: For parsing command line arguments.
Usage:
    python3 create_two_lane_map.py --filename output.osm --total_length <total_length> --lane_width <lane_width> --points_per_meter <points_per_meter>
"""

from pyproj import Proj, Transformer
import xml.etree.ElementTree as ET
from xml.dom import minidom
import argparse


# Adjusted geoReference (removed vertical geoid grid)
geo_reference = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs"

# Coordinate transformer (meters → lat/lon)
proj = Proj(geo_reference)
transformer = Transformer.from_proj(proj, "epsg:4326", always_xy=True)

# Globals
node_id = 1000000
way_id = 1000
relation_id = 100
nodes, ways, relations = [], [], []

def add_node(x, y, z=0.0):
    global node_id
    lon, lat = transformer.transform(x, y)
    node = ET.Element("node", id=str(node_id), version="1", lat=f"{lat:.9f}", lon=f"{lon:.9f}", visible="true")
    ET.SubElement(node, "tag", k="ele", v=f"{z:.2f}")
    ET.SubElement(node, "tag", k="lat", v=f"{lat:.9f}")
    ET.SubElement(node, "tag", k="lon", v=f"{lon:.9f}")
    nodes.append((node_id, node))
    node_id += 1
    return node_id - 1

def create_way(node_ids, tags):
    global way_id
    way = ET.Element("way", id=str(way_id), version="1", visible="true")
    for nid in node_ids:
        ET.SubElement(way, "nd", ref=str(nid))
    for k, v in tags.items():
        ET.SubElement(way, "tag", k=k, v=v)
    ways.append((way_id, way))
    way_id += 1
    return way_id - 1

def create_lanelet(left_id, right_id, tags):
    global relation_id
    rel = ET.Element("relation", id=str(relation_id), version="1", visible="true")
    ET.SubElement(rel, "member", type="way", ref=str(left_id), role="left")
    ET.SubElement(rel, "member", type="way", ref=str(right_id), role="right")
    for k, v in tags.items():
        ET.SubElement(rel, "tag", k=k, v=v)
    ET.SubElement(rel, "tag", k="cad_id", v=str(relation_id))
    relations.append((relation_id, rel))
    relation_id += 1

def create_vector_map(filename, total_length, lane_width, points_per_meter):
    """
    Create a vector map with two parallel lanes.
    Inputs:
    - filename: The output filename for the vector map.
    - total_length: The length of the lanes.
    - lane_width: The width of the lanes.
    - points_per_meter: The number of points per meter.
    """
    # Road geometry
    lanelet_length = 25.0
    total_points = int(total_length * points_per_meter) + 1
    points_per_lanelet = int(lanelet_length * points_per_meter) + 1
    # Generate lane boundaries
    x_offset = -total_length / 2
    y_offset = -lane_width
    left1, right1, left2, right2 = [], [], [], []
    for i in range(total_points):
        x = i / points_per_meter + x_offset
        right1.append(add_node(x, 0 + y_offset))
        left1.append(add_node(x, lane_width + y_offset))
        right2.append(add_node(x, lane_width + y_offset))
        left2.append(add_node(x, 2 * lane_width + y_offset))

    stride = points_per_lanelet - 1
    way_dict = {"type": "line_thin", "subtype": "solid"}
    lanelet_dict = {"type": "lanelet",
                    "subtype": "road",
                    "road_type": "road",
                    "turn_direction": "straight",
                    "from_cad_id" : [],
                    "direction" : "ONE_WAY",
                    "level" : "0",
                    "location" : "private",
                    "near_spaces" : [],
                    "participant:vehicle" : "yes",
                    "to_cad_id" : [],
                    }
    for i in range(0, total_points - stride, stride):
         # Outer left boundary of lane 2
        l2_nodes = left2[i:i + points_per_lanelet]
        l2_id = create_way(l2_nodes, way_dict)  # solid

        # Shared dashed centerline between lanes
        center_nodes = right2[i:i + points_per_lanelet]
        center_id = create_way(center_nodes, {"type": "line_thin", "subtype": "dashed"})

        # Outer right boundary of lane 1
        r1_nodes = right1[i:i + points_per_lanelet]
        r1_id = create_way(r1_nodes, way_dict)  # solid

        # Lanelet for lane 2 (left: outer left, right: dashed center)
        create_lanelet(l2_id, center_id, lanelet_dict)

        # Lanelet for lane 1 (left: dashed center, right: outer right)
        create_lanelet(center_id, r1_id, lanelet_dict)

    # Build OSM XML tree
    osm = ET.Element("osm", version="0.6")
    ET.SubElement(osm, "geoReference", v=geo_reference)
    for _, node in nodes:
        osm.append(node)
    for _, way in ways:
        osm.append(way)
    for _, rel in relations:
        osm.append(rel)

    # Write to file
    with open(filename, "w", encoding="utf-8") as f:
        xml_str = minidom.parseString(ET.tostring(osm)).toprettyxml(indent="  ")
        f.write(xml_str)

    print(f"Map saved to {filename}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Create a vector map with two parallel lanes.")
    parser.add_argument("--filename", type=str, default="two_lane_straight.osm", help="Output filename for the vector map.")
    parser.add_argument("--total_length", type=float, default=50.0, help="Length of the lanes.")
    parser.add_argument("--lane_width", type=float, default=3.7, help="Width of the lanes.")
    parser.add_argument("--points_per_meter", type=int, default=5, help="Number of points in the lane.")
    args = parser.parse_args()
    create_vector_map(args.filename, args.total_length, args.lane_width, args.points_per_meter)
    print("Vector map with two parallel lanes created successfully.")