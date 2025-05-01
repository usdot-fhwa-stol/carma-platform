from pyproj import Proj, Transformer
import xml.etree.ElementTree as ET
from xml.dom import minidom





# Road geometry
lane_width = 3.5
length = 100
num_points = 11
dx = length / (num_points - 1)

# Adjusted geoReference (removed vertical geoid grid)
# TODO: Needs to be centroid of route file
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
    node = ET.Element("node", id=str(node_id), lat=f"{lat:.9f}", lon=f"{lon:.9f}", visible="true")
    ET.SubElement(node, "tag", k="ele", v=f"{z:.2f}")
    ET.SubElement(node, "tag", k="lat", v=f"{lat:.9f}")
    ET.SubElement(node, "tag", k="lon", v=f"{lon:.9f}")
    nodes.append((node_id, node))
    node_id += 1
    return node_id - 1

def create_way(node_ids, tags):
    global way_id
    way = ET.Element("way", id=str(way_id), visible="true")
    for nid in node_ids:
        ET.SubElement(way, "nd", ref=str(nid))
    for k, v in tags.items():
        ET.SubElement(way, "tag", k=k, v=v)
    ways.append((way_id, way))
    way_id += 1
    return way_id - 1

def create_lanelet(left_id, right_id, tags):
    global relation_id
    rel = ET.Element("relation", id=str(relation_id), visible="true")
    ET.SubElement(rel, "member", type="way", ref=str(left_id), role="left")
    ET.SubElement(rel, "member", type="way", ref=str(right_id), role="right")
    for k, v in tags.items():
        ET.SubElement(rel, "tag", k=k, v=v)
    ET.SubElement(rel, "tag", k="cad_id", v=str(relation_id))
    relations.append((relation_id, rel))
    relation_id += 1

# Generate lane boundaries
x_offset = -length / 2
y_offset = -lane_width
left1, right1, left2, right2 = [], [], [], []
for i in range(num_points):
    x = i * dx + x_offset
    right1.append(add_node(x, 0 + y_offset))
    left1.append(add_node(x, lane_width + y_offset))
    right2.append(add_node(x, lane_width + y_offset))
    left2.append(add_node(x, 2 * lane_width + y_offset))

# Create ways for boundaries
way_dict = {"type": "line_thin", "subtype": "solid"}
left1_id = create_way(left1, way_dict)
right1_id = create_way(right1, way_dict)
left2_id = create_way(left2, way_dict)
right2_id = create_way(right2, way_dict)

# Create lanelet relations
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
create_lanelet(left1_id, right1_id, lanelet_dict)
create_lanelet(left2_id, right2_id, lanelet_dict)

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
output_file = "two_lane_straight_map.osm"
with open(output_file, "w", encoding="utf-8") as f:
    xml_str = minidom.parseString(ET.tostring(osm)).toprettyxml(indent="  ")
    f.write(xml_str)

print(f"Map saved to {output_file}")
