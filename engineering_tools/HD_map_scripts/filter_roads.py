"""
Purpose:
This script filters an .xodr map file to retain only a specified subset of roads based on their IDs.

Key Features:

- Allows user to define a set of road IDs to keep (road_ids_to_keep).
- Removes all roads and junctions not associated with the specified IDs.
- Outputs a new .xodr file containing only the filtered elements.

Use Case:
Creating a trimmed-down version of a map with only selected road segments for focused simulation or testing.
"""
from lxml import etree

# === EDIT THIS LIST: IDs of roads you want to keep ===
road_ids_to_keep = {"23"}

# === Input and output file paths ===
# TODO: Change these to match actual files
input_file = "original_map.xodr"
output_file = "filtered_map.xodr"

def filter_xodr(input_file, output_file, road_ids_to_keep):
    tree = etree.parse(input_file)
    root = tree.getroot()

    # Remove roads not in the list
    for road in root.findall("road"):
        if road.get("id") not in road_ids_to_keep:
            root.remove(road)

    # Remove unrelated junctions
    for junction in root.findall("junction"):
        remove = True
        for connection in junction.findall("connection"):
            if (connection.get("incomingRoad") in road_ids_to_keep or
                connection.get("connectingRoad") in road_ids_to_keep):
                remove = False
                break
        if remove:
            root.remove(junction)

    # Write the result to output
    tree.write(output_file, pretty_print=True, xml_declaration=True, encoding='UTF-8')
    print(f"Filtered XODR written to: {output_file}")

# === Run the filter ===
filter_xodr(input_file, output_file, road_ids_to_keep)
