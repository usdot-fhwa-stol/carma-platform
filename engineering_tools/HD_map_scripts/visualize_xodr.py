"""

Purpose:
This script parses an OpenDRIVE (.xodr) map file and visualizes the basic road geometries using matplotlib.

Key Features:

- Loads and parses an .xodr file using xml.etree.ElementTree.
- Extracts position, heading, and length for each road geometry segment.
- Plots each road segment as an arrow on a 2D plane.
- Optionally overlays road names on the visualization for identification.

Use Case:
Quick inspection and debugging of road layouts from .xodr files in a readable 2D plot.

"""



import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import numpy as np

# === Load and parse the XODR file ===
xodr_file_path = "xodr_map.xodr"  # Change this to your actual path
tree = ET.parse(xodr_file_path)
root = tree.getroot()

# === Extract road geometries ===
road_geometries = []

for road in root.findall(".//road"):
    road_name = road.get("name", "")
    for geometry in road.findall(".//geometry"):
        s = float(geometry.get("s", 0))
        x = float(geometry.get("x", 0))
        y = float(geometry.get("y", 0))
        hdg = float(geometry.get("hdg", 0))
        length = float(geometry.get("length", 0))
        road_geometries.append((x, y, length, hdg, road_name))

# === Plot the road geometries ===
fig, ax = plt.subplots(figsize=(10, 10))

# === Toggle to show or hide road names ===
SHOW_ROAD_NAMES = True

# ... (rest of your parsing and plotting setup)

for x, y, length, hdg, name in road_geometries:
    dx = length * np.cos(hdg)
    dy = length * np.sin(hdg)
    ax.arrow(x, y, dx, dy, head_width=1.0, length_includes_head=True)

    if SHOW_ROAD_NAMES and name:
        ax.text(x, y, name, fontsize=6)



ax.set_title("Basic Road Geometry Visualization (from XODR)")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_aspect("equal")
plt.grid(True)
plt.tight_layout()
plt.show()
