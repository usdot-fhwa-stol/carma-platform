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


import matplotlib
matplotlib.use('Agg')  # Use this on headless systems

import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
import numpy as np

# === Load and parse the XODR file ===
xodr_file_path = "xodr_map.xodr"  # TODO: Change this to your actual xodr file path
tree = ET.parse(xodr_file_path)
root = tree.getroot()

fig, ax = plt.subplots(figsize=(10, 10))

# === Go through each road geometry ===
for road in root.findall(".//road"):
    road_name = road.get("name", "")
    for geometry in road.findall(".//geometry"):
        try:
            x = float(geometry.get("x"))
            y = float(geometry.get("y"))
            hdg = float(geometry.get("hdg"))
            length = float(geometry.get("length"))

            dx = length * np.cos(hdg)
            dy = length * np.sin(hdg)

            ax.arrow(x, y, dx, dy, head_width=1.0, length_includes_head=True)

            if road_name:
                ax.text(x, y, road_name, fontsize=6)
        except Exception as e:
            print("Error in geometry:", ET.tostring(geometry, encoding='unicode'))
            print("Reason:", e)

# === Plot Settings ===
ax.set_title("Basic Road Geometry Visualization (XODR)")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_aspect("equal")
plt.grid(True)
plt.tight_layout()
plt.show()
