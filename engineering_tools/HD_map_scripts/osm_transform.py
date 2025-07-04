"""
This script relocates and optionally rotates an OpenStreetMap (OSM) file that uses a custom geoReference. It's useful when you want to:
- Move a local map to a new geographic location.
- Preserve geometry, scale, and layout.
- Rotate the map around its origin (for alignment or testing).
---

Steps:

1. Read the input OSM XML file.
2. Extract the original geoReference (a Transverse Mercator projection centered at some latitude/longitude).
3. Convert each nodes lat/lon to projected (X, Y) coordinates using the original projection.
4. Apply a 2D rotation (optional) around the local origin (0, 0).
5. Transform the rotated (X, Y) into new lat/lon coordinates using a new projection centered at a new location.
6. Update the <geoReference> tag to reflect the new center.
7. Save the updated OSM XML file with transformed coordinates.

---

Inputs:

1- input_file.osm: An OSM XML file that:
- Contains a <geoReference> string using +proj=tmerc
- Has <node> elements with lat and lon attributes

2- output_file.osm: The transformed OSM map name that:
- All node positions have been relocated and optionally rotated
- The <geoReference> tag is updated to match the new map center
- The map geometry is preserved in relative terms but relocated globally

---

Parameters to adjust to transform:

- rotation_deg: Rotation angle in degrees (positive = counter-clockwise)
- new_lat_0, new_lon_0: New center location in geographic coordinates (latitude, longitude)

--

Dependency:
pip install lxml pyproj

--
how to run the script:

python osm_transform.py suntrax.osm suntrax_transformed.osm
"""

import argparse
from lxml import etree
from pyproj import CRS, Transformer
import math
import numpy as np

# === Fixed reference and rotation config ===
# TODO Add the desired geoReference and rotation reference for the output map here (They can both be the same or different based on the application)
# New Georeference
new_lat_0 = 0.0
new_lon_0 = 0.0
# New Rotation Reference
rotate_lat = 0.0
rotate_lon = 0.0
# Rotation Reference
# TODO Add the desired rotation of the output map here
rotation_deg = 0.0  # Counter-clockwise

theta_rad = math.radians(rotation_deg)

# === Parse command-line arguments ===
parser = argparse.ArgumentParser(description="Relocate and rotate OSM map geometry around a new reference point.")
parser.add_argument("input_file", help="Path to the input .osm file")
parser.add_argument("output_file", help="Path to the output .osm file")
args = parser.parse_args()

# === Parse XML ===
tree = etree.parse(args.input_file)
root = tree.getroot()

# === Read original geoReference string from map file ===
geo_ref_elem = root.find("geoReference")
if geo_ref_elem is None or (not geo_ref_elem.text and not geo_ref_elem.attrib.get("v")):
    raise ValueError("❌ geoReference tag not found or is empty in the OSM file.")

if geo_ref_elem.text:
    old_proj_str = geo_ref_elem.text.strip()
else:
    old_proj_str = geo_ref_elem.attrib.get("v").strip()
print(f"📌 Extracted old geoReference:\n{old_proj_str}\n")

new_proj_str = f"+proj=tmerc +lat_0={new_lat_0} +lon_0={new_lon_0} +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs"
print(f"📌 Updated new geoReference:\n{new_proj_str}\n")

# === Coordinate Systems ===
crs_wgs84 = CRS.from_epsg(4326)
crs_old = CRS.from_proj4(old_proj_str)
crs_new = CRS.from_proj4(new_proj_str)

# === Transformers ===
to_old_xy = Transformer.from_crs(crs_wgs84, crs_old, always_xy=True)
to_new_latlon = Transformer.from_crs(crs_new, crs_wgs84, always_xy=True)

# === Update <geoReference> to new projection ===
if geo_ref_elem.text:
    geo_ref_elem.text = new_proj_str
else:
    geo_ref_elem.set("v", new_proj_str)

# === Step 1: Convert all nodes to old projected coordinates and compute centroid ===
xs_old, ys_old = [], []
for node in root.findall("node"):
    lat = float(node.get("lat"))
    lon = float(node.get("lon"))
    x_old, y_old = to_old_xy.transform(lon, lat)
    xs_old.append(x_old)
    ys_old.append(y_old)

cx_old = np.mean(xs_old)
cy_old = np.mean(ys_old)

# === Step 2: Compute shift offset to move centroid to new reference point ===
to_new_xy = Transformer.from_crs(crs_wgs84, crs_new, always_xy=True)
rotate_x, rotate_y = to_new_xy.transform(rotate_lon, rotate_lat)

# Offset from old centroid to new rotation center
offset_x = rotate_x - cx_old
offset_y = rotate_y - cy_old

# === Step 3: Apply shift + rotation ===
for i, node in enumerate(root.findall("node")):
    x = xs_old[i] + offset_x
    y = ys_old[i] + offset_y

    # Rotate around new center
    x_rel, y_rel = x - rotate_x, y - rotate_y
    x_rot = x_rel * math.cos(theta_rad) - y_rel * math.sin(theta_rad)
    y_rot = x_rel * math.sin(theta_rad) + y_rel * math.cos(theta_rad)
    x_rot += rotate_x
    y_rot += rotate_y

    # Convert to final lat/lon using new projection
    new_lon, new_lat = to_new_latlon.transform(x_rot, y_rot)
    node.set("lat", f"{new_lat:.10f}")
    node.set("lon", f"{new_lon:.10f}")
    # Update tag values for lat and lon
    for tag in node.findall('tag'):
        if tag.get('k') == 'lat':
            tag.set('v', f"{new_lat:.10f}")
        elif tag.get('k') == 'lon':
            tag.set('v', f"{new_lon:.10f}")

# === Save transformed file ===
tree.write(args.output_file, pretty_print=True, xml_declaration=True, encoding="UTF-8")
print(f"\n✅ Map shifted and rotated. Output saved to: {args.output_file}")
