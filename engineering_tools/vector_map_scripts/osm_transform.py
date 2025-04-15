import argparse
from lxml import etree
from pyproj import CRS, Transformer
import math
import numpy as np

# === Parse command-line arguments ===
parser = argparse.ArgumentParser(description="Transform and rotate an OSM map based on geoReference.")
parser.add_argument("input_file", help="Path to the input .osm file")
parser.add_argument("output_file", help="Path to the output .osm file")
args = parser.parse_args()

# === Fixed rotation angle ===
rotation_deg = 0  # Counter-clockwise
theta_rad = math.radians(rotation_deg)

# === Parse XML ===
tree = etree.parse(args.input_file)
root = tree.getroot()

# === Read original geoReference string from map file ===
geo_ref_elem = root.find("geoReference")
if geo_ref_elem is None or not geo_ref_elem.text:
    raise ValueError("❌ geoReference tag not found or is empty in the OSM file.")

old_proj_str = geo_ref_elem.text.strip()
print(f"📌 Extracted old geoReference:\n{old_proj_str}\n")

# === New map center for updated geoReference ===
new_lat_0 = 38.955789
new_lon_0 = -77.150789
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
geo_ref_elem.text = new_proj_str

# --- Compute centroid of original map to rotate around ---
xs, ys = [], []
for node in root.findall("node"):
    lat = float(node.get("lat"))
    lon = float(node.get("lon"))
    x, y = to_old_xy.transform(lon, lat)
    xs.append(x)
    ys.append(y)

cx, cy = np.mean(xs), np.mean(ys)

# === Process nodes ===
for node in root.findall("node"):
    lat = float(node.get("lat"))
    lon = float(node.get("lon"))

    x, y = to_old_xy.transform(lon, lat)

    # Rotate around centroid
    x_rel, y_rel = x - cx, y - cy
    x_rot = x_rel * math.cos(theta_rad) - y_rel * math.sin(theta_rad)
    y_rot = x_rel * math.sin(theta_rad) + y_rel * math.cos(theta_rad)
    x_rot += cx
    y_rot += cy

    new_lon, new_lat = to_new_latlon.transform(x_rot, y_rot)

    node.set("lat", f"{new_lat:.10f}")
    node.set("lon", f"{new_lon:.10f}")

# === Save transformed file ===
tree.write(args.output_file, pretty_print=True, xml_declaration=True, encoding="UTF-8")
print(f"\n✅ Transformed + rotated map saved to: {args.output_file}")
