# Map Tools for XODR and OSM Processing

This repository contains a collection of Python scripts for manipulating, visualizing, and transforming OpenDRIVE (`.xodr`) and OpenStreetMap (`.osm`) files. These tools are designed to support map trimming, relocation, and visualization workflows.

- [**visualize_xodr.py**](visualize_xodr.py)
  Visualizes basic road geometries from a `.xodr` file using matplotlib. Useful for quick inspection and debugging.

- [**filter_roads.py**](filter_roads.py)
  Filters an `.xodr` map to retain only specific road segments by ID. Helps create minimal maps for focused testing.
  For step-by-step trimming and editing instructions, refer to:  [**Trimming_XODR_Maps.md**](Trimming_XODR_Maps.md)

- [**xodr_transform.py**](xodr_transform.py)
  Applies geographic transformations to `.xodr` files by updating the `geoReference` and rotating local coordinates.

- [**osm_transform.py**](osm_transform.py)
  Shifts and optionally rotates `.osm` maps based on updated georeferencing and rotation logic. Useful for map relocation.

- [**create_two_lane_map.py**](create_two_lane_map.py)
  Generates a basic two-lane road map in `.osm` format using parametric inputs like length, width, and point density.
