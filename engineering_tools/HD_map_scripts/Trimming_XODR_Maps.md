# Trimming XODR Maps

This guide outlines the steps required to trim an OpenDRIVE (`.xodr`) map effectively, ensuring it is suitable for visualization, simulation, and testing.

---

## 1. Visualize the XODR

Use the `visualize_xodr.py` script to get an overview of the map layout.

* This helps in identifying which road segments are of interest.
* Optional: Enable road names for easier identification.

---

## 2. Trim the Map

Use the `filter_roads.py` script to retain only the desired road segments.

* Edit the `road_ids_to_keep` set in the script.
* The script removes all other roads and cleans up unrelated junctions.

**Note:** In the final map, ensure that junctions involving deleted road segments are fully removed to avoid broken references.

---

## 3. Truncate the Map

Manually truncate each road to the desired length:

* Sum the lengths of all `<geometry>` segments under each `<road><planView>`.
* Update the corresponding `<road length="...">` attribute to reflect this new length.

---

## 4. Review and Fix Lane Sections

Check for any lane sections that exceed the truncated road length:

* If a `<laneSection>` uses an `s` value greater than the road’s updated length, remove or adjust it.

**Example Fix:**
Change: `s="5.0632...e+2"`
To something within bounds, e.g., `s="0.0"` or `s="100.0"`.

This prevents runtime errors such as "out-of-bounds" during simulation or conversion.
