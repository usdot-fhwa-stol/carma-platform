# approximate_intersection

This library contains a fast occupancy grid creation and intersection implementation. The user provides 2d min/max bounds on the grid as well as cell side length (cells are always square). The user can then add points into the grid. Cells which contain points are marked as occupied. Once the grid is populated, intersections can be checked against. If the queried point lands in an occupied cell the intersection is reported as true.

The original intent for this library was fast filtering of lidar data against static road maps.
