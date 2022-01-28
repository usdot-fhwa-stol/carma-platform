# points_map_filter

The points_map_filter node performs an approximate filtering of lidar data to keep it within the bounds of the lanes in a Lanelet2 semantic map. The map space is discritized into square cells with length specified by the node parameters. The lane boundaries in the Lanelet2 map are then used to create an occupancy grid of the wold. When lidar data is received only points which intersect the occupied cells (where the lanes are) will be forwarded out of this node.
