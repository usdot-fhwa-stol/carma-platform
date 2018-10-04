#!/usr/bin/python
# This script can be used to reverse all waypoints order from a CARMA route file
# To Run:
# Create a text file and only copy and paste lines after 'waypoints:'
# Add an empty line at the start of the file
# Run "python reverse_waypoints.py <file_name>" to get a text file with reversed points

import sys

def main():
    filepath = sys.argv[1]
    waypoints = []
    with open(filepath, 'r') as fp:
        line = fp.readline()
        counter = 0
        while line:
            if len(line) == 1:
                counter += 1
                waypoint = []
                waypoint.append(line)
                waypoints.append(waypoint)
            else:
                waypoints[counter - 1].append(line)
            line = fp.readline()
        print("Processed %d waypoints" % len(waypoints))
    output_path = filepath + '_output'
    with open(output_path, 'w') as fp:
        for wp in reversed(waypoints):
            for line in wp:
                fp.write(line)
    print("Done!")

if __name__ == "__main__":
    print("Running ReverseWaypoints.py...")
    main()
