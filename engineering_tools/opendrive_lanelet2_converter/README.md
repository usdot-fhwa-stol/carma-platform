# opendrive2lanlet2

opendrive2lanlet2 is a Python script for converting opendrive maps to lanelet2 map readable by autoware. this scripts only converts 2d roads with no elevation.

## Installation

as a dependency need to install opendrive2lanelet library with a GNU General Public License v3.0

```bash
pip install opendrive2lanelet
```

## Usage

```bash
python3.7 map_convertor.py -i input.xodr -o output.osm
```

## Description

Node class representing node in lanelet2
Way class representing way in lanelet2
Relation class representing relation in lanelet2
Opendrive2Lanelet2Convertor class used to convert opendrive map to lanelet2 map
