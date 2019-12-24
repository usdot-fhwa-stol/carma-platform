import os
import sys, getopt
from pyproj import Proj, transform
import numpy as np
from opendrive2lanelet.opendriveparser.parser import parse_opendrive
from opendrive2lanelet.network import Network

import xml.etree.ElementTree as xml
import xml.dom.minidom as pxml
from lxml import etree

# class representing node object in Way object 
class Node:
    def __init__(self, id, lat, lon, local_x, local_y):
        self.id = id
        self.lat = lat
        self.lon = lon
        self.local_x = local_x
        self.local_y = local_y

    def create_xml_node_object(self):
        node_element = xml.Element('node',{'id': str(self.id), 'lat': str(self.lat), 'lon': str(self.lon), 'version': str(1), 'visible': 'true'})
        xml.SubElement(node_element, "tag", {"k": "ele", "v": "0.0"})
        return node_element

# class representing Way object in Relation object members
class Way:
    def __init__(self, id, nodes):
        self.id = id
        self.nodes = nodes
    
    def create_xml_way_object(self):
        way_element = xml.Element("way", {"id": self.id, "version": str(1), "visible": "true"})
        for i in self.nodes:
            xml.SubElement(way_element, "nd", {"ref": str(i.id)})
        xml.SubElement(way_element, "tag", {"k": "type", "v": "road_border"})
        return way_element

# class representing relation in lanelet2
class Relation:
    def __init__(self, id, member_left, member_right, from_cad_id, to_cad_id, cad_id, relation_type):
        self.id = str(id)
        self.member_left = member_left
        self.member_right = member_right
        self.member_left_id = str(member_left.id)
        self.member_right_id = str(member_right.id)
        self.from_cad_id = from_cad_id
        self.to_cad_id = to_cad_id
        self.relation_type = relation_type
        self.cad_id = cad_id
        self.turn_direction = "straight"
        self.set_turn_direction()

    def create_xml_relation_object(self):
        relation_element = xml.Element("relation", {"id": self.id, "version": str(1), "visible": "true"})
        xml.SubElement(relation_element, "member", {"ref": self.member_left_id, "role": str("left"), "type": "way"})
        xml.SubElement(relation_element, "member", {"ref": self.member_right_id, "role": str("right"), "type": "way"})
        xml.SubElement(relation_element, "tag", {"k": "cad_id", "v": self.cad_id})
        xml.SubElement(relation_element, "tag", {"k": "direction", "v": "ONE_WAY"})
        xml.SubElement(relation_element, "tag", {"k": "level", "v": "0"})
        xml.SubElement(relation_element, "tag", {"k": "location", "v": "private"})
        xml.SubElement(relation_element, "tag", {"k": "participant:vehicle", "v": "yes"})
        xml.SubElement(relation_element, "tag", {"k": "road_type", "v": "road"})
        xml.SubElement(relation_element, "tag", {"k": "subtype", "v": "road"})
        xml.SubElement(relation_element, "tag", {"k": "type", "v": "lanelet"})
        xml.SubElement(relation_element, "tag", {"k": "from_cad_id", "v": self.from_cad_id})
        xml.SubElement(relation_element, "tag", {"k": "to_cad_id", "v": self.to_cad_id})
        xml.SubElement(relation_element, "tag", {"k": "near_spaces", "v": "[]"})
        xml.SubElement(relation_element, "tag", {"k": "turn_direction", "v": self.turn_direction})
        return relation_element
    
    # start is the first point of lanelet
    # mid is the secound point of lanlelet
    # end is the last point of lanelet
    # set_direction is used by set_turn direction to check if the lanelet is turning left or right
    def set_direction(self, start, mid, end):
        if(((b[0] - a[0])*(c[1] - a[1]) - (b[1] - a[1])*(c[0] - a[0])) > 0):
            return 1
        else:
            return 0
    #set_turn_direction is used to set set the turn_direction tag in lanelet
    def set_turn_direction(self):
        turn_direction = "straight"
        if(len(self.from_cad_id) == 1 and len(self.to_cad_id) == 1):
            start = np.array([self.member_left.nodes[0].local_x,self.member_left.nodes[0].local_y])
            mid = np.array([self.member_left.nodes[1].local_x,self.member_left.nodes[1].local_y])
            end = np.array([self.member_left.nodes[-1].local_x,self.member_left.nodes[-1].local_y])

            start_mid = mid - start
            start_end = end - start
            cosine_angle = np.dot(start_mid, start_end) / (np.linalg.norm(start_mid) * np.linalg.norm(start_end))
            alpha = np.arccos(cosine_angle)

            if(np.degrees(alpha) > 10):
                if(self.set_direction(start,mid,end) == 0):
                    turn_direction = "right"
                else:
                    turn_direction = "left"

        self.turn_direction = turn_direction

# class used to convert opendrive map to lanelet2 map
class Opendrive2Lanelet2Convertor:
    def __init__(self, fn):
        self.scenario, self.geoReference = self.open_drive_loader(fn)
        self.root = xml.Element('osm',{'generator': 'lanelet2','version': '0.6'})
        self.nodes = xml.Element('nodes')
        self.ways = xml.Element('ways')
        self.relations = xml.Element('relations')

    def open_drive_loader(self, fn):
        fi = open(fn.format(os.path.dirname(os.path.realpath(__file__))), "r")
        tree = etree.parse(fi)
        root = tree.getroot()
        geoReference = root[0][0].text
        open_drive = parse_opendrive(root)
        road_network = Network()
        road_network.load_opendrive(open_drive)
        return road_network.export_commonroad_scenario(), geoReference

    # convert x,y values to geo points using the geo reference defined in the input file
    def get_point_geo(self,x,y):
        inProj = Proj(self.geoReference)
        outProj = Proj(init="epsg:4326")
        return transform(inProj,outProj,x,y)

    def write_xml_to_file(self,fn):
                
        for child in self.nodes.getchildren():
            self.root.append(child)

        for child in self.ways.getchildren():
            self.root.append(child)

        for child in self.relations.getchildren():
            self.root.append(child)

        tree = xml.ElementTree(self.root)
        fh = open(fn, "wb")
        tree.write(fh)

    # convert vertice from opendrive to a node in lanelet 
    def convert_vertice_to_node(self,node_id,vertice):
        x = vertice[0]
        y = vertice[1]
        lon, lat = self.get_point_geo(x,y)
        local_x, local_y = x, y
        return Node(node_id,lat,lon, local_x, local_y)
    
    # apply convert_vertice_to_node to a list of nodes
    # bound_id 0,1 for left and right 
    def process_vertices(self, vertices, relation_id, bound_id):
        nodes = []
        for j in range(len(vertices)):
            node_id = relation_id + str(bound_id) + "{0:0=3d}".format(j)
            node = self.convert_vertice_to_node(node_id, vertices[j])
            nodes.append(node)
            self.nodes.append(node.create_xml_node_object())
        return nodes

    def convert(self, fn):
        for i in self.scenario._id_set:
            left_nodes = []
            right_nodes = []
            relation_id = str(i)
            cad_id = str(i)

            left_nodes = self.process_vertices(self.scenario._lanelet_network._lanelets[i]._left_vertices, relation_id, 0)
            right_nodes = self.process_vertices(self.scenario._lanelet_network._lanelets[i]._right_vertices, relation_id, 1)

            left_way_id = relation_id + '0'
            left_way = Way(left_way_id,left_nodes)

            right_way_id = relation_id + '1'
            right_way = Way(right_way_id,right_nodes)

            self.ways.append(left_way.create_xml_way_object())
            self.ways.append(right_way.create_xml_way_object())

            from_cad_id = self.scenario._lanelet_network._lanelets[i]._successor
            to_cad_id = self.scenario._lanelet_network._lanelets[i]._predecessor
            relation = Relation(relation_id, left_way, right_way, from_cad_id, to_cad_id, cad_id, "lanelet")
            self.relations.append(relation.create_xml_relation_object())

        self.write_xml_to_file(fn)

def main(argv):

    inputfile = ''
    outputfile = ''
    try:
        opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
    except getopt.GetoptError:
        print('opendrive2lanelet2convertor.py -i <inputfile> -o <outputfile>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('opendrive2lanelet2convertor.py -i <inputfile> -o <outputfile>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-o", "--ofile"):
            outputfile = arg
    if(inputfile is not None and outputfile is not None):
        open_drive2_lanelet2_convertor = Opendrive2Lanelet2Convertor(inputfile)
        open_drive2_lanelet2_convertor.convert(outputfile)

if __name__== "__main__":
    main(sys.argv[1:])
