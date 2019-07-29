# -*- coding=utf-8 -*-

import numpy as np
from lxml import etree

def get_string(node, name):
    return node.get(name)

def get_int(node, name):
    return int(node.get(name))

def get_float(node, name):
    return float(node.get(name))


class Header:
    def __init__(self, node_header):
        self.date = get_string(node_header, 'date')
        self.name = get_string(node_header, 'name')
        self.east = get_float(node_header, 'east')
        self.north = get_float(node_header, 'north')
        self.south = get_float(node_header, 'south')
        self.west = get_float(node_header, 'west')
        self.revMajor = get_int(node_header, 'revMajor')
        self.revMinor = get_int(node_header, 'revMinor')
        self.vendor = get_string(node_header, 'vendor')
        self.version = get_string(node_header, 'version')

        if node_header.find("geoReference") is not None:
            self.geo_reference = node_header.find("geoReference").text

class RoadLink:
    def __init__(self, node_roadlink):
        pass

class RoadType:
    def __init__(self, node_roadlink):
        pass

class RoadGeometry:
    def __init__(self, node_roadlink):
        pass

class RoadElevation:
    def __init__(self, node_roadlink):
        pass

class RoadLaneOffset:
    def __init__(self, node_roadlink):
        pass

class RoadLaneSection:
    def __init__(self, node_roadlink):
        pass

class LaneLink: 
    def __init__(self, node_roadlink):
        pass       

class LaneWidth: 
    def __init__(self, node_roadlink):
        pass 

class LaneMark: 
    def __init__(self, node_roadlink):
        pass 

class LaneSpeed: 
    def __init__(self, node_roadlink):
        pass 

class Lane:
    def __init__(self, node_lane):
        self.id = get_int(node_lane, 'id')
        self.level = get_string(node_lane, 'level')
        self.type = get_string(node_lane, 'type')
        self.link = None
        self.width = None
        self.roadMark = None
        self.speed = None

        node_link = node_lane.find('link')
        if node_link is not None:
            self.link = LaneLink(node_link)



class Road:
    def __init__(self, node_road):
        self.link = None
        self.type = None
        self.planView = []
        self.elevationProfile = []
        self.laneOffsets = []
        self.laneSections = []


class OpenDrive:
    def __init__(self):
        self.header = None
        self.roads = []
        self.junctions = []
        self.controllers = []

def parse_opendrive(node_root) -> OpenDrive:
    opendrive = OpenDrive()

    node_header = node_root.find('header')
    if node_header is not None:
        opendrive.header = Header(node_header)

    for node_road in node_root.findall('road'):
        road = Road(node_road)
        opendrive.roads.append(road)



if __name__ == "__main__":

    xodr_file = r'C:\Users\jobge\Desktop\opendrive\yongfeng_sample_largezone.xodr'
    
    opendrive = parse_opendrive(etree.parse(xodr_file).getroot())
    
    
     