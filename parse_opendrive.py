# -*- coding=utf-8 -*-

import numpy as np
from lxml import etree
from enum import Enum
import os

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

        # print(self.name, self.date, self.geo_reference)

class RoadLinkElement:
    def __init__(self, node_roadlink_element):
        # <predecessor contactPoint="end" elementId="91" elementType="junction" />
        self.contactPoint = get_string(node_roadlink_element, 'contactPoint')
        self.elementId = get_int(node_roadlink_element, 'elementId')
        self.elementType = get_string(node_roadlink_element, 'elementType') 

        # print(self.contactPoint, self.elementId, self.elementType)

class RoadLink:
    def __init__(self, node_roadlink):
        self.predecessor = None
        self.successor = None

        node_predecessor = node_roadlink.find('predecessor')
        if node_predecessor is not None:
            self.predecessor = RoadLinkElement(node_predecessor)

        node_successor = node_roadlink.find('successor')
        if node_successor is not None:
            self.successor = RoadLinkElement(node_successor)

class RoadType:
    def __init__(self, node_roadtype):
        # <type s="0" type="motorway" /> 
        self.s = get_float(node_roadtype, 's')
        self.type = get_string(node_roadtype, 'type')

class GeometryType(Enum):
    unknown = 0
    Line = 1
    Arc = 2
    Poly3 = 3
    ParamPoly3 = 4

class Geometry:
    def __init__(self, node_geometry):
        # <geometry hdg="1.97298146738386" length="204.215477960887" s="0" x="-901.947546366" y="1610.90849739965">
        self.s = get_float(node_geometry, 's')
        self.x = get_float(node_geometry, 'x')
        self.y = get_float(node_geometry, 'y')
        self.hdg = get_float(node_geometry, 'hdg')
        self.heading = self.hdg
        self.length = get_float(node_geometry, 'length')
        self.geo_type = GeometryType.unknown
        self.start_position = np.array([self.x, self.y])         

class GeometryLine(Geometry):
    def __init__(self, node_geometry):        
        super().__init__(node_geometry)  
        self.geo_type = GeometryType.Line           

class GeometryArc(Geometry):
    def __init__(self, node_geometry):
        super().__init__(node_geometry)
        self.geo_type = GeometryType.Arc
        self.curvature = None

        node_arc = node_geometry.find('arc')
        if node_arc is not None:
            self.curvature = get_float(node_arc, 'curvature')

class GeometryPoly3(Geometry):
    def __init__(self, node_geometry):
        super().__init__(node_geometry)
        self.geo_type = GeometryType.Poly3
        self.a = 0
        self.b = 0
        self.c = 0
        self.d = 0

        node_poly3 = node_geometry.find('poly3')
        if node_poly3 is not None:
            self.a = get_float(node_poly3, 'a')
            self.b = get_float(node_poly3, 'b')
            self.c = get_float(node_poly3, 'c')
            self.d = get_float(node_poly3, 'd')

class GeometryParamPoly3(Geometry):
    def __init__(self, node_geometry):
        super().__init__(node_geometry)
        self.geo_type = GeometryType.ParamPoly3
        self.aU = 0
        self.bU = 0
        self.cU = 0
        self.dU = 0
        self.aV = 0
        self.bV = 0
        self.cV = 0
        self.dV = 0 
        self.pRange = 0

        node_param_poly3 = node_geometry.find('ParamPoly3')
        if node_param_poly3 is not None:
            self.aU = get_float(node_param_poly3, 'aU')
            self.bU = get_float(node_param_poly3, 'bU')
            self.cU = get_float(node_param_poly3, 'cU')
            self.dU = get_float(node_param_poly3, 'dU')
            self.aV = get_float(node_param_poly3, 'aV')
            self.bV = get_float(node_param_poly3, 'bV')
            self.cV = get_float(node_param_poly3, 'cV')
            self.dV = get_float(node_param_poly3, 'dV')            
            self.pRange = get_float(node_param_poly3, 'pRange')

class RoadElevation:
    def __init__(self, node_elevation):
        # <elevation s="0" a="12" b="0" c="0" d="0" />
        self.s = get_float(node_elevation, 's')
        self.a = get_float(node_elevation, 'a')
        self.b = get_float(node_elevation, 'b')
        self.c = get_float(node_elevation, 'c')
        self.d = get_float(node_elevation, 'd')
        # print('RoadElevation', self.s, self.a, self.b, self.c, self.d)

class RoadLaneOffset:
    def __init__(self, node_laneOffset):
        # <laneOffset s="0" a="0" b="0" c="0" d="0" />
        self.s = get_float(node_laneOffset, 's')
        self.a = get_float(node_laneOffset, 'a')
        self.b = get_float(node_laneOffset, 'b')
        self.c = get_float(node_laneOffset, 'c')
        self.d = get_float(node_laneOffset, 'd')

class RoadLaneSection:
    def __init__(self, node_laneSection):
        self.id = -1
        self.s = get_float(node_laneSection, 's')
        self.left_lanes = []
        self.center_lanes = []
        self.right_lanes = []
        self.all_lanes = {}

        node_left = node_laneSection.find('left')
        if node_left is not None:
            for node_lane in node_left.findall('lane'):
                lane = Lane(node_lane)
                self.left_lanes.append(lane)
                self.all_lanes[lane.id] = lane
        
        node_center = node_laneSection.find('center')
        if node_center is not None:
            for node_lane in node_center.findall('lane'):
                lane = Lane(node_lane)
                self.center_lanes.append(lane)
                self.all_lanes[lane.id] = lane
        
        node_right = node_laneSection.find('right')
        if node_right is not None:
            for node_lane in node_right.findall('lane'):
                lane = Lane(node_lane)
                self.right_lanes.append(lane)
                self.all_lanes[lane.id] = lane

class LaneLinkElement: 
    def __init__(self, node_element):
        self.id = get_int(node_element, 'id')

class LaneLink: 
    def __init__(self, node_roadlink):
        # <link>
        #     <predecessor id="-1" />
        #     <successor id="-1" />
        # </link>  
        self.predecessor = None
        self.successor = None

        node_predecessor = node_roadlink.find('predecessor')
        if node_predecessor is not None:
            self.predecessor = LaneLinkElement(node_predecessor)

        node_successor = node_roadlink.find('successor')
        if node_successor is not None:
            self.successor = LaneLinkElement(node_successor)


class LaneWidth: 
    def __init__(self, node_width):
        # <width a="0.2" b="0" c="0" d="0" sOffset="0" /> 
        self.a = get_float(node_width, 'a')
        self.b = get_float(node_width, 'b')
        self.c = get_float(node_width, 'c')
        self.d = get_float(node_width, 'd')
        self.sOffset = get_float(node_width, 'sOffset')

class LaneMark: 
    def __init__(self, node_mark):
        # <roadMark color="standard" sOffset="0" type="none" weight="standard" />
        self.color = get_string(node_mark, 'color')
        self.sOffset = get_float(node_mark, 'sOffset')
        self.type = get_string(node_mark, 'type')
        self.weight = get_string(node_mark, 'weight') 

class LaneSpeed: 
    def __init__(self, node_speed):
        # <speed sOffset="0" max="13.8" /> 
        self.sOffset = get_float(node_speed, 'sOffset')
        self.max = get_float(node_speed, 'max')

class laneHeight:
    def __init__(self, node_height):
        # <height sOffset="0" inner="0.2" outer="0.2" /> 
        self.sOffset = get_float(node_height, 'sOffset')
        self.inner = get_float(node_height, 'inner')
        self.outer = get_float(node_height, 'outer')

class LaneUserDataStyle:
    def __init__(self, node_style):
        # <style sOffset="0" laneStyle="spec_concrete_3d" mapping="auto" />
        self.sOffset = get_float(node_style, 'sOffset')
        self.laneStyle = get_string(node_style, 'laneStyle')
        self.mapping = get_string(node_style, 'mapping')

class LaneUserDataFillet:
    def __init__(self, node_fillet):
        # <fillet pos="leftOfLane" style="grass" id="650" />        
        self.id = get_int(node_fillet, 'id')
        self.pos = get_string(node_fillet, 'pos')
        self.style = get_string(node_fillet, 'style')

class LaneUserData:
    def __init__(self, node_user):
        # <userData code="viStyleDef">
        #     <style sOffset="0" laneStyle="spec_concrete_3d" mapping="auto" />
        #     <fillet pos="leftOfLane" style="grass" id="650" />
        # </userData>
        self.code = get_string(node_user, 'code')
        self.style = None
        self.fillet = None

        node_style = node_user.find('style')
        if node_style is not None:
            self.style = LaneUserDataStyle(node_style)

        node_fillet = node_user.find('fillet')
        if node_fillet is not None:
            self.fillet = LaneUserDataFillet(node_fillet)

class Lane:
    def __init__(self, node_lane):
        self.id = get_int(node_lane, 'id')
        self.level = get_string(node_lane, 'level')
        self.type = get_string(node_lane, 'type')
        self.link = None
        self.width = None
        self.roadMarks = []
        self.speed = None
        self.height = None
        self.userData = None

        node_link = node_lane.find('link')
        if node_link is not None:
            self.link = LaneLink(node_link)

        node_width = node_lane.find('width')
        if node_width is not None:
            self.width = LaneWidth(node_width)

        for node_mark in node_lane.findall('roadMark'):
            roadMark = LaneMark(node_mark)
            self.roadMarks.append(roadMark)

        node_speed = node_lane.find('speed')
        if node_speed is not None:
            self.speed = LaneSpeed(node_speed)

        node_height = node_lane.find('height')
        if node_height is not None:
            self.height = laneHeight(node_height)

        node_user = node_lane.find('userData')
        if node_user is not None:
            self.userData = LaneUserData(node_user)

class RoadObject:
    def __init__(self, node_object):
        # <object id="100101" type="vegetation" name="VegBush04.flt" s="0" t="1" zOffset="0" length="3" width="3" height="3" hdg="0.7055475" pitch="0" roll="0" />
        self.id = get_int(node_object, 'id')
        self.type = get_string(node_object, 'type')
        self.name = get_string(node_object, 'name')
        self.s = get_float(node_object, 's')
        self.t = get_float(node_object, 't')
        self.zOffset = get_float(node_object, 'zOffset')
        self.length = get_float(node_object, 'length')
        self.width = get_float(node_object, 'width')
        self.height = get_float(node_object, 'height')
        self.hdg = get_float(node_object, 'hdg')
        self.pitch = get_float(node_object, 'pitch')
        self.roll = get_float(node_object, 'roll')

class Road:
    def __init__(self, node_road):        
        self.id = None
        self.name = None
        self.junction = None
        self.length = None

        self.link = None
        self.type = None
        self.planView = []
        self.elevationProfile = []
        self.laneOffsets = []
        self.laneSections = []
        self.objects = []

        # <road id="201" name="" junction="-1" length="70.6552863027954">
        self.id = get_int(node_road, 'id')
        self.name = get_string(node_road, 'name')
        self.junction = get_int(node_road, 'junction')
        self.length = get_float(node_road, 'length')        

        if self.id == 1002 or self.id==104431 or True:
            print('road id', self.id)
            # print(self.id, self.name, self.junction, self.length)

            node_link = node_road.find('link')
            if node_link is not None:
                self.link = RoadLink(node_link)

            node_type = node_road.find('type')
            if node_type is not None:
                self.type = RoadType(node_type)

            node_planview = node_road.find('planView')
            if node_planview is not None:
                for node_geometry in node_planview.findall('geometry'): 
                    geometry = None
                    if node_geometry.find('line') is not None:                        
                        geometry = GeometryLine(node_geometry)
                    elif node_geometry.find('arc') is not None:                        
                        geometry = GeometryArc(node_geometry)
                    elif node_geometry.find('poly3') is not None:                        
                        geometry = GeometryPoly3(node_geometry)                    
                    if geometry is not None:
                        self.planView.append(geometry)

            node_elevations = node_road.find('elevationProfile')
            if node_elevations is not None:
                for node_elevation in node_elevations.findall('elevation'):
                    elevation = RoadElevation(node_elevation)
                    self.elevationProfile.append(elevation)
                    
            node_lanes = node_road.find('lanes')
            if node_lanes is not None:
                for node_laneOffset in node_lanes.findall('laneOffset'):
                    laneOffset = RoadLaneOffset(node_laneOffset)
                    self.laneOffsets.append(laneOffset)
                
                for node_laneSection in node_lanes.findall('laneSection'):
                    laneSection = RoadLaneSection(node_laneSection)
                    laneSection.id = len(self.laneSections)
                    self.laneSections.append(laneSection)

            node_objects = node_road.find('objects')
            if node_objects is not None:
                for node_object in node_objects.findall('object'):
                    road_object =  RoadObject(node_object)  
                    self.objects.append(road_object)     

class Control:
    def __init__(self, node_control):
        self.signalId = get_int(node_control, 'signalId')
        self.type = get_string(node_control, 'type')

class Controller:
    def __init__(self, node_controller):
        self.id = get_int(node_controller, 'id')
        self.controls = []

        for node_control in node_controller.findall('control'):
            control = Control(node_control)
            self.controls.append(control)

class JunctionLaneLink:
    def __init__(self, node_laneLink):        
        self.from_lane = get_int(node_laneLink, 'from')
        self.to_lane = get_int(node_laneLink, 'to')

class JunctionConnection:
    def __init__(self, node_connection):
        self.id = get_int(node_connection, 'id')
        self.incomingRoad = get_int(node_connection, 'incomingRoad')
        self.connectingRoad = get_int(node_connection, 'connectingRoad')
        self.contactPoint = get_string(node_connection, 'contactPoint')
        self.laneLinks = []

        for node_laneLink in node_connection.findall('laneLink'):
            laneLink = JunctionLaneLink(node_laneLink)
            self.laneLinks.append(laneLink)
     
class Junction:
    def __init__(self, node_junction):
        self.id = get_int(node_junction, 'id')
        self.name = get_string(node_junction, 'name')
        self.connections = []

        for node_connection in node_junction.findall('connection'):
            connection = JunctionConnection(node_connection)
            self.connections.append(connection)
        
class OpenDrive:
    def __init__(self, node_root):
        self.header = None
        self.roads = []
        self.junctions = []
        self.controllers = []

        node_header = node_root.find('header')
        if node_header is not None:
            self.header = Header(node_header)

        for node_road in node_root.findall('road'):
            road = Road(node_road)
            self.roads.append(road)

        for node_controller in node_root.findall('controller'):
            controller = Controller(node_controller)
            print('controller', controller.id)
            self.controllers.append(controller)

        for node_junction in node_root.findall('junction'):
            junction = Junction(node_junction)
            print('junction', junction.id)
            self.junctions.append(junction)


if __name__ == "__main__":
    
    xodr_file = r'sample_largezone.xodr'
    
    if os.path.exists(xodr_file):
        doc = etree.parse(xodr_file)
        opendrive = OpenDrive(doc.getroot())    
        print('parse ok')

        for road in opendrive.roads:
            for lanesection in road.laneSections:
                for lane in lanesection.all_lanes.values():
                    if len(lane.roadMarks) > 1:
                        print(road.id, lanesection.id, lanesection.s)
    else:
        print(xodr_file, 'not found')
     