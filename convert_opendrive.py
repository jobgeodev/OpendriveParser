# -*- coding=utf-8 -*-

import opendrive_parser_14H as opendrive_parser
import numpy as np
import math

# pip install generateDS
# https://pypi.org/project/generateDS/
# 必须生成指定版本的数据
# generateDS.py -o opendrive_parser_15M.py OpenDRIVE_1.5M.xsd 
# generateDS.py -o opendrive_parser_14H.py OpenDRIVE_1.4H.xsd  ok
# https://sourceforge.net/projects/generateds/
# http://www.davekuhlman.org/generateDS.html

# https://pypi.org/project/Shapely/
# pip install Shapely
# import shapely
from shapely.geometry import (Point, LineString, Polygon, MultiPolygon, LinearRing, MultiPoint)
from shapely.validation import explain_validity
from shapely.ops import (unary_union, triangulate, polygonize, polygonize_full)

# https://pypi.org/project/geojson/
# pip install geojson
import geojson

class Geometry():
    def __init__(self, start_position: float, heading: float, s : float, length: float):
        self.start_position = np.array(start_position)
        self.length = length
        self.heading = heading
        self.s = s

    def calc_position(self, s_pos):
        pass

class Line(Geometry):
    def calc_position(self, s_pos):        
        pos = self.start_position + np.array( [s_pos * np.cos(self.heading), s_pos * np.sin(self.heading)] )
        tangent = self.heading

        return (pos, tangent)    
    
class Arc(Geometry):
    def __init__(self, start_position, heading, s, length, curvature):
        self.curvature = curvature
        super().__init__(start_position=start_position, heading=heading, s = s, length=length)

    def calc_position(self, s_pos):
        c = self.curvature
        hdg = self.heading - np.pi / 2

        a = 2 / c * np.sin(s_pos * c / 2)
        alpha = (np.pi - s_pos * c) / 2 - hdg

        dx = -1 * a * np.cos(alpha)
        dy = a * np.sin(alpha)

        pos = self.start_position + np.array([dx, dy])
        tangent = self.heading + s_pos * self.curvature

        return (pos, tangent)
    
class Poly3(Geometry):
    def __init__(self, start_position, heading, s, length, a, b, c, d):
        self._a = a
        self._b = b
        self._c = c
        self._d = d
        super().__init__(start_position=start_position, heading=heading, s = s, length=length)       

    def calc_position(self, s_pos):
        # Calculate new point in s_pos/t coordinate system
        coeffs = [self._a, self._b, self._c, self._d]

        t = np.polynomial.polynomial.polyval(s_pos, coeffs)

        # Rotate and translate
        srot = s_pos * np.cos(self.heading) - t * np.sin(self.heading)
        trot = s_pos * np.sin(self.heading) + t * np.cos(self.heading)

        # Derivate to get heading change
        dCoeffs = coeffs[1:] * np.array(np.arange(1, len(coeffs)))
        tangent = np.polynomial.polynomial.polyval(s_pos, dCoeffs)

        return (self.start_position + np.array([srot, trot]), self.heading + tangent)

        
class ConvertOpenDrive:
    def __init__(self, open_drive, interval:float = 0.5):
        self.open_drive = open_drive
        self.interval = interval
        # 路段的所有插值点
        # self.road_inters = []
        self.road_length = 0
        self.road_geometrys = []
        self.road_geometrys_pos = []
        self.road_laneOffsets = []
        self.road_laneOffsets_pos = []
        self.road_laneSections = []
        self.road_laneSections_pos = []
        self.road_elevations = []
        self.road_elevations_pos = []

        # self.road_geometry_inters = {}
        # self.

    def calc_interpolates(self, pos_offset0, pos_offset1):
        vals = []
        p0 = pos_offset0
        p1 = pos_offset1
        if p1 > p0 :
            vals = np.append(np.arange(p0, p1, self.interval), p1)        
        return vals

    def calc_lateral_offset(self, start_position, heading, lateral_offset, is_left) : 
        pi_half = math.pi * 0.5 
        if is_left == False:
            pi_half *= -1.0    
        pos = start_position + np.array( [lateral_offset * np.cos(heading + pi_half), lateral_offset * np.sin(heading + pi_half)] )
        tangent = heading    
        return (pos, tangent) 

    def init_road(self, road):
        # self.road_inters = []
        self.road_length = road.length
        self.road_geometrys = []
        self.road_geometrys_pos = []
        self.road_laneOffsets = []
        self.road_laneOffsets_pos = []
        self.road_laneSections = []
        self.road_laneSections_pos = []
        self.road_elevations = []
        self.road_elevations_pos = []
        # self.road_geometry_inters = {}

        # self.road_inters = self.calc_interpolates(0, road.length)
        # print(road.id, 'inters', len(self.road_inters))

        for geometry in road.planView.get_geometry():
            # print(geometry.s, geometry.x, geometry.y, geometry.hdg)
            start_position = np.array([geometry.x, geometry.y])  
            heading = geometry.hdg
            start_pos = geometry.s
            length = geometry.length
            
            self.road_geometrys_pos.append(geometry.s)

            if geometry.line is not None:
                # print('is line')
                self.road_geometrys.append(Line(start_position, heading, start_pos, length))
            elif geometry.arc is not None:
                # print('is arc')
                self.road_geometrys.append(Arc(start_position, heading, start_pos, length, geometry.arc.curvature))
            elif geometry.poly3 is not None:
                # print('is poly3')
                self.road_geometrys.append(Poly3(start_position, heading, start_pos, length, geometry.poly3.a, geometry.poly3.b, geometry.poly3.c, geometry.poly3.d))
            elif geometry.spiral is not None:
                print('is spiral')
            elif geometry.paramPoly3 is not None:
                print('is paramPoly3')

        if road.lanes is not None:
            for laneOffset in road.lanes.get_laneOffset():
                self.road_laneOffsets.append(laneOffset)
                self.road_laneOffsets_pos.append(laneOffset.s)
            
            if len(self.road_laneOffsets) == 0:
                laneOffset = opendrive_parser.laneOffsetType(0,0,0,0,0)
                self.road_laneOffsets.append(laneOffset)
                self.road_laneOffsets_pos.append(laneOffset.s)

            for laneSection in road.lanes.get_laneSection():
                self.road_laneSections.append(laneSection)    
                self.road_laneSections_pos.append(laneSection.s)   

        if road.elevationProfile is not None:
            for elevation in road.elevationProfile.elevation:
                self.road_elevations.append(elevation)
                self.road_elevations_pos.append(elevation.s)
        # print(self.road_elevations_pos)

    def json_all_baseroad(self, json_file):
        fc = []
        for road in self.open_drive.get_road(): 
            self.init_road(road)
            geometry_count = len(self.road_geometrys)
            for geometry_idx, geometry in enumerate(self.road_geometrys):
                pts = []
                for s_pos in self.calc_interpolates(0, geometry.length):
                    pos, tangent = geometry.calc_position(s_pos)
                    pts.append(pos)
                props = {}
                props['road_id'] = road.id 
                props['road_name'] = road.name  
                props['road_length'] = road.length
                props['geometry_length'] = geometry.length
                props['geometry_index'] = geometry_idx
                props['geometry_count'] = geometry_count

                f = geojson.Feature(geometry=LineString(pts), properties=props)
                fc.append(f)
        self.save_json_features(fc, json_file)    
        
    def calc_abcd(self, a:float, b:float, c:float, d:float, s_pos:float):
        ds = s_pos
        return a + b * ds + c * ds * ds + d * ds * ds * ds
    
    # part_type == 1   planView geometry
    # part_type == 2   laneOffset
    # part_type == 3   laneSection
    # part_type == 4   elevation
    def calc_part_index(self, r_pos:float, part_type:int):
        parts_pos = []
        if part_type==1:
            parts_pos = self.road_geometrys_pos.copy()
        elif part_type==2:
            parts_pos = self.road_laneOffsets_pos.copy()
        elif part_type==3:
            parts_pos = self.road_laneSections_pos.copy()
        elif part_type==4:
            parts_pos = self.road_elevations_pos.copy()
        parts_pos.append(self.road_length)    
        
        part_index = -1
        for idx, spos in enumerate(parts_pos):
            epos = parts_pos[idx+1]
            if r_pos >= spos and r_pos <= epos:
                part_index = idx
                break
        return part_index

    def calc_geometry_position(self, r_pos:float):
        tuple_pos = None
        part_idx = self.calc_part_index(r_pos, part_type=1)
        if part_idx > -1:
            geometry = self.road_geometrys[part_idx]
            s_pos = r_pos - geometry.s
            tuple_pos = geometry.calc_position(s_pos)
        return tuple_pos 

    def calc_offset_value(self, r_pos:float):
        val_offset = 0
        part_idx = self.calc_part_index(r_pos, part_type=2)
        if part_idx > -1:
            offset = self.road_laneOffsets[part_idx]
            s_pos = r_pos - offset.s
            val_offset = self.calc_abcd(offset.a, offset.b, offset.c, offset.d, s_pos)
        return val_offset 

    def calc_road_section_lane_ids(self, lane_section):
        lane_ids = []
        lanes = []
        if lane_section.left is not None:
            lanes.extend(lane_section.left.lane)
        if lane_section.right is not None:
            lanes.extend(lane_section.right.lane)
        if lane_section.center is not None:
            lanes.append(lane_section.center.lane)

        for lane in lanes:
            lane_ids.append(lane.id)
        return lane_ids

    def calc_lane_width(self, lane, s_pos):
        width = 0
        if lane.id != 0:
            lane_widths = lane.width.copy()
            lane_widths.reverse()
            for lane_width in lane_widths:
                if s_pos >= lane_width.sOffset :
                    width = self.calc_abcd(lane_width.a, lane_width.b, lane_width.c, lane_width.d, s_pos - lane_width.sOffset)
                    break

        return width   
    
    def calc_elevation(self, r_pos):
        val_elev = 0
        elevation_idx = self.calc_part_index(r_pos, part_type=4)
        if elevation_idx > -1:
            elevation = self.road_elevations[elevation_idx]
            s_pos = r_pos - elevation.s
            val_elev = self.calc_abcd(elevation.a, elevation.b, elevation.c, elevation.d, s_pos)
        
        return val_elev

    def get_lane(self, lane_section, lane_id):
        lane = None
        
        lanes = []
        if lane_section.left is not None:
            lanes.extend(lane_section.left.lane)
        if lane_section.right is not None:
            lanes.extend(lane_section.right.lane)
        if lane_section.center is not None:
            lanes.append(lane_section.center.lane)

        for _lane in lanes:
            if _lane.id == lane_id :
                lane = _lane

        return lane

    def calc_road_section_lane_pts(self, road_id, section_id, lane_id):
        sections_pos = self.road_laneSections_pos.copy()
        sections_pos.append(self.road_length)        

        laneSection = self.road_laneSections[section_id]
        cur_section_s_pos = laneSection.s
        cur_section_e_pos = sections_pos[section_id + 1]

        pts = []
        for r_pos in self.calc_interpolates(cur_section_s_pos, cur_section_e_pos):
            s_pos = r_pos - cur_section_s_pos

            pos, tangent = self.calc_geometry_position(r_pos)
            val_offset = self.calc_offset_value(r_pos)
            pos2, tangent2 = self.calc_lateral_offset(pos, tangent, abs(val_offset), True if val_offset > 0 else False)
            
            lane_ids = []
            if lane_id == 0 :
                lane_ids.append(lane_id)
            elif lane_id > 0 :
                for id in range(1, lane_id + 1):
                    lane_ids.append(id)
            elif lane_id < 0 :
                for id in range(1, abs(lane_id) + 1):
                    lane_ids.append(id * -1)

            width_offset = 0        
            for lane_id in lane_ids:
                lane = self.get_lane(laneSection, lane_id)
                width_offset += self.calc_lane_width(lane, s_pos)

            pos3, tangent3 = self.calc_lateral_offset(pos2, tangent2, abs(width_offset), True if lane_id > 0 else False)

            # add z value
            x,y = pos3
            z = self.calc_elevation(r_pos)
            pos3 = (pos3[0], pos3[1], z) 

            pts.append(pos3)              
        
        return pts
    
    # 用于计算mark标线
    def calc_road_section_lane_range_pts(self, road_id, section_id, lane_id, offset_pos0:float, offset_pos1:float):
        sections_pos = self.road_laneSections_pos.copy()
        sections_pos.append(self.road_length)        

        laneSection = self.road_laneSections[section_id]
        cur_section_s_pos = laneSection.s
        cur_section_e_pos = sections_pos[section_id + 1]

        pts = []
        for r_pos in self.calc_interpolates(offset_pos0, offset_pos1):
            s_pos = r_pos - cur_section_s_pos

            # print(r_pos, s_pos, cur_section_s_pos, cur_section_e_pos, offset_pos0, offset_pos1)

            pos, tangent = self.calc_geometry_position(r_pos)
            val_offset = self.calc_offset_value(r_pos)
            pos2, tangent2 = self.calc_lateral_offset(pos, tangent, abs(val_offset), True if val_offset > 0 else False)
            
            lane_ids = []
            if lane_id == 0 :
                lane_ids.append(lane_id)
            elif lane_id > 0 :
                for id in range(1, lane_id + 1):
                    lane_ids.append(id)
            elif lane_id < 0 :
                for id in range(1, abs(lane_id) + 1):
                    lane_ids.append(id * -1)

            width_offset = 0        
            for lane_id in lane_ids:
                lane = self.get_lane(laneSection, lane_id)
                width_offset += self.calc_lane_width(lane, s_pos)

            pos3, tangent3 = self.calc_lateral_offset(pos2, tangent2, abs(width_offset), True if lane_id > 0 else False)

            # add z value
            x,y = pos3
            z = self.calc_elevation(r_pos)
            pos3 = (pos3[0], pos3[1], z) 

            pts.append(pos3)
            # print(lane_ids)
        
        return pts

    def json_all_baselane(self, json_file):
        fc = []
        for road in self.open_drive.get_road():  
            self.init_road(road)
            offset_count = len(self.road_laneOffsets)
            offsets_pos = self.road_laneOffsets_pos.copy()
            offsets_pos.append(road.length)

            print(road.id, offset_count)

            for offset_idx, base_offset in enumerate(self.road_laneOffsets):
                cur_offset_s_pos = base_offset.s
                cur_offset_e_pos = offsets_pos[offset_idx + 1]                

                pts = []
                for r_pos in self.calc_interpolates(cur_offset_s_pos, cur_offset_e_pos):
                    s_pos = r_pos - cur_offset_s_pos
                    val_offset = self.calc_abcd(base_offset.a, base_offset.b, base_offset.c, base_offset.d, s_pos)
                    pos, tangent = self.calc_geometry_position(r_pos)
                    pos2, tangent2 = self.calc_lateral_offset(pos, tangent, abs(val_offset), True if val_offset > 0 else False)
                    pts.append(pos2)

                props = {}
                props['road_id'] = road.id 
                props['road_name'] = road.name  
                props['road_length'] = road.length
                props['offset_length'] = cur_offset_e_pos - cur_offset_s_pos
                props['offset_idx'] = offset_idx
                props['offset_count'] = offset_count

                f = geojson.Feature(geometry=LineString(pts), properties=props)
                fc.append(f)

        self.save_json_features(fc, json_file)

    def json_all_baselane2(self, json_file):
        fc = []
        for road in self.open_drive.get_road():  
            self.init_road(road)
            offset_count = len(self.road_laneOffsets)
            
            pts = []
            for r_pos in self.calc_interpolates(0, road.length):
                pos, tangent = self.calc_geometry_position(r_pos)
                val_offset = self.calc_offset_value(r_pos)
                pos2, tangent2 = self.calc_lateral_offset(pos, tangent, abs(val_offset), True if val_offset > 0 else False)
                pts.append(pos2)

            props = {}
            props['road_id'] = road.id 
            props['road_name'] = road.name  
            props['road_length'] = road.length
            # props['offset_length'] = cur_offset_e_pos - cur_offset_s_pos
            # props['offset_idx'] = offset_idx
            props['offset_count'] = offset_count

            f = geojson.Feature(geometry=LineString(pts), properties=props)
            fc.append(f)

        self.save_json_features(fc, json_file)
               
    def json_all_lanes(self, json_file):
        fc = []
        for road in self.open_drive.get_road():  
            self.init_road(road)
            
            for section_idx, laneSection in enumerate(self.road_laneSections):
                
                lane_ids = self.calc_road_section_lane_ids(laneSection)
                for lane_id in lane_ids:
                    pts = self.calc_road_section_lane_pts(road.id, section_idx, lane_id)
                    
                    props = {}
                    props['road_id'] = road.id 
                    props['road_name'] = road.name  
                    props['road_length'] = road.length                    
                    props['section_idx'] = section_idx
                    props['lane_id'] = lane_id

                    f = geojson.Feature(geometry=LineString(pts), properties=props)
                    fc.append(f)

        self.save_json_features(fc, json_file)    

    def json_all_marks(self, json_file):
        fc = []
        for road in self.open_drive.get_road():  
            self.init_road(road)

            # if int(road.id) != 501:
            #     continue

            sections_pos = self.road_laneSections_pos.copy()
            sections_pos.append(road.length)
            
            for section_idx, laneSection in enumerate(self.road_laneSections):
                section_pos_range = laneSection.s, sections_pos[section_idx+1]
                print('section', road.id, section_idx, section_pos_range)
                
                lane_ids = self.calc_road_section_lane_ids(laneSection)
                for lane_id in lane_ids:
                    lane = self.get_lane(laneSection, lane_id)
                    
                    lanemarks_pos = []
                    for lanemark in lane.roadMark:
                        lanemarks_pos.append(lanemark.sOffset)
                    lanemarks_pos.append(section_pos_range[1])
                    
                    for lane_mark_idx, lane_mark in enumerate(lane.roadMark):
                        lane_mark_pos_range = section_pos_range[0] + lane_mark.sOffset, lanemarks_pos[lane_mark_idx+1]
                        print('lanemark', road.id, section_idx, lane_id, lane_mark_idx, lane_mark.sOffset, lane_mark_pos_range)

                        pts = self.calc_road_section_lane_range_pts(road.id, section_idx, lane_id, lane_mark_pos_range[0], lane_mark_pos_range[1])
                    
                        props = {}
                        props['road_id'] = road.id 
                        props['road_name'] = road.name  
                        props['road_length'] = road.length                    
                        props['section_idx'] = section_idx
                        props['lane_id'] = lane_id
                        props['lane_mark_idx'] = lane_mark_idx
                        props['lanemark_color'] = lane_mark.color
                        props['lanemark_type'] = lane_mark.type_
                        props['lanemark_weight'] = lane_mark.weight
                        # color="standard" type="none" weight="standard

                        f = geojson.Feature(geometry=LineString(pts), properties=props)
                        fc.append(f)

        self.save_json_features(fc, json_file) 

    def json_all_objects(self, json_file):
        fc = []
        for road in self.open_drive.get_road():  
            self.init_road(road)

            for road_object in road.objects.object:
                print(road.id, road_object.id, road_object.s, road_object.t, road.length)   
                
                if road_object.s < 0 or road_object.s > road.length:
                    continue

                pos, tangent = self.calc_geometry_position(road_object.s)

                offset = road_object.t
                pos2, tangent2 = self.calc_lateral_offset(pos, tangent, abs(offset), True if offset>0 else False)

                props = {}
                props['roadid'] = road.id 
                props['id'] = road_object.id 
                props['type'] = road_object.type_
                props['name'] = road_object.name
                props['zOffset'] = road_object.zOffset                
                props['length'] = road_object.length
                props['width'] = road_object.width
                props['height'] = road_object.height
                props['hdg'] = road_object.hdg + tangent2
                props['pitch'] = road_object.pitch
                props['roll'] = road_object.roll   
                
                x,y = pos2
                z = self.calc_elevation(road_object.s)
                pos2 = (pos2[0], pos2[1], z)   

                f = geojson.Feature(geometry=Point(pos2), properties=props)
                fc.append(f)  

        self.save_json_features(fc, json_file) 

    def json_all_signals(self, json_file):
        fc = []
        for road in self.open_drive.get_road():  
            self.init_road(road)

            if road.signals is None:
                continue

            for signal in road.signals.signal:
                if signal.s < 0 or signal.s > road.length:
                    continue

                pos, tangent = self.calc_geometry_position(signal.s)

                offset = signal.t
                pos2, tangent2 = self.calc_lateral_offset(pos, tangent, abs(offset), True if offset>0 else False)

                # print(tangent, tangent2)
        
                props = {}
                props['id'] = signal.id 
                props['name'] = signal.name
                props['dynamic'] = signal.dynamic 
                props['orientation'] = signal.orientation
                props['zOffset'] = signal.zOffset  
                props['width'] = signal.width
                props['height'] = signal.height
                props['country'] = signal.country
                props['type'] = signal.type_
                props['subtype'] = signal.subtype
                props['value'] = signal.value
                props['text'] = signal.text
                props['hOffset'] = signal.hOffset + tangent
                props['pitch'] = signal.pitch
                props['roll'] = signal.roll
                props['signal_tag'] = 'signal'                 

                x,y = pos2
                z = self.calc_elevation(signal.s)
                pos2 = (pos2[0], pos2[1], z)             

                f = geojson.Feature(geometry=Point(pos2), properties=props)
                fc.append(f)  

        self.save_json_features(fc, json_file)        

    # 计算交叉路口的连接车道
    def calc_connection_lane_id(self, junction, connection_id):
        from_lanes = []
        to_lanes = []
   
        for connection in junction.connection:
            if connection.id == connection_id:
                for laneLink in connection.laneLink:
                    from_lanes.append(laneLink.from_)
                    from_lanes.append(laneLink.from_ + 1)
                    to_lanes.append(laneLink.to)
                    to_lanes.append(laneLink.to + 1)

        # print(from_lanes, to_lanes)

        from_lanes = list(set(from_lanes)) 
        from_lanes.sort(reverse=True)

        to_lanes = list(set(to_lanes))
        to_lanes.sort(reverse=True)

        # print(from_lanes, to_lanes)
        return (from_lanes, to_lanes)
    
    # 获取道路road对象
    def get_road(self, road_id):
        road = None
        for _road in self.open_drive.get_road():  
            if _road.id == road_id:
                road = _road
        return road

    # 输出所有的路口面
    def json_all_crossings(self, json_file):
        fc = []
        for junction in self.open_drive.junction:
            polygons = []
            props = {}
            for connection in junction.connection:
                # print(junction.id, connection.id)
                road = self.get_road(connection.connectingRoad)
                self.init_road(road)
                
                from_lanes, to_lanes = self.calc_connection_lane_id(junction, connection.id)
                print(junction.id, connection.id, 'sections', len(self.road_laneSections), to_lanes)

                # tail_pts = []
                # section_id = 0
                # for lane_id in to_lanes:
                #     pts = self.calc_road_section_lane_pts(road.id, section_id, lane_id)
                #     tail_pts.append(pts[0])

                # head_pts = []
                # section_id = len(self.road_laneSections) - 1 
                # for lane_id in to_lanes:
                #     pts = self.calc_road_section_lane_pts(road.id, section_id, lane_id)
                #     head_pts.append(pts[0])

                l_pts = []
                for section_id, section in enumerate(self.road_laneSections):
                    pts = self.calc_road_section_lane_pts(road.id, section_id, to_lanes[0])
                    l_pts.extend(pts) 
                    
                r_pts = []
                for section_id, section in enumerate(self.road_laneSections):
                    pts = self.calc_road_section_lane_pts(road.id, section_id, to_lanes[-1])
                    r_pts.extend(pts) 
                
                # 闭合多边形
                fill_pts = []
                # fill_pts.extend(tail_pts)
                fill_pts.extend(r_pts)
                # head_pts.reverse()
                # fill_pts.extend(head_pts)
                l_pts.reverse()
                fill_pts.extend(l_pts)                
                    
                # 使用多边形的方式构建
                ring = LinearRing(fill_pts)
                ring = ring.simplify(0.1)
                gon = Polygon(ring)
                polygon = gon.buffer(0)
                polygons.append(polygon)
    
                # 输出中间数据
                # props = {}
                # f = geojson.Feature(geometry=polygon, properties=props)
                # fc.append(f)    

            out_gon = unary_union(polygons)
            out_gon = out_gon.buffer(0.1)
            # print(out_gon.area, out_gon.is_simple, out_gon.is_valid, out_gon.geom_type)

            if out_gon.geom_type == 'MultiPolygon':   
                for geom in out_gon.geoms:
                    # print(geom)
                    gon = Polygon(geom.exterior.coords)
                    f = geojson.Feature(geometry=gon, properties=props)
                    fc.append(f)
            elif out_gon.geom_type == 'Polygon':
                gon = Polygon(out_gon.exterior)
                f = geojson.Feature(geometry=gon, properties=props)
                fc.append(f)
            else:
                print('else else', out_gon.geometryType)               
                        
        self.save_json_features(fc, json_file)   

    def save_json_features(self, fc, json_file):
        with open(json_file , mode='w+') as f:
            fs = geojson.FeatureCollection(fc)
            dump = geojson.dumps(fs, sort_keys=True)
            f.write(dump)
            f.close()
            
            
if __name__ == "__main__":

    xodr_file = r'sample_largezone.xodr'
    opendrive = opendrive_parser.parse(xodr_file, silence=True)

    convert = ConvertOpenDrive(opendrive, 0.5)

    convert.json_all_baseroad(r'./tmp/all_baseroad_new.json')

    convert.json_all_baselane(r'./tmp/all_baselane_new.json')

    convert.json_all_baselane2(r'./tmp/all_baselane2_new.json')

    convert.json_all_lanes(r'./tmp/all_lanes_new.json')

    convert.json_all_marks(r'./tmp/all_marks_new.json')

    convert.json_all_signals(r'./tmp/all_signals_new.json')

    convert.json_all_objects(r'./tmp/all_objects_new.json')

    convert.json_all_crossings(r'./tmp/all_crossings_new.json')

