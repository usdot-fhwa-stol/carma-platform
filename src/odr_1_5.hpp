// Copyright (c) 2020 Jens Klimke (jens.klimke@rwth-aachen.de). All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Generated with xsd2cpp (https://github.com/JensKlimke/xsd2cpp) on 2020-08-13
//

#ifndef XML_PARSER_ODR_1_5_HPP
#define XML_PARSER_ODR_1_5_HPP

#include <vector>
#include <string>
#include <tinyxml2.h>

#include <odrparser/odr_1_5.h>

namespace odr_1_5 {

    bool __parse__t_header(const tinyxml2::XMLElement *elem, t_header &obj);

    bool __parse__t_header_GeoReference(const tinyxml2::XMLElement *elem, t_header_GeoReference &obj);

    bool __parse__t_header_Offset(const tinyxml2::XMLElement *elem, t_header_Offset &obj);

    bool __parse__t_road(const tinyxml2::XMLElement *elem, t_road &obj);

    bool __parse__t_road_link(const tinyxml2::XMLElement *elem, t_road_link &obj);

    bool
    __parse__t_road_link_predecessorSuccessor(const tinyxml2::XMLElement *elem, t_road_link_predecessorSuccessor &obj);

    bool __parse__t_road_link_neighbor(const tinyxml2::XMLElement *elem, t_road_link_neighbor &obj);

    bool __parse__t_road_type(const tinyxml2::XMLElement *elem, t_road_type &obj);

    bool __parse__t_road_type_speed(const tinyxml2::XMLElement *elem, t_road_type_speed &obj);

    bool __parse__t_road_planView(const tinyxml2::XMLElement *elem, t_road_planView &obj);

    bool __parse__t_road_planView_geometry(const tinyxml2::XMLElement *elem, t_road_planView_geometry &obj);

    bool __parse__t_road_planView_geometry_line(const tinyxml2::XMLElement *elem, t_road_planView_geometry_line &obj);

    bool
    __parse__t_road_planView_geometry_spiral(const tinyxml2::XMLElement *elem, t_road_planView_geometry_spiral &obj);

    bool __parse__t_road_planView_geometry_arc(const tinyxml2::XMLElement *elem, t_road_planView_geometry_arc &obj);

    bool __parse__t_road_planView_geometry_poly3(const tinyxml2::XMLElement *elem, t_road_planView_geometry_poly3 &obj);

    bool __parse__t_road_planView_geometry_paramPoly3(const tinyxml2::XMLElement *elem,
                                                      t_road_planView_geometry_paramPoly3 &obj);

    bool __parse__t_road_elevationProfile(const tinyxml2::XMLElement *elem, t_road_elevationProfile &obj);

    bool __parse__t_road_elevationProfile_elevation(const tinyxml2::XMLElement *elem,
                                                    t_road_elevationProfile_elevation &obj);

    bool __parse__t_road_lateralProfile(const tinyxml2::XMLElement *elem, t_road_lateralProfile &obj);

    bool __parse__t_road_lateralProfile_superelevation(const tinyxml2::XMLElement *elem,
                                                       t_road_lateralProfile_superelevation &obj);

    bool
    __parse__t_road_lateralProfile_crossfall(const tinyxml2::XMLElement *elem, t_road_lateralProfile_crossfall &obj);

    bool __parse__t_road_lateralProfile_shape(const tinyxml2::XMLElement *elem, t_road_lateralProfile_shape &obj);

    bool __parse__t_road_lanes(const tinyxml2::XMLElement *elem, t_road_lanes &obj);

    bool __parse__t_road_lanes_laneOffset(const tinyxml2::XMLElement *elem, t_road_lanes_laneOffset &obj);

    bool __parse__t_road_lanes_laneSection(const tinyxml2::XMLElement *elem, t_road_lanes_laneSection &obj);

    bool __parse__t_road_lanes_laneSection_left(const tinyxml2::XMLElement *elem, t_road_lanes_laneSection_left &obj);

    bool
    __parse__t_road_lanes_laneSection_center(const tinyxml2::XMLElement *elem, t_road_lanes_laneSection_center &obj);

    bool __parse__t_road_lanes_laneSection_right(const tinyxml2::XMLElement *elem, t_road_lanes_laneSection_right &obj);

    bool __parse__t_road_lanes_laneSection_center_lane(const tinyxml2::XMLElement *elem,
                                                       t_road_lanes_laneSection_center_lane &obj);

    bool
    __parse__t_road_lanes_laneSection_lr_lane(const tinyxml2::XMLElement *elem, t_road_lanes_laneSection_lr_lane &obj);

    bool __parse__t_road_lanes_laneSection_left_lane(const tinyxml2::XMLElement *elem,
                                                     t_road_lanes_laneSection_left_lane &obj);

    bool __parse__t_road_lanes_laneSection_right_lane(const tinyxml2::XMLElement *elem,
                                                      t_road_lanes_laneSection_right_lane &obj);

    bool __parse__t_road_lanes_laneSection_lcr_lane_link(const tinyxml2::XMLElement *elem,
                                                         t_road_lanes_laneSection_lcr_lane_link &obj);

    bool __parse__t_road_lanes_laneSection_lcr_lane_link_predecessorSuccessor(const tinyxml2::XMLElement *elem,
                                                                              t_road_lanes_laneSection_lcr_lane_link_predecessorSuccessor &obj);

    bool __parse__t_road_lanes_laneSection_lr_lane_width(const tinyxml2::XMLElement *elem,
                                                         t_road_lanes_laneSection_lr_lane_width &obj);

    bool __parse__t_road_lanes_laneSection_lr_lane_border(const tinyxml2::XMLElement *elem,
                                                          t_road_lanes_laneSection_lr_lane_border &obj);

    bool __parse__t_road_lanes_laneSection_lcr_lane_roadMark(const tinyxml2::XMLElement *elem,
                                                             t_road_lanes_laneSection_lcr_lane_roadMark &obj);

    bool __parse__t_road_lanes_laneSection_lcr_lane_roadMark_sway(const tinyxml2::XMLElement *elem,
                                                                  t_road_lanes_laneSection_lcr_lane_roadMark_sway &obj);

    bool __parse__t_road_lanes_laneSection_lcr_lane_roadMark_type(const tinyxml2::XMLElement *elem,
                                                                  t_road_lanes_laneSection_lcr_lane_roadMark_type &obj);

    bool __parse__t_road_lanes_laneSection_lcr_lane_roadMark_type_line(const tinyxml2::XMLElement *elem,
                                                                       t_road_lanes_laneSection_lcr_lane_roadMark_type_line &obj);

    bool __parse__t_road_lanes_laneSection_lcr_lane_roadMark_explicit(const tinyxml2::XMLElement *elem,
                                                                      t_road_lanes_laneSection_lcr_lane_roadMark_explicit &obj);

    bool __parse__t_road_lanes_laneSection_lcr_lane_roadMark_explicit_line(const tinyxml2::XMLElement *elem,
                                                                           t_road_lanes_laneSection_lcr_lane_roadMark_explicit_line &obj);

    bool __parse__t_road_lanes_laneSection_lr_lane_material(const tinyxml2::XMLElement *elem,
                                                            t_road_lanes_laneSection_lr_lane_material &obj);

    bool __parse__t_road_lanes_laneSection_lr_lane_visibility(const tinyxml2::XMLElement *elem,
                                                              t_road_lanes_laneSection_lr_lane_visibility &obj);

    bool __parse__t_road_lanes_laneSection_lr_lane_speed(const tinyxml2::XMLElement *elem,
                                                         t_road_lanes_laneSection_lr_lane_speed &obj);

    bool __parse__t_road_lanes_laneSection_lr_lane_access(const tinyxml2::XMLElement *elem,
                                                          t_road_lanes_laneSection_lr_lane_access &obj);

    bool __parse__t_road_lanes_laneSection_lr_lane_height(const tinyxml2::XMLElement *elem,
                                                          t_road_lanes_laneSection_lr_lane_height &obj);

    bool __parse__t_road_lanes_laneSection_lr_lane_rule(const tinyxml2::XMLElement *elem,
                                                        t_road_lanes_laneSection_lr_lane_rule &obj);

    bool __parse__t_road_objects(const tinyxml2::XMLElement *elem, t_road_objects &obj);

    bool __parse__t_road_objects_object(const tinyxml2::XMLElement *elem, t_road_objects_object &obj);

    bool __parse__t_road_objects_object_repeat(const tinyxml2::XMLElement *elem, t_road_objects_object_repeat &obj);

    bool __parse__t_road_objects_object_outlines(const tinyxml2::XMLElement *elem, t_road_objects_object_outlines &obj);

    bool __parse__t_road_objects_object_outlines_outline(const tinyxml2::XMLElement *elem,
                                                         t_road_objects_object_outlines_outline &obj);

    bool __parse__t_road_objects_object_outlines_outline_cornerRoad(const tinyxml2::XMLElement *elem,
                                                                    t_road_objects_object_outlines_outline_cornerRoad &obj);

    bool __parse__t_road_objects_object_outlines_outline_cornerLocal(const tinyxml2::XMLElement *elem,
                                                                     t_road_objects_object_outlines_outline_cornerLocal &obj);

    bool __parse__t_road_objects_object_material(const tinyxml2::XMLElement *elem, t_road_objects_object_material &obj);

    bool __parse__t_road_objects_object_laneValidity(const tinyxml2::XMLElement *elem,
                                                     t_road_objects_object_laneValidity &obj);

    bool __parse__t_road_objects_object_parkingSpace(const tinyxml2::XMLElement *elem,
                                                     t_road_objects_object_parkingSpace &obj);

    bool __parse__t_road_objects_object_markings(const tinyxml2::XMLElement *elem, t_road_objects_object_markings &obj);

    bool __parse__t_road_objects_object_markings_marking(const tinyxml2::XMLElement *elem,
                                                         t_road_objects_object_markings_marking &obj);

    bool __parse__t_road_objects_object_markings_marking_cornerReference(const tinyxml2::XMLElement *elem,
                                                                         t_road_objects_object_markings_marking_cornerReference &obj);

    bool __parse__t_road_objects_object_borders(const tinyxml2::XMLElement *elem, t_road_objects_object_borders &obj);

    bool __parse__t_road_objects_object_borders_border(const tinyxml2::XMLElement *elem,
                                                       t_road_objects_object_borders_border &obj);

    bool __parse__t_road_objects_objectReference(const tinyxml2::XMLElement *elem, t_road_objects_objectReference &obj);

    bool __parse__t_road_objects_tunnel(const tinyxml2::XMLElement *elem, t_road_objects_tunnel &obj);

    bool __parse__t_road_objects_bridge(const tinyxml2::XMLElement *elem, t_road_objects_bridge &obj);

    bool __parse__t_road_signals(const tinyxml2::XMLElement *elem, t_road_signals &obj);

    bool __parse__t_road_signals_signal(const tinyxml2::XMLElement *elem, t_road_signals_signal &obj);

    bool
    __parse__t_road_signals_signal_dependency(const tinyxml2::XMLElement *elem, t_road_signals_signal_dependency &obj);

    bool
    __parse__t_road_signals_signal_reference(const tinyxml2::XMLElement *elem, t_road_signals_signal_reference &obj);

    bool __parse__t_road_signals_signal_positionRoad(const tinyxml2::XMLElement *elem,
                                                     t_road_signals_signal_positionRoad &obj);

    bool __parse__t_road_signals_signal_positionInertial(const tinyxml2::XMLElement *elem,
                                                         t_road_signals_signal_positionInertial &obj);

    bool __parse__t_road_signals_signalReference(const tinyxml2::XMLElement *elem, t_road_signals_signalReference &obj);

    bool __parse__t_road_surface(const tinyxml2::XMLElement *elem, t_road_surface &obj);

    bool __parse__t_road_surface_CRG(const tinyxml2::XMLElement *elem, t_road_surface_CRG &obj);

    bool __parse__t_road_railroad(const tinyxml2::XMLElement *elem, t_road_railroad &obj);

    bool __parse__t_road_railroad_switch(const tinyxml2::XMLElement *elem, t_road_railroad_switch &obj);

    bool
    __parse__t_road_railroad_switch_mainTrack(const tinyxml2::XMLElement *elem, t_road_railroad_switch_mainTrack &obj);

    bool
    __parse__t_road_railroad_switch_sideTrack(const tinyxml2::XMLElement *elem, t_road_railroad_switch_sideTrack &obj);

    bool __parse__t_road_railroad_switch_partner(const tinyxml2::XMLElement *elem, t_road_railroad_switch_partner &obj);

    bool __parse__t_controller(const tinyxml2::XMLElement *elem, t_controller &obj);

    bool __parse__t_controller_control(const tinyxml2::XMLElement *elem, t_controller_control &obj);

    bool __parse__t_junction(const tinyxml2::XMLElement *elem, t_junction &obj);

    bool __parse__t_junction_connection(const tinyxml2::XMLElement *elem, t_junction_connection &obj);

    bool
    __parse__t_junction_predecessorSuccessor(const tinyxml2::XMLElement *elem, t_junction_predecessorSuccessor &obj);

    bool __parse__t_junction_connection_laneLink(const tinyxml2::XMLElement *elem, t_junction_connection_laneLink &obj);

    bool __parse__t_junction_priority(const tinyxml2::XMLElement *elem, t_junction_priority &obj);

    bool __parse__t_junction_controller(const tinyxml2::XMLElement *elem, t_junction_controller &obj);

    bool __parse__t_junction_surface(const tinyxml2::XMLElement *elem, t_junction_surface &obj);

    bool __parse__t_junction_surface_CRG(const tinyxml2::XMLElement *elem, t_junction_surface_CRG &obj);

    bool __parse__t_junctionGroup(const tinyxml2::XMLElement *elem, t_junctionGroup &obj);

    bool __parse__t_junctionGroup_junctionReference(const tinyxml2::XMLElement *elem,
                                                    t_junctionGroup_junctionReference &obj);

    bool __parse__t_station(const tinyxml2::XMLElement *elem, t_station &obj);

    bool __parse__t_station_platform(const tinyxml2::XMLElement *elem, t_station_platform &obj);

    bool __parse__t_station_platform_segment(const tinyxml2::XMLElement *elem, t_station_platform_segment &obj);

    bool __parse__t_userData(const tinyxml2::XMLElement *elem, t_userData &obj);

    bool __parse__t_include(const tinyxml2::XMLElement *elem, t_include &obj);

    bool __parse__t_dataQuality(const tinyxml2::XMLElement *elem, t_dataQuality &obj);

    bool __parse__t_dataQuality_Error(const tinyxml2::XMLElement *elem, t_dataQuality_Error &obj);

    bool __parse__t_dataQuality_RawData(const tinyxml2::XMLElement *elem, t_dataQuality_RawData &obj);

    bool __parse__OpenDRIVE(const tinyxml2::XMLElement *elem, OpenDRIVE &obj);

} // namespace odr_1_5

#endif // XML_PARSER_ODR_1_5_HPP
