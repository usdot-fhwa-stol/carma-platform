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

#ifndef ODR_1_5_STRUCTURE_H
#define ODR_1_5_STRUCTURE_H

#include <string>
#include <memory>
#include <vector>

namespace odr_1_5 {

    namespace xsd {
        template<typename T>
        struct Attribute : public std::shared_ptr<T> {
            Attribute() = default;

            virtual ~Attribute() = default;

            Attribute<T> &operator=(const T &v) {
                this->reset(new T(v));
                return *this;
            }

            Attribute<T> &create() {
                this->reset(new T);
                return *this;
            }
        };

        typedef Attribute<double> d_double;
        typedef Attribute<int> d_int;
        typedef Attribute<unsigned int> d_uint;
        typedef Attribute<std::string> d_string;
        typedef Attribute<float> d_float;
        template<typename T>
        using Vector = std::vector<T>;
    } // namespace xsd

    typedef xsd::d_double t_grEqZero;
    typedef xsd::d_double t_grZero;
    typedef xsd::d_double t_zeroOne;
    typedef xsd::d_string t_bool;
    typedef xsd::d_string t_yesNo;
    typedef xsd::d_float t_header_Version;
    typedef xsd::d_string e_maxSpeedString;
    typedef xsd::d_string t_junction_id;
    typedef xsd::d_string e_trafficRule;
    typedef xsd::d_string e_road_link_elementType;
    typedef xsd::d_string e_road_link_neighbor_side;
    typedef xsd::d_string e_paramPoly3_pRange;
    typedef xsd::d_string e_road_lateralProfile_crossfall_side;
    typedef xsd::d_string e_road_lanes_laneSection_lcr_lane_roadMark_laneChange;
    typedef xsd::d_string e_road_lanes_laneSection_lr_lane_access_rule;
    typedef xsd::d_string e_road_objects_object_parkingSpace_access;
    typedef xsd::d_string e_road_signals_signal_reference_elementType;
    typedef xsd::d_string e_road_surface_CRG_purpose;
    typedef xsd::d_string e_road_surface_CRG_mode;
    typedef xsd::d_string e_road_railroad_switch_position;
    typedef xsd::d_string e_junction_type;
    typedef xsd::d_string e_junctionGroup_type;
    typedef xsd::d_string e_station_type;
    typedef xsd::d_string e_station_platform_segment_side;
    typedef xsd::d_string e_dataQuality_RawData_Source;
    typedef xsd::d_string e_dataQuality_RawData_PostProcessing;
    typedef xsd::d_string e_unitDistance;
    typedef xsd::d_string e_unitSpeed;
    typedef xsd::d_string e_unitMass;
    typedef xsd::d_string e_unitSlope;
    typedef xsd::d_string e_roadType;
    typedef xsd::d_string e_roadMarkType;
    typedef xsd::d_string e_roadMarkWeight;
    typedef xsd::d_string e_roadMarkColor;
    typedef xsd::d_string e_laneType;
    typedef xsd::d_string e_objectType;
    typedef xsd::d_string e_tunnelType;
    typedef xsd::d_string e_bridgeType;
    typedef xsd::d_string e_accessRestrictionType;
    typedef xsd::d_string e_countryCode_deprecated;
    typedef xsd::d_string e_countryCode_iso3166alpha3;
    typedef xsd::d_string e_sideType;
    typedef xsd::d_string e_outlineFillType;
    typedef xsd::d_string e_borderType;
    typedef xsd::d_string e_contactPoint;
    typedef xsd::d_string e_elementDir;
    typedef xsd::d_string e_direction;
    typedef xsd::d_string e_roadMarkRule;
    typedef xsd::d_string e_orientation;
    typedef xsd::d_string t_maxSpeed;
    typedef xsd::d_string e_unit;
    typedef xsd::d_string e_countryCode;
    struct t_header;
    struct t_header_GeoReference;
    struct t_header_Offset;
    struct t_road;
    struct t_road_link;
    struct t_road_link_predecessorSuccessor;
    struct t_road_link_neighbor;
    struct t_road_type;
    struct t_road_type_speed;
    struct t_road_planView;
    struct t_road_planView_geometry;
    struct t_road_planView_geometry_line;
    struct t_road_planView_geometry_spiral;
    struct t_road_planView_geometry_arc;
    struct t_road_planView_geometry_poly3;
    struct t_road_planView_geometry_paramPoly3;
    struct t_road_elevationProfile;
    struct t_road_elevationProfile_elevation;
    struct t_road_lateralProfile;
    struct t_road_lateralProfile_superelevation;
    struct t_road_lateralProfile_crossfall;
    struct t_road_lateralProfile_shape;
    struct t_road_lanes;
    struct t_road_lanes_laneOffset;
    struct t_road_lanes_laneSection;
    struct t_road_lanes_laneSection_left;
    struct t_road_lanes_laneSection_center;
    struct t_road_lanes_laneSection_right;
    struct t_road_lanes_laneSection_center_lane;
    struct t_road_lanes_laneSection_lr_lane;
    struct t_road_lanes_laneSection_left_lane;
    struct t_road_lanes_laneSection_right_lane;
    struct t_road_lanes_laneSection_lcr_lane_link;
    struct t_road_lanes_laneSection_lcr_lane_link_predecessorSuccessor;
    struct t_road_lanes_laneSection_lr_lane_width;
    struct t_road_lanes_laneSection_lr_lane_border;
    struct t_road_lanes_laneSection_lcr_lane_roadMark;
    struct t_road_lanes_laneSection_lcr_lane_roadMark_sway;
    struct t_road_lanes_laneSection_lcr_lane_roadMark_type;
    struct t_road_lanes_laneSection_lcr_lane_roadMark_type_line;
    struct t_road_lanes_laneSection_lcr_lane_roadMark_explicit;
    struct t_road_lanes_laneSection_lcr_lane_roadMark_explicit_line;
    struct t_road_lanes_laneSection_lr_lane_material;
    struct t_road_lanes_laneSection_lr_lane_visibility;
    struct t_road_lanes_laneSection_lr_lane_speed;
    struct t_road_lanes_laneSection_lr_lane_access;
    struct t_road_lanes_laneSection_lr_lane_height;
    struct t_road_lanes_laneSection_lr_lane_rule;
    struct t_road_objects;
    struct t_road_objects_object;
    struct t_road_objects_object_repeat;
    struct t_road_objects_object_outlines;
    struct t_road_objects_object_outlines_outline;
    struct t_road_objects_object_outlines_outline_cornerRoad;
    struct t_road_objects_object_outlines_outline_cornerLocal;
    struct t_road_objects_object_material;
    struct t_road_objects_object_laneValidity;
    struct t_road_objects_object_parkingSpace;
    struct t_road_objects_object_markings;
    struct t_road_objects_object_markings_marking;
    struct t_road_objects_object_markings_marking_cornerReference;
    struct t_road_objects_object_borders;
    struct t_road_objects_object_borders_border;
    struct t_road_objects_objectReference;
    struct t_road_objects_tunnel;
    struct t_road_objects_bridge;
    struct t_road_signals;
    struct t_road_signals_signal;
    struct t_road_signals_signal_dependency;
    struct t_road_signals_signal_reference;
    struct t_road_signals_signal_positionRoad;
    struct t_road_signals_signal_positionInertial;
    struct t_road_signals_signalReference;
    struct t_road_surface;
    struct t_road_surface_CRG;
    struct t_road_railroad;
    struct t_road_railroad_switch;
    struct t_road_railroad_switch_mainTrack;
    struct t_road_railroad_switch_sideTrack;
    struct t_road_railroad_switch_partner;
    struct t_controller;
    struct t_controller_control;
    struct t_junction;
    struct t_junction_connection;
    struct t_junction_predecessorSuccessor;
    struct t_junction_connection_laneLink;
    struct t_junction_priority;
    struct t_junction_controller;
    struct t_junction_surface;
    struct t_junction_surface_CRG;
    struct t_junctionGroup;
    struct t_junctionGroup_junctionReference;
    struct t_station;
    struct t_station_platform;
    struct t_station_platform_segment;
    struct t_userData;
    struct t_include;
    struct t_dataQuality;
    struct t_dataQuality_Error;
    struct t_dataQuality_RawData;
    struct OpenDRIVE;


    struct t_header {
        xsd::Attribute<t_header_GeoReference> sub_geoReference;
        xsd::Attribute<t_header_Offset> sub_offset;
        xsd::d_string _date;
        xsd::d_double _east;
        xsd::d_string _name;
        xsd::d_double _north;
        xsd::d_int _revMajor;
        xsd::d_int _revMinor;
        xsd::d_double _south;
        xsd::d_string _vendor;
        t_header_Version _version;
        xsd::d_double _west;
    };

    struct t_header_GeoReference {
    };

    struct t_header_Offset {
        xsd::d_float _hdg;
        xsd::d_double _x;
        xsd::d_double _y;
        xsd::d_double _z;
    };

    struct t_road {
        xsd::Attribute<t_road_elevationProfile> sub_elevationProfile;
        xsd::Attribute<t_road_lanes> sub_lanes;
        xsd::Attribute<t_road_lateralProfile> sub_lateralProfile;
        xsd::Attribute<t_road_link> sub_link;
        xsd::Attribute<t_road_objects> sub_objects;
        xsd::Attribute<t_road_planView> sub_planView;
        xsd::Attribute<t_road_railroad> sub_railroad;
        xsd::Attribute<t_road_signals> sub_signals;
        xsd::Attribute<t_road_surface> sub_surface;
        xsd::Vector<t_road_type> sub_type{};
        xsd::d_string _id;
        xsd::d_string _junction;
        t_grEqZero _length;
        xsd::d_string _name;
        e_trafficRule _rule;
    };

    struct t_road_link {
        xsd::Attribute<t_road_link_neighbor> sub_neighbor;
        xsd::Attribute<t_road_link_predecessorSuccessor> sub_predecessor;
        xsd::Attribute<t_road_link_predecessorSuccessor> sub_successor;
    };

    struct t_road_link_predecessorSuccessor {
        e_contactPoint _contactPoint;
        e_elementDir _elementDir;
        xsd::d_string _elementId;
        t_grEqZero _elementS;
        e_road_link_elementType _elementType;
    };

    struct t_road_link_neighbor {
        e_direction _direction;
        xsd::d_string _elementId;
        e_road_link_neighbor_side _side;
    };

    struct t_road_type {
        xsd::Attribute<t_road_type_speed> sub_speed;
        e_countryCode _country;
        t_grEqZero _s;
        e_roadType _type;
    };

    struct t_road_type_speed {
        t_maxSpeed _max;
        e_unitSpeed _unit;
    };

    struct t_road_planView {
        xsd::Vector<t_road_planView_geometry> sub_geometry{};
    };

    struct t_road_planView_geometry {
        xsd::Attribute<t_road_planView_geometry_arc> sub_arc;
        xsd::Attribute<t_road_planView_geometry_line> sub_line;
        xsd::Attribute<t_road_planView_geometry_paramPoly3> sub_paramPoly3;
        xsd::Attribute<t_road_planView_geometry_poly3> sub_poly3;
        xsd::Attribute<t_road_planView_geometry_spiral> sub_spiral;
        xsd::d_double _hdg;
        t_grEqZero _length;
        t_grEqZero _s;
        xsd::d_double _x;
        xsd::d_double _y;
    };

    struct t_road_planView_geometry_line {
    };

    struct t_road_planView_geometry_spiral {
        xsd::d_double _curvEnd;
        xsd::d_double _curvStart;
    };

    struct t_road_planView_geometry_arc {
        xsd::d_double _curvature;
    };

    struct t_road_planView_geometry_poly3 {
        xsd::d_double _a;
        xsd::d_double _b;
        xsd::d_double _c;
        xsd::d_double _d;
    };

    struct t_road_planView_geometry_paramPoly3 {
        xsd::d_double _aU;
        xsd::d_double _aV;
        xsd::d_double _bU;
        xsd::d_double _bV;
        xsd::d_double _cU;
        xsd::d_double _cV;
        xsd::d_double _dU;
        xsd::d_double _dV;
        e_paramPoly3_pRange _pRange;
    };

    struct t_road_elevationProfile {
        xsd::Vector<t_road_elevationProfile_elevation> sub_elevation{};
    };

    struct t_road_elevationProfile_elevation {
        xsd::d_double _a;
        xsd::d_double _b;
        xsd::d_double _c;
        xsd::d_double _d;
        t_grEqZero _s;
    };

    struct t_road_lateralProfile {
        xsd::Vector<t_road_lateralProfile_crossfall> sub_crossfall{};
        xsd::Vector<t_road_lateralProfile_shape> sub_shape{};
        xsd::Vector<t_road_lateralProfile_superelevation> sub_superelevation{};
    };

    struct t_road_lateralProfile_superelevation {
        xsd::d_double _a;
        xsd::d_double _b;
        xsd::d_double _c;
        xsd::d_double _d;
        t_grEqZero _s;
    };

    struct t_road_lateralProfile_crossfall {
        xsd::d_double _a;
        xsd::d_double _b;
        xsd::d_double _c;
        xsd::d_double _d;
        t_grEqZero _s;
        e_road_lateralProfile_crossfall_side _side;
    };

    struct t_road_lateralProfile_shape {
        xsd::d_double _a;
        xsd::d_double _b;
        xsd::d_double _c;
        xsd::d_double _d;
        t_grEqZero _s;
        xsd::d_double _t;
    };

    struct t_road_lanes {
        xsd::Vector<t_road_lanes_laneOffset> sub_laneOffset{};
        xsd::Vector<t_road_lanes_laneSection> sub_laneSection{};
    };

    struct t_road_lanes_laneOffset {
        xsd::d_double _a;
        xsd::d_double _b;
        xsd::d_double _c;
        xsd::d_double _d;
        t_grEqZero _s;
    };

    struct t_road_lanes_laneSection {
        xsd::Attribute<t_road_lanes_laneSection_center> sub_center;
        xsd::Attribute<t_road_lanes_laneSection_left> sub_left;
        xsd::Attribute<t_road_lanes_laneSection_right> sub_right;
        t_grEqZero _s;
        t_bool _singleSide;
    };

    struct t_road_lanes_laneSection_left {
        xsd::Vector<t_road_lanes_laneSection_left_lane> sub_lane{};
    };

    struct t_road_lanes_laneSection_center {
        xsd::Vector<t_road_lanes_laneSection_center_lane> sub_lane{};
    };

    struct t_road_lanes_laneSection_right {
        xsd::Vector<t_road_lanes_laneSection_right_lane> sub_lane{};
    };

    struct t_road_lanes_laneSection_center_lane {
        xsd::Attribute<t_road_lanes_laneSection_lcr_lane_link> sub_link;
        xsd::Vector<t_road_lanes_laneSection_lcr_lane_roadMark> sub_roadMark{};
        xsd::d_int _id;
        t_bool _level;
        e_laneType _type;
    };

    struct t_road_lanes_laneSection_lr_lane {
        xsd::Vector<t_road_lanes_laneSection_lr_lane_access> sub_access{};
        xsd::Vector<t_road_lanes_laneSection_lr_lane_border> sub_border{};
        xsd::Vector<t_road_lanes_laneSection_lr_lane_height> sub_height{};
        xsd::Attribute<t_road_lanes_laneSection_lcr_lane_link> sub_link;
        xsd::Vector<t_road_lanes_laneSection_lr_lane_material> sub_material{};
        xsd::Vector<t_road_lanes_laneSection_lcr_lane_roadMark> sub_roadMark{};
        xsd::Vector<t_road_lanes_laneSection_lr_lane_rule> sub_rule{};
        xsd::Vector<t_road_lanes_laneSection_lr_lane_speed> sub_speed{};
        xsd::Vector<t_road_lanes_laneSection_lr_lane_visibility> sub_visibility{};
        xsd::Vector<t_road_lanes_laneSection_lr_lane_width> sub_width{};
        t_bool _level;
        e_laneType _type;
    };

    struct t_road_lanes_laneSection_left_lane : public t_road_lanes_laneSection_lr_lane {
        xsd::d_uint _id;
    };

    struct t_road_lanes_laneSection_right_lane : public t_road_lanes_laneSection_lr_lane {
        xsd::d_int _id;
    };

    struct t_road_lanes_laneSection_lcr_lane_link {
        xsd::Vector<t_road_lanes_laneSection_lcr_lane_link_predecessorSuccessor> sub_predecessor{};
        xsd::Vector<t_road_lanes_laneSection_lcr_lane_link_predecessorSuccessor> sub_successor{};
    };

    struct t_road_lanes_laneSection_lcr_lane_link_predecessorSuccessor {
        xsd::d_int _id;
    };

    struct t_road_lanes_laneSection_lr_lane_width {
        xsd::d_double _a;
        xsd::d_double _b;
        xsd::d_double _c;
        xsd::d_double _d;
        t_grEqZero _sOffset;
    };

    struct t_road_lanes_laneSection_lr_lane_border {
        xsd::d_double _a;
        xsd::d_double _b;
        xsd::d_double _c;
        xsd::d_double _d;
        t_grEqZero _sOffset;
    };

    struct t_road_lanes_laneSection_lcr_lane_roadMark {
        xsd::Attribute<t_road_lanes_laneSection_lcr_lane_roadMark_explicit> sub_explicit;
        xsd::Vector<t_road_lanes_laneSection_lcr_lane_roadMark_sway> sub_sway{};
        xsd::Attribute<t_road_lanes_laneSection_lcr_lane_roadMark_type> sub_type;
        e_roadMarkColor _color;
        xsd::d_double _height;
        e_road_lanes_laneSection_lcr_lane_roadMark_laneChange _laneChange;
        xsd::d_string _material;
        t_grEqZero _sOffset;
        e_roadMarkType _type;
        e_roadMarkWeight _weight;
        t_grEqZero _width;
    };

    struct t_road_lanes_laneSection_lcr_lane_roadMark_sway {
        xsd::d_double _a;
        xsd::d_double _b;
        xsd::d_double _c;
        xsd::d_double _d;
        t_grEqZero _ds;
    };

    struct t_road_lanes_laneSection_lcr_lane_roadMark_type {
        xsd::Vector<t_road_lanes_laneSection_lcr_lane_roadMark_type_line> sub_line{};
        xsd::d_string _name;
        t_grEqZero _width;
    };

    struct t_road_lanes_laneSection_lcr_lane_roadMark_type_line {
        e_roadMarkColor _color;
        t_grEqZero _length;
        e_roadMarkRule _rule;
        t_grEqZero _sOffset;
        t_grEqZero _space;
        xsd::d_double _tOffset;
        t_grEqZero _width;
    };

    struct t_road_lanes_laneSection_lcr_lane_roadMark_explicit {
        xsd::Vector<t_road_lanes_laneSection_lcr_lane_roadMark_explicit_line> sub_line{};
    };

    struct t_road_lanes_laneSection_lcr_lane_roadMark_explicit_line {
        t_grEqZero _length;
        e_roadMarkRule _rule;
        t_grEqZero _sOffset;
        xsd::d_double _tOffset;
        t_grEqZero _width;
    };

    struct t_road_lanes_laneSection_lr_lane_material {
        t_grEqZero _friction;
        t_grEqZero _roughness;
        t_grEqZero _sOffset;
        xsd::d_string _surface;
    };

    struct t_road_lanes_laneSection_lr_lane_visibility {
        t_grEqZero _back;
        t_grEqZero _forward;
        t_grEqZero _left;
        t_grEqZero _right;
        t_grEqZero _sOffset;
    };

    struct t_road_lanes_laneSection_lr_lane_speed {
        t_grEqZero _max;
        t_grEqZero _sOffset;
        e_unitSpeed _unit;
    };

    struct t_road_lanes_laneSection_lr_lane_access {
        e_accessRestrictionType _restriction;
        e_road_lanes_laneSection_lr_lane_access_rule _rule;
        t_grEqZero _sOffset;
    };

    struct t_road_lanes_laneSection_lr_lane_height {
        xsd::d_double _inner;
        xsd::d_double _outer;
        t_grEqZero _sOffset;
    };

    struct t_road_lanes_laneSection_lr_lane_rule {
        t_grEqZero _sOffset;
        xsd::d_string _value;
    };

    struct t_road_objects {
        xsd::Vector<t_road_objects_bridge> sub_bridge{};
        xsd::Vector<t_road_objects_object> sub_object{};
        xsd::Vector<t_road_objects_objectReference> sub_objectReference{};
        xsd::Vector<t_road_objects_tunnel> sub_tunnel{};
    };

    struct t_road_objects_object {
        xsd::Attribute<t_road_objects_object_borders> sub_borders;
        xsd::Attribute<t_road_objects_object_markings> sub_markings;
        xsd::Vector<t_road_objects_object_material> sub_material{};
        xsd::Attribute<t_road_objects_object_outlines_outline> sub_outline;
        xsd::Attribute<t_road_objects_object_outlines> sub_outlines;
        xsd::Attribute<t_road_objects_object_parkingSpace> sub_parkingSpace;
        xsd::Attribute<t_road_objects_object_repeat> sub_repeat;
        xsd::Vector<t_road_objects_object_laneValidity> sub_validity{};
        t_yesNo _dynamic;
        xsd::d_double _hdg;
        xsd::d_double _height;
        xsd::d_string _id;
        xsd::d_double _length;
        xsd::d_string _name;
        e_orientation _orientation;
        xsd::d_double _pitch;
        xsd::d_double _radius;
        xsd::d_double _roll;
        t_grEqZero _s;
        xsd::d_string _subtype;
        xsd::d_double _t;
        e_objectType _type;
        t_grEqZero _validLength;
        xsd::d_double _width;
        xsd::d_double _zOffset;
    };

    struct t_road_objects_object_repeat {
        t_grEqZero _distance;
        xsd::d_double _heightEnd;
        xsd::d_double _heightStart;
        t_grEqZero _length;
        t_grEqZero _lengthEnd;
        t_grEqZero _lengthStart;
        t_grEqZero _radiusEnd;
        t_grEqZero _radiusStart;
        t_grEqZero _s;
        xsd::d_double _tEnd;
        xsd::d_double _tStart;
        t_grEqZero _widthEnd;
        t_grEqZero _widthStart;
        xsd::d_double _zOffsetEnd;
        xsd::d_double _zOffsetStart;
    };

    struct t_road_objects_object_outlines {
        xsd::Vector<t_road_objects_object_outlines_outline> sub_outline{};
    };

    struct t_road_objects_object_outlines_outline {
        xsd::Vector<t_road_objects_object_outlines_outline_cornerLocal> sub_cornerLocal{};
        xsd::Vector<t_road_objects_object_outlines_outline_cornerRoad> sub_cornerRoad{};
        t_bool _closed;
        e_outlineFillType _fillType;
        xsd::d_uint _id;
        e_laneType _laneType;
        t_bool _outer;
    };

    struct t_road_objects_object_outlines_outline_cornerRoad {
        xsd::d_double _dz;
        xsd::d_double _height;
        xsd::d_uint _id;
        t_grEqZero _s;
        xsd::d_double _t;
    };

    struct t_road_objects_object_outlines_outline_cornerLocal {
        xsd::d_double _height;
        xsd::d_uint _id;
        xsd::d_double _u;
        xsd::d_double _v;
        xsd::d_double _z;
    };

    struct t_road_objects_object_material {
        t_grEqZero _friction;
        t_grEqZero _roughness;
        xsd::d_string _surface;
    };

    struct t_road_objects_object_laneValidity {
        xsd::d_int _fromLane;
        xsd::d_int _toLane;
    };

    struct t_road_objects_object_parkingSpace {
        e_road_objects_object_parkingSpace_access _access;
        xsd::d_string _restrictions;
    };

    struct t_road_objects_object_markings {
        xsd::Vector<t_road_objects_object_markings_marking> sub_marking{};
    };

    struct t_road_objects_object_markings_marking {
        xsd::Vector<t_road_objects_object_markings_marking_cornerReference> sub_cornerReference{};
        e_roadMarkColor _color;
        t_grEqZero _lineLength;
        e_sideType _side;
        t_grEqZero _spaceLength;
        xsd::d_double _startOffset;
        xsd::d_double _stopOffset;
        e_roadMarkWeight _weight;
        t_grEqZero _width;
        t_grEqZero _zOffset;
    };

    struct t_road_objects_object_markings_marking_cornerReference {
        xsd::d_uint _id;
    };

    struct t_road_objects_object_borders {
        xsd::Vector<t_road_objects_object_borders_border> sub_border{};
    };

    struct t_road_objects_object_borders_border {
        xsd::Vector<t_road_objects_object_markings_marking_cornerReference> sub_cornerReference{};
        xsd::d_uint _outlineId;
        e_borderType _type;
        t_bool _useCompleteOutline;
        t_grEqZero _width;
    };

    struct t_road_objects_objectReference {
        xsd::Vector<t_road_objects_object_laneValidity> sub_validity{};
        xsd::d_string _id;
        e_orientation _orientation;
        t_grEqZero _s;
        xsd::d_double _t;
        t_grEqZero _validLength;
        xsd::d_double _zOffset;
    };

    struct t_road_objects_tunnel {
        xsd::Vector<t_road_objects_object_laneValidity> sub_validity{};
        t_zeroOne _daylight;
        xsd::d_string _id;
        t_grEqZero _length;
        t_zeroOne _lighting;
        xsd::d_string _name;
        t_grEqZero _s;
        e_tunnelType _type;
    };

    struct t_road_objects_bridge {
        xsd::Vector<t_road_objects_object_laneValidity> sub_validity{};
        xsd::d_string _id;
        t_grEqZero _length;
        xsd::d_string _name;
        t_grEqZero _s;
        e_bridgeType _type;
    };

    struct t_road_signals {
        xsd::Vector<t_road_signals_signal> sub_signal{};
        xsd::Vector<t_road_signals_signalReference> sub_signalReference{};
    };

    struct t_road_signals_signal {
        xsd::Vector<t_road_signals_signal_dependency> sub_dependency{};
        xsd::Vector<t_road_signals_signal_positionInertial> sub_positionInertial{};
        xsd::Vector<t_road_signals_signal_positionRoad> sub_positionRoad{};
        xsd::Vector<t_road_signals_signal_reference> sub_reference{};
        xsd::Vector<t_road_objects_object_laneValidity> sub_validity{};
        e_countryCode _country;
        xsd::d_string _countryRevision;
        t_yesNo _dynamic;
        xsd::d_double _hOffset;
        t_grEqZero _height;
        xsd::d_string _id;
        xsd::d_string _name;
        e_orientation _orientation;
        xsd::d_double _pitch;
        xsd::d_double _roll;
        t_grEqZero _s;
        xsd::d_string _subtype;
        xsd::d_double _t;
        xsd::d_string _text;
        xsd::d_string _type;
        e_unit _unit;
        xsd::d_double _value;
        t_grEqZero _width;
        xsd::d_double _zOffset;
    };

    struct t_road_signals_signal_dependency {
        xsd::d_string _id;
        xsd::d_string _type;
    };

    struct t_road_signals_signal_reference {
        xsd::d_string _elementId;
        e_road_signals_signal_reference_elementType _elementType;
        xsd::d_string _type;
    };

    struct t_road_signals_signal_positionRoad {
        xsd::d_double _hOffset;
        xsd::d_double _pitch;
        xsd::d_string _roadId;
        xsd::d_double _roll;
        t_grEqZero _s;
        xsd::d_double _t;
        xsd::d_double _zOffset;
    };

    struct t_road_signals_signal_positionInertial {
        xsd::d_double _hdg;
        xsd::d_double _pitch;
        xsd::d_double _roll;
        xsd::d_double _x;
        xsd::d_double _y;
        xsd::d_double _z;
    };

    struct t_road_signals_signalReference {
        xsd::Vector<t_road_objects_object_laneValidity> sub_validity{};
        xsd::d_string _id;
        e_orientation _orientation;
        t_grEqZero _s;
        xsd::d_double _t;
    };

    struct t_road_surface {
        xsd::Vector<t_road_surface_CRG> sub_CRG{};
    };

    struct t_road_surface_CRG {
        xsd::d_string _file;
        xsd::d_double _hOffset;
        e_road_surface_CRG_mode _mode;
        e_direction _orientation;
        e_road_surface_CRG_purpose _purpose;
        t_grEqZero _sEnd;
        xsd::d_double _sOffset;
        t_grEqZero _sStart;
        xsd::d_double _tOffset;
        xsd::d_double _zOffset;
        xsd::d_double _zScale;
    };

    struct t_road_railroad {
        xsd::Vector<t_road_railroad_switch> sub_switch{};
    };

    struct t_road_railroad_switch {
        xsd::Attribute<t_road_railroad_switch_mainTrack> sub_mainTrack;
        xsd::Attribute<t_road_railroad_switch_partner> sub_partner;
        xsd::Attribute<t_road_railroad_switch_sideTrack> sub_sideTrack;
        xsd::d_string _id;
        xsd::d_string _name;
        e_road_railroad_switch_position _position;
    };

    struct t_road_railroad_switch_mainTrack {
        e_elementDir _dir;
        xsd::d_string _id;
        t_grEqZero _s;
    };

    struct t_road_railroad_switch_sideTrack {
        e_elementDir _dir;
        xsd::d_string _id;
        t_grEqZero _s;
    };

    struct t_road_railroad_switch_partner {
        xsd::d_string _id;
        xsd::d_string _name;
    };

    struct t_controller {
        xsd::Vector<t_controller_control> sub_control{};
        xsd::d_string _id;
        xsd::d_string _name;
        xsd::d_uint _sequence;
    };

    struct t_controller_control {
        xsd::d_string _signalId;
        xsd::d_string _type;
    };

    struct t_junction {
        xsd::Vector<t_junction_connection> sub_connection{};
        xsd::Vector<t_junction_controller> sub_controller{};
        xsd::Vector<t_junction_priority> sub_priority{};
        xsd::Attribute<t_junction_surface> sub_surface;
        t_junction_id _id;
        xsd::d_string _name;
        e_junction_type _type;
    };

    struct t_junction_connection {
        xsd::Vector<t_junction_connection_laneLink> sub_laneLink{};
        xsd::Attribute<t_junction_predecessorSuccessor> sub_predecessor;
        xsd::Attribute<t_junction_predecessorSuccessor> sub_successor;
        xsd::d_string _connectingRoad;
        xsd::d_string _connectionMaster;
        e_contactPoint _contactPoint;
        xsd::d_string _id;
        xsd::d_string _incomingRoad;
        e_junction_type _type;
    };

    struct t_junction_predecessorSuccessor {
        e_elementDir _elementDir;
        xsd::d_string _elementId;
        t_grZero _elementS;
        xsd::d_string _elementType;
    };

    struct t_junction_connection_laneLink {
        xsd::d_int _from;
        xsd::d_int _to;
    };

    struct t_junction_priority {
        xsd::d_string _high;
        xsd::d_string _low;
    };

    struct t_junction_controller {
        xsd::d_string _id;
        xsd::d_uint _sequence;
        xsd::d_string _type;
    };

    struct t_junction_surface {
        xsd::Vector<t_junction_surface_CRG> sub_CRG{};
    };

    struct t_junction_surface_CRG {
        xsd::d_string _file;
        e_road_surface_CRG_mode _mode;
        e_road_surface_CRG_purpose _purpose;
        xsd::d_double _zOffset;
        xsd::d_double _zScale;
    };

    struct t_junctionGroup {
        xsd::Vector<t_junctionGroup_junctionReference> sub_junctionReference{};
        xsd::d_string _id;
        xsd::d_string _name;
        e_junctionGroup_type _type;
    };

    struct t_junctionGroup_junctionReference {
        xsd::d_string _junction;
    };

    struct t_station {
        xsd::Vector<t_station_platform> sub_platform{};
        xsd::d_string _id;
        xsd::d_string _name;
        e_station_type _type;
    };

    struct t_station_platform {
        xsd::Vector<t_station_platform_segment> sub_segment{};
        xsd::d_string _id;
        xsd::d_string _name;
    };

    struct t_station_platform_segment {
        xsd::d_string _roadId;
        t_grEqZero _sEnd;
        t_grEqZero _sStart;
        e_station_platform_segment_side _side;
    };

    struct t_userData {
        xsd::d_string _code;
        xsd::d_string _value;
    };

    struct t_include {
        xsd::d_string _file;
    };

    struct t_dataQuality {
        xsd::Attribute<t_dataQuality_Error> sub_error;
        xsd::Attribute<t_dataQuality_RawData> sub_rawData;
    };

    struct t_dataQuality_Error {
        xsd::d_double _xyAbsolute;
        xsd::d_double _xyRelative;
        xsd::d_double _zAbsolute;
        xsd::d_double _zRelative;
    };

    struct t_dataQuality_RawData {
        xsd::d_string _date;
        e_dataQuality_RawData_PostProcessing _postProcessing;
        xsd::d_string _postProcessingComment;
        e_dataQuality_RawData_Source _source;
        xsd::d_string _sourceComment;
    };

    struct OpenDRIVE {
        xsd::Vector<t_controller> sub_controller{};
        xsd::Attribute<t_header> sub_header;
        xsd::Vector<t_junction> sub_junction{};
        xsd::Vector<t_junctionGroup> sub_junctionGroup{};
        xsd::Vector<t_road> sub_road{};
        xsd::Vector<t_station> sub_station{};
    };


} // namespace odr_1_5

#endif // ODR_1_5_STRUCTURE_H
