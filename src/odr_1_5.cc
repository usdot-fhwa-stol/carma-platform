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

#include "odr_1_5.hpp"

namespace odr_1_5 {

    bool __parse__t_header(const tinyxml2::XMLElement *elem, t_header &obj) {

        if (elem->Attribute("date") != nullptr)
            obj._date = std::string(elem->Attribute("date"));

        if (elem->Attribute("east") != nullptr)
            obj._east = elem->DoubleAttribute("east", 0.0);

        if (elem->Attribute("name") != nullptr)
            obj._name = std::string(elem->Attribute("name"));

        if (elem->Attribute("north") != nullptr)
            obj._north = elem->DoubleAttribute("north", 0.0);

        if (elem->Attribute("revMajor") != nullptr)
            obj._revMajor = elem->IntAttribute("revMajor", 0);

        if (elem->Attribute("revMinor") != nullptr)
            obj._revMinor = elem->IntAttribute("revMinor", 0);

        if (elem->Attribute("south") != nullptr)
            obj._south = elem->DoubleAttribute("south", 0.0);

        if (elem->Attribute("vendor") != nullptr)
            obj._vendor = std::string(elem->Attribute("vendor"));

        if (elem->Attribute("version") != nullptr)
            obj._version = elem->FloatAttribute("version", 0.0f);

        if (elem->Attribute("west") != nullptr)
            obj._west = elem->DoubleAttribute("west", 0.0);

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("geoReference");
        if (e != nullptr)
            __parse__t_header_GeoReference(e, *(obj.sub_geoReference.create()));

        e = elem->FirstChildElement("offset");
        if (e != nullptr)
            __parse__t_header_Offset(e, *(obj.sub_offset.create()));


        return true;

    }

    bool __parse__t_header_GeoReference(const tinyxml2::XMLElement *elem, t_header_GeoReference &obj) {


        return true;

    }

    bool __parse__t_header_Offset(const tinyxml2::XMLElement *elem, t_header_Offset &obj) {

        if (elem->Attribute("hdg") != nullptr)
            obj._hdg = elem->FloatAttribute("hdg", 0.0f);

        if (elem->Attribute("x") != nullptr)
            obj._x = elem->DoubleAttribute("x", 0.0);

        if (elem->Attribute("y") != nullptr)
            obj._y = elem->DoubleAttribute("y", 0.0);

        if (elem->Attribute("z") != nullptr)
            obj._z = elem->DoubleAttribute("z", 0.0);


        return true;

    }

    bool __parse__t_road(const tinyxml2::XMLElement *elem, t_road &obj) {

        if (elem->Attribute("id") != nullptr)
            obj._id = std::string(elem->Attribute("id"));

        if (elem->Attribute("junction") != nullptr)
            obj._junction = std::string(elem->Attribute("junction"));

        if (elem->Attribute("length") != nullptr)
            obj._length = elem->DoubleAttribute("length", 0.0);

        if (elem->Attribute("name") != nullptr)
            obj._name = std::string(elem->Attribute("name"));

        if (elem->Attribute("rule") != nullptr)
            obj._rule = std::string(elem->Attribute("rule"));

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("elevationProfile");
        if (e != nullptr)
            __parse__t_road_elevationProfile(e, *(obj.sub_elevationProfile.create()));

        e = elem->FirstChildElement("lanes");
        if (e != nullptr)
            __parse__t_road_lanes(e, *(obj.sub_lanes.create()));

        e = elem->FirstChildElement("lateralProfile");
        if (e != nullptr)
            __parse__t_road_lateralProfile(e, *(obj.sub_lateralProfile.create()));

        e = elem->FirstChildElement("link");
        if (e != nullptr)
            __parse__t_road_link(e, *(obj.sub_link.create()));

        e = elem->FirstChildElement("objects");
        if (e != nullptr)
            __parse__t_road_objects(e, *(obj.sub_objects.create()));

        e = elem->FirstChildElement("planView");
        if (e != nullptr)
            __parse__t_road_planView(e, *(obj.sub_planView.create()));

        e = elem->FirstChildElement("railroad");
        if (e != nullptr)
            __parse__t_road_railroad(e, *(obj.sub_railroad.create()));

        e = elem->FirstChildElement("signals");
        if (e != nullptr)
            __parse__t_road_signals(e, *(obj.sub_signals.create()));

        e = elem->FirstChildElement("surface");
        if (e != nullptr)
            __parse__t_road_surface(e, *(obj.sub_surface.create()));

        e = elem->FirstChildElement("type");
        while (e != nullptr) {

            obj.sub_type.emplace_back();
            __parse__t_road_type(e, obj.sub_type.back());

            e = e->NextSiblingElement("type");
        }


        return true;

    }

    bool __parse__t_road_link(const tinyxml2::XMLElement *elem, t_road_link &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("neighbor");
        if (e != nullptr)
            __parse__t_road_link_neighbor(e, *(obj.sub_neighbor.create()));

        e = elem->FirstChildElement("predecessor");
        if (e != nullptr)
            __parse__t_road_link_predecessorSuccessor(e, *(obj.sub_predecessor.create()));

        e = elem->FirstChildElement("successor");
        if (e != nullptr)
            __parse__t_road_link_predecessorSuccessor(e, *(obj.sub_successor.create()));


        return true;

    }

    bool
    __parse__t_road_link_predecessorSuccessor(const tinyxml2::XMLElement *elem, t_road_link_predecessorSuccessor &obj) {

        if (elem->Attribute("contactPoint") != nullptr)
            obj._contactPoint = std::string(elem->Attribute("contactPoint"));

        if (elem->Attribute("elementDir") != nullptr)
            obj._elementDir = std::string(elem->Attribute("elementDir"));

        if (elem->Attribute("elementId") != nullptr)
            obj._elementId = std::string(elem->Attribute("elementId"));

        if (elem->Attribute("elementS") != nullptr)
            obj._elementS = elem->DoubleAttribute("elementS", 0.0);

        if (elem->Attribute("elementType") != nullptr)
            obj._elementType = std::string(elem->Attribute("elementType"));


        return true;

    }

    bool __parse__t_road_link_neighbor(const tinyxml2::XMLElement *elem, t_road_link_neighbor &obj) {

        if (elem->Attribute("direction") != nullptr)
            obj._direction = std::string(elem->Attribute("direction"));

        if (elem->Attribute("elementId") != nullptr)
            obj._elementId = std::string(elem->Attribute("elementId"));

        if (elem->Attribute("side") != nullptr)
            obj._side = std::string(elem->Attribute("side"));


        return true;

    }

    bool __parse__t_road_type(const tinyxml2::XMLElement *elem, t_road_type &obj) {

        if (elem->Attribute("country") != nullptr)
            obj._country = std::string(elem->Attribute("country"));

        if (elem->Attribute("s") != nullptr)
            obj._s = elem->DoubleAttribute("s", 0.0);

        if (elem->Attribute("type") != nullptr)
            obj._type = std::string(elem->Attribute("type"));

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("speed");
        if (e != nullptr)
            __parse__t_road_type_speed(e, *(obj.sub_speed.create()));


        return true;

    }

    bool __parse__t_road_type_speed(const tinyxml2::XMLElement *elem, t_road_type_speed &obj) {

        if (elem->Attribute("max") != nullptr)
            obj._max = std::string(elem->Attribute("max"));

        if (elem->Attribute("unit") != nullptr)
            obj._unit = std::string(elem->Attribute("unit"));


        return true;

    }

    bool __parse__t_road_planView(const tinyxml2::XMLElement *elem, t_road_planView &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("geometry");
        while (e != nullptr) {

            obj.sub_geometry.emplace_back();
            __parse__t_road_planView_geometry(e, obj.sub_geometry.back());

            e = e->NextSiblingElement("geometry");
        }


        return true;

    }

    bool __parse__t_road_planView_geometry(const tinyxml2::XMLElement *elem, t_road_planView_geometry &obj) {

        if (elem->Attribute("hdg") != nullptr)
            obj._hdg = elem->DoubleAttribute("hdg", 0.0);

        if (elem->Attribute("length") != nullptr)
            obj._length = elem->DoubleAttribute("length", 0.0);

        if (elem->Attribute("s") != nullptr)
            obj._s = elem->DoubleAttribute("s", 0.0);

        if (elem->Attribute("x") != nullptr)
            obj._x = elem->DoubleAttribute("x", 0.0);

        if (elem->Attribute("y") != nullptr)
            obj._y = elem->DoubleAttribute("y", 0.0);

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("arc");
        if (e != nullptr)
            __parse__t_road_planView_geometry_arc(e, *(obj.sub_arc.create()));

        e = elem->FirstChildElement("line");
        if (e != nullptr)
            __parse__t_road_planView_geometry_line(e, *(obj.sub_line.create()));

        e = elem->FirstChildElement("paramPoly3");
        if (e != nullptr)
            __parse__t_road_planView_geometry_paramPoly3(e, *(obj.sub_paramPoly3.create()));

        e = elem->FirstChildElement("poly3");
        if (e != nullptr)
            __parse__t_road_planView_geometry_poly3(e, *(obj.sub_poly3.create()));

        e = elem->FirstChildElement("spiral");
        if (e != nullptr)
            __parse__t_road_planView_geometry_spiral(e, *(obj.sub_spiral.create()));


        return true;

    }

    bool __parse__t_road_planView_geometry_line(const tinyxml2::XMLElement *elem, t_road_planView_geometry_line &obj) {


        return true;

    }

    bool
    __parse__t_road_planView_geometry_spiral(const tinyxml2::XMLElement *elem, t_road_planView_geometry_spiral &obj) {

        if (elem->Attribute("curvEnd") != nullptr)
            obj._curvEnd = elem->DoubleAttribute("curvEnd", 0.0);

        if (elem->Attribute("curvStart") != nullptr)
            obj._curvStart = elem->DoubleAttribute("curvStart", 0.0);


        return true;

    }

    bool __parse__t_road_planView_geometry_arc(const tinyxml2::XMLElement *elem, t_road_planView_geometry_arc &obj) {

        if (elem->Attribute("curvature") != nullptr)
            obj._curvature = elem->DoubleAttribute("curvature", 0.0);


        return true;

    }

    bool
    __parse__t_road_planView_geometry_poly3(const tinyxml2::XMLElement *elem, t_road_planView_geometry_poly3 &obj) {

        if (elem->Attribute("a") != nullptr)
            obj._a = elem->DoubleAttribute("a", 0.0);

        if (elem->Attribute("b") != nullptr)
            obj._b = elem->DoubleAttribute("b", 0.0);

        if (elem->Attribute("c") != nullptr)
            obj._c = elem->DoubleAttribute("c", 0.0);

        if (elem->Attribute("d") != nullptr)
            obj._d = elem->DoubleAttribute("d", 0.0);


        return true;

    }

    bool __parse__t_road_planView_geometry_paramPoly3(const tinyxml2::XMLElement *elem,
                                                      t_road_planView_geometry_paramPoly3 &obj) {

        if (elem->Attribute("aU") != nullptr)
            obj._aU = elem->DoubleAttribute("aU", 0.0);

        if (elem->Attribute("aV") != nullptr)
            obj._aV = elem->DoubleAttribute("aV", 0.0);

        if (elem->Attribute("bU") != nullptr)
            obj._bU = elem->DoubleAttribute("bU", 0.0);

        if (elem->Attribute("bV") != nullptr)
            obj._bV = elem->DoubleAttribute("bV", 0.0);

        if (elem->Attribute("cU") != nullptr)
            obj._cU = elem->DoubleAttribute("cU", 0.0);

        if (elem->Attribute("cV") != nullptr)
            obj._cV = elem->DoubleAttribute("cV", 0.0);

        if (elem->Attribute("dU") != nullptr)
            obj._dU = elem->DoubleAttribute("dU", 0.0);

        if (elem->Attribute("dV") != nullptr)
            obj._dV = elem->DoubleAttribute("dV", 0.0);

        if (elem->Attribute("pRange") != nullptr)
            obj._pRange = std::string(elem->Attribute("pRange"));


        return true;

    }

    bool __parse__t_road_elevationProfile(const tinyxml2::XMLElement *elem, t_road_elevationProfile &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("elevation");
        while (e != nullptr) {

            obj.sub_elevation.emplace_back();
            __parse__t_road_elevationProfile_elevation(e, obj.sub_elevation.back());

            e = e->NextSiblingElement("elevation");
        }


        return true;

    }

    bool __parse__t_road_elevationProfile_elevation(const tinyxml2::XMLElement *elem,
                                                    t_road_elevationProfile_elevation &obj) {

        if (elem->Attribute("a") != nullptr)
            obj._a = elem->DoubleAttribute("a", 0.0);

        if (elem->Attribute("b") != nullptr)
            obj._b = elem->DoubleAttribute("b", 0.0);

        if (elem->Attribute("c") != nullptr)
            obj._c = elem->DoubleAttribute("c", 0.0);

        if (elem->Attribute("d") != nullptr)
            obj._d = elem->DoubleAttribute("d", 0.0);

        if (elem->Attribute("s") != nullptr)
            obj._s = elem->DoubleAttribute("s", 0.0);


        return true;

    }

    bool __parse__t_road_lateralProfile(const tinyxml2::XMLElement *elem, t_road_lateralProfile &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("crossfall");
        while (e != nullptr) {

            obj.sub_crossfall.emplace_back();
            __parse__t_road_lateralProfile_crossfall(e, obj.sub_crossfall.back());

            e = e->NextSiblingElement("crossfall");
        }

        e = elem->FirstChildElement("shape");
        while (e != nullptr) {

            obj.sub_shape.emplace_back();
            __parse__t_road_lateralProfile_shape(e, obj.sub_shape.back());

            e = e->NextSiblingElement("shape");
        }

        e = elem->FirstChildElement("superelevation");
        while (e != nullptr) {

            obj.sub_superelevation.emplace_back();
            __parse__t_road_lateralProfile_superelevation(e, obj.sub_superelevation.back());

            e = e->NextSiblingElement("superelevation");
        }


        return true;

    }

    bool __parse__t_road_lateralProfile_superelevation(const tinyxml2::XMLElement *elem,
                                                       t_road_lateralProfile_superelevation &obj) {

        if (elem->Attribute("a") != nullptr)
            obj._a = elem->DoubleAttribute("a", 0.0);

        if (elem->Attribute("b") != nullptr)
            obj._b = elem->DoubleAttribute("b", 0.0);

        if (elem->Attribute("c") != nullptr)
            obj._c = elem->DoubleAttribute("c", 0.0);

        if (elem->Attribute("d") != nullptr)
            obj._d = elem->DoubleAttribute("d", 0.0);

        if (elem->Attribute("s") != nullptr)
            obj._s = elem->DoubleAttribute("s", 0.0);


        return true;

    }

    bool
    __parse__t_road_lateralProfile_crossfall(const tinyxml2::XMLElement *elem, t_road_lateralProfile_crossfall &obj) {

        if (elem->Attribute("a") != nullptr)
            obj._a = elem->DoubleAttribute("a", 0.0);

        if (elem->Attribute("b") != nullptr)
            obj._b = elem->DoubleAttribute("b", 0.0);

        if (elem->Attribute("c") != nullptr)
            obj._c = elem->DoubleAttribute("c", 0.0);

        if (elem->Attribute("d") != nullptr)
            obj._d = elem->DoubleAttribute("d", 0.0);

        if (elem->Attribute("s") != nullptr)
            obj._s = elem->DoubleAttribute("s", 0.0);

        if (elem->Attribute("side") != nullptr)
            obj._side = std::string(elem->Attribute("side"));


        return true;

    }

    bool __parse__t_road_lateralProfile_shape(const tinyxml2::XMLElement *elem, t_road_lateralProfile_shape &obj) {

        if (elem->Attribute("a") != nullptr)
            obj._a = elem->DoubleAttribute("a", 0.0);

        if (elem->Attribute("b") != nullptr)
            obj._b = elem->DoubleAttribute("b", 0.0);

        if (elem->Attribute("c") != nullptr)
            obj._c = elem->DoubleAttribute("c", 0.0);

        if (elem->Attribute("d") != nullptr)
            obj._d = elem->DoubleAttribute("d", 0.0);

        if (elem->Attribute("s") != nullptr)
            obj._s = elem->DoubleAttribute("s", 0.0);

        if (elem->Attribute("t") != nullptr)
            obj._t = elem->DoubleAttribute("t", 0.0);


        return true;

    }

    bool __parse__t_road_lanes(const tinyxml2::XMLElement *elem, t_road_lanes &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("laneOffset");
        while (e != nullptr) {

            obj.sub_laneOffset.emplace_back();
            __parse__t_road_lanes_laneOffset(e, obj.sub_laneOffset.back());

            e = e->NextSiblingElement("laneOffset");
        }

        e = elem->FirstChildElement("laneSection");
        while (e != nullptr) {

            obj.sub_laneSection.emplace_back();
            __parse__t_road_lanes_laneSection(e, obj.sub_laneSection.back());

            e = e->NextSiblingElement("laneSection");
        }


        return true;

    }

    bool __parse__t_road_lanes_laneOffset(const tinyxml2::XMLElement *elem, t_road_lanes_laneOffset &obj) {

        if (elem->Attribute("a") != nullptr)
            obj._a = elem->DoubleAttribute("a", 0.0);

        if (elem->Attribute("b") != nullptr)
            obj._b = elem->DoubleAttribute("b", 0.0);

        if (elem->Attribute("c") != nullptr)
            obj._c = elem->DoubleAttribute("c", 0.0);

        if (elem->Attribute("d") != nullptr)
            obj._d = elem->DoubleAttribute("d", 0.0);

        if (elem->Attribute("s") != nullptr)
            obj._s = elem->DoubleAttribute("s", 0.0);


        return true;

    }

    bool __parse__t_road_lanes_laneSection(const tinyxml2::XMLElement *elem, t_road_lanes_laneSection &obj) {

        if (elem->Attribute("s") != nullptr)
            obj._s = elem->DoubleAttribute("s", 0.0);

        if (elem->Attribute("singleSide") != nullptr)
            obj._singleSide = std::string(elem->Attribute("singleSide"));

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("center");
        if (e != nullptr)
            __parse__t_road_lanes_laneSection_center(e, *(obj.sub_center.create()));

        e = elem->FirstChildElement("left");
        if (e != nullptr)
            __parse__t_road_lanes_laneSection_left(e, *(obj.sub_left.create()));

        e = elem->FirstChildElement("right");
        if (e != nullptr)
            __parse__t_road_lanes_laneSection_right(e, *(obj.sub_right.create()));


        return true;

    }

    bool __parse__t_road_lanes_laneSection_left(const tinyxml2::XMLElement *elem, t_road_lanes_laneSection_left &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("lane");
        while (e != nullptr) {

            obj.sub_lane.emplace_back();
            __parse__t_road_lanes_laneSection_left_lane(e, obj.sub_lane.back());

            e = e->NextSiblingElement("lane");
        }


        return true;

    }

    bool
    __parse__t_road_lanes_laneSection_center(const tinyxml2::XMLElement *elem, t_road_lanes_laneSection_center &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("lane");
        while (e != nullptr) {

            obj.sub_lane.emplace_back();
            __parse__t_road_lanes_laneSection_center_lane(e, obj.sub_lane.back());

            e = e->NextSiblingElement("lane");
        }


        return true;

    }

    bool
    __parse__t_road_lanes_laneSection_right(const tinyxml2::XMLElement *elem, t_road_lanes_laneSection_right &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("lane");
        while (e != nullptr) {

            obj.sub_lane.emplace_back();
            __parse__t_road_lanes_laneSection_right_lane(e, obj.sub_lane.back());

            e = e->NextSiblingElement("lane");
        }


        return true;

    }

    bool __parse__t_road_lanes_laneSection_center_lane(const tinyxml2::XMLElement *elem,
                                                       t_road_lanes_laneSection_center_lane &obj) {

        if (elem->Attribute("id") != nullptr)
            obj._id = elem->IntAttribute("id", 0);

        if (elem->Attribute("level") != nullptr)
            obj._level = std::string(elem->Attribute("level"));

        if (elem->Attribute("type") != nullptr)
            obj._type = std::string(elem->Attribute("type"));

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("link");
        if (e != nullptr)
            __parse__t_road_lanes_laneSection_lcr_lane_link(e, *(obj.sub_link.create()));

        e = elem->FirstChildElement("roadMark");
        while (e != nullptr) {

            obj.sub_roadMark.emplace_back();
            __parse__t_road_lanes_laneSection_lcr_lane_roadMark(e, obj.sub_roadMark.back());

            e = e->NextSiblingElement("roadMark");
        }


        return true;

    }

    bool
    __parse__t_road_lanes_laneSection_lr_lane(const tinyxml2::XMLElement *elem, t_road_lanes_laneSection_lr_lane &obj) {

        if (elem->Attribute("level") != nullptr)
            obj._level = std::string(elem->Attribute("level"));

        if (elem->Attribute("type") != nullptr)
            obj._type = std::string(elem->Attribute("type"));

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("access");
        while (e != nullptr) {

            obj.sub_access.emplace_back();
            __parse__t_road_lanes_laneSection_lr_lane_access(e, obj.sub_access.back());

            e = e->NextSiblingElement("access");
        }

        e = elem->FirstChildElement("border");
        while (e != nullptr) {

            obj.sub_border.emplace_back();
            __parse__t_road_lanes_laneSection_lr_lane_border(e, obj.sub_border.back());

            e = e->NextSiblingElement("border");
        }

        e = elem->FirstChildElement("height");
        while (e != nullptr) {

            obj.sub_height.emplace_back();
            __parse__t_road_lanes_laneSection_lr_lane_height(e, obj.sub_height.back());

            e = e->NextSiblingElement("height");
        }

        e = elem->FirstChildElement("link");
        if (e != nullptr)
            __parse__t_road_lanes_laneSection_lcr_lane_link(e, *(obj.sub_link.create()));

        e = elem->FirstChildElement("material");
        while (e != nullptr) {

            obj.sub_material.emplace_back();
            __parse__t_road_lanes_laneSection_lr_lane_material(e, obj.sub_material.back());

            e = e->NextSiblingElement("material");
        }

        e = elem->FirstChildElement("roadMark");
        while (e != nullptr) {

            obj.sub_roadMark.emplace_back();
            __parse__t_road_lanes_laneSection_lcr_lane_roadMark(e, obj.sub_roadMark.back());

            e = e->NextSiblingElement("roadMark");
        }

        e = elem->FirstChildElement("rule");
        while (e != nullptr) {

            obj.sub_rule.emplace_back();
            __parse__t_road_lanes_laneSection_lr_lane_rule(e, obj.sub_rule.back());

            e = e->NextSiblingElement("rule");
        }

        e = elem->FirstChildElement("speed");
        while (e != nullptr) {

            obj.sub_speed.emplace_back();
            __parse__t_road_lanes_laneSection_lr_lane_speed(e, obj.sub_speed.back());

            e = e->NextSiblingElement("speed");
        }

        e = elem->FirstChildElement("visibility");
        while (e != nullptr) {

            obj.sub_visibility.emplace_back();
            __parse__t_road_lanes_laneSection_lr_lane_visibility(e, obj.sub_visibility.back());

            e = e->NextSiblingElement("visibility");
        }

        e = elem->FirstChildElement("width");
        while (e != nullptr) {

            obj.sub_width.emplace_back();
            __parse__t_road_lanes_laneSection_lr_lane_width(e, obj.sub_width.back());

            e = e->NextSiblingElement("width");
        }


        return true;

    }

    bool __parse__t_road_lanes_laneSection_left_lane(const tinyxml2::XMLElement *elem,
                                                     t_road_lanes_laneSection_left_lane &obj) {

        __parse__t_road_lanes_laneSection_lr_lane(elem, obj);

        if (elem->Attribute("id") != nullptr)
            obj._id = elem->UnsignedAttribute("id", 0);


        return true;

    }

    bool __parse__t_road_lanes_laneSection_right_lane(const tinyxml2::XMLElement *elem,
                                                      t_road_lanes_laneSection_right_lane &obj) {

        __parse__t_road_lanes_laneSection_lr_lane(elem, obj);

        if (elem->Attribute("id") != nullptr)
            obj._id = elem->IntAttribute("id", 0);


        return true;

    }

    bool __parse__t_road_lanes_laneSection_lcr_lane_link(const tinyxml2::XMLElement *elem,
                                                         t_road_lanes_laneSection_lcr_lane_link &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("predecessor");
        while (e != nullptr) {

            obj.sub_predecessor.emplace_back();
            __parse__t_road_lanes_laneSection_lcr_lane_link_predecessorSuccessor(e, obj.sub_predecessor.back());

            e = e->NextSiblingElement("predecessor");
        }

        e = elem->FirstChildElement("successor");
        while (e != nullptr) {

            obj.sub_successor.emplace_back();
            __parse__t_road_lanes_laneSection_lcr_lane_link_predecessorSuccessor(e, obj.sub_successor.back());

            e = e->NextSiblingElement("successor");
        }


        return true;

    }

    bool __parse__t_road_lanes_laneSection_lcr_lane_link_predecessorSuccessor(const tinyxml2::XMLElement *elem,
                                                                              t_road_lanes_laneSection_lcr_lane_link_predecessorSuccessor &obj) {

        if (elem->Attribute("id") != nullptr)
            obj._id = elem->IntAttribute("id", 0);


        return true;

    }

    bool __parse__t_road_lanes_laneSection_lr_lane_width(const tinyxml2::XMLElement *elem,
                                                         t_road_lanes_laneSection_lr_lane_width &obj) {

        if (elem->Attribute("a") != nullptr)
            obj._a = elem->DoubleAttribute("a", 0.0);

        if (elem->Attribute("b") != nullptr)
            obj._b = elem->DoubleAttribute("b", 0.0);

        if (elem->Attribute("c") != nullptr)
            obj._c = elem->DoubleAttribute("c", 0.0);

        if (elem->Attribute("d") != nullptr)
            obj._d = elem->DoubleAttribute("d", 0.0);

        if (elem->Attribute("sOffset") != nullptr)
            obj._sOffset = elem->DoubleAttribute("sOffset", 0.0);


        return true;

    }

    bool __parse__t_road_lanes_laneSection_lr_lane_border(const tinyxml2::XMLElement *elem,
                                                          t_road_lanes_laneSection_lr_lane_border &obj) {

        if (elem->Attribute("a") != nullptr)
            obj._a = elem->DoubleAttribute("a", 0.0);

        if (elem->Attribute("b") != nullptr)
            obj._b = elem->DoubleAttribute("b", 0.0);

        if (elem->Attribute("c") != nullptr)
            obj._c = elem->DoubleAttribute("c", 0.0);

        if (elem->Attribute("d") != nullptr)
            obj._d = elem->DoubleAttribute("d", 0.0);

        if (elem->Attribute("sOffset") != nullptr)
            obj._sOffset = elem->DoubleAttribute("sOffset", 0.0);


        return true;

    }

    bool __parse__t_road_lanes_laneSection_lcr_lane_roadMark(const tinyxml2::XMLElement *elem,
                                                             t_road_lanes_laneSection_lcr_lane_roadMark &obj) {

        if (elem->Attribute("color") != nullptr)
            obj._color = std::string(elem->Attribute("color"));

        if (elem->Attribute("height") != nullptr)
            obj._height = elem->DoubleAttribute("height", 0.0);

        if (elem->Attribute("laneChange") != nullptr)
            obj._laneChange = std::string(elem->Attribute("laneChange"));

        if (elem->Attribute("material") != nullptr)
            obj._material = std::string(elem->Attribute("material"));

        if (elem->Attribute("sOffset") != nullptr)
            obj._sOffset = elem->DoubleAttribute("sOffset", 0.0);

        if (elem->Attribute("type") != nullptr)
            obj._type = std::string(elem->Attribute("type"));

        if (elem->Attribute("weight") != nullptr)
            obj._weight = std::string(elem->Attribute("weight"));

        if (elem->Attribute("width") != nullptr)
            obj._width = elem->DoubleAttribute("width", 0.0);

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("explicit");
        if (e != nullptr)
            __parse__t_road_lanes_laneSection_lcr_lane_roadMark_explicit(e, *(obj.sub_explicit.create()));

        e = elem->FirstChildElement("sway");
        while (e != nullptr) {

            obj.sub_sway.emplace_back();
            __parse__t_road_lanes_laneSection_lcr_lane_roadMark_sway(e, obj.sub_sway.back());

            e = e->NextSiblingElement("sway");
        }

        e = elem->FirstChildElement("type");
        if (e != nullptr)
            __parse__t_road_lanes_laneSection_lcr_lane_roadMark_type(e, *(obj.sub_type.create()));


        return true;

    }

    bool __parse__t_road_lanes_laneSection_lcr_lane_roadMark_sway(const tinyxml2::XMLElement *elem,
                                                                  t_road_lanes_laneSection_lcr_lane_roadMark_sway &obj) {

        if (elem->Attribute("a") != nullptr)
            obj._a = elem->DoubleAttribute("a", 0.0);

        if (elem->Attribute("b") != nullptr)
            obj._b = elem->DoubleAttribute("b", 0.0);

        if (elem->Attribute("c") != nullptr)
            obj._c = elem->DoubleAttribute("c", 0.0);

        if (elem->Attribute("d") != nullptr)
            obj._d = elem->DoubleAttribute("d", 0.0);

        if (elem->Attribute("ds") != nullptr)
            obj._ds = elem->DoubleAttribute("ds", 0.0);


        return true;

    }

    bool __parse__t_road_lanes_laneSection_lcr_lane_roadMark_type(const tinyxml2::XMLElement *elem,
                                                                  t_road_lanes_laneSection_lcr_lane_roadMark_type &obj) {

        if (elem->Attribute("name") != nullptr)
            obj._name = std::string(elem->Attribute("name"));

        if (elem->Attribute("width") != nullptr)
            obj._width = elem->DoubleAttribute("width", 0.0);

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("line");
        while (e != nullptr) {

            obj.sub_line.emplace_back();
            __parse__t_road_lanes_laneSection_lcr_lane_roadMark_type_line(e, obj.sub_line.back());

            e = e->NextSiblingElement("line");
        }


        return true;

    }

    bool __parse__t_road_lanes_laneSection_lcr_lane_roadMark_type_line(const tinyxml2::XMLElement *elem,
                                                                       t_road_lanes_laneSection_lcr_lane_roadMark_type_line &obj) {

        if (elem->Attribute("color") != nullptr)
            obj._color = std::string(elem->Attribute("color"));

        if (elem->Attribute("length") != nullptr)
            obj._length = elem->DoubleAttribute("length", 0.0);

        if (elem->Attribute("rule") != nullptr)
            obj._rule = std::string(elem->Attribute("rule"));

        if (elem->Attribute("sOffset") != nullptr)
            obj._sOffset = elem->DoubleAttribute("sOffset", 0.0);

        if (elem->Attribute("space") != nullptr)
            obj._space = elem->DoubleAttribute("space", 0.0);

        if (elem->Attribute("tOffset") != nullptr)
            obj._tOffset = elem->DoubleAttribute("tOffset", 0.0);

        if (elem->Attribute("width") != nullptr)
            obj._width = elem->DoubleAttribute("width", 0.0);


        return true;

    }

    bool __parse__t_road_lanes_laneSection_lcr_lane_roadMark_explicit(const tinyxml2::XMLElement *elem,
                                                                      t_road_lanes_laneSection_lcr_lane_roadMark_explicit &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("line");
        while (e != nullptr) {

            obj.sub_line.emplace_back();
            __parse__t_road_lanes_laneSection_lcr_lane_roadMark_explicit_line(e, obj.sub_line.back());

            e = e->NextSiblingElement("line");
        }


        return true;

    }

    bool __parse__t_road_lanes_laneSection_lcr_lane_roadMark_explicit_line(const tinyxml2::XMLElement *elem,
                                                                           t_road_lanes_laneSection_lcr_lane_roadMark_explicit_line &obj) {

        if (elem->Attribute("length") != nullptr)
            obj._length = elem->DoubleAttribute("length", 0.0);

        if (elem->Attribute("rule") != nullptr)
            obj._rule = std::string(elem->Attribute("rule"));

        if (elem->Attribute("sOffset") != nullptr)
            obj._sOffset = elem->DoubleAttribute("sOffset", 0.0);

        if (elem->Attribute("tOffset") != nullptr)
            obj._tOffset = elem->DoubleAttribute("tOffset", 0.0);

        if (elem->Attribute("width") != nullptr)
            obj._width = elem->DoubleAttribute("width", 0.0);


        return true;

    }

    bool __parse__t_road_lanes_laneSection_lr_lane_material(const tinyxml2::XMLElement *elem,
                                                            t_road_lanes_laneSection_lr_lane_material &obj) {

        if (elem->Attribute("friction") != nullptr)
            obj._friction = elem->DoubleAttribute("friction", 0.0);

        if (elem->Attribute("roughness") != nullptr)
            obj._roughness = elem->DoubleAttribute("roughness", 0.0);

        if (elem->Attribute("sOffset") != nullptr)
            obj._sOffset = elem->DoubleAttribute("sOffset", 0.0);

        if (elem->Attribute("surface") != nullptr)
            obj._surface = std::string(elem->Attribute("surface"));


        return true;

    }

    bool __parse__t_road_lanes_laneSection_lr_lane_visibility(const tinyxml2::XMLElement *elem,
                                                              t_road_lanes_laneSection_lr_lane_visibility &obj) {

        if (elem->Attribute("back") != nullptr)
            obj._back = elem->DoubleAttribute("back", 0.0);

        if (elem->Attribute("forward") != nullptr)
            obj._forward = elem->DoubleAttribute("forward", 0.0);

        if (elem->Attribute("left") != nullptr)
            obj._left = elem->DoubleAttribute("left", 0.0);

        if (elem->Attribute("right") != nullptr)
            obj._right = elem->DoubleAttribute("right", 0.0);

        if (elem->Attribute("sOffset") != nullptr)
            obj._sOffset = elem->DoubleAttribute("sOffset", 0.0);


        return true;

    }

    bool __parse__t_road_lanes_laneSection_lr_lane_speed(const tinyxml2::XMLElement *elem,
                                                         t_road_lanes_laneSection_lr_lane_speed &obj) {

        if (elem->Attribute("max") != nullptr)
            obj._max = elem->DoubleAttribute("max", 0.0);

        if (elem->Attribute("sOffset") != nullptr)
            obj._sOffset = elem->DoubleAttribute("sOffset", 0.0);

        if (elem->Attribute("unit") != nullptr)
            obj._unit = std::string(elem->Attribute("unit"));


        return true;

    }

    bool __parse__t_road_lanes_laneSection_lr_lane_access(const tinyxml2::XMLElement *elem,
                                                          t_road_lanes_laneSection_lr_lane_access &obj) {

        if (elem->Attribute("restriction") != nullptr)
            obj._restriction = std::string(elem->Attribute("restriction"));

        if (elem->Attribute("rule") != nullptr)
            obj._rule = std::string(elem->Attribute("rule"));

        if (elem->Attribute("sOffset") != nullptr)
            obj._sOffset = elem->DoubleAttribute("sOffset", 0.0);


        return true;

    }

    bool __parse__t_road_lanes_laneSection_lr_lane_height(const tinyxml2::XMLElement *elem,
                                                          t_road_lanes_laneSection_lr_lane_height &obj) {

        if (elem->Attribute("inner") != nullptr)
            obj._inner = elem->DoubleAttribute("inner", 0.0);

        if (elem->Attribute("outer") != nullptr)
            obj._outer = elem->DoubleAttribute("outer", 0.0);

        if (elem->Attribute("sOffset") != nullptr)
            obj._sOffset = elem->DoubleAttribute("sOffset", 0.0);


        return true;

    }

    bool __parse__t_road_lanes_laneSection_lr_lane_rule(const tinyxml2::XMLElement *elem,
                                                        t_road_lanes_laneSection_lr_lane_rule &obj) {

        if (elem->Attribute("sOffset") != nullptr)
            obj._sOffset = elem->DoubleAttribute("sOffset", 0.0);

        if (elem->Attribute("value") != nullptr)
            obj._value = std::string(elem->Attribute("value"));


        return true;

    }

    bool __parse__t_road_objects(const tinyxml2::XMLElement *elem, t_road_objects &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("bridge");
        while (e != nullptr) {

            obj.sub_bridge.emplace_back();
            __parse__t_road_objects_bridge(e, obj.sub_bridge.back());

            e = e->NextSiblingElement("bridge");
        }

        e = elem->FirstChildElement("object");
        while (e != nullptr) {

            obj.sub_object.emplace_back();
            __parse__t_road_objects_object(e, obj.sub_object.back());

            e = e->NextSiblingElement("object");
        }

        e = elem->FirstChildElement("objectReference");
        while (e != nullptr) {

            obj.sub_objectReference.emplace_back();
            __parse__t_road_objects_objectReference(e, obj.sub_objectReference.back());

            e = e->NextSiblingElement("objectReference");
        }

        e = elem->FirstChildElement("tunnel");
        while (e != nullptr) {

            obj.sub_tunnel.emplace_back();
            __parse__t_road_objects_tunnel(e, obj.sub_tunnel.back());

            e = e->NextSiblingElement("tunnel");
        }


        return true;

    }

    bool __parse__t_road_objects_object(const tinyxml2::XMLElement *elem, t_road_objects_object &obj) {

        if (elem->Attribute("dynamic") != nullptr)
            obj._dynamic = std::string(elem->Attribute("dynamic"));

        if (elem->Attribute("hdg") != nullptr)
            obj._hdg = elem->DoubleAttribute("hdg", 0.0);

        if (elem->Attribute("height") != nullptr)
            obj._height = elem->DoubleAttribute("height", 0.0);

        if (elem->Attribute("id") != nullptr)
            obj._id = std::string(elem->Attribute("id"));

        if (elem->Attribute("length") != nullptr)
            obj._length = elem->DoubleAttribute("length", 0.0);

        if (elem->Attribute("name") != nullptr)
            obj._name = std::string(elem->Attribute("name"));

        if (elem->Attribute("orientation") != nullptr)
            obj._orientation = std::string(elem->Attribute("orientation"));

        if (elem->Attribute("pitch") != nullptr)
            obj._pitch = elem->DoubleAttribute("pitch", 0.0);

        if (elem->Attribute("radius") != nullptr)
            obj._radius = elem->DoubleAttribute("radius", 0.0);

        if (elem->Attribute("roll") != nullptr)
            obj._roll = elem->DoubleAttribute("roll", 0.0);

        if (elem->Attribute("s") != nullptr)
            obj._s = elem->DoubleAttribute("s", 0.0);

        if (elem->Attribute("subtype") != nullptr)
            obj._subtype = std::string(elem->Attribute("subtype"));

        if (elem->Attribute("t") != nullptr)
            obj._t = elem->DoubleAttribute("t", 0.0);

        if (elem->Attribute("type") != nullptr)
            obj._type = std::string(elem->Attribute("type"));

        if (elem->Attribute("validLength") != nullptr)
            obj._validLength = elem->DoubleAttribute("validLength", 0.0);

        if (elem->Attribute("width") != nullptr)
            obj._width = elem->DoubleAttribute("width", 0.0);

        if (elem->Attribute("zOffset") != nullptr)
            obj._zOffset = elem->DoubleAttribute("zOffset", 0.0);

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("borders");
        if (e != nullptr)
            __parse__t_road_objects_object_borders(e, *(obj.sub_borders.create()));

        e = elem->FirstChildElement("markings");
        if (e != nullptr)
            __parse__t_road_objects_object_markings(e, *(obj.sub_markings.create()));

        e = elem->FirstChildElement("material");
        while (e != nullptr) {

            obj.sub_material.emplace_back();
            __parse__t_road_objects_object_material(e, obj.sub_material.back());

            e = e->NextSiblingElement("material");
        }

        e = elem->FirstChildElement("outline");
        if (e != nullptr)
            __parse__t_road_objects_object_outlines_outline(e, *(obj.sub_outline.create()));

        e = elem->FirstChildElement("outlines");
        if (e != nullptr)
            __parse__t_road_objects_object_outlines(e, *(obj.sub_outlines.create()));

        e = elem->FirstChildElement("parkingSpace");
        if (e != nullptr)
            __parse__t_road_objects_object_parkingSpace(e, *(obj.sub_parkingSpace.create()));

        e = elem->FirstChildElement("repeat");
        if (e != nullptr)
            __parse__t_road_objects_object_repeat(e, *(obj.sub_repeat.create()));

        e = elem->FirstChildElement("validity");
        while (e != nullptr) {

            obj.sub_validity.emplace_back();
            __parse__t_road_objects_object_laneValidity(e, obj.sub_validity.back());

            e = e->NextSiblingElement("validity");
        }


        return true;

    }

    bool __parse__t_road_objects_object_repeat(const tinyxml2::XMLElement *elem, t_road_objects_object_repeat &obj) {

        if (elem->Attribute("distance") != nullptr)
            obj._distance = elem->DoubleAttribute("distance", 0.0);

        if (elem->Attribute("heightEnd") != nullptr)
            obj._heightEnd = elem->DoubleAttribute("heightEnd", 0.0);

        if (elem->Attribute("heightStart") != nullptr)
            obj._heightStart = elem->DoubleAttribute("heightStart", 0.0);

        if (elem->Attribute("length") != nullptr)
            obj._length = elem->DoubleAttribute("length", 0.0);

        if (elem->Attribute("lengthEnd") != nullptr)
            obj._lengthEnd = elem->DoubleAttribute("lengthEnd", 0.0);

        if (elem->Attribute("lengthStart") != nullptr)
            obj._lengthStart = elem->DoubleAttribute("lengthStart", 0.0);

        if (elem->Attribute("radiusEnd") != nullptr)
            obj._radiusEnd = elem->DoubleAttribute("radiusEnd", 0.0);

        if (elem->Attribute("radiusStart") != nullptr)
            obj._radiusStart = elem->DoubleAttribute("radiusStart", 0.0);

        if (elem->Attribute("s") != nullptr)
            obj._s = elem->DoubleAttribute("s", 0.0);

        if (elem->Attribute("tEnd") != nullptr)
            obj._tEnd = elem->DoubleAttribute("tEnd", 0.0);

        if (elem->Attribute("tStart") != nullptr)
            obj._tStart = elem->DoubleAttribute("tStart", 0.0);

        if (elem->Attribute("widthEnd") != nullptr)
            obj._widthEnd = elem->DoubleAttribute("widthEnd", 0.0);

        if (elem->Attribute("widthStart") != nullptr)
            obj._widthStart = elem->DoubleAttribute("widthStart", 0.0);

        if (elem->Attribute("zOffsetEnd") != nullptr)
            obj._zOffsetEnd = elem->DoubleAttribute("zOffsetEnd", 0.0);

        if (elem->Attribute("zOffsetStart") != nullptr)
            obj._zOffsetStart = elem->DoubleAttribute("zOffsetStart", 0.0);


        return true;

    }

    bool
    __parse__t_road_objects_object_outlines(const tinyxml2::XMLElement *elem, t_road_objects_object_outlines &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("outline");
        while (e != nullptr) {

            obj.sub_outline.emplace_back();
            __parse__t_road_objects_object_outlines_outline(e, obj.sub_outline.back());

            e = e->NextSiblingElement("outline");
        }


        return true;

    }

    bool __parse__t_road_objects_object_outlines_outline(const tinyxml2::XMLElement *elem,
                                                         t_road_objects_object_outlines_outline &obj) {

        if (elem->Attribute("closed") != nullptr)
            obj._closed = std::string(elem->Attribute("closed"));

        if (elem->Attribute("fillType") != nullptr)
            obj._fillType = std::string(elem->Attribute("fillType"));

        if (elem->Attribute("id") != nullptr)
            obj._id = elem->UnsignedAttribute("id", 0);

        if (elem->Attribute("laneType") != nullptr)
            obj._laneType = std::string(elem->Attribute("laneType"));

        if (elem->Attribute("outer") != nullptr)
            obj._outer = std::string(elem->Attribute("outer"));

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("cornerLocal");
        while (e != nullptr) {

            obj.sub_cornerLocal.emplace_back();
            __parse__t_road_objects_object_outlines_outline_cornerLocal(e, obj.sub_cornerLocal.back());

            e = e->NextSiblingElement("cornerLocal");
        }

        e = elem->FirstChildElement("cornerRoad");
        while (e != nullptr) {

            obj.sub_cornerRoad.emplace_back();
            __parse__t_road_objects_object_outlines_outline_cornerRoad(e, obj.sub_cornerRoad.back());

            e = e->NextSiblingElement("cornerRoad");
        }


        return true;

    }

    bool __parse__t_road_objects_object_outlines_outline_cornerRoad(const tinyxml2::XMLElement *elem,
                                                                    t_road_objects_object_outlines_outline_cornerRoad &obj) {

        if (elem->Attribute("dz") != nullptr)
            obj._dz = elem->DoubleAttribute("dz", 0.0);

        if (elem->Attribute("height") != nullptr)
            obj._height = elem->DoubleAttribute("height", 0.0);

        if (elem->Attribute("id") != nullptr)
            obj._id = elem->UnsignedAttribute("id", 0);

        if (elem->Attribute("s") != nullptr)
            obj._s = elem->DoubleAttribute("s", 0.0);

        if (elem->Attribute("t") != nullptr)
            obj._t = elem->DoubleAttribute("t", 0.0);


        return true;

    }

    bool __parse__t_road_objects_object_outlines_outline_cornerLocal(const tinyxml2::XMLElement *elem,
                                                                     t_road_objects_object_outlines_outline_cornerLocal &obj) {

        if (elem->Attribute("height") != nullptr)
            obj._height = elem->DoubleAttribute("height", 0.0);

        if (elem->Attribute("id") != nullptr)
            obj._id = elem->UnsignedAttribute("id", 0);

        if (elem->Attribute("u") != nullptr)
            obj._u = elem->DoubleAttribute("u", 0.0);

        if (elem->Attribute("v") != nullptr)
            obj._v = elem->DoubleAttribute("v", 0.0);

        if (elem->Attribute("z") != nullptr)
            obj._z = elem->DoubleAttribute("z", 0.0);


        return true;

    }

    bool
    __parse__t_road_objects_object_material(const tinyxml2::XMLElement *elem, t_road_objects_object_material &obj) {

        if (elem->Attribute("friction") != nullptr)
            obj._friction = elem->DoubleAttribute("friction", 0.0);

        if (elem->Attribute("roughness") != nullptr)
            obj._roughness = elem->DoubleAttribute("roughness", 0.0);

        if (elem->Attribute("surface") != nullptr)
            obj._surface = std::string(elem->Attribute("surface"));


        return true;

    }

    bool __parse__t_road_objects_object_laneValidity(const tinyxml2::XMLElement *elem,
                                                     t_road_objects_object_laneValidity &obj) {

        if (elem->Attribute("fromLane") != nullptr)
            obj._fromLane = elem->IntAttribute("fromLane", 0);

        if (elem->Attribute("toLane") != nullptr)
            obj._toLane = elem->IntAttribute("toLane", 0);


        return true;

    }

    bool __parse__t_road_objects_object_parkingSpace(const tinyxml2::XMLElement *elem,
                                                     t_road_objects_object_parkingSpace &obj) {

        if (elem->Attribute("access") != nullptr)
            obj._access = std::string(elem->Attribute("access"));

        if (elem->Attribute("restrictions") != nullptr)
            obj._restrictions = std::string(elem->Attribute("restrictions"));


        return true;

    }

    bool
    __parse__t_road_objects_object_markings(const tinyxml2::XMLElement *elem, t_road_objects_object_markings &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("marking");
        while (e != nullptr) {

            obj.sub_marking.emplace_back();
            __parse__t_road_objects_object_markings_marking(e, obj.sub_marking.back());

            e = e->NextSiblingElement("marking");
        }


        return true;

    }

    bool __parse__t_road_objects_object_markings_marking(const tinyxml2::XMLElement *elem,
                                                         t_road_objects_object_markings_marking &obj) {

        if (elem->Attribute("color") != nullptr)
            obj._color = std::string(elem->Attribute("color"));

        if (elem->Attribute("lineLength") != nullptr)
            obj._lineLength = elem->DoubleAttribute("lineLength", 0.0);

        if (elem->Attribute("side") != nullptr)
            obj._side = std::string(elem->Attribute("side"));

        if (elem->Attribute("spaceLength") != nullptr)
            obj._spaceLength = elem->DoubleAttribute("spaceLength", 0.0);

        if (elem->Attribute("startOffset") != nullptr)
            obj._startOffset = elem->DoubleAttribute("startOffset", 0.0);

        if (elem->Attribute("stopOffset") != nullptr)
            obj._stopOffset = elem->DoubleAttribute("stopOffset", 0.0);

        if (elem->Attribute("weight") != nullptr)
            obj._weight = std::string(elem->Attribute("weight"));

        if (elem->Attribute("width") != nullptr)
            obj._width = elem->DoubleAttribute("width", 0.0);

        if (elem->Attribute("zOffset") != nullptr)
            obj._zOffset = elem->DoubleAttribute("zOffset", 0.0);

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("cornerReference");
        while (e != nullptr) {

            obj.sub_cornerReference.emplace_back();
            __parse__t_road_objects_object_markings_marking_cornerReference(e, obj.sub_cornerReference.back());

            e = e->NextSiblingElement("cornerReference");
        }


        return true;

    }

    bool __parse__t_road_objects_object_markings_marking_cornerReference(const tinyxml2::XMLElement *elem,
                                                                         t_road_objects_object_markings_marking_cornerReference &obj) {

        if (elem->Attribute("id") != nullptr)
            obj._id = elem->UnsignedAttribute("id", 0);


        return true;

    }

    bool __parse__t_road_objects_object_borders(const tinyxml2::XMLElement *elem, t_road_objects_object_borders &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("border");
        while (e != nullptr) {

            obj.sub_border.emplace_back();
            __parse__t_road_objects_object_borders_border(e, obj.sub_border.back());

            e = e->NextSiblingElement("border");
        }


        return true;

    }

    bool __parse__t_road_objects_object_borders_border(const tinyxml2::XMLElement *elem,
                                                       t_road_objects_object_borders_border &obj) {

        if (elem->Attribute("outlineId") != nullptr)
            obj._outlineId = elem->UnsignedAttribute("outlineId", 0);

        if (elem->Attribute("type") != nullptr)
            obj._type = std::string(elem->Attribute("type"));

        if (elem->Attribute("useCompleteOutline") != nullptr)
            obj._useCompleteOutline = std::string(elem->Attribute("useCompleteOutline"));

        if (elem->Attribute("width") != nullptr)
            obj._width = elem->DoubleAttribute("width", 0.0);

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("cornerReference");
        while (e != nullptr) {

            obj.sub_cornerReference.emplace_back();
            __parse__t_road_objects_object_markings_marking_cornerReference(e, obj.sub_cornerReference.back());

            e = e->NextSiblingElement("cornerReference");
        }


        return true;

    }

    bool
    __parse__t_road_objects_objectReference(const tinyxml2::XMLElement *elem, t_road_objects_objectReference &obj) {

        if (elem->Attribute("id") != nullptr)
            obj._id = std::string(elem->Attribute("id"));

        if (elem->Attribute("orientation") != nullptr)
            obj._orientation = std::string(elem->Attribute("orientation"));

        if (elem->Attribute("s") != nullptr)
            obj._s = elem->DoubleAttribute("s", 0.0);

        if (elem->Attribute("t") != nullptr)
            obj._t = elem->DoubleAttribute("t", 0.0);

        if (elem->Attribute("validLength") != nullptr)
            obj._validLength = elem->DoubleAttribute("validLength", 0.0);

        if (elem->Attribute("zOffset") != nullptr)
            obj._zOffset = elem->DoubleAttribute("zOffset", 0.0);

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("validity");
        while (e != nullptr) {

            obj.sub_validity.emplace_back();
            __parse__t_road_objects_object_laneValidity(e, obj.sub_validity.back());

            e = e->NextSiblingElement("validity");
        }


        return true;

    }

    bool __parse__t_road_objects_tunnel(const tinyxml2::XMLElement *elem, t_road_objects_tunnel &obj) {

        if (elem->Attribute("daylight") != nullptr)
            obj._daylight = elem->DoubleAttribute("daylight", 0.0);

        if (elem->Attribute("id") != nullptr)
            obj._id = std::string(elem->Attribute("id"));

        if (elem->Attribute("length") != nullptr)
            obj._length = elem->DoubleAttribute("length", 0.0);

        if (elem->Attribute("lighting") != nullptr)
            obj._lighting = elem->DoubleAttribute("lighting", 0.0);

        if (elem->Attribute("name") != nullptr)
            obj._name = std::string(elem->Attribute("name"));

        if (elem->Attribute("s") != nullptr)
            obj._s = elem->DoubleAttribute("s", 0.0);

        if (elem->Attribute("type") != nullptr)
            obj._type = std::string(elem->Attribute("type"));

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("validity");
        while (e != nullptr) {

            obj.sub_validity.emplace_back();
            __parse__t_road_objects_object_laneValidity(e, obj.sub_validity.back());

            e = e->NextSiblingElement("validity");
        }


        return true;

    }

    bool __parse__t_road_objects_bridge(const tinyxml2::XMLElement *elem, t_road_objects_bridge &obj) {

        if (elem->Attribute("id") != nullptr)
            obj._id = std::string(elem->Attribute("id"));

        if (elem->Attribute("length") != nullptr)
            obj._length = elem->DoubleAttribute("length", 0.0);

        if (elem->Attribute("name") != nullptr)
            obj._name = std::string(elem->Attribute("name"));

        if (elem->Attribute("s") != nullptr)
            obj._s = elem->DoubleAttribute("s", 0.0);

        if (elem->Attribute("type") != nullptr)
            obj._type = std::string(elem->Attribute("type"));

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("validity");
        while (e != nullptr) {

            obj.sub_validity.emplace_back();
            __parse__t_road_objects_object_laneValidity(e, obj.sub_validity.back());

            e = e->NextSiblingElement("validity");
        }


        return true;

    }

    bool __parse__t_road_signals(const tinyxml2::XMLElement *elem, t_road_signals &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("signal");
        while (e != nullptr) {

            obj.sub_signal.emplace_back();
            __parse__t_road_signals_signal(e, obj.sub_signal.back());

            e = e->NextSiblingElement("signal");
        }

        e = elem->FirstChildElement("signalReference");
        while (e != nullptr) {

            obj.sub_signalReference.emplace_back();
            __parse__t_road_signals_signalReference(e, obj.sub_signalReference.back());

            e = e->NextSiblingElement("signalReference");
        }


        return true;

    }

    bool __parse__t_road_signals_signal(const tinyxml2::XMLElement *elem, t_road_signals_signal &obj) {

        if (elem->Attribute("country") != nullptr)
            obj._country = std::string(elem->Attribute("country"));

        if (elem->Attribute("countryRevision") != nullptr)
            obj._countryRevision = std::string(elem->Attribute("countryRevision"));

        if (elem->Attribute("dynamic") != nullptr)
            obj._dynamic = std::string(elem->Attribute("dynamic"));

        if (elem->Attribute("hOffset") != nullptr)
            obj._hOffset = elem->DoubleAttribute("hOffset", 0.0);

        if (elem->Attribute("height") != nullptr)
            obj._height = elem->DoubleAttribute("height", 0.0);

        if (elem->Attribute("id") != nullptr)
            obj._id = std::string(elem->Attribute("id"));

        if (elem->Attribute("name") != nullptr)
            obj._name = std::string(elem->Attribute("name"));

        if (elem->Attribute("orientation") != nullptr)
            obj._orientation = std::string(elem->Attribute("orientation"));

        if (elem->Attribute("pitch") != nullptr)
            obj._pitch = elem->DoubleAttribute("pitch", 0.0);

        if (elem->Attribute("roll") != nullptr)
            obj._roll = elem->DoubleAttribute("roll", 0.0);

        if (elem->Attribute("s") != nullptr)
            obj._s = elem->DoubleAttribute("s", 0.0);

        if (elem->Attribute("subtype") != nullptr)
            obj._subtype = std::string(elem->Attribute("subtype"));

        if (elem->Attribute("t") != nullptr)
            obj._t = elem->DoubleAttribute("t", 0.0);

        if (elem->Attribute("text") != nullptr)
            obj._text = std::string(elem->Attribute("text"));

        if (elem->Attribute("type") != nullptr)
            obj._type = std::string(elem->Attribute("type"));

        if (elem->Attribute("unit") != nullptr)
            obj._unit = std::string(elem->Attribute("unit"));

        if (elem->Attribute("value") != nullptr)
            obj._value = elem->DoubleAttribute("value", 0.0);

        if (elem->Attribute("width") != nullptr)
            obj._width = elem->DoubleAttribute("width", 0.0);

        if (elem->Attribute("zOffset") != nullptr)
            obj._zOffset = elem->DoubleAttribute("zOffset", 0.0);

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("dependency");
        while (e != nullptr) {

            obj.sub_dependency.emplace_back();
            __parse__t_road_signals_signal_dependency(e, obj.sub_dependency.back());

            e = e->NextSiblingElement("dependency");
        }

        e = elem->FirstChildElement("positionInertial");
        while (e != nullptr) {

            obj.sub_positionInertial.emplace_back();
            __parse__t_road_signals_signal_positionInertial(e, obj.sub_positionInertial.back());

            e = e->NextSiblingElement("positionInertial");
        }

        e = elem->FirstChildElement("positionRoad");
        while (e != nullptr) {

            obj.sub_positionRoad.emplace_back();
            __parse__t_road_signals_signal_positionRoad(e, obj.sub_positionRoad.back());

            e = e->NextSiblingElement("positionRoad");
        }

        e = elem->FirstChildElement("reference");
        while (e != nullptr) {

            obj.sub_reference.emplace_back();
            __parse__t_road_signals_signal_reference(e, obj.sub_reference.back());

            e = e->NextSiblingElement("reference");
        }

        e = elem->FirstChildElement("validity");
        while (e != nullptr) {

            obj.sub_validity.emplace_back();
            __parse__t_road_objects_object_laneValidity(e, obj.sub_validity.back());

            e = e->NextSiblingElement("validity");
        }


        return true;

    }

    bool
    __parse__t_road_signals_signal_dependency(const tinyxml2::XMLElement *elem, t_road_signals_signal_dependency &obj) {

        if (elem->Attribute("id") != nullptr)
            obj._id = std::string(elem->Attribute("id"));

        if (elem->Attribute("type") != nullptr)
            obj._type = std::string(elem->Attribute("type"));


        return true;

    }

    bool
    __parse__t_road_signals_signal_reference(const tinyxml2::XMLElement *elem, t_road_signals_signal_reference &obj) {

        if (elem->Attribute("elementId") != nullptr)
            obj._elementId = std::string(elem->Attribute("elementId"));

        if (elem->Attribute("elementType") != nullptr)
            obj._elementType = std::string(elem->Attribute("elementType"));

        if (elem->Attribute("type") != nullptr)
            obj._type = std::string(elem->Attribute("type"));


        return true;

    }

    bool __parse__t_road_signals_signal_positionRoad(const tinyxml2::XMLElement *elem,
                                                     t_road_signals_signal_positionRoad &obj) {

        if (elem->Attribute("hOffset") != nullptr)
            obj._hOffset = elem->DoubleAttribute("hOffset", 0.0);

        if (elem->Attribute("pitch") != nullptr)
            obj._pitch = elem->DoubleAttribute("pitch", 0.0);

        if (elem->Attribute("roadId") != nullptr)
            obj._roadId = std::string(elem->Attribute("roadId"));

        if (elem->Attribute("roll") != nullptr)
            obj._roll = elem->DoubleAttribute("roll", 0.0);

        if (elem->Attribute("s") != nullptr)
            obj._s = elem->DoubleAttribute("s", 0.0);

        if (elem->Attribute("t") != nullptr)
            obj._t = elem->DoubleAttribute("t", 0.0);

        if (elem->Attribute("zOffset") != nullptr)
            obj._zOffset = elem->DoubleAttribute("zOffset", 0.0);


        return true;

    }

    bool __parse__t_road_signals_signal_positionInertial(const tinyxml2::XMLElement *elem,
                                                         t_road_signals_signal_positionInertial &obj) {

        if (elem->Attribute("hdg") != nullptr)
            obj._hdg = elem->DoubleAttribute("hdg", 0.0);

        if (elem->Attribute("pitch") != nullptr)
            obj._pitch = elem->DoubleAttribute("pitch", 0.0);

        if (elem->Attribute("roll") != nullptr)
            obj._roll = elem->DoubleAttribute("roll", 0.0);

        if (elem->Attribute("x") != nullptr)
            obj._x = elem->DoubleAttribute("x", 0.0);

        if (elem->Attribute("y") != nullptr)
            obj._y = elem->DoubleAttribute("y", 0.0);

        if (elem->Attribute("z") != nullptr)
            obj._z = elem->DoubleAttribute("z", 0.0);


        return true;

    }

    bool
    __parse__t_road_signals_signalReference(const tinyxml2::XMLElement *elem, t_road_signals_signalReference &obj) {

        if (elem->Attribute("id") != nullptr)
            obj._id = std::string(elem->Attribute("id"));

        if (elem->Attribute("orientation") != nullptr)
            obj._orientation = std::string(elem->Attribute("orientation"));

        if (elem->Attribute("s") != nullptr)
            obj._s = elem->DoubleAttribute("s", 0.0);

        if (elem->Attribute("t") != nullptr)
            obj._t = elem->DoubleAttribute("t", 0.0);

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("validity");
        while (e != nullptr) {

            obj.sub_validity.emplace_back();
            __parse__t_road_objects_object_laneValidity(e, obj.sub_validity.back());

            e = e->NextSiblingElement("validity");
        }


        return true;

    }

    bool __parse__t_road_surface(const tinyxml2::XMLElement *elem, t_road_surface &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("CRG");
        while (e != nullptr) {

            obj.sub_CRG.emplace_back();
            __parse__t_road_surface_CRG(e, obj.sub_CRG.back());

            e = e->NextSiblingElement("CRG");
        }


        return true;

    }

    bool __parse__t_road_surface_CRG(const tinyxml2::XMLElement *elem, t_road_surface_CRG &obj) {

        if (elem->Attribute("file") != nullptr)
            obj._file = std::string(elem->Attribute("file"));

        if (elem->Attribute("hOffset") != nullptr)
            obj._hOffset = elem->DoubleAttribute("hOffset", 0.0);

        if (elem->Attribute("mode") != nullptr)
            obj._mode = std::string(elem->Attribute("mode"));

        if (elem->Attribute("orientation") != nullptr)
            obj._orientation = std::string(elem->Attribute("orientation"));

        if (elem->Attribute("purpose") != nullptr)
            obj._purpose = std::string(elem->Attribute("purpose"));

        if (elem->Attribute("sEnd") != nullptr)
            obj._sEnd = elem->DoubleAttribute("sEnd", 0.0);

        if (elem->Attribute("sOffset") != nullptr)
            obj._sOffset = elem->DoubleAttribute("sOffset", 0.0);

        if (elem->Attribute("sStart") != nullptr)
            obj._sStart = elem->DoubleAttribute("sStart", 0.0);

        if (elem->Attribute("tOffset") != nullptr)
            obj._tOffset = elem->DoubleAttribute("tOffset", 0.0);

        if (elem->Attribute("zOffset") != nullptr)
            obj._zOffset = elem->DoubleAttribute("zOffset", 0.0);

        if (elem->Attribute("zScale") != nullptr)
            obj._zScale = elem->DoubleAttribute("zScale", 0.0);


        return true;

    }

    bool __parse__t_road_railroad(const tinyxml2::XMLElement *elem, t_road_railroad &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("switch");
        while (e != nullptr) {

            obj.sub_switch.emplace_back();
            __parse__t_road_railroad_switch(e, obj.sub_switch.back());

            e = e->NextSiblingElement("switch");
        }


        return true;

    }

    bool __parse__t_road_railroad_switch(const tinyxml2::XMLElement *elem, t_road_railroad_switch &obj) {

        if (elem->Attribute("id") != nullptr)
            obj._id = std::string(elem->Attribute("id"));

        if (elem->Attribute("name") != nullptr)
            obj._name = std::string(elem->Attribute("name"));

        if (elem->Attribute("position") != nullptr)
            obj._position = std::string(elem->Attribute("position"));

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("mainTrack");
        if (e != nullptr)
            __parse__t_road_railroad_switch_mainTrack(e, *(obj.sub_mainTrack.create()));

        e = elem->FirstChildElement("partner");
        if (e != nullptr)
            __parse__t_road_railroad_switch_partner(e, *(obj.sub_partner.create()));

        e = elem->FirstChildElement("sideTrack");
        if (e != nullptr)
            __parse__t_road_railroad_switch_sideTrack(e, *(obj.sub_sideTrack.create()));


        return true;

    }

    bool
    __parse__t_road_railroad_switch_mainTrack(const tinyxml2::XMLElement *elem, t_road_railroad_switch_mainTrack &obj) {

        if (elem->Attribute("dir") != nullptr)
            obj._dir = std::string(elem->Attribute("dir"));

        if (elem->Attribute("id") != nullptr)
            obj._id = std::string(elem->Attribute("id"));

        if (elem->Attribute("s") != nullptr)
            obj._s = elem->DoubleAttribute("s", 0.0);


        return true;

    }

    bool
    __parse__t_road_railroad_switch_sideTrack(const tinyxml2::XMLElement *elem, t_road_railroad_switch_sideTrack &obj) {

        if (elem->Attribute("dir") != nullptr)
            obj._dir = std::string(elem->Attribute("dir"));

        if (elem->Attribute("id") != nullptr)
            obj._id = std::string(elem->Attribute("id"));

        if (elem->Attribute("s") != nullptr)
            obj._s = elem->DoubleAttribute("s", 0.0);


        return true;

    }

    bool
    __parse__t_road_railroad_switch_partner(const tinyxml2::XMLElement *elem, t_road_railroad_switch_partner &obj) {

        if (elem->Attribute("id") != nullptr)
            obj._id = std::string(elem->Attribute("id"));

        if (elem->Attribute("name") != nullptr)
            obj._name = std::string(elem->Attribute("name"));


        return true;

    }

    bool __parse__t_controller(const tinyxml2::XMLElement *elem, t_controller &obj) {

        if (elem->Attribute("id") != nullptr)
            obj._id = std::string(elem->Attribute("id"));

        if (elem->Attribute("name") != nullptr)
            obj._name = std::string(elem->Attribute("name"));

        if (elem->Attribute("sequence") != nullptr)
            obj._sequence = elem->UnsignedAttribute("sequence", 0);

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("control");
        while (e != nullptr) {

            obj.sub_control.emplace_back();
            __parse__t_controller_control(e, obj.sub_control.back());

            e = e->NextSiblingElement("control");
        }


        return true;

    }

    bool __parse__t_controller_control(const tinyxml2::XMLElement *elem, t_controller_control &obj) {

        if (elem->Attribute("signalId") != nullptr)
            obj._signalId = std::string(elem->Attribute("signalId"));

        if (elem->Attribute("type") != nullptr)
            obj._type = std::string(elem->Attribute("type"));


        return true;

    }

    bool __parse__t_junction(const tinyxml2::XMLElement *elem, t_junction &obj) {

        if (elem->Attribute("id") != nullptr)
            obj._id = std::string(elem->Attribute("id"));

        if (elem->Attribute("name") != nullptr)
            obj._name = std::string(elem->Attribute("name"));

        if (elem->Attribute("type") != nullptr)
            obj._type = std::string(elem->Attribute("type"));

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("connection");
        while (e != nullptr) {

            obj.sub_connection.emplace_back();
            __parse__t_junction_connection(e, obj.sub_connection.back());

            e = e->NextSiblingElement("connection");
        }

        e = elem->FirstChildElement("controller");
        while (e != nullptr) {

            obj.sub_controller.emplace_back();
            __parse__t_junction_controller(e, obj.sub_controller.back());

            e = e->NextSiblingElement("controller");
        }

        e = elem->FirstChildElement("priority");
        while (e != nullptr) {

            obj.sub_priority.emplace_back();
            __parse__t_junction_priority(e, obj.sub_priority.back());

            e = e->NextSiblingElement("priority");
        }

        e = elem->FirstChildElement("surface");
        if (e != nullptr)
            __parse__t_junction_surface(e, *(obj.sub_surface.create()));


        return true;

    }

    bool __parse__t_junction_connection(const tinyxml2::XMLElement *elem, t_junction_connection &obj) {

        if (elem->Attribute("connectingRoad") != nullptr)
            obj._connectingRoad = std::string(elem->Attribute("connectingRoad"));

        if (elem->Attribute("connectionMaster") != nullptr)
            obj._connectionMaster = std::string(elem->Attribute("connectionMaster"));

        if (elem->Attribute("contactPoint") != nullptr)
            obj._contactPoint = std::string(elem->Attribute("contactPoint"));

        if (elem->Attribute("id") != nullptr)
            obj._id = std::string(elem->Attribute("id"));

        if (elem->Attribute("incomingRoad") != nullptr)
            obj._incomingRoad = std::string(elem->Attribute("incomingRoad"));

        if (elem->Attribute("type") != nullptr)
            obj._type = std::string(elem->Attribute("type"));

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("laneLink");
        while (e != nullptr) {

            obj.sub_laneLink.emplace_back();
            __parse__t_junction_connection_laneLink(e, obj.sub_laneLink.back());

            e = e->NextSiblingElement("laneLink");
        }

        e = elem->FirstChildElement("predecessor");
        if (e != nullptr)
            __parse__t_junction_predecessorSuccessor(e, *(obj.sub_predecessor.create()));

        e = elem->FirstChildElement("successor");
        if (e != nullptr)
            __parse__t_junction_predecessorSuccessor(e, *(obj.sub_successor.create()));


        return true;

    }

    bool
    __parse__t_junction_predecessorSuccessor(const tinyxml2::XMLElement *elem, t_junction_predecessorSuccessor &obj) {

        if (elem->Attribute("elementDir") != nullptr)
            obj._elementDir = std::string(elem->Attribute("elementDir"));

        if (elem->Attribute("elementId") != nullptr)
            obj._elementId = std::string(elem->Attribute("elementId"));

        if (elem->Attribute("elementS") != nullptr)
            obj._elementS = elem->DoubleAttribute("elementS", 0.0);

        if (elem->Attribute("elementType") != nullptr)
            obj._elementType = std::string(elem->Attribute("elementType"));


        return true;

    }

    bool
    __parse__t_junction_connection_laneLink(const tinyxml2::XMLElement *elem, t_junction_connection_laneLink &obj) {

        if (elem->Attribute("from") != nullptr)
            obj._from = elem->IntAttribute("from", 0);

        if (elem->Attribute("to") != nullptr)
            obj._to = elem->IntAttribute("to", 0);


        return true;

    }

    bool __parse__t_junction_priority(const tinyxml2::XMLElement *elem, t_junction_priority &obj) {

        if (elem->Attribute("high") != nullptr)
            obj._high = std::string(elem->Attribute("high"));

        if (elem->Attribute("low") != nullptr)
            obj._low = std::string(elem->Attribute("low"));


        return true;

    }

    bool __parse__t_junction_controller(const tinyxml2::XMLElement *elem, t_junction_controller &obj) {

        if (elem->Attribute("id") != nullptr)
            obj._id = std::string(elem->Attribute("id"));

        if (elem->Attribute("sequence") != nullptr)
            obj._sequence = elem->UnsignedAttribute("sequence", 0);

        if (elem->Attribute("type") != nullptr)
            obj._type = std::string(elem->Attribute("type"));


        return true;

    }

    bool __parse__t_junction_surface(const tinyxml2::XMLElement *elem, t_junction_surface &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("CRG");
        while (e != nullptr) {

            obj.sub_CRG.emplace_back();
            __parse__t_junction_surface_CRG(e, obj.sub_CRG.back());

            e = e->NextSiblingElement("CRG");
        }


        return true;

    }

    bool __parse__t_junction_surface_CRG(const tinyxml2::XMLElement *elem, t_junction_surface_CRG &obj) {

        if (elem->Attribute("file") != nullptr)
            obj._file = std::string(elem->Attribute("file"));

        if (elem->Attribute("mode") != nullptr)
            obj._mode = std::string(elem->Attribute("mode"));

        if (elem->Attribute("purpose") != nullptr)
            obj._purpose = std::string(elem->Attribute("purpose"));

        if (elem->Attribute("zOffset") != nullptr)
            obj._zOffset = elem->DoubleAttribute("zOffset", 0.0);

        if (elem->Attribute("zScale") != nullptr)
            obj._zScale = elem->DoubleAttribute("zScale", 0.0);


        return true;

    }

    bool __parse__t_junctionGroup(const tinyxml2::XMLElement *elem, t_junctionGroup &obj) {

        if (elem->Attribute("id") != nullptr)
            obj._id = std::string(elem->Attribute("id"));

        if (elem->Attribute("name") != nullptr)
            obj._name = std::string(elem->Attribute("name"));

        if (elem->Attribute("type") != nullptr)
            obj._type = std::string(elem->Attribute("type"));

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("junctionReference");
        while (e != nullptr) {

            obj.sub_junctionReference.emplace_back();
            __parse__t_junctionGroup_junctionReference(e, obj.sub_junctionReference.back());

            e = e->NextSiblingElement("junctionReference");
        }


        return true;

    }

    bool __parse__t_junctionGroup_junctionReference(const tinyxml2::XMLElement *elem,
                                                    t_junctionGroup_junctionReference &obj) {

        if (elem->Attribute("junction") != nullptr)
            obj._junction = std::string(elem->Attribute("junction"));


        return true;

    }

    bool __parse__t_station(const tinyxml2::XMLElement *elem, t_station &obj) {

        if (elem->Attribute("id") != nullptr)
            obj._id = std::string(elem->Attribute("id"));

        if (elem->Attribute("name") != nullptr)
            obj._name = std::string(elem->Attribute("name"));

        if (elem->Attribute("type") != nullptr)
            obj._type = std::string(elem->Attribute("type"));

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("platform");
        while (e != nullptr) {

            obj.sub_platform.emplace_back();
            __parse__t_station_platform(e, obj.sub_platform.back());

            e = e->NextSiblingElement("platform");
        }


        return true;

    }

    bool __parse__t_station_platform(const tinyxml2::XMLElement *elem, t_station_platform &obj) {

        if (elem->Attribute("id") != nullptr)
            obj._id = std::string(elem->Attribute("id"));

        if (elem->Attribute("name") != nullptr)
            obj._name = std::string(elem->Attribute("name"));

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("segment");
        while (e != nullptr) {

            obj.sub_segment.emplace_back();
            __parse__t_station_platform_segment(e, obj.sub_segment.back());

            e = e->NextSiblingElement("segment");
        }


        return true;

    }

    bool __parse__t_station_platform_segment(const tinyxml2::XMLElement *elem, t_station_platform_segment &obj) {

        if (elem->Attribute("roadId") != nullptr)
            obj._roadId = std::string(elem->Attribute("roadId"));

        if (elem->Attribute("sEnd") != nullptr)
            obj._sEnd = elem->DoubleAttribute("sEnd", 0.0);

        if (elem->Attribute("sStart") != nullptr)
            obj._sStart = elem->DoubleAttribute("sStart", 0.0);

        if (elem->Attribute("side") != nullptr)
            obj._side = std::string(elem->Attribute("side"));


        return true;

    }

    bool __parse__t_userData(const tinyxml2::XMLElement *elem, t_userData &obj) {

        if (elem->Attribute("code") != nullptr)
            obj._code = std::string(elem->Attribute("code"));

        if (elem->Attribute("value") != nullptr)
            obj._value = std::string(elem->Attribute("value"));


        return true;

    }

    bool __parse__t_include(const tinyxml2::XMLElement *elem, t_include &obj) {

        if (elem->Attribute("file") != nullptr)
            obj._file = std::string(elem->Attribute("file"));


        return true;

    }

    bool __parse__t_dataQuality(const tinyxml2::XMLElement *elem, t_dataQuality &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("error");
        if (e != nullptr)
            __parse__t_dataQuality_Error(e, *(obj.sub_error.create()));

        e = elem->FirstChildElement("rawData");
        if (e != nullptr)
            __parse__t_dataQuality_RawData(e, *(obj.sub_rawData.create()));


        return true;

    }

    bool __parse__t_dataQuality_Error(const tinyxml2::XMLElement *elem, t_dataQuality_Error &obj) {

        if (elem->Attribute("xyAbsolute") != nullptr)
            obj._xyAbsolute = elem->DoubleAttribute("xyAbsolute", 0.0);

        if (elem->Attribute("xyRelative") != nullptr)
            obj._xyRelative = elem->DoubleAttribute("xyRelative", 0.0);

        if (elem->Attribute("zAbsolute") != nullptr)
            obj._zAbsolute = elem->DoubleAttribute("zAbsolute", 0.0);

        if (elem->Attribute("zRelative") != nullptr)
            obj._zRelative = elem->DoubleAttribute("zRelative", 0.0);


        return true;

    }

    bool __parse__t_dataQuality_RawData(const tinyxml2::XMLElement *elem, t_dataQuality_RawData &obj) {

        if (elem->Attribute("date") != nullptr)
            obj._date = std::string(elem->Attribute("date"));

        if (elem->Attribute("postProcessing") != nullptr)
            obj._postProcessing = std::string(elem->Attribute("postProcessing"));

        if (elem->Attribute("postProcessingComment") != nullptr)
            obj._postProcessingComment = std::string(elem->Attribute("postProcessingComment"));

        if (elem->Attribute("source") != nullptr)
            obj._source = std::string(elem->Attribute("source"));

        if (elem->Attribute("sourceComment") != nullptr)
            obj._sourceComment = std::string(elem->Attribute("sourceComment"));


        return true;

    }

    bool __parse__OpenDRIVE(const tinyxml2::XMLElement *elem, OpenDRIVE &obj) {

        const tinyxml2::XMLElement *e;

        e = elem->FirstChildElement("controller");
        while (e != nullptr) {

            obj.sub_controller.emplace_back();
            __parse__t_controller(e, obj.sub_controller.back());

            e = e->NextSiblingElement("controller");
        }

        e = elem->FirstChildElement("header");
        if (e != nullptr)
            __parse__t_header(e, *(obj.sub_header.create()));

        e = elem->FirstChildElement("junction");
        while (e != nullptr) {

            obj.sub_junction.emplace_back();
            __parse__t_junction(e, obj.sub_junction.back());

            e = e->NextSiblingElement("junction");
        }

        e = elem->FirstChildElement("junctionGroup");
        while (e != nullptr) {

            obj.sub_junctionGroup.emplace_back();
            __parse__t_junctionGroup(e, obj.sub_junctionGroup.back());

            e = e->NextSiblingElement("junctionGroup");
        }

        e = elem->FirstChildElement("road");
        while (e != nullptr) {

            obj.sub_road.emplace_back();
            __parse__t_road(e, obj.sub_road.back());

            e = e->NextSiblingElement("road");
        }

        e = elem->FirstChildElement("station");
        while (e != nullptr) {

            obj.sub_station.emplace_back();
            __parse__t_station(e, obj.sub_station.back());

            e = e->NextSiblingElement("station");
        }


        return true;

    }


} // namespace odr_1_5
