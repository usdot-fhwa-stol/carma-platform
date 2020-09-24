#pragma once

#include <iostream>
#include <odrparser/odrparser.h>

namespace opendrive_to_lanelet2 {

  class RoadReferencePath {
    odr_1_5::t_road_planView plan_view;

  };

void convert() {
  // create container instance
  odr::OpenDRIVEFile odrData;

  // load xml file content to container (replace <...> by the file name)
  odr::loadFile("/workspaces/carma_ws/carma/test.xodr", odrData); 

  // pointer to the ODR data
  odr_1_5::OpenDRIVE *odrr = odrData.OpenDRIVE1_5.get();

  // access the header
  const auto header = odrr->sub_header.get();
  
  //std::string georeference = odr->sub_header->sub_geoReference._reference // TODO: Default package does not parse georeference https://github.com/JensKlimke/odrparser/issues/3

  std::cout << *header->_date << std::endl; // e.g. "Thu Feb  8 14:24:06 2007"

  // access the roads vector
  const auto &roads = odrr->sub_road;
  std::cout << roads.size() << std::endl; // e.g. 36

  // access a single road content
  const auto &rd = odrr->sub_road.front();
  std::cout << rd.sub_lanes->sub_laneSection.size() << std::endl; // e.g. 1

  for (auto road : odrr->sub_road) {
    referencePath = road.sub_planView->sub_geometry + road.sub_lanes->sub_laneOffset; // TODO how to process the lane offset which is in sections with the geometry. A unified reference line is needed before conversion can occur?
    for (auto lane_section : road.sub_lanes->sub_laneSection) {// Note there is also a lane offset which is probably needed
      
      for (auto lanelet : lane_section.sub_center->sub_lane) {

      }


      for (auto lanelet : lane_section.sub_left->sub_lane) {
        
      }
      for (auto lanelet : lane_section.sub_right->sub_lane) {
        
      }
    }
  }

/* NOTES:
Each lane section must have a lane offset field to receive an offset. However, the lane offset does not need to have started or ended in that section


*/


// Algorithm for convert opendrive to lanelets(opendrive)
// for road in opendrive do // Convert each section of each road into lanelets
//   referencePath ← road.planView + road.laneOffset // planView contains the reference path without offset
//   for section in road do
//     lanelets.add(convert to lanelets(section, referenceP ath))
// for lanelet in lanelets do // Create connectivity graph
//     lanelet.predecessor ← find predecessor for lane(lanelet.lane, opendrive)
//     lanelet.successor ← find successor for lane(lanelet.lane, opendrive)
//     lanelet.left neighbor ← find left neighbor for lane(lanelet.lane)
//     lanelet.right neighbor ← find right neighbor for lane(lanelet.lane)
// return lanelets

}

}