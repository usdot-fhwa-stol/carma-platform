#pragma once

#include <iostream>
#include <odrparser/odrparser.h>

namespace opendrive_to_lanelet2 {

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

}

}