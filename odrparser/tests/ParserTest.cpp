//
// Copyright (c) 2019 Jens Klimke <jens.klimke@rwth-aachen.de>. All rights reserved.
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
// Created by Jens Klimke on 2019-04-25.
//

#include <gtest/gtest.h>
#include <odrparser/odrparser.h>
#include <sstream>

#ifndef TRACKS_DIR
#define TRACKS_DIR "./"
#endif


TEST(ParserTest, DemoCode) {

    // create file name
    std::stringstream ss;
    ss << TRACKS_DIR << "/sample1.1.xodr";

    // create container instance
    odr::OpenDRIVEFile odrData;

    // load xml file content to container (replace <...> by the file name)
    odr::loadFile(ss.str(), odrData);

    // pointer to the ODR data
    odr_1_5::OpenDRIVE *odrr = odrData.OpenDRIVE1_5.get();

    // access the header
    const auto header = odrr->sub_header.get();
    std::cout << *header->_date << std::endl; // e.g. "Thu Feb  8 14:24:06 2007"

    // access the roads vector
    const auto &roads = odrr->sub_road;
    std::cout << roads.size() << std::endl; // e.g. 36

    // access a single road content
    const auto &rd = odrr->sub_road.front();
    std::cout << rd.sub_lanes->sub_laneSection.size() << std::endl; // e.g. 1

}



TEST(ParserTest, LoadODRFile15) {

    odr::OpenDRIVEFile odrFile;

    // create file name
    std::stringstream ss;
    ss << TRACKS_DIR << "/sample1.1.xodr";

    // load file
    odr::loadFile(ss.str(), odrFile);
    odr_1_5::OpenDRIVE *odrr = odrFile.OpenDRIVE1_5.get();

    const auto header = odrr->sub_header.get();
    EXPECT_EQ("Thu Feb  8 14:24:06 2007", *header->_date);

    const auto &roads = odrr->sub_road;
    EXPECT_EQ(36, roads.size());


    const auto &rd = odrr->sub_road.front();

    EXPECT_EQ(1, rd.sub_lanes->sub_laneSection.size());
    EXPECT_EQ(2, rd.sub_lanes->sub_laneSection.front().sub_left->sub_lane.size());
    EXPECT_EQ(1, *rd.sub_lanes->sub_laneSection.front().sub_left->sub_lane.back()._id);
    EXPECT_EQ(2, *rd.sub_lanes->sub_laneSection.front().sub_left->sub_lane.front()._id);
    EXPECT_DOUBLE_EQ(1.5, *rd.sub_lanes->sub_laneSection.front().sub_left->sub_lane.front().sub_width.front()._a);

    EXPECT_TRUE(rd.sub_planView->sub_geometry.front().sub_line);
    EXPECT_TRUE(rd.sub_planView->sub_geometry.at(1).sub_spiral);
    EXPECT_TRUE(rd.sub_planView->sub_geometry.at(2).sub_arc);

    EXPECT_FALSE(rd.sub_planView->sub_geometry.front().sub_arc);
    EXPECT_FALSE(rd.sub_planView->sub_geometry.at(1).sub_line);
    EXPECT_FALSE(rd.sub_planView->sub_geometry.at(2).sub_spiral);

}
