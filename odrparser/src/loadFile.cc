#include <memory>

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
// Created by Jens Klimke on 2019-04-27.
//

#include <string>
#include <tinyxml2.h>
#include <odrparser/odrparser.h>
#include "odr_1_5.hpp"

namespace odr {

    void loadFile(const std::string &filename, OpenDRIVEFile &data) {

        // xml document
        tinyxml2::XMLDocument xml_doc;

        // load and check file
        tinyxml2::XMLError eResult = xml_doc.LoadFile(filename.c_str());
        if (eResult != tinyxml2::XML_SUCCESS) {
            if (eResult == tinyxml2::XML_ERROR_FILE_NOT_FOUND) {
                throw std::runtime_error("File not found!");
            } else {
                throw std::runtime_error("File could not be loaded!");
            }
        }

        // get OpenDRIVE element
        auto od = xml_doc.FirstChildElement("OpenDRIVE");
        if (od == nullptr)
            throw std::runtime_error("OpenDRIVE element bot found");

        // parse OpenDRIVE element
        data.OpenDRIVE1_5 = std::make_shared<odr_1_5::OpenDRIVE>();
        __parse__OpenDRIVE(od, *data.OpenDRIVE1_5);

    }

}