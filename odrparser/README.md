# odrparser
A very simple OpenDRIVE parser

[![Build Status](https://travis-ci.com/JensKlimke/odrparser.svg?branch=master)](https://travis-ci.com/JensKlimke/odrparser)

## Motivation

There are several self-driven parser for OpenDRIVE. The problem with that is, that features are missing or missinterpreted. I used a script to parse the scheme file available on the [OpenDRIVE website](http://www.opendrive.org/download.html) and auto-generate code from that scheme. The output is a very simple header file. 

What it is: It's a parser (a XML file content to data structure in memory converter) for the OpenDRIVE specification 1.5 Revision M. 

What it isn't: It's not a OpenDRIVE interpreter: It does not link IDs with it associated objects. It is also not an OpenDRIVE generator (currently you cannot save the content to an XML file.). It is also not a parser for any other version of OpenDRIVE (note that OpenDRIVE 1.6 became an official standard by [ASAM e.V.](https://www.asam.net/standards/detail/opendrive/))

ODR header file: ```include/odr/odr1_5_structure.h```. 

## Installation

Just add_subdirectory it in cmake if you want to use the library in your project:

    // CmakeLists.txt
    ...
    add_subdirectory(<path to library root>)
    ...
    
You don't need to set any options. If the ``BUILD_TESTS`` option is set, you should unset it before adding the subdirectory. The ``odrparser`` target is a static library.

To create and run the tests, use cmake in the terminal:

    > cd <source folder>
    > mkdir build && cd build
    > cmake -DBUILD_TESTS=ON ..
    > make 
    > make test
    ...
    
To create the tests set ``-DBUILD_TESTS=ON``. Googletest need to be installed on your system. If you don't have googletest clone googletest (https://github.com/google/googletest.git) into the root folder of the project and set the option ``-DBUILD_GTESTS=ON``. 
    
## Usage

To access the content of the data structure, you can use the function ``odr::loadFile(<filename>, <container>)``. The function will load the ODR content from the file into the container, which shall be an instance of the class ``odr::OpenDRIVEFile``. Of course you can derive your own class, instantiate it and use it as the container.

Sample code:

    // create container instance
    odr::OpenDRIVEFile odrData;
    
    // load xml file content to container (replace <...> by the file name)
    odr::loadFile(<filename.xml>, odrData); 
    
    // pointer to the ODR data
    odr1_5::OpenDRIVE *odrr = odrFile.OpenDRIVE1_5.get();
   
    // access the header
    const auto header = odrr->sub_header.get();
    std::cout << *header->_date << std::endl; // e.g. "Thu Feb  8 14:24:06 2007"
    
    // access the roads vector
    const auto &roads = odrr->sub_road;
    std::cout << roads.size() << std::endl; // e.g. 36
    
    // access a single road content
    const auto &rd = odrr->sub_road.front();
    std::cout << rd.sub_lanes->sub_laneSection.size() << std::endl; // e.g. 1
    
To learn more about the OpenDRIVE structure read [this](http://www.opendrive.org/docs/OpenDRIVEFormatSpecRev1.5M.pdf).

## Used Libraries

* TinyXML2 - TinyXML2 is a simple, small, efficient, C++ XML parser that can be easily integrated into other programs. (https://github.com/leethomason/tinyxml2, )
* Googletest - Google Testing and Mocking Framework (https://github.com/google/googletest.git)