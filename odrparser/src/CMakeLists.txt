add_library(odrparser STATIC
        odr_1_5.cc
        odr_1_5.hpp
        loadFile.cc
        )

# include directories
target_include_directories(odrparser
        PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        PRIVATE
        ${tinyxml_SOURCE_DIR}
        )

# link tinyxml2
target_link_libraries(odrparser PRIVATE tinyxml2)

# set public headers
file(GLOB_RECURSE HEADER_INCLUDE_FILES "../include/odrparser/*.h")
set_target_properties(PROPERTIES
        PUBLIC_HEADER "${HEADER_INCLUDE_FILES}"
        )

# installation
include(GNUInstallDirs)

# install commands
install(TARGETS odrparser
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        COMPONENT library)
install(FILES ${HEADER_INCLUDE_FILES}
        DESTINATION include/odrparser)