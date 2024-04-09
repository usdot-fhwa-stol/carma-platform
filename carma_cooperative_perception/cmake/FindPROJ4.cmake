# BSD 3-Clause License
#
# Copyright (c) 2009, Mateusz Loskot <mateusz@loskot.net>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

###############################################################################
# CMake module to search for PROJ.4 library
#
# On success, the macro sets the following variables:
# PROJ4_FOUND       = if the library found
# PROJ4_VERSION     = version number of PROJ.4 found
# PROJ4_LIBRARIES   = full path to the library
# PROJ4_INCLUDE_DIR = where to find the library headers
# Also defined, but not for general use are
# PROJ4_LIBRARY, where to find the PROJ.4 library.
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#
###############################################################################

# lint_cmake: -linelength
# Reference: https://raw.githubusercontent.com/mloskot/cmake-modules/master/modules/FindPROJ4.cmake

# Try to use OSGeo4W installation
if(WIN32)
  set(PROJ4_OSGEO4W_HOME "C:/OSGeo4W")

  if($ENV{OSGEO4W_HOME})
    set(PROJ4_OSGEO4W_HOME "$ENV{OSGEO4W_HOME}")
  endif()
endif()

find_path(PROJ4_INCLUDE_DIR proj_api.h
  PATHS ${PROJ4_OSGEO4W_HOME}/include
  DOC "Path to PROJ.4 library include directory")

if(PROJ4_INCLUDE_DIR)
  # Extract version from proj_api.h (ex: 480)
  file(STRINGS ${PROJ4_INCLUDE_DIR}/proj_api.h
    PJ_VERSIONSTR
    REGEX "#define[ ]+PJ_VERSION[ ]+[0-9]+")
  string(REGEX MATCH "[0-9]+" PJ_VERSIONSTR ${PJ_VERSIONSTR})

  # TODO: Should be formatted as 4.8.0?
  set(PROJ4_VERSION ${PJ_VERSIONSTR})
endif()

set(PROJ4_NAMES ${PROJ4_NAMES} proj proj_i)
find_library(PROJ4_LIBRARY
  NAMES ${PROJ4_NAMES}
  PATHS ${PROJ4_OSGEO4W_HOME}/lib
  DOC "Path to PROJ.4 library file")

if(PROJ4_LIBRARY)
  set(PROJ4_LIBRARIES ${PROJ4_LIBRARY})
endif()

# Handle the QUIETLY and REQUIRED arguments and set PROJ4_FOUND to TRUE
# if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PROJ4 DEFAULT_MSG
  PROJ4_LIBRARY
  PROJ4_INCLUDE_DIR)
