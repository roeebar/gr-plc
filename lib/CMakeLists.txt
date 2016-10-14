#
# Gr-plc - IEEE 1901 module for GNU Radio
# Copyright (C) 2016 Roee Bar <roeeb@ece.ubc.ca>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

# gnuradio-plc
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})
list(APPEND plc_sources
    phy_tx_impl.cc
    phy_rx_impl.cc
    app_out_impl.cc
    app_in_impl.cc
    impulse_source_impl.cc
)

set(plc_sources "${plc_sources}" PARENT_SCOPE)
if(NOT plc_sources)
	MESSAGE(STATUS "No C++ sources... skipping lib/")
	return()
endif(NOT plc_sources)

list(APPEND plc_libs
  lightplc
  ${GNURADIO_ALL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(gnuradio-plc SHARED ${plc_sources})
target_link_libraries(gnuradio-plc ${plc_libs})
set_target_properties(gnuradio-plc PROPERTIES DEFINE_SYMBOL "gnuradio_plc_EXPORTS")

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
endif()

if(APPLE)
    set_target_properties(gnuradio-plc PROPERTIES
        INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    )
endif(APPLE)

########################################################################
# Install built library files
########################################################################
install(TARGETS gnuradio-plc
    LIBRARY DESTINATION lib${LIB_SUFFIX} # .so/.dylib file
    ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
    RUNTIME DESTINATION bin              # .dll file
)

########################################################################
# Build and register unit test
########################################################################
include(GrTest)

include_directories(${CPPUNIT_INCLUDE_DIRS})
list(APPEND test_plc_sources
    )
