# Copyright 2011,2012 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})
list(APPEND plc_sources
    mac_receiver_impl.cc
    mac_transmitter_impl.cc
    phy_impl.cc
    app_out_impl.cc
    app_in_impl.cc
    phy_tx_impl.cc
    phy_rx_impl.cc )

set(plc_sources "${plc_sources}" PARENT_SCOPE)
if(NOT plc_sources)
	MESSAGE(STATUS "No C++ sources... skipping lib/")
	return()
endif(NOT plc_sources)

add_library(gnuradio-plc SHARED ${plc_sources})
target_link_libraries(gnuradio-plc ${Boost_LIBRARIES} ${GNURADIO_ALL_LIBRARIES} lightplc)
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
    ${CMAKE_CURRENT_SOURCE_DIR}/test_plc.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/qa_plc.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/qa_app_in.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/qa_app_out.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/qa_phy.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/qa_mac_transmitter.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/qa_mac_receiver.cc
    )

add_executable(test-plc ${test_plc_sources})

target_link_libraries(
  test-plc
  ${GNURADIO_RUNTIME_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CPPUNIT_LIBRARIES}
  lightplc
  itpp
 )

# GR_ADD_TEST(test_plc test-plc)
