# libodcantools - Library to wrap a CAN interface.
# Copyright (C) 2015 Christian Berger
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

CMAKE_MINIMUM_REQUIRED (VERSION 2.8)

PROJECT (librplidarsdk)

# Set include directory.
INCLUDE_DIRECTORIES(sdk/include)
INCLUDE_DIRECTORIES(sdk/src)

SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -Wall")
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wall -pipe")

SET(librplidarsdk_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/sdk")

SET (librplidarsdk-sources
    ${librplidarsdk_SOURCE_DIR}/src/rplidar_driver.cpp
    ${librplidarsdk_SOURCE_DIR}/src/arch/linux/net_serial.cpp
    ${librplidarsdk_SOURCE_DIR}/src/arch/linux/timer.cpp
    ${librplidarsdk_SOURCE_DIR}/src/hal/thread.cpp
)

###############################################################################
# Resulting artifacts.
ADD_LIBRARY (rplidarsdk-core   OBJECT ${librplidarsdk-sources})
ADD_LIBRARY (rplidarsdk-static STATIC $<TARGET_OBJECTS:rplidarsdk-core>)
ADD_LIBRARY (rplidarsdk        SHARED $<TARGET_OBJECTS:rplidarsdk-core>)

