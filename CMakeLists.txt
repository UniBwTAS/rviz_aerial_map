cmake_minimum_required(VERSION 3.0.2)
project(rviz_aerial_map)

set(RVIZ_AERIAL_MAP_VERSION "1.0.0")

# ROS dependencies
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rviz
    sensor_msgs
    ogre_primitives
    tas_proj)

# System dependencies
find_package(GDAL)
find_package(Boost REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Network Concurrent REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OGRE_OV OGRE OGRE-Overlay)
# Old versions of OGRE (pre 1.9) included OGRE-Overlay in the main package
# (i.e. there was no OGRE-Overlay component).  So if the above
# pkg_check_modules() failed, try looking for just OGRE.
if (NOT OGRE_OV_FOUND)
    pkg_check_modules(OGRE_OV REQUIRED OGRE)
endif ()

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES map_helper
    CATKIN_DEPENDS tas_proj
    DEPENDS Qt5Core GDAL Boost)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Network_INCLUDE_DIRS}
    ${Qt5Concurrent_INCLUDE_DIRS}
    ${GDAL_INCLUDE_DIRS}
    ${BOOST_INCLUDE_DIRS}
    ${OGRE_OV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Build map helper library
add_library(map_helper
    src/map_helpers/tiff_tile_helper.cpp
    src/map_helpers/osm_tile_helper.cpp
    )
target_link_libraries(map_helper
    ${catkin_LIBRARIES}
    ${GDAL_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${BOOST_LIBRARIES}
    )

# Build rviz plugin library
add_definitions("-Wall -Wunused -std=c++11 -DRVIZ_AERIAL_MAP_VERSION=\"\\\"${RVIZ_AERIAL_MAP_VERSION}\\\"\"")
set(${PROJECT_NAME}_HEADERS
    src/aerial_map_display.h
    src/vis_helpers/image_downloader.h
    )
qt5_wrap_cpp(${PROJECT_NAME}_MOCSrcs ${${PROJECT_NAME}_HEADERS})

add_library(${PROJECT_NAME}
    src/aerial_map_display.cpp
    src/vis_helpers/image_downloader.cpp
    src/vis_helpers/texture_cache.cpp
    ${${PROJECT_NAME}_MOCSrcs}
    )
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Gui_LIBRARIES}
    ${Qt5Network_LIBRARIES}
    ${Qt5Concurrent_LIBRARIES}
    map_helper
    )

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )

install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
