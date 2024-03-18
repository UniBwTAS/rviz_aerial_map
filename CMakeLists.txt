cmake_minimum_required(VERSION 3.1)
project(rviz_aerial_map)

set(RVIZ_AERIAL_MAP_VERSION "1.0.0")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rviz
    sensor_msgs
    ogre_primitives
    tas_proj)

# System dependencies
find_package(GDAL)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES map_helper
    CATKIN_DEPENDS tas_proj
    DEPENDS GDAL)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GDAL_INCLUDE_DIRS}
)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if (rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui QtNetwork)
    ## pull in all required include dirs, define QT_LIBRARIES, etc.
    include(${QT_USE_FILE})
else ()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets Network)
    ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
    set(QT_LIBRARIES Qt5::Widgets Qt5::Network)
endif ()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

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
add_definitions("-DRVIZ_AERIAL_MAP_VERSION=\"\\\"${RVIZ_AERIAL_MAP_VERSION}\\\"\"")

add_library(${PROJECT_NAME}
    src/aerial_map_display.cpp
    src/vis_helpers/image_downloader.cpp
    src/vis_helpers/texture_cache.cpp
    )
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} map_helper)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
