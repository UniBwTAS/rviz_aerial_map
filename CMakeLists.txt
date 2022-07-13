cmake_minimum_required(VERSION 3.5)
project(rviz_aerial_map)

set(RVIZ_AERIAL_MAP_VERSION "1.0.0")
add_definitions("-DRVIZ_AERIAL_MAP_VERSION=\"\\\"${RVIZ_AERIAL_MAP_VERSION}\\\"\"")

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif ()

# We specifically don't turn on CMAKE_AUTOMOC, since it generates one huge
# mocs_compilation.cpp file that takes a lot of memory to compile.  Instead
# we create individual moc files that can be compiled separately.

find_package(ament_cmake REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Core Network Concurrent)

find_package(geometry_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tas_proj REQUIRED)
find_package(ogre_primitives REQUIRED)
find_package(GDAL)

# Build map helper library
add_library(map_helper SHARED
    src/map_helpers/tiff_tile_helper.cpp
    src/map_helpers/osm_tile_helper.cpp
    )
target_include_directories(map_helper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${Qt5Core_INCLUDE_DIRS}
    ${GDAL_INCLUDE_DIRS}
    )
target_link_libraries(map_helper PUBLIC
    ${GDAL_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    )
ament_target_dependencies(map_helper
    PUBLIC
    tas_proj
    )

set(rviz_aerial_map_headers_to_moc
    src/aerial_map_display.h
    src/vis_helpers/image_downloader.h
    )

foreach (header "${rviz_aerial_map_headers_to_moc}")
    qt5_wrap_cpp(rviz_aerial_map_moc_files "${header}")
endforeach ()

set(rviz_aerial_map_source_files
    src/aerial_map_display.cpp
    src/vis_helpers/image_downloader.cpp
    src/vis_helpers/texture_cache.cpp
    )

add_library(rviz_aerial_map SHARED
    ${rviz_aerial_map_moc_files}
    ${rviz_aerial_map_source_files}
    )

target_include_directories(rviz_aerial_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Network_INCLUDE_DIRS}
    ${Qt5Concurrent_INCLUDE_DIRS}
    )

target_link_libraries(rviz_aerial_map PUBLIC
    rviz_ogre_vendor::OgreMain
    rviz_ogre_vendor::OgreOverlay
    ${Qt5Core_LIBRARIES}
    ${Qt5Network_LIBRARIES}
    ${Qt5Concurrent_LIBRARIES}
    map_helper
    )

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(rviz_aerial_map
    PUBLIC
    rclcpp
    rviz_common
    rviz_ogre_vendor
    sensor_msgs
    ogre_primitives
    tas_proj
    tf2
    tf2_geometry_msgs
    tf2_ros
    )

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(rviz_aerial_map HAS_LIBRARY_TARGET)

ament_export_dependencies(
    rclcpp
    rviz_common
    rviz_ogre_vendor
    sensor_msgs
    ogre_primitives
    tas_proj
    rviz_rendering
    tf2
    tf2_geometry_msgs
    tf2_ros
)

install(
    TARGETS rviz_aerial_map
    EXPORT rviz_aerial_map
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    TARGETS map_helper
    EXPORT rviz_aerial_map
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

ament_package(
    CONFIG_EXTRAS "rviz_aerial_map-extras.cmake"
)

