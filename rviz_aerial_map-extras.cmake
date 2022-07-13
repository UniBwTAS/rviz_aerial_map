# find package Qt5 because otherwise using the rviz_aerial_map::rviz_aerial_map
# exported target will complain that the Qt5::Widgets target does not exist
find_package(Qt5 REQUIRED QUIET COMPONENTS Widgets Core Network Concurrent)
