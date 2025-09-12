cmake_minimum_required(VERSION 3.8)
project(elfin_robot_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#    add_compile_options(-Wall -Wextra -Wpedantic -Wno-error)
#endif()

set(DEPENDENCIES
    rclcpp
    pluginlib
    hardware_interface
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${DEPENDENCIES})
    find_package(${Dependency} REQUIRED)
endforeach()

add_library(elfin_hardware_interface SHARED src/elfin_hardware_interface.cpp)

target_include_directories(elfin_hardware_interface PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/elfin_sdk_vendor/include>)
target_link_directories(elfin_hardware_interface PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/elfin_sdk_vendor/lib>)
target_link_libraries(elfin_hardware_interface PUBLIC HR_Pro.so)

ament_target_dependencies(elfin_hardware_interface PUBLIC ${DEPENDENCIES})

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

install(
    TARGETS elfin_hardware_interface
    EXPORT export_elfin_hardware_interface
    LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY elfin_sdk_vendor/lib/ DESTINATION lib)

ament_export_targets(export_elfin_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
