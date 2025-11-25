cmake_minimum_required(VERSION 3.8)
project(address_value)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "address_value::AddressValue"
    EXECUTABLE address_value_node
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
