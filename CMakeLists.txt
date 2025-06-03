cmake_minimum_required(VERSION 3.10)

project(cartesio_xbot2)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(XBOT2_ENABLE_XENO OFF "Compile against xenomai")

find_package(xbot2 REQUIRED)
find_package(cartesian_interface REQUIRED)

# cartesio RT
add_xbot2_plugin(cartesio_rt
    src/cartesio_rt.cpp)

target_link_libraries(cartesio_rt PRIVATE ${cartesian_interface_LIBRARIES})

install(
    TARGETS cartesio_rt
    DESTINATION lib
    )

add_executable(test_deserialize src/test_deserialize.cpp)

install(TARGETS test_deserialize
        DESTINATION bin)

install(FILES serialized.txt 
        DESTINATION bin) 
        
add_executable(test_sender_udp src/test_sender_udp.cpp)

install(TARGETS test_sender_udp
        DESTINATION bin)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(test_deserialize_udp src/test_deserialize_udp.cpp)

target_link_libraries(test_deserialize_udp
  ${catkin_LIBRARIES}
)

install(TARGETS test_deserialize_udp
        DESTINATION bin)