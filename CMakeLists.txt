cmake_minimum_required(VERSION 3.0.2)
project(async_comm)

if (NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif (NOT DEFINED CMAKE_BUILD_TYPE)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads)

include_directories(
  ${Boost_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# examples
option(ASYNC_COMM_BUILD_EXAMPLES "Build examples" ON)

add_executable(udp_hello_world examples/udp_hello_world.cpp)
target_link_libraries(udp_hello_world ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable(serial_loopback examples/serial_loopback.cpp)
target_link_libraries(serial_loopback ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable(serial_protocol examples/serial_protocol.cpp)
target_link_libraries(serial_protocol ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable(tcp_client_hello_world examples/tcp_client_hello_world.cpp)
target_link_libraries(tcp_client_hello_world ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
