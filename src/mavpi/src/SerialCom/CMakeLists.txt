cmake_minimum_required(VERSION 2.8.3)
project(SerialCom)
find_package(Boost REQUIRED COMPONENTS
  system
)
include_directories(
  ${Boost_INCLUDE_DIR}
)

add_executable(
          SerialCom
          test.cpp
          ../protocol.c
#          ../SerialCom.cpp
          ../serial.cpp
          ../timeout.cpp
        )

target_link_libraries(SerialCom
  ${Boost_LIBRARIES}
)
