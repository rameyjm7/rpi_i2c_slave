 cmake_minimum_required(VERSION 2.8)
set(NAME rpi_i2c_slave)
project(${NAME} CXX)
set(CMAKE_CXX_STANDARD 14)

LINK_DIRECTORIES(
)

set(INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(SRC
	src/main.cpp
)

set(LIBS pigpio pthread)

include_directories(${INCLUDES})
add_executable(${NAME}  ${SRC})
target_link_libraries(${NAME}  ${LIBS} )
