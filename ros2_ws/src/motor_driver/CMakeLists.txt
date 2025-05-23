cmake_minimum_required(VERSION 3.5)
project(motor_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(soem REQUIRED)
# find_package(xenomai REQUIRED)

# include directories
include_directories(include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/src)
include_directories(/usr/include/soem)

# Setting for Xenomai
execute_process(COMMAND /usr/xenomai/bin/xeno-config --posix --alchemy --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND /usr/xenomai/bin/xeno-config --posix --alchemy --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}${XENO_CFLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${XENO_LDFLAGS}")

# build executable
add_executable(combine_test src/combine_test.cpp src/Humanoid_EtherCAT_DC.cpp)

target_link_libraries(combine_test 
  alchemy
  cobalt
  copperplate
  soem
)


install(TARGETS combine_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
