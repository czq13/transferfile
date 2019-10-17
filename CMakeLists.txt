cmake_minimum_required (VERSION 2.6)

project (try_connect_mavlink)

include_directories(/home/czq/firmware/mavlink/include)
add_executable(try_connect_mavlink try_connect_mavlink.cpp)
