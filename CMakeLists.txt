cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(main)

add_executable(autopilot_control
    src/main.cpp
    src/autopilot_params.cpp
    src/autopilot_mission.cpp
    src/timer.cpp
)

find_package(MAVSDK REQUIRED)
find_package(Threads REQUIRED)

target_link_libraries(autopilot_control
    MAVSDK::mavsdk
    Threads::Threads
)

if(NOT MSVC)
    add_compile_options(autopilot_control PRIVATE -Wall -Wextra)
else()
    add_compile_options(autopilot_control PRIVATE -WX -W2)
endif()
