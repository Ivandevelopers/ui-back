# cmake_minimum_required(VERSION 3.10.2)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# include_directories(/home/mazurak/mavlink/generated/include/mavlink/v2.0)

# project(main)

# add_executable(autopilot_control
#     src/main.cpp
#     src/autopilot_params.cpp
#     src/autopilot_mission.cpp
#     src/autopilot_sensors_calibration.cpp
# )

# find_package(MAVSDK REQUIRED)
# find_package(Threads REQUIRED)

# target_link_libraries(autopilot_control
#     MAVSDK::mavsdk
#     Threads::Threads
# )

# if(NOT MSVC)
#     add_compile_options(autopilot_control PRIVATE -Wall -Wextra)
# else()
#     add_compile_options(autopilot_control PRIVATE -WX -W2)
# endif()


cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Назва проекту
project(autopilot_control)

set(HOME_DIR $ENV{HOME})

set(MAVLINK_INCLUDE_DIR "${HOME_DIR}/mavlink/generated/include/mavlink/v2.0")

# Перевірка на існування заголовкових файлів MAVLink
if(NOT EXISTS ${MAVLINK_INCLUDE_DIR}/ardupilotmega/ardupilotmega.h)
    message(FATAL_ERROR "MAVLink header file not found in ${MAVLINK_INCLUDE_DIR}!")
endif()

include_directories(${MAVLINK_INCLUDE_DIR})

# Встановити шлях до заголовкових файлів MAVSDK
find_package(MAVSDK REQUIRED)
include_directories(${MAVSDK_INCLUDE_DIRS})

# Додати джерельні файли проекту
add_executable(autopilot_control
    src/main.cpp
    src/autopilot_params.cpp
    src/autopilot_mission.cpp
    src/autopilot_sensors_calibration.cpp
)

# Знайти пакети для потоків
find_package(Threads REQUIRED)

# Зв'язати необхідні бібліотеки
target_link_libraries(autopilot_control
    MAVSDK::mavsdk
    Threads::Threads
)

# Додати опції компіляції
if(NOT MSVC)
    target_compile_options(autopilot_control PRIVATE -Wall -Wextra)
else()
    target_compile_options(autopilot_control PRIVATE -WX -W2)
endif()
