cmake_minimum_required(VERSION 3.10)
project(RTK_NAV C)

set(CMAKE_C_STANDARD 11)
set(BUILD_ROOT ${CMAKE_HOME_DIRECTORY})
message(STATUS "BUILD_ROOT: ${BUILD_ROOT}")
include_directories(${BUILD_ROOT} /usr/local/include)


add_executable(rtk-navication main.c
                transfer/corsclient.c transfer/serial.c  transfer/command.c transfer/udp_piece.c
                msg/msg_rtk.c msg/msg_cors.c msg/msg_trace.c
                control/control.c msg/time_runner.c utility/mem.c msg/nema.c
                task/task.c utility/log.c utility/cmath.c)
target_link_libraries(rtk-navication pthread m)
