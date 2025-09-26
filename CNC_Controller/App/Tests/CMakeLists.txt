cmake_minimum_required(VERSION 3.15)
project(stm32_unit_tests C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_compile_options(
    -Wall -Wextra -Wpedantic
    $<$<C_COMPILER_ID:GNU>:-Wno-unused-parameter>
)

# This CMake lives in CNC_Controller/App/Tests
set(APP_DIR  ${CMAKE_CURRENT_LIST_DIR}/..)
set(APP_INC  ${APP_DIR}/Inc)
set(APP_SRC  ${APP_DIR}/Src)

include_directories(${APP_INC})

add_executable(unit_tests
    test_protocol.c
    ${APP_SRC}/Protocol/Requests/move_home_request.c
    ${APP_SRC}/Protocol/Requests/move_probe_level_request.c
    ${APP_SRC}/Protocol/Requests/move_queue_add_request.c
    ${APP_SRC}/Protocol/Requests/led_control_request.c
    ${APP_SRC}/Protocol/Requests/move_end_request.c
    ${APP_SRC}/Protocol/Requests/move_queue_status_request.c
    ${APP_SRC}/Protocol/Requests/start_move_request.c

    ${APP_SRC}/Protocol/Responses/move_queue_add_ack_response.c
    ${APP_SRC}/Protocol/Responses/move_queue_status_response.c
    ${APP_SRC}/Protocol/Responses/move_home_response.c
    ${APP_SRC}/Protocol/Responses/move_probe_level_response.c
    ${APP_SRC}/Protocol/Responses/led_control_response.c
    ${APP_SRC}/Protocol/Responses/move_end_response.c
    ${APP_SRC}/Protocol/Responses/start_move_response.c

    ${APP_SRC}/Protocol/router.c
    ${APP_SRC}/app_spi_handshake.c
)

enable_testing()
add_test(NAME unit_tests COMMAND unit_tests)
