cmake_minimum_required(VERSION 3.16)
project(adcs_sil LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(FSW_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../FlightSoftware")

add_executable(
    sensor_receiver
    sil_connection.cpp
    sensor_receiver.cpp
    command_sender.cpp
    boot_config_receiver.cpp
    ${FSW_ROOT}/flight_software.cpp
    ${FSW_ROOT}/mode_director.cpp
    ${FSW_ROOT}/eps/eps_manager.cpp
    ${FSW_ROOT}/fdir/fdir_manager.cpp
    ${FSW_ROOT}/modes/detumble_mode.cpp
    ${FSW_ROOT}/modes/standby_mode.cpp
    ${FSW_ROOT}/modes/pointing_mode.cpp
    ${FSW_ROOT}/modes/safe_mode.cpp
    ${FSW_ROOT}/estimation/state_estimator.cpp
    ${FSW_ROOT}/estimation/orbit_propagator.cpp
    ${FSW_ROOT}/estimation/sun_model.cpp
    ${FSW_ROOT}/estimation/dipole_mag_model.cpp
    ${FSW_ROOT}/estimation/triad.cpp
    ${FSW_ROOT}/estimation/css_wls.cpp
    ${FSW_ROOT}/estimation/attitude_filter.cpp
    ${FSW_ROOT}/math/attitude.cpp
    ${FSW_ROOT}/control/pd_controller.cpp
    ${FSW_ROOT}/actuators/rw_mapper.cpp
    ${FSW_ROOT}/pointing/sun_reference.cpp
    ${FSW_ROOT}/pointing/nadir_reference.cpp
)

target_include_directories(
    sensor_receiver
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${FSW_ROOT}
        ${FSW_ROOT}/modes
        ${FSW_ROOT}/estimation
        ${FSW_ROOT}/control
        ${FSW_ROOT}/actuators
        ${FSW_ROOT}/pointing
)

if(WIN32)
    target_link_libraries(sensor_receiver PRIVATE ws2_32)
endif()
