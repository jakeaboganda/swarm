cmake_minimum_required(VERSION 3.18)
project(opencardynamics_dt_fmu LANGUAGES CXX)

# Trimmed double-track OCD-FMU build: compiles ONLY the concrete combo's source
# subset (WHEEL_TORQUE x PT1 x DOUBLE_TRACK x MF52 x DEFAULT aero) plus the fmi3
# wrapper. Excludes ccma/track/vehicle_handler (Boost) and matplotlib/ROS2.
# Eigen is the only third-party dependency. Double-track VD is header-only.

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(OCD_ROOT "" CACHE PATH "Path to the Open-Car-Dynamics checkout")
set(EIGEN_INCLUDE "" CACHE PATH "Path to the Eigen include dir (the one CONTAINING eigen3/)")
if(NOT OCD_ROOT OR NOT EIGEN_INCLUDE)
  message(FATAL_ERROR "Set -DOCD_ROOT=<checkout> and -DEIGEN_INCLUDE=<dir containing eigen3/>")
endif()

set(TAM ${OCD_ROOT}/submodules/tam__common)
set(PMG ${TAM}/submodules/parameter_management/param_management_cpp)

file(GLOB OCD_LIB_INCLUDES ${OCD_ROOT}/lib/*/include)

set(OCD_SOURCES
  ${OCD_ROOT}/lib/ocd_drivetrain_wheel_torque_cpp/src/drivetrain_direct_torque_model.cpp
  ${OCD_ROOT}/lib/ocd_drivetrain_wheel_torque_cpp/src/drivetrain_direct_torque_eqns.cpp
  ${OCD_ROOT}/lib/ocd_steering_actuator_pt1_cpp/src/steering_actuator_pt1_model.cpp
  ${OCD_ROOT}/lib/ocd_tire_models_cpp/src/mf_52.cpp
  ${OCD_ROOT}/lib/ocd_aerodynamics_models_cpp/src/default.cpp
  ${OCD_ROOT}/lib/ocd_helpers_cpp/src/time_delay.cpp
  ${PMG}/src/param_value_manager.cpp
  ${PMG}/src/param_reference_manager.cpp
  ${PMG}/src/param_manager_composer.cpp
  ${PMG}/src/initialization.cpp
  ${PMG}/src/init_backend.cpp
)

add_library(opencardynamics_dt SHARED wrapper.cpp ${OCD_SOURCES})
set_target_properties(opencardynamics_dt PROPERTIES PREFIX "" OUTPUT_NAME "opencardynamics_dt")

target_include_directories(opencardynamics_dt PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/fmi3
  ${OCD_LIB_INCLUDES}
  ${PMG}/include
  ${TAM}/submodules/time_series_logging/tsl_logger_cpp/include
  ${TAM}/tum_types_cpp/include
  ${TAM}/tum_helpers_cpp/include
  ${EIGEN_INCLUDE}
  ${EIGEN_INCLUDE}/eigen3
)

add_executable(harness harness.cpp)
target_include_directories(harness PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/fmi3)
target_link_libraries(harness ${CMAKE_DL_LIBS})
