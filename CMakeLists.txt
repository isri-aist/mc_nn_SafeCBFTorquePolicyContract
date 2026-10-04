cmake_minimum_required(VERSION 3.10)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

project(mc_nn_SafeCBFTorquePolicyContract CXX)

if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

# mc_nn installation: -Dmc_nn_DIR=<mc_nn prefix>/lib/cmake/mc_nn or CMAKE_PREFIX_PATH.
# Defaults to an mc_nn built next to this directory (mcrtc workspace layout).
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../mc_nn/install")
find_package(mc_nn REQUIRED)

# Documentation and example policies (also keeps an install target when the
# contract itself is skipped).
install(FILES README.md LICENSE DESTINATION "share/${PROJECT_NAME}")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/policies/" DESTINATION "share/${PROJECT_NAME}/policies"
        FILES_MATCHING PATTERN "*.onnx" PATTERN "*.yaml")

# Needs the mc_rtc/TVM versions providing mc_tasks::TorqueJointTask and the
# CBF DynamicsConstraint (see README.md). Skipped otherwise.
# Installed in ${MC_RTC_LIBDIR}/mc_nn_contracts: list that folder in RunNN's contracts_dirs.
mc_nn_add_contract(mc_nn_SafeCBFTorquePolicyContract
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_nn_SafeCBFTorquePolicyContract.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_nn_SafeCBFTorquePolicyContract.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/observation/Observation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/observation/Observations.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/observation/ObservationManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/policy/PolicyConfig.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/policy/RLPolicyRuntime.cpp
  INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  REQUIRES_HEADERS
    mc_tasks/TorqueJointTask.h
)

if(TARGET mcnn_contract_mc_nn_SafeCBFTorquePolicyContract)
  # Optional joystick commands through mc_joystick_plugin (keyboard and GUI commands always work).
  get_target_property(MC_RTC_INCLUDE_DIRS mc_rtc::mc_control INTERFACE_INCLUDE_DIRECTORIES)
  find_path(MC_JOYSTICK_PLUGIN_INCLUDE_DIR mc_joystick_plugin/joystick_inputs.h
            PATHS ${MC_RTC_INCLUDE_DIRS} ${CMAKE_PREFIX_PATH} PATH_SUFFIXES include)
  if(MC_JOYSTICK_PLUGIN_INCLUDE_DIR)
    target_include_directories(mcnn_contract_mc_nn_SafeCBFTorquePolicyContract PRIVATE ${MC_JOYSTICK_PLUGIN_INCLUDE_DIR})
    target_compile_definitions(mcnn_contract_mc_nn_SafeCBFTorquePolicyContract PRIVATE MC_NN_HAS_JOYSTICK_PLUGIN)
    message(STATUS "mc_nn contract mc_nn_SafeCBFTorquePolicyContract: joystick support enabled")
  else()
    message(STATUS "mc_nn contract mc_nn_SafeCBFTorquePolicyContract: mc_joystick_plugin not found, joystick commands disabled")
  endif()
endif()
