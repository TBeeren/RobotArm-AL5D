cmake_minimum_required(VERSION 3.0.0)
project(OneDoFDrivers VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(OneDoFDrivers main.cpp CRobotHighLevel.cpp CRobotLowLevel.cpp)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
