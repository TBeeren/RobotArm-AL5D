cmake_minimum_required(VERSION 3.0.0)
project(RobotHighLevel VERSION 0.1.0)

include(CTest)
enable_testing()

SET(SOURCES
	CCalibration.cpp
	CConfiguration.cpp
	CMove.cpp
	CCalibration.cpp
	CConfiguration.cpp
)

add_library(RobotHighLevel ${SOURCES})
target_compile_options(RobotHighLevel PRIVATE -Wall -Wextra)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

