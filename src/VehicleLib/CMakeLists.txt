cmake_minimum_required(VERSION 3.5)

# set include directories
include_directories(${ODEDIR}/include
	${ODEDIR}/ode/src)

add_library(VehicleLib STATIC
	vmCar.cpp
	vmWishboneCar.cpp)

set_target_properties(VehicleLib
PROPERTIES
ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

target_link_libraries(VehicleLib debug ${ODEDIR}/lib/Debug/oded.lib)
target_link_libraries(VehicleLib optimized ${ODEDIR}/lib/Release/ode.lib)
