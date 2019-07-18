cmake_minimum_required(VERSION 3.3.0)

message("Camera NDI Registration")
project(CameraRegistration)

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(ndicapi REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)


# Define and configure the targets
set(PLUGIN_SOURCES
	charuco.cpp
	Maxer_NDI_Tracker.cpp
)

set(PLUGIN_HEADERS
	ndicapimaxer.h
	Maxer_NDI_Tracker.h

)




file(GLOB eigen_files
	"Eigen/*"
	"Eigen/src/*"
)

add_executable(exec ${PLUGIN_SOURCES})
target_sources(exec PUBLIC ${PLUGIN_SOURCES} ${PLUGIN_HEADERS}  ${eigen_files} )
target_include_directories(exec PUBLIC include
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
	${OpenCV_LIBS}
	include/Eigen
)

target_link_libraries(exec  ${OpenCV_LIBS} ndicapi)
