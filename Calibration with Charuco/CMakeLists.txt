cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 14)
SET( CMAKE_BUILD_TYPE debug)
project( CharucoCalibration )


find_package(ndicapi REQUIRED)
find_package(OpenCV REQUIRED)

file(GLOB eigen_files
	"Eigen/*"
	"Eigen/src/*"
)


set(Sources

		  Maxer_NDI_Tracker.cpp
		  calibrate_camera_charuco.cpp
)
set(Headers

		  Maxer_NDI_Tracker.h
)

SET(${PROJECT_NAME}_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  )

add_executable( ${PROJECT_NAME} ${Sources} ${Headers})

target_include_directories(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC OpenCV include/Eigen)

target_sources(${PROJECT_NAME}  PUBLIC  ${eigen_files} )

target_link_libraries(${PROJECT_NAME} PUBLIC ndicapi)

target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})

