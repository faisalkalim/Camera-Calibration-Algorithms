cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 14)
SET( CMAKE_BUILD_TYPE debug)

project( Charuco )
find_package( OpenCV REQUIRED )

set(Sources

		detect_board_charuco.cpp
)

SET(${PROJECT_NAME}_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  )

add_executable( ${PROJECT_NAME} ${Sources})




target_include_directories(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC OpenCV )


target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS}) 
