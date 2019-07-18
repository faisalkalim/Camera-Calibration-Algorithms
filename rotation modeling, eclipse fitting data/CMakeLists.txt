cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 14)
SET( CMAKE_BUILD_TYPE debug)
project( RotationDataIntrinsics )

# FIND_PACKAGE(Threads)
# LIST(APPEND ${PROJECT_NAME}_LIBS
#   ${CMAKE_THREAD_LIBS_INIT}
#   )
find_package(OpenCV REQUIRED)
set(Sources
		  rotation_intrinsics__data.cpp
)

SET(${PROJECT_NAME}_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  )

add_executable( RotationDataIntrinsics ${Sources})

target_include_directories(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC OpenCV)


target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})


#CONFIGURE_FILE(ndicapiConfig.cmake.in
 # "${CMAKE_BINARY_DIR}/ndicapiConfig.cmake" @ONLY)

#CONFIGURE_FILE(ndicapiConfigVersion.cmake.in
 # "${CMAKE_BINARY_DIR}/ndicapiConfigVersion.cmake" @ONLY)

#CONFIGURE_FILE(ndicapiExport.h.in
 # "${CMAKE_BINARY_DIR}/ndicapiExport.h" @ONLY)
 #Enable building of python module, configure the export file in the source directory as well
#CONFIGURE_FILE(ndicapiExport.h.in
 # "${CMAKE_SOURCE_DIR}/ndicapiExport.h" @ONLY)

#target_link_libraries( NDIAPIExample ${OpenCV_LIBS} )
#target_link_libraries( NDIAPIExample ${ImFusionLib} )

