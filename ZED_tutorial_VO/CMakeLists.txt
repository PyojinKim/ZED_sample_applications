SET(execName ZED_tutorial_VO)
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")    
SET(VERSION_REQ_OCV "3.1")
SET(VERSION_REQ_CUDA "8.0")
SET(SRC_FOLDER src)


##For Jetson, OpenCV4Tegra is based on OpenCV2.4
exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" )
   SET(VERSION_REQ_OCV "2.4")
   SET(VERSION_REQ_CUDA "")

   unset(CUDA_USE_STATIC_CUDA_RUNTIME CACHE)
   option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
   message ("CUDA_USE_STATIC_CUDA_RUNTIME : ${CUDA_USE_STATIC_CUDA_RUNTIME}")
   ##to prevent from opencv_dep_cudart dependencies error...
   ## cmake with -DCUDA_USE_STATIC_CUDA_RUNTIME=false can also be called.
endif()


# Add plain cmake packages
find_package(ZED 2.0 REQUIRED)
find_package(OpenCV ${VERSION_REQ_OCV} REQUIRED)
find_package(CUDA ${VERSION_REQ_CUDA} REQUIRED)


# Include dirs
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


# Set link libraries
link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})


# Set sourcefiles
FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/*.cpp")


################################################################################
ADD_EXECUTABLE(${execName} ${SRC_FILES})
add_definitions(-std=c++0x -g -O3)
TARGET_LINK_LIBRARIES(${execName}
                        ${ZED_LIBRARIES}
                        ${OpenCV_LIBRARIES}
			${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
                    )
