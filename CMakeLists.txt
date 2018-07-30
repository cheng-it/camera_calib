cmake_minimum_required(VERSION 2.8)
project(camera_calib)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

#set(CMAKE_PREFIX_PATH "/usr/local/opencv-3.1.0/share/OpenCV")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(HALCONROOT /home/jackymond/software/halcon)

find_package(Boost COMPONENTS filesystem regex REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenCV 3.1.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${HALCONROOT}/include
  ${HALCONROOT}/include/halconcpp
)
link_directories(
  /usr/local/lib
  ${PROJECT_SOURCE_DIR}/lib
  ${HALCONROOT}/lib/x64-linux
)

set(HALCON_LIBS 
	halconcpp
	halcon
	X11
)

add_executable(monocular_calibration
	src/monocular_calibration.cpp
	src/halcon_wrapper/cv_convert.cpp
	src/halcon_wrapper/calibrate.cpp
)

target_link_libraries(monocular_calibration
	${OpenCV_LIBS}
	#${Boost_LIBS}
	${HALCON_LIBS}
)

add_executable(planar_pose_extraction
	src/planar_pose_extraction.cpp
	src/halcon_wrapper/cv_convert.cpp
	src/halcon_wrapper/calibrate.cpp
)
target_link_libraries(planar_pose_extraction
	${OpenCV_LIBS}
	${Boost_LIBS}
	${HALCON_LIBS}
)

add_subdirectory(src/halcon_wrapper)
#add_subdirectory(include/cv_calib)
add_subdirectory(include/kalibr)
add_subdirectory(include/utils)