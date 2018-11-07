cmake_minimum_required( VERSION 2.8 )
project ( slam_lesson4 )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

############### dependencies ######################
# Eigen
include_directories( "/usr/include/eigen3" )
# OpenCV
set(OpenCV_DIR /usr/local/opencv3/share/OpenCV)
find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Sophus 
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
#Pangolin
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )


set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
)
############### source and test ######################
include_directories( ${PROJECT_SOURCE_DIR}/include )

add_executable( gaussnewton gaussnewton.cpp )
target_link_libraries( gaussnewton ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})

add_executable( disparity disparity.cpp )
target_link_libraries( disparity ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})
