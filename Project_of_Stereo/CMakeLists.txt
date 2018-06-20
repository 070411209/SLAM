cmake_minimum_required( VERSION 2.8 )
project( imageBasics )

# 添加c++ 11标准支持
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

# 寻找OpenCV库
find_package( OpenCV REQUIRED )
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )


# ./imageBasics 
# ORBextractor.h ORBextractor.cpp
add_executable( imageBasics main.cpp motract.h motract.cpp)

# 链接OpenCV库
target_link_libraries( imageBasics ${OpenCV_LIBS} )

# Debug
set( CMAKE_BUILD_TYPE "Debug" )