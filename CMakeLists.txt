cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(Kinect_LM_PCL)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} "C:/Program Files/Microsoft SDKs/Kinect/v1.8/inc" "D:/Leap/LeapDeveloperKit/LeapSDK/include"
"D:/OpenCV/build/include")
link_directories(${PCL_LIBRARY_DIRS} "C:/Program Files/Microsoft SDKs/Kinect/v1.8/lib/amd64" "D:/Leap/LeapDeveloperKit/LeapSDK/lib/x64"
"D:/OpenCV/build/x64/vc10/lib")
add_definitions(${PCL_DEFINITIONS})

set(HEADER_FILES KLApplication.h KLCloud.h KAPI.h LListener.h KHandDetector.h LProcessor.h LeapToKinect.h)
add_executable (klApplication KLApplication.cpp KLCloud.cpp KAPI.cpp LListener.cpp KHandDetector.h LProcessor.cpp ${HEADER_FILES})
target_link_libraries (klApplication ${PCL_LIBRARIES} Kinect10.lib Leap.lib opencv_core248.lib opencv_imgproc248.lib opencv_highgui248.lib)