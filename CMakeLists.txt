cmake_minimum_required(VERSION 2.8)

project(BaslerCameraHardTrigger)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#OpenCV (here we need is OpenCV3)
Message("Checking for OpenCV3 lib :")
#in case you did not install opencv in default directory, please specify exact location
find_package(OpenCV 3 REQUIRED)
if(OpenCV_FOUND)
    Message("OpenCV has Found")
    message("OpenCV Version " ${OpenCV_VERSION})
    include_directories(${OpenCV_INCLUDE_DIRS})
else(OpenCV_FOUND)
    Message(FATAL_ERROR "OpenCV did not found")
endif(OpenCV_FOUND)


# Use Basler Pylon API for Basler camera
# The Basler Pylon API must be installed on your system
# The Path to Pylon should be :
# Include directory should be : /path/to/pylon5/include
# Libraries : /path/to/pylon5/lib64
MESSAGE("Trying to find Pylon API for Basler Cameras")
OPTION(USE_BASLER "If we use a Basler camera , we should find the Pylon API" ON)
#Pylon Libraries:A Basler Camera
#if(USE_BASLER)
#    set(BASLER_PYLON_INCLUDE_PATH /opt/pylon5/include)
#    set(BASLER_PYLON_LIBRARY -L/opt/pylon5/lib64 -Wl,-E -lpylonbase -lpylonutility -lGenApi_gcc_v3_1_Basler_pylon_v5_1 -lGCBase_gcc_v3_1_Basler_pylon_v5_1)
#    INCLUDE_DIRECTORIES(${BASLER_PYLON_INCLUDE_PATH})
#endif(USE_BASLER)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} SHARED
src/camthread.cpp
include/camthread.h
)

#Basler camera grabber library
#add_library(CameraGrabberBasler "CameraGrabberBasler.h" "CameraGrabberBasler.cpp")

#add_executable(${PROJECT_NAME} "ImageGrabController.cpp" )
#target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} CameraGrabberBasler ${BASLER_PYLON_LIBRARY})

#add_library(Grab "./include/camthread.h" "./src/camthread.cpp")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Grab)
add_executable(Grab Examples/Grab.cpp )
target_link_libraries(Grab Threads::Threads )
target_link_libraries(Grab ${BASLER_PYLON_LIBRARY} ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
)
# 包含头文件
include_directories(/opt/pylon5/include)
link_directories(/opt/pylon5/lib64)

# target_link
target_link_libraries(Grab
            ${OpenCV_LIBS}
            X11
            /opt/pylon5/lib64/libpylonbase.so
            /opt/pylon5/lib64/libpylonutility.so
            /opt/pylon5/lib64/libGCBase_gcc_v3_1_Basler_pylon_v5_1.so
            /opt/pylon5/lib64/libGenApi_gcc_v3_1_Basler_pylon_v5_1.so
)
