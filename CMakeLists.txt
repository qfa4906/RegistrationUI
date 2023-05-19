cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0074 NEW)

project(RegistrationUI VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    add_definitions(-DQT_NO_DEBUG_OUTPUT)
    add_definitions(-DQT_FORCE_ASSERTS)
endif()
#set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/config")
find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED) 
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)
find_package(FLANN REQUIRED)

if(PCL_FOUND)
    add_definitions(-DPCL_DATA_TYPES_H_)
endif()

set(PROJECT_RESOURCE_FILE
    ./RegistrationDemo.qrc
)

set(PROJECT_SOURCES
        ./main.cpp
        ./Registration.cpp
        ./Registration.h
        ./RegistrationDemo.h
        ./RegistrationDemo.cpp
        ./RegistrationDemo.ui
        ./OPENGR/include/gr/algorithms/match4pcsBase.h
)


#include_directories(./CameraCalibration)
include_directories(./OPENGR/include)
if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(${PROJECT_NAME}
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET ImgPeaker APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(${PROJECT_NAME} SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(${PROJECT_NAME}
            ${PROJECT_SOURCES}
        )
    endif()
endif()
target_compile_options(RegistrationUI 
	PRIVATE 
	$<$<CXX_COMPILER_ID:MSVC>:/bigobj> ) 
target_link_libraries(${PROJECT_NAME} PRIVATE 
                        Qt${QT_VERSION_MAJOR}::Widgets
                        FLANN::FLANN
                        ${OpenCV_LIBS}
                        ${VTK_LIBRARIES}
                        ${PCL_COMMON_LIBRARIES}
                        ${PCL_FILTERS_LIBRARIES}
                        ${PCL_IO_LIBRARIES}
                        ${PCL_VISUALIZATION_LIBRARIES}
                        ${PCL_REGISTRATION_LIBARAIES}
                        ${PCL_KDTREE_LIBRARIES}
                        ${PCL_FEATURES_LIBRARIES}
)
