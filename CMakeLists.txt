# A minimal example on how to combine Qt3D and Bullet physics
# Copyright (C) 2017 Jens Jacob Støren
# Permission is granted to anyone to use this software for any purpose, 
cmake_minimum_required(VERSION 3.8.1)

project(Qt5BulletExample)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)


# Find Qt5 with modules. 
# Needs to set CMAKE variable Qt5_DIR = path-to-your-qt-innstallation/5.9/msvcxxx/lib/cmake/Qt5 
set(Qt5_DIR "" CACHE PATH "Path to Qt5 cmake stuff: ..Qt/5.9/msvcxxx/lib/cmake/Qt5" )
find_package(Qt5 REQUIRED COMPONENTS  3DCore 3DExtras 3DRender 3DInput)

# Find Bullet
# Needs to set CMAKE variable BULLET_ROOT 
set(BULLET_ROOT "" CACHE PATH "Path to installation of Bullet" )
find_package(Bullet REQUIRED )

include_directories(${BULLET_INCLUDE_DIR})

add_executable(Qt5BulletExample WIN32 
		       main.cpp 
		       )

target_link_libraries(Qt5BulletExample 
	                  Qt5::3DCore
	                  Qt5::3DRender
	                  Qt5::3DInput
	                  Qt5::3DExtras
	                  debug ${BULLET_DYNAMICS_LIBRARY_DEBUG}
	                  debug ${BULLET_COLLISION_LIBRARY_DEBUG}
	                  debug ${BULLET_MATH_LIBRARY_DEBUG}
                      optimized ${BULLET_DYNAMICS_LIBRARY}
	                  optimized ${BULLET_COLLISION_LIBRARY}
	                  optimized ${BULLET_MATH_LIBRARY}
	                  )
	
# Deploing Qt Dlls				   
if(WIN32)
    get_target_property(QT5_QMAKE_EXECUTABLE Qt5::qmake IMPORTED_LOCATION)
    get_filename_component(QT5_WINDEPLOYQT_EXECUTABLE ${QT5_QMAKE_EXECUTABLE} PATH)
    set(QT5_WINDEPLOYQT_EXECUTABLE "${QT5_WINDEPLOYQT_EXECUTABLE}/windeployqt.exe")

    add_custom_command(TARGET Qt5BulletExample POST_BUILD
       COMMAND ${QT5_WINDEPLOYQT_EXECUTABLE} --qmldir ${CMAKE_SOURCE_DIR} $<TARGET_FILE_DIR:Qt5BulletExample>)
endif(WIN32)
