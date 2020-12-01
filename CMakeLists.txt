cmake_minimum_required(VERSION 3.17)
project(vmlab1)

set(CMAKE_CXX_STANDARD 14)

add_executable(vmlab1 main.cpp RapidBody.cpp RapidBody.h Pyramid.cpp Pyramid.h)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
