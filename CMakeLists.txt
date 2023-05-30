cmake_minimum_required(VERSION 2.8)
project(calSDF)
find_package(CGAL )
find_package(Boost REQUIRED  COMPONENTS  system filesystem)#BOOST大写
set(CMAKE_CXX_STANDARD 17)
# find_package(Qt5 QUIET COMPONENTS Xml Script OpenGL Svg)
include(${CGAL_USE_FILE})
add_executable(main main.cpp)
target_link_libraries(main ${CGAL_LIBS}  ${Boost_LIBRARY_DIRS} -lboost_filesystem -lboost_system)
