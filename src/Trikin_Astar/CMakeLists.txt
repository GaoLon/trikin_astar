# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

cmake_minimum_required(VERSION 3.1...3.15)

project( trikin_astar )


# CGAL and its components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cmake_modules 
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package( CGAL QUIET COMPONENTS  )

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()


# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()

# include for local directory
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
set(PCL_INCLUDE_DIRS ${PCL_INCLUDE_DIR})
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp rospy std_msgs
 DEPENDS PCL Eigen3
)
# include for local package
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIR})
include_directories("/usr/include/pcl-1.8")
# include_directories( BEFORE ../include )

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

set(LIBS ${LIBS} ${catkin_LIBRARIES})

# Creating entries for target: Trikin-Astar
# ############################
file(GLOB NODE_SRC_FILES src/*.cpp)
add_executable( Trikin_Astar  ${NODE_SRC_FILES} )

# add_to_cached_list( CGAL_EXECUTABLE_TARGETS Trikin-Astar )

# Link the executable to CGAL and third-party libraries
# target_link_libraries(Trikin-Astar PRIVATE CGAL::CGAL)
target_link_libraries(Trikin_Astar PRIVATE CGAL::CGAL ${LIBS} ${PCL_LIBRARIES} )