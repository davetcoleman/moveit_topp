cmake_minimum_required(VERSION 2.8.3)
project(moveit_topp)

# C++ 11
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rosparam_shortcuts
  cmake_modules
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rosparam_shortcuts
  DEPENDS
    Eigen
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    #${PROJECT_NAME}_wrapper
)

###########
## Build ##
###########

include_directories(
  include
  external/TOPP/src/
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

# --------------------------------------------
# Configure Locations
set(topp_LOCATION ${PROJECT_SOURCE_DIR}/external/TOPP/build/src/TOPPbindings.so)
set(topp_DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
MESSAGE( STATUS "topp_LOCATION:         " ${topp_LOCATION} )
MESSAGE( STATUS "topp_DESTINATION:      " ${topp_DESTINATION} )

# Topp library
add_library(${PROJECT_NAME} SHARED IMPORTED)
set_property(TARGET ${PROJECT_NAME} PROPERTY IMPORTED_LOCATION ${topp_LOCATION})

# Unfortuantly an imported target is not copied to the devel/lib folder, so we do it manually
# so that installation of a catkin workspace is not required
# See http://answers.ros.org/question/223866/cmakecatkin-how-to-export-imported-target/
file(COPY ${topp_LOCATION}
  DESTINATION ${topp_DESTINATION}
)

# --------------------------------------------

# Library
add_executable(${PROJECT_NAME}_wrapper
  src/moveit_topp.cpp
)
target_link_libraries(${PROJECT_NAME}_wrapper
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Testing ##
#############

## Test for correct C++ source code
roslint_cpp()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} #${PROJECT_NAME}_wrapper
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# # Mark roslaunch files for installation
# # install(FILES launch
# #   FILES_MATCHING PATTERN "*.launch"
# #   PATTERN ".svn" EXCLUDE
# #   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# # )
# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# # Mark config files for installation
# install(FILES config
#   FILES_MATCHING PATTERN "*.yaml"
#   PATTERN ".svn" EXCLUDE
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
