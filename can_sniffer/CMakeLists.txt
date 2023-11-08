cmake_minimum_required(VERSION 2.8.3)
project(can_sniffer)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# Add ros version check
find_package(catkin REQUIRED COMPONENTS message_generation
   roscpp
  std_msgs
  thorvald_base
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CanMsgId.msg
  CanMsgIdList.msg
  OnOffMsg.msg
  CanVariableData.msg
  BitMaskData.msg
  VarData.msg
  CanMsgVariable.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   GetVarList.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS  message_runtime roscpp std_msgs thorvald_base
#  DEPENDS eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})

find_package(LibXml2 REQUIRED)
if (LIBXML2_FOUND)
  #message("Libxml2_INCLUDE_DIRS="${Libxml2_INCLUDE_DIRS})
  set(Libxml2_INCLUDE_DIRS "/usr/include/libxml2") #Don't know why even if it finds it the include folder variable is empty
  include_directories(${Libxml2_INCLUDE_DIRS})
endif (LIBXML2_FOUND)

#if (EIGEN_FOUND)
#  message("EIGEN_INCLUDE_DIRS=" ${EIGEN_INCLUDE_DIRS})
#  include_directories(${EIGEN_INCLUDE_DIRS})
#endif (EIGEN_FOUND)

## Declare a cpp executable
add_executable(can_sniffer_node src/can_sniffer_node.cpp src/can_sniffer.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(can_sniffer_node can_sniffer_generate_messages_cpp can_reader_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(can_sniffer_node  ${catkin_LIBRARIES} )

if (LIBXML2_FOUND)
  message("LIBXML2_LIBRARIES=" ${LIBXML2_LIBRARIES})
  target_link_libraries (can_sniffer_node ${LIBXML2_LIBRARIES})
endif (LIBXML2_FOUND)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 install(TARGETS can_sniffer_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(DIRECTORY
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
