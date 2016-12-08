cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(heel_toe_planner)

find_package(cmake_modules)
find_package(Eigen REQUIRED)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(IPOPT REQUIRED)
if(IPOPT_FOUND)
  link_directories(${IPOPT_LIBRARY_DIRS})
    message([STATUS]"${IPOPT_LIBRARY_DIRS}")
endif()
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)
if(gazebo_FOUND)
    link_directories(${GAZEBO_LIBRARY_DIRS})
endif()
find_package(catkin REQUIRED COMPONENTS	roscpp
					rbdl
					visualization_msgs
					tf
					roslib
					trac_ik_lib
					pal_robot_tools
					pal_multicontact_planner
					std_msgs
					gazebo_msgs
					rosgraph_msgs
					dynamic_reconfigure
					message_generation
					std_srvs

)

find_package(PCL 1.3 REQUIRED COMPONENTS common io)
find_package(orocos_kdl REQUIRED)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_C_FLAGS "-std=c99 ${CMAKE_C_FLAGS} ")
#set(CMAKE_LINK_LIBRARY_FLAG "-Wl,--rpath -Wl,/home/viki/Desktop/CoinIpopt/build/lib ${CMAKE_LINK_LIBRARY_FLAG}")
#set(CMAKE_CXX_LINKER_PREFERENCE "-Wl,--rpath -Wl,/home/viki/Desktop/CoinIpopt/build/lib ${CMAKE_CXX_LINKER_PREFERENCE}")



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp visualization_msgs tf trac_ik_lib std_srvs rosgraph_msgs dynamic_reconfigure message_generation std_msgs gazebo_msgs
  DEPENDS orocos_kdl
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${SDFormat_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(${PROJECT_NAME}	src/COM_Generation/COM_Boundary_Condition.c
				src/COM_Generation/COM_Generation.c
				src/COM_Generation/Cosine_Cosine_Interpolation.c
				src/COM_Generation/fileManager.c
				src/COM_Generation/Forward_Kinematic.c
				src/COM_Generation/fprintf.c
				src/COM_Generation/Gait_Generation.c
				src/COM_Generation/Heel_Toe_Planner_Part1_emxAPI.c
				src/COM_Generation/Heel_Toe_Planner_Part1_emxutil.c
				src/COM_Generation/Heel_Toe_Planner_Part1_initialize.c
				src/COM_Generation/Heel_Toe_Planner_Part1_rtwutil.c
				src/COM_Generation/Heel_Toe_Planner_Part1_terminate.c
				src/COM_Generation/mldivide.c
				src/COM_Generation/Quadratic_Cosine_Interpolation.c
				src/COM_Generation/rtGetInf.c
				src/COM_Generation/rtGetNaN.c
				src/COM_Generation/rt_nonfinite.c
				src/COM_Generation/X_DS_Differential_Equa_Solver.c
				src/COM_Generation/X_SS_Differential_Equa_Solver.c
				src/COM_Generation/Y_Correction.c
				src/COM_Generation/Y_DS_Differential_Equa_Solver.c
				src/COM_Generation/Y_End.c
				src/COM_Generation/Y_SS_Differential_Equa_Solver.c
				src/COM_Generation/ZMP_Boundary_Condition.c
				src/COM_Generation/ZMP_END_FITTING.c
				src/COM_Generation/ZMP_Generation.c
				src/COM_Generation/COM_Generation_All.cpp
				src/COM_Generation/Ipopt_Solver/Nonlinear_Eqn_Solver1.cpp
				src/COM_Generation/Ipopt_Solver/Nonlinear_Eqn_Solver2.cpp
				src/COM_Generation/Ipopt_Solver/Nonlinear_Eqn_Solver3.cpp
				src/COM_Generation/Ipopt_Solver/NLP_Interface.cpp
				src/IK_Solver.cpp
				src/Three_Mass.cpp
				src/Controller.cpp

)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${IPOPT_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${SDFormat_LIBRARIES} ${orocos_kdl_LIBRARIES} ${GAZEBO_libraries} ${roscpp_LIBRARIES} lapack)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(heel_toe_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(demo src/test/demo.cpp)
add_executable(test_com_gen src/test/Test_COM_Generation.cpp)
add_executable(test_nonlinear_solver src/test/Test_Solver.cpp)
add_executable(heel_toe_planner_complete src/heel_toe_planner_complete.cpp)
add_executable(test_controller src/test/Test_Controller.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(heel_toe_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(demo  ${PROJECT_NAME})
target_link_libraries(test_com_gen ${PROJECT_NAME})
target_link_libraries(test_nonlinear_solver ${PROJECT_NAME})
target_link_libraries(heel_toe_planner_complete ${PROJECT_NAME})
target_link_libraries(test_controller ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/
	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY gazebo/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY gazebo/worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
