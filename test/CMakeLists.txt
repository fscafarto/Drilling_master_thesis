cmake_minimum_required(VERSION 3.5)
project(anymals_like LANGUAGES CXX) #mod

#set(CMAKE_CXX_STANDARD 17)

#==
# Dependencies
#==

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(iDynTree REQUIRED)
find_package( ifopt REQUIRED )
find_package(Threads REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  #eigen_conversions
  tf
  tf_conversions
  #roslib
  ifopt
)



if(WIN32)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/win32/mt_release/bin")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/win32/mt_debug/bin")
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../rsc" DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE})
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../rsc" DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG})
else()
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../rsc" DESTINATION ${CMAKE_BINARY_DIR}/test) #mod
endif()

include_directories (${EIGEN3_INCLUDE_DIRS})

include_directories(/usr/include/eigen3)

include_directories(/opt/ros/noetic/include)

include_directories(/opt/ros/noetic/lib)

include_directories(include)

include_directories(/usr/local/lib)




SET( ALGLIB_SRC 
  src/alglib/alglibinternal.cpp
  src/alglib/alglibmisc.cpp
  src/alglib/ap.cpp
  src/alglib/linalg.cpp
  src/alglib/optimization.cpp
  src/alglib/solvers.cpp
)

SET( TOWR_SRC 
  src/towr/nlp_formulation.cc
  src/towr/parameters.cc
  src/towr/nodes_variables.cc
  src/towr/nodes_variables_all.cc
  src/towr/nodes_variables_phase_based.cc
  src/towr/phase_durations.cc
  src/towr/robot_model.cc
  src/towr/dynamic_model.cc
  src/towr/single_rigid_body_dynamics.cc
  src/towr/time_discretization_constraint.cc
  src/towr/base_motion_constraint.cc
  src/towr/terrain_constraint.cc
  src/towr/swing_constraint.cc
  src/towr/force_constraint.cc
  src/towr/total_duration_constraint.cc
  src/towr/dynamic_constraint.cc
  src/towr/range_of_motion_constraint.cc
  src/towr/spline_acc_constraint.cc
  src/towr/linear_constraint.cc
  src/towr/node_cost.cc
  src/towr/soft_constraint.cc
  src/towr/gait_generator.cc
  src/towr/monoped_gait_generator.cc
  src/towr/biped_gait_generator.cc
  src/towr/quadruped_gait_generator.cc
  src/towr/height_map_examples.cc
  src/towr/height_map.cc
  src/towr/state.cc
  src/towr/polynomial.cc
  src/towr/spline.cc
  src/towr/phase_spline.cc
  src/towr/node_spline.cc
  src/towr/nodes_observer.cc
  src/towr/spline_holder.cc
  src/towr/euler_converter.cc
  src/towr/phase_durations_observer.cc
)

add_library( optimal src/optimal.cpp ${ALGLIB_SRC} )
target_link_libraries( optimal ${catkin_LIBRARIES}  )

add_library( planning src/planning.cpp ${TOWR_SRC} ) 
target_link_libraries( planning ${catkin_LIBRARIES})



#==
# Build Targets
#==

function(create_executable app_name file_name)
    add_executable(${app_name} ${file_name})
    set_target_properties(${app_name} PROPERTIES MACOSX_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/mac/lib")
    if(WIN32)
        target_link_libraries(${app_name} PUBLIC raisim::raisim Ws2_32 Winmm)
        target_compile_options(${app_name} PRIVATE "/MP")
    else()
        target_link_libraries(${app_name} PUBLIC raisim::raisim pthread)
    endif()

    if(APPLE)
        execute_process(COMMAND sysctl -q hw.optional.arm64
                OUTPUT_VARIABLE _sysctl_stdout
                ERROR_VARIABLE _sysctl_stderr
                RESULT_VARIABLE _sysctl_result
                )
        if(_sysctl_result EQUAL 0 AND _sysctl_stdout MATCHES "hw.optional.arm64: 1")
            target_compile_options(${app_name} PRIVATE -mcpu=apple-m1)
        endif()
    endif()

    target_include_directories(${app_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
endfunction()


# benchmark examples
#create_executable(speed_test src/benchmark/speed_test.cpp)

# server examples
#create_executable(primitives src/server/primitives.cpp)
#create_executable(meshes src/server/meshes.cpp)
#create_executable(laikago src/server/laikago.cpp)
#create_executable(balls src/server/balls.cpp)
#create_executable(atlas src/server/atlas.cpp)
#create_executable(heightmap src/server/heightmap.cpp)
#create_executable(robots src/server/aliengo.cpp)
#create_executable(materials src/server/material.cpp)
#create_executable(materialStaticFriction src/server/materialStaticFriction.cpp)
#create_executable(heightMapUsingPng src/server/heightMapUsingPng.cpp)
#create_executable(wheeledRobot src/server/wheeledRobot.cpp)
#create_executable(newtonsCradle src/server/newtonsCradle.cpp)
#create_executable(visualObjects src/server/visualObjects.cpp)
#create_executable(rayDemo src/server/rayDemo.cpp)
#create_executable(rayDemo2 src/server/rayDemo2.cpp)
create_executable(anymals_like src/anymals_like.cpp)
#create_executable(anymal_stress_test src/server/anymals_stress_test.cpp)
#create_executable(compound src/server/compound.cpp)
#create_executable(kinova src/server/kinova.cpp)
#create_executable(cartPole src/server/cartPole.cpp)
#create_executable(springTest src/server/springTest.cpp)
#create_executable(trackedRobotAndTemplatedURDF src/server/trackedRobotAndTemplatedURDF.cpp)
#create_executable(kinematicObject src/server/kinematicObject.cpp)

# xml reader
#create_executable(xmlRader src/xml/xmlReader.cpp)

# mjcf files
#create_executable(mjcf_humanoid src/mjcf/humanoid.cpp)
#create_executable(mjcf_ant src/mjcf/ant.cpp)
#create_executable(mjcf_cassie src/mjcf/cassie.cpp)

target_link_libraries(anymals_like   PUBLIC raisim::raisim ${iDynTree_LIBRARIES} Eigen3::Eigen ${Boost_LIBRARIES} ${catkin_LIBRARIES} -I/opt/ros/noetic/include -L/opt/ros/noetic/lib -lroscpp -ltf -lrostime -lrosconsole -lroscpp_serialization -lroslib -lboost_thread optimal planning towr ifopt_core ifopt_ipopt -L/usr/local/lib -ltowr -lifopt_core -lifopt_ipopt)

if(NOT WIN32)
  target_link_libraries(anymals_like PUBLIC pthread)
endif()

target_include_directories(anymals_like PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)







