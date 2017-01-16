# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST,
#                 Florent Lamiraux (CNRS/LAAS)
#
# This file is part of sot-stabilizer.
# sot-stabilizer is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-dyninv is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-dyninv. If not, see <http://www.gnu.org/licenses/>.

INCLUDE(../cmake/python.cmake)

# prefix and suffix each element of list by ${prefix}elemnt${suffix}
macro(ADDPREFIX newlist prefix list_name)
    # create empty list - necessary?
    SET(${newlist})

     # prefix and suffix elements
    foreach(l ${${list_name}})
      list(APPEND ${newlist} ${prefix}${l} )
    endforeach()

endmacro(ADDPREFIX)

ADDPREFIX(${PROJECT_NAME}_ABSOLUTE_HEADERS  "${CMAKE_SOURCE_DIR}/"  ${PROJECT_NAME}_HEADERS)

SET(LIBRARY_NAME ${PROJECT_NAME})

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  hrp2-decoupled-stabilizer.cpp
  hrp2-lqr-decoupled-stabilizer.cpp
  hrp2-lqr-twoDof-coupled-stabilizer.cpp
  hrp2-linear-state-space-stabilizer.cpp
  perturbations-generator.cpp
  discrete-time-non-alg-riccati-eqn.cpp
  discrete-time-lti-lqr.cpp
  discrete-time-ordinary-smc.cpp
  state-space-linear-controller.cpp
  prototyping/decoupled-elastic-inv-pendulum-simulator.cpp
  prototyping/zmp-to-com-algebraic-controller.cpp
  prototyping/linearized-table-cart-device.cc
#  prototyping/linearized-rotational-table-cart-device.cpp
#  prototyping/non-linear-rotational-table-cart-device.cpp
  ${${PROJECT_NAME}_ABSOLUTE_HEADERS}
)


SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${PROJECT_VERSION}
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

# provide path to dependency libraries
LINK_DIRECTORIES( ${STATE_OBSERVATION_LIBRARY_DIRS})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

ADD_DEPENDENCIES(${LIBRARY_NAME} state-observation sot-state-observation sot-core dynamic-graph dynamic-graph-python)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} state-observation sot-state-observation sot-core dynamic-graph dynamic-graph-python)


# Add dynamic-graph-python compilation flags and link to dependency libraries
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} state-observation)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} sot-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} sot-state-observation)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} state-observation)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} sot-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} sot-state-observation)


INSTALL(TARGETS ${LIBRARY_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

#STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})

SET(NEW_ENTITY_CLASS
#  DGIMUFlexibilityEstimation
#  DGIMUAttitudeEstimation
#  MovingFrameTransformation
  )

DYNAMIC_GRAPH_PYTHON_MODULE("sot/application/stabilizer" ${LIBRARY_NAME} wrap)

###
# PYTHON PART
###

#cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()

#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})

INCLUDE(../cmake/python.cmake)
FINDPYTHON ()

INSTALL(FILES
  dynamic_graph/sot/application/stabilizer/scenarii/hand_compensater.py
  dynamic_graph/sot/application/stabilizer/scenarii/hand_compensater_oscillator.py
  dynamic_graph/sot/application/stabilizer/scenarii/ds_stabilizer.py
  dynamic_graph/sot/application/stabilizer/scenarii/ds_stabilizer_hrp2.py
  dynamic_graph/sot/application/stabilizer/scenarii/ds_linear_stabilizer_hrp2.py
  dynamic_graph/sot/application/stabilizer/scenarii/ds_lqr_twoDof_coupled_stabilizer_hrp2.py
  dynamic_graph/sot/application/stabilizer/scenarii/seqplay_stabilizer.py
  dynamic_graph/sot/application/stabilizer/scenarii/seqplay_stabilizer_hrp2.py
  dynamic_graph/sot/application/stabilizer/scenarii/seqplay_lqr_twoDof_coupled_stabilizer_hrp2.py
  dynamic_graph/sot/application/stabilizer/scenarii/seqplay_lqr_twoDof_coupled_stabilizer_hrp2_encoders.py
  dynamic_graph/sot/application/stabilizer/scenarii/pg_lqr_twoDof_coupled_stabilizer_hrp2_encoders.py
  dynamic_graph/sot/application/stabilizer/scenarii/pg_lqr_twoDof_coupled_stabilizer_hrp2.py
  dynamic_graph/sot/application/stabilizer/scenarii/seqplay_lqr_twoDof_coupled_stabilizer.py
  dynamic_graph/sot/application/stabilizer/scenarii/pg_lqr_twoDof_coupled_stabilizer.py
  dynamic_graph/sot/application/stabilizer/scenarii/hand_compensater_chicken.py
  dynamic_graph/sot/application/stabilizer/scenarii/hand_compensater_solver_kine.py
  dynamic_graph/sot/application/stabilizer/scenarii/leg_oscillator.py
  dynamic_graph/sot/application/stabilizer/scenarii/hrp2_stabilizer_simulated_decoupled_model.py
  dynamic_graph/sot/application/stabilizer/scenarii/hrp2_stabilizer.py
  dynamic_graph/sot/application/stabilizer/scenarii/hrp2_linear_stabilizer.py
  dynamic_graph/sot/application/stabilizer/scenarii/hrp2_model_free_observer_stabilizer.py
  dynamic_graph/sot/application/stabilizer/scenarii/ds_model_free_observer_stabilizer_hrp2.py
  dynamic_graph/sot/application/stabilizer/scenarii/hrp2_lqr_twoDof_coupled_stabilizer.py
  dynamic_graph/sot/application/stabilizer/scenarii/hrp2_lqr_twoDof_coupled_stabilizer_encoders.py
  dynamic_graph/sot/application/stabilizer/scenarii/__init__.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/application/stabilizer/scenarii
  )

INSTALL(FILES
  dynamic_graph/sot/application/stabilizer/scripts/appli_hands_compensater_chicken.py
  dynamic_graph/sot/application/stabilizer/scripts/appli_two_hands_compensater_hrp.py
  dynamic_graph/sot/application/stabilizer/scripts/appli_model_base_elast_pendulum_two_hands_compensater_hrp.py
  dynamic_graph/sot/application/stabilizer/scripts/appli_model_base_two_hands_compensater_hrp.py
  dynamic_graph/sot/application/stabilizer/scripts/appli_two_hands_compensater_oscillator_hrp.py
  dynamic_graph/sot/application/stabilizer/scripts/appli_ds_stabilizer_hrp.py
  dynamic_graph/sot/application/stabilizer/scripts/appli_ds_linear_stabilizer_hrp.py
  dynamic_graph/sot/application/stabilizer/scripts/appli_ds_model_free_observer_stabilizer.py
  dynamic_graph/sot/application/stabilizer/scripts/appli_seqplay_stabilizer_hrp.py
  dynamic_graph/sot/application/stabilizer/scripts/appli_seqplay_lqr_twoDof_coupled_stabilizer_hrp.py
  dynamic_graph/sot/application/stabilizer/scripts/appli_seqplay_lqr_twoDof_coupled_stabilizer_hrp_encoders.py
  dynamic_graph/sot/application/stabilizer/scripts/appli_pg_lqr_twoDof_coupled_stabilizer_hrp_encoders.py
  dynamic_graph/sot/application/stabilizer/scripts/appli_pg_lqr_twoDof_coupled_stabilizer_hrp.py

  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/application/stabilizer/scripts
  )
