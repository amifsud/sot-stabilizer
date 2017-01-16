# Copyright 2014, François Bleibel, Mehdi Benallegue, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of sot-stabilizer.
# sot-dyninv is free software: you can redistribute it and/or
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

ADD_DEFINITIONS(-DDEBUG=2)

macro(ADDPREFIX newlist prefix list_name)
    # create empty list - necessary?
    SET(${newlist})

     # prefix and suffix elements
    foreach(l ${${list_name}})
      list(APPEND ${newlist} ${prefix}${l} )
    endforeach()

endmacro(ADDPREFIX)

ADDPREFIX(${PROJECT_NAME}_ABSOLUTE_HEADERS  "${CMAKE_SOURCE_DIR}/"  ${PROJECT_NAME}_HEADERS)

SET(${PROJECT_NAME}_test_CXXFLAGS "-I${PROJECT_SOURCE_DIR}/include/ ${${PROJECT_NAME}_CXXFLAGS}")

include_directories(${Boost_INCLUDE_DIR} ${PROJECT_INCLUDE_DIR})

SET(tests
    test_state-space-linear-controller
    test_hrp2-linear-state-space-stabilizer
    test_hrp2-lqr-twoDof-coupled-stabilizer
    test_hrp2-smc-stabilizer
  )

FOREACH(test ${tests})
  SET(EXECUTABLE_NAME "${test}_exe")
  ADD_EXECUTABLE(${EXECUTABLE_NAME}
    ${test}.cpp ${${PROJECT_NAME}_ABSOLUTE_HEADERS})

  #TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
    #controller-pd
   # )

  PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} sot-core)
  PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} state-observation)
  PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} dynamic-graph)

  TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${Boost_LIBRARIES} ${PROJECT_NAME} state-observation)

  ADD_TEST(${EXECUTABLE_NAME} ${EXECUTABLE_NAME})

ENDFOREACH(test)

SET(execs
  )

FOREACH(exec ${execs})
  SET(EXECUTABLE_NAME "${test}_exe")
  ADD_EXECUTABLE(${EXECUTABLE_NAME}
    ${test}.cpp ${${PROJECT_NAME}_ABSOLUTE_HEADERS})

  #TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
    #controller-pd
   # )

  PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} sot-core)
  PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} state-observation)
  PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} dynamic-graph)

  TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${Boost_LIBRARIES} ${PROJECT_NAME} state-observation)
ENDFOREACH(exec)


