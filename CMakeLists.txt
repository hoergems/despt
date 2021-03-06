cmake_minimum_required (VERSION 2.6)

set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -std=c++11 -pg -w")
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE RelWithDebInfo) 
endif()

string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)
if (CMAKE_BUILD_TYPE MATCHES RELWITHDEBINFO OR CMAKE_BUILD_TYPE MATCHES DEBUG)
   set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
   set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)


include(FindPkgConfig)
include(GNUInstallDirs)


find_package(SDFormat REQUIRED)
find_package(oppt REQUIRED)
find_package(kdl_parser REQUIRED)
include_directories(${oppt_INCLUDE_DIRS})
link_directories(${oppt_LIBRARY_DIRS})


if(PKG_CONFIG_FOUND)
    pkg_check_modules(DESPOT despot)
    if(NOT DESPOT_FOUND)
       message(FATAL_ERROR "DESPOT could not be found")
    endif()       
endif()
include_directories(${DESPOT_INCLUDE_DIRS})
link_directories(${DESPOT_LIBRARY_DIRS})
link_directories(${kdl_parser_LIBRARY_DIRS})
list(APPEND oppt_LIBRARY_DIRS "${ASSIMP_LIBRARY_DIRS}")

message("====================================")
message(${oppt_LIBRARIES})
message("====================================")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(despotSolver
               ${CMAKE_CURRENT_LIST_DIR}/main.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/Model.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/State.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/Belief.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/World.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/Planner.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/Bound.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/POMCPPrior.cpp)
               
target_link_libraries (despotSolver                
                       ${oppt_LIBRARIES}
                       ${DESPOT_LIBRARIES}
                       ${kdl_parser_LIBRARIES})
#set_target_properties( despott
#    PROPERTIES    
#    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
#)
