cmake_minimum_required (VERSION 2.6)

set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -std=c++11 -pg")
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE RelWithDebInfo) 
endif()

string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)
if (CMAKE_BUILD_TYPE MATCHES RELWITHDEBINFO OR CMAKE_BUILD_TYPE MATCHES DEBUG)
   set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
   set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")
endif()

include(FindPkgConfig)
include(GNUInstallDirs)


find_package(SDFormat REQUIRED)
find_package(oppt REQUIRED)

if(PKG_CONFIG_FOUND)
    pkg_check_modules(DESPOT despot)
    if(NOT DESPOT_FOUND)
       message(FATAL_ERROR "DESPOT could not be found")
    endif()       
endif()
include_directories(${DESPOT_INCLUDE_DIRS})
link_directories(${DESPOT_LIBRARY_DIRS})
list(APPEND oppt_LIBRARY_DIRS "${ASSIMP_LIBRARY_DIRS}")

message(${oppt_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(despott
               ${CMAKE_CURRENT_LIST_DIR}/main.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/Model.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/State.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/Belief.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/World.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/Planner.cpp
               ${CMAKE_CURRENT_LIST_DIR}/src/Bound.cpp)
               
target_link_libraries (despott                   
                       ${oppt_LIBRARIES}
                       ${DESPOT_LIBRARIES})
