# @author Can Erdogan
# @date Jan 15, 2012

cmake_minimum_required(VERSION 3.1)
project(balancing)

find_package(DART 6.3.0 REQUIRED COMPONENTS utils-urdf gui CONFIG)

option(balancing_SYSTEM_EIGEN "Use system-installed version of Eigen" OFF)
#option(DDP_BUILD_EXAMPLES "Build examples" ON)

# Set the top level directory for use in the code
add_definitions(-DTOP_LEVEL_PATH="${CMAKE_SOURCE_DIR}")

# Set the C99 standard for the C files
set(CMAKE_INSTALL_PREFIX /usr)
#set(CMAKE_C_FLAGS --std=c++11 -g)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=gnu++0x")


# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed. You may need to remove CMakeCache.txt.")
endif()

# Set the mass table path
set(massTablePathVal "/home/cerdogan/Documents/Krang/Documentation/Design/Mass\ Table/MassProp.table" 
	CACHE STRING "Path to the mass table in your system - should be in doc/krang.git/design/mass table")
add_definitions(-DMASS_TABLE_PATH="${massTablePathVal}")

# Set eigen folder given in directory
set(EIGEN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/eigen)
if(balancing_SYSTEM_EIGEN)
    find_package(Eigen)
    if(EIGEN_FOUND)
        message(STATUS "Eigen found: ${EIGEN_INCLUDE_DIR}")
    else()
        message(FATAL_ERROR "Eigen NOT found!")
    endif()
endif()

# ================================================================================================
# Include and link to libraries

# Link to somatic, amino and ach
# NOTE: Ideally we would like to 'find' these packages but for now, we assume they are either 
# in /usr/lib or /usr/local/lib
#link_libraries(amino ntcan protobuf-c ach somatic stdc++ filter pcio pthread ncurses dart fcl tinyxml tinyxml2 kore assimp GL)
link_libraries(amino ntcan protobuf-c ach somatic stdc++ filter pthread ncurses dart fcl tinyxml tinyxml2 kore assimp GL config4cpp)
#include_directories(/usr/include/dart)
include_directories(/usr/local/include/config4cpp)


# Include Eigen
#include_directories(/usr/include/eigen3)
#include_directories(/usr/local/include/eigen3)

# ================================================================================================
# Build the codebase library and the executables



# Include the project files
include_directories(src)
include_directories(/home/munzir/project/krang/experiments/common/library)
include_directories(/home/munzir/project/krang/experiments/common/)
include_directories(${DART_INCLUDE_DIRS})
#including the Eigen direcotry given in folder
include_directories(${EIGEN_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
# =====================================================
#Including DDP Files
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

#target_include_directories(ddp INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
#file(GLOB DDP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/include/ddp/*.hpp")
add_library(ddp INTERFACE)
target_include_directories(ddp INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
file(GLOB DDP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/include/ddp/*.hpp")
target_sources(ddp INTERFACE ${DDP_SOURCES})

# Collect the source, script and fortran files
file(GLOB main_source "src/*.cpp" "src/*.c" "/home/munzir/project/krang/experiments/common/library/*.cpp")
file(GLOB scripts_source "exe/*.cpp")
list(SORT scripts_source)

# Build the library and set the executables that use it (have to hardcode for now)
add_library(Main SHARED ${main_source})
set(MainDependents kore balancing)

#add_library(ddp INTERFACE)
#target_include_directories(ddp INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
#target_sources(ddp INTERFACE ${DDP_SOURCES})

# Build scripts
message(STATUS "\n-- SCRIPTS: ")
foreach(script_src_file ${scripts_source})
	get_filename_component(script_base ${script_src_file} NAME_WE)
	message(STATUS "Adding script ${script_src_file} with base name ${script_base}" )
	add_executable(${script_base} ${script_src_file})
	target_link_libraries(${script_base} Main )
	#target_link_libraries(${script_base} ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES}) 
	target_link_libraries(${script_base} ${DART_LIBRARIES} ${wxWidgets_LIBRARIES}) 
	add_custom_target(${script_base}.run ${script_base} ${ARGN})
endforeach(script_src_file)
message(STATUS " ")

