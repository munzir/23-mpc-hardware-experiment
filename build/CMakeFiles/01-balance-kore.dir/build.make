# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.8

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cerdogan/Documents/Software/project/krang/demos/balancing/mpc_steps

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cerdogan/Documents/Software/project/krang/demos/balancing/mpc_steps/build

# Include any dependencies generated for this target.
include CMakeFiles/01-balance-kore.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/01-balance-kore.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/01-balance-kore.dir/flags.make

CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.o: CMakeFiles/01-balance-kore.dir/flags.make
CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.o: ../exe/01-balance-kore.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cerdogan/Documents/Software/project/krang/demos/balancing/mpc_steps/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.o -c /home/cerdogan/Documents/Software/project/krang/demos/balancing/mpc_steps/exe/01-balance-kore.cpp

CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cerdogan/Documents/Software/project/krang/demos/balancing/mpc_steps/exe/01-balance-kore.cpp > CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.i

CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cerdogan/Documents/Software/project/krang/demos/balancing/mpc_steps/exe/01-balance-kore.cpp -o CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.s

CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.o.requires:

.PHONY : CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.o.requires

CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.o.provides: CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.o.requires
	$(MAKE) -f CMakeFiles/01-balance-kore.dir/build.make CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.o.provides.build
.PHONY : CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.o.provides

CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.o.provides.build: CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.o


# Object files for target 01-balance-kore
01__balance__kore_OBJECTS = \
"CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.o"

# External object files for target 01-balance-kore
01__balance__kore_EXTERNAL_OBJECTS =

01-balance-kore: CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.o
01-balance-kore: CMakeFiles/01-balance-kore.dir/build.make
01-balance-kore: libMain.so
01-balance-kore: CMakeFiles/01-balance-kore.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cerdogan/Documents/Software/project/krang/demos/balancing/mpc_steps/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable 01-balance-kore"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/01-balance-kore.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/01-balance-kore.dir/build: 01-balance-kore

.PHONY : CMakeFiles/01-balance-kore.dir/build

CMakeFiles/01-balance-kore.dir/requires: CMakeFiles/01-balance-kore.dir/exe/01-balance-kore.cpp.o.requires

.PHONY : CMakeFiles/01-balance-kore.dir/requires

CMakeFiles/01-balance-kore.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/01-balance-kore.dir/cmake_clean.cmake
.PHONY : CMakeFiles/01-balance-kore.dir/clean

CMakeFiles/01-balance-kore.dir/depend:
	cd /home/cerdogan/Documents/Software/project/krang/demos/balancing/mpc_steps/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cerdogan/Documents/Software/project/krang/demos/balancing/mpc_steps /home/cerdogan/Documents/Software/project/krang/demos/balancing/mpc_steps /home/cerdogan/Documents/Software/project/krang/demos/balancing/mpc_steps/build /home/cerdogan/Documents/Software/project/krang/demos/balancing/mpc_steps/build /home/cerdogan/Documents/Software/project/krang/demos/balancing/mpc_steps/build/CMakeFiles/01-balance-kore.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/01-balance-kore.dir/depend

