# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/crazyflie/crazyswarm/ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/CDPParser

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/crazyflie/crazyswarm/ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/cdpBuild

# Include any dependencies generated for this target.
include src/CMakeFiles/testCDP.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/testCDP.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/testCDP.dir/flags.make

src/CMakeFiles/testCDP.dir/test.cpp.o: src/CMakeFiles/testCDP.dir/flags.make
src/CMakeFiles/testCDP.dir/test.cpp.o: /home/crazyflie/crazyswarm/ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/CDPParser/src/test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/crazyflie/crazyswarm/ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/cdpBuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/testCDP.dir/test.cpp.o"
	cd /home/crazyflie/crazyswarm/ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/cdpBuild/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testCDP.dir/test.cpp.o -c /home/crazyflie/crazyswarm/ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/CDPParser/src/test.cpp

src/CMakeFiles/testCDP.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testCDP.dir/test.cpp.i"
	cd /home/crazyflie/crazyswarm/ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/cdpBuild/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/crazyflie/crazyswarm/ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/CDPParser/src/test.cpp > CMakeFiles/testCDP.dir/test.cpp.i

src/CMakeFiles/testCDP.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testCDP.dir/test.cpp.s"
	cd /home/crazyflie/crazyswarm/ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/cdpBuild/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/crazyflie/crazyswarm/ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/CDPParser/src/test.cpp -o CMakeFiles/testCDP.dir/test.cpp.s

src/CMakeFiles/testCDP.dir/test.cpp.o.requires:

.PHONY : src/CMakeFiles/testCDP.dir/test.cpp.o.requires

src/CMakeFiles/testCDP.dir/test.cpp.o.provides: src/CMakeFiles/testCDP.dir/test.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/testCDP.dir/build.make src/CMakeFiles/testCDP.dir/test.cpp.o.provides.build
.PHONY : src/CMakeFiles/testCDP.dir/test.cpp.o.provides

src/CMakeFiles/testCDP.dir/test.cpp.o.provides.build: src/CMakeFiles/testCDP.dir/test.cpp.o


# Object files for target testCDP
testCDP_OBJECTS = \
"CMakeFiles/testCDP.dir/test.cpp.o"

# External object files for target testCDP
testCDP_EXTERNAL_OBJECTS =

bin/testCDP: src/CMakeFiles/testCDP.dir/test.cpp.o
bin/testCDP: src/CMakeFiles/testCDP.dir/build.make
bin/testCDP: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
bin/testCDP: /usr/lib/x86_64-linux-gnu/libboost_system.so
bin/testCDP: /usr/lib/x86_64-linux-gnu/libboost_thread.so
bin/testCDP: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
bin/testCDP: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
bin/testCDP: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
bin/testCDP: /usr/lib/x86_64-linux-gnu/libpthread.so
bin/testCDP: src/CMakeFiles/testCDP.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/crazyflie/crazyswarm/ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/cdpBuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/testCDP"
	cd /home/crazyflie/crazyswarm/ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/cdpBuild/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testCDP.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/testCDP.dir/build: bin/testCDP

.PHONY : src/CMakeFiles/testCDP.dir/build

src/CMakeFiles/testCDP.dir/requires: src/CMakeFiles/testCDP.dir/test.cpp.o.requires

.PHONY : src/CMakeFiles/testCDP.dir/requires

src/CMakeFiles/testCDP.dir/clean:
	cd /home/crazyflie/crazyswarm/ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/cdpBuild/src && $(CMAKE_COMMAND) -P CMakeFiles/testCDP.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/testCDP.dir/clean

src/CMakeFiles/testCDP.dir/depend:
	cd /home/crazyflie/crazyswarm/ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/cdpBuild && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/crazyflie/crazyswarm/ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/CDPParser /home/crazyflie/crazyswarm/ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/CDPParser/src /home/crazyflie/crazyswarm/ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/cdpBuild /home/crazyflie/crazyswarm/ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/cdpBuild/src /home/crazyflie/crazyswarm/ros_ws/src/crazyflie_ros/externalDependencies/libmotioncapture/externalDependencies/cdpBuild/src/CMakeFiles/testCDP.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/testCDP.dir/depend
