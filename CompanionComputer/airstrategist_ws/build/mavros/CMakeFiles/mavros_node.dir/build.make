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
CMAKE_SOURCE_DIR = /home/pi/airstrategist_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/airstrategist_ws/build

# Include any dependencies generated for this target.
include mavros/CMakeFiles/mavros_node.dir/depend.make

# Include the progress variables for this target.
include mavros/CMakeFiles/mavros_node.dir/progress.make

# Include the compile flags for this target's objects.
include mavros/CMakeFiles/mavros_node.dir/flags.make

mavros/CMakeFiles/mavros_node.dir/src/mavros_node.cpp.o: mavros/CMakeFiles/mavros_node.dir/flags.make
mavros/CMakeFiles/mavros_node.dir/src/mavros_node.cpp.o: /home/pi/airstrategist_ws/src/mavros/src/mavros_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/airstrategist_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object mavros/CMakeFiles/mavros_node.dir/src/mavros_node.cpp.o"
	cd /home/pi/airstrategist_ws/build/mavros && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mavros_node.dir/src/mavros_node.cpp.o -c /home/pi/airstrategist_ws/src/mavros/src/mavros_node.cpp

mavros/CMakeFiles/mavros_node.dir/src/mavros_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mavros_node.dir/src/mavros_node.cpp.i"
	cd /home/pi/airstrategist_ws/build/mavros && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/airstrategist_ws/src/mavros/src/mavros_node.cpp > CMakeFiles/mavros_node.dir/src/mavros_node.cpp.i

mavros/CMakeFiles/mavros_node.dir/src/mavros_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mavros_node.dir/src/mavros_node.cpp.s"
	cd /home/pi/airstrategist_ws/build/mavros && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/airstrategist_ws/src/mavros/src/mavros_node.cpp -o CMakeFiles/mavros_node.dir/src/mavros_node.cpp.s

mavros/CMakeFiles/mavros_node.dir/src/mavros_node.cpp.o.requires:

.PHONY : mavros/CMakeFiles/mavros_node.dir/src/mavros_node.cpp.o.requires

mavros/CMakeFiles/mavros_node.dir/src/mavros_node.cpp.o.provides: mavros/CMakeFiles/mavros_node.dir/src/mavros_node.cpp.o.requires
	$(MAKE) -f mavros/CMakeFiles/mavros_node.dir/build.make mavros/CMakeFiles/mavros_node.dir/src/mavros_node.cpp.o.provides.build
.PHONY : mavros/CMakeFiles/mavros_node.dir/src/mavros_node.cpp.o.provides

mavros/CMakeFiles/mavros_node.dir/src/mavros_node.cpp.o.provides.build: mavros/CMakeFiles/mavros_node.dir/src/mavros_node.cpp.o


# Object files for target mavros_node
mavros_node_OBJECTS = \
"CMakeFiles/mavros_node.dir/src/mavros_node.cpp.o"

# External object files for target mavros_node
mavros_node_EXTERNAL_OBJECTS =

/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: mavros/CMakeFiles/mavros_node.dir/src/mavros_node.cpp.o
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: mavros/CMakeFiles/mavros_node.dir/build.make
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: /opt/ros/kinetic/lib/libroscpp.so
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: /usr/lib/arm-linux-gnueabihf/libboost_signals.so
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: /opt/ros/kinetic/lib/librosconsole.so
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: /usr/lib/arm-linux-gnueabihf/liblog4cxx.so
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: /opt/ros/kinetic/lib/librostime.so
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: /opt/ros/kinetic/lib/libcpp_common.so
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: /usr/lib/arm-linux-gnueabihf/libboost_system.so
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: /usr/lib/arm-linux-gnueabihf/libpthread.so
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so
/home/pi/airstrategist_ws/devel/lib/mavros/mavros_node: mavros/CMakeFiles/mavros_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/airstrategist_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/pi/airstrategist_ws/devel/lib/mavros/mavros_node"
	cd /home/pi/airstrategist_ws/build/mavros && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mavros_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
mavros/CMakeFiles/mavros_node.dir/build: /home/pi/airstrategist_ws/devel/lib/mavros/mavros_node

.PHONY : mavros/CMakeFiles/mavros_node.dir/build

mavros/CMakeFiles/mavros_node.dir/requires: mavros/CMakeFiles/mavros_node.dir/src/mavros_node.cpp.o.requires

.PHONY : mavros/CMakeFiles/mavros_node.dir/requires

mavros/CMakeFiles/mavros_node.dir/clean:
	cd /home/pi/airstrategist_ws/build/mavros && $(CMAKE_COMMAND) -P CMakeFiles/mavros_node.dir/cmake_clean.cmake
.PHONY : mavros/CMakeFiles/mavros_node.dir/clean

mavros/CMakeFiles/mavros_node.dir/depend:
	cd /home/pi/airstrategist_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/airstrategist_ws/src /home/pi/airstrategist_ws/src/mavros /home/pi/airstrategist_ws/build /home/pi/airstrategist_ws/build/mavros /home/pi/airstrategist_ws/build/mavros/CMakeFiles/mavros_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : mavros/CMakeFiles/mavros_node.dir/depend

