# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/yxc/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yxc/catkin_ws/build

# Include any dependencies generated for this target.
include serial_demo/CMakeFiles/serial_demo.dir/depend.make

# Include the progress variables for this target.
include serial_demo/CMakeFiles/serial_demo.dir/progress.make

# Include the compile flags for this target's objects.
include serial_demo/CMakeFiles/serial_demo.dir/flags.make

serial_demo/CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.o: serial_demo/CMakeFiles/serial_demo.dir/flags.make
serial_demo/CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.o: /home/yxc/catkin_ws/src/serial_demo/src/serial_demo_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yxc/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object serial_demo/CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.o"
	cd /home/yxc/catkin_ws/build/serial_demo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.o -c /home/yxc/catkin_ws/src/serial_demo/src/serial_demo_node.cpp

serial_demo/CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.i"
	cd /home/yxc/catkin_ws/build/serial_demo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yxc/catkin_ws/src/serial_demo/src/serial_demo_node.cpp > CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.i

serial_demo/CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.s"
	cd /home/yxc/catkin_ws/build/serial_demo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yxc/catkin_ws/src/serial_demo/src/serial_demo_node.cpp -o CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.s

serial_demo/CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.o.requires:

.PHONY : serial_demo/CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.o.requires

serial_demo/CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.o.provides: serial_demo/CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.o.requires
	$(MAKE) -f serial_demo/CMakeFiles/serial_demo.dir/build.make serial_demo/CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.o.provides.build
.PHONY : serial_demo/CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.o.provides

serial_demo/CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.o.provides.build: serial_demo/CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.o


# Object files for target serial_demo
serial_demo_OBJECTS = \
"CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.o"

# External object files for target serial_demo
serial_demo_EXTERNAL_OBJECTS =

/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: serial_demo/CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.o
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: serial_demo/CMakeFiles/serial_demo.dir/build.make
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: /opt/ros/melodic/lib/libroscpp.so
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: /opt/ros/melodic/lib/librosconsole.so
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: /opt/ros/melodic/lib/librostime.so
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: /opt/ros/melodic/lib/libcpp_common.so
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: /opt/ros/melodic/lib/libserial.so
/home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo: serial_demo/CMakeFiles/serial_demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yxc/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo"
	cd /home/yxc/catkin_ws/build/serial_demo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/serial_demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
serial_demo/CMakeFiles/serial_demo.dir/build: /home/yxc/catkin_ws/devel/lib/serial_demo/serial_demo

.PHONY : serial_demo/CMakeFiles/serial_demo.dir/build

serial_demo/CMakeFiles/serial_demo.dir/requires: serial_demo/CMakeFiles/serial_demo.dir/src/serial_demo_node.cpp.o.requires

.PHONY : serial_demo/CMakeFiles/serial_demo.dir/requires

serial_demo/CMakeFiles/serial_demo.dir/clean:
	cd /home/yxc/catkin_ws/build/serial_demo && $(CMAKE_COMMAND) -P CMakeFiles/serial_demo.dir/cmake_clean.cmake
.PHONY : serial_demo/CMakeFiles/serial_demo.dir/clean

serial_demo/CMakeFiles/serial_demo.dir/depend:
	cd /home/yxc/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yxc/catkin_ws/src /home/yxc/catkin_ws/src/serial_demo /home/yxc/catkin_ws/build /home/yxc/catkin_ws/build/serial_demo /home/yxc/catkin_ws/build/serial_demo/CMakeFiles/serial_demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : serial_demo/CMakeFiles/serial_demo.dir/depend

