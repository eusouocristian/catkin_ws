# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/cristian/Documents/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cristian/Documents/catkin_ws/build

# Include any dependencies generated for this target.
include simple_arm/CMakeFiles/look_away.dir/depend.make

# Include the progress variables for this target.
include simple_arm/CMakeFiles/look_away.dir/progress.make

# Include the compile flags for this target's objects.
include simple_arm/CMakeFiles/look_away.dir/flags.make

simple_arm/CMakeFiles/look_away.dir/src/look_away.cpp.o: simple_arm/CMakeFiles/look_away.dir/flags.make
simple_arm/CMakeFiles/look_away.dir/src/look_away.cpp.o: /home/cristian/Documents/catkin_ws/src/simple_arm/src/look_away.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cristian/Documents/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object simple_arm/CMakeFiles/look_away.dir/src/look_away.cpp.o"
	cd /home/cristian/Documents/catkin_ws/build/simple_arm && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/look_away.dir/src/look_away.cpp.o -c /home/cristian/Documents/catkin_ws/src/simple_arm/src/look_away.cpp

simple_arm/CMakeFiles/look_away.dir/src/look_away.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/look_away.dir/src/look_away.cpp.i"
	cd /home/cristian/Documents/catkin_ws/build/simple_arm && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cristian/Documents/catkin_ws/src/simple_arm/src/look_away.cpp > CMakeFiles/look_away.dir/src/look_away.cpp.i

simple_arm/CMakeFiles/look_away.dir/src/look_away.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/look_away.dir/src/look_away.cpp.s"
	cd /home/cristian/Documents/catkin_ws/build/simple_arm && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cristian/Documents/catkin_ws/src/simple_arm/src/look_away.cpp -o CMakeFiles/look_away.dir/src/look_away.cpp.s

# Object files for target look_away
look_away_OBJECTS = \
"CMakeFiles/look_away.dir/src/look_away.cpp.o"

# External object files for target look_away
look_away_EXTERNAL_OBJECTS =

/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: simple_arm/CMakeFiles/look_away.dir/src/look_away.cpp.o
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: simple_arm/CMakeFiles/look_away.dir/build.make
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /opt/ros/noetic/lib/libcontroller_manager.so
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /opt/ros/noetic/lib/libclass_loader.so
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /usr/lib/x86_64-linux-gnu/libPocoFoundation.so
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /usr/lib/x86_64-linux-gnu/libdl.so
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /opt/ros/noetic/lib/libroslib.so
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /opt/ros/noetic/lib/librospack.so
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /usr/lib/x86_64-linux-gnu/libpython3.8.so
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /opt/ros/noetic/lib/libroscpp.so
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /opt/ros/noetic/lib/librosconsole.so
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /opt/ros/noetic/lib/librostime.so
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /opt/ros/noetic/lib/libcpp_common.so
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away: simple_arm/CMakeFiles/look_away.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cristian/Documents/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away"
	cd /home/cristian/Documents/catkin_ws/build/simple_arm && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/look_away.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
simple_arm/CMakeFiles/look_away.dir/build: /home/cristian/Documents/catkin_ws/devel/lib/simple_arm/look_away

.PHONY : simple_arm/CMakeFiles/look_away.dir/build

simple_arm/CMakeFiles/look_away.dir/clean:
	cd /home/cristian/Documents/catkin_ws/build/simple_arm && $(CMAKE_COMMAND) -P CMakeFiles/look_away.dir/cmake_clean.cmake
.PHONY : simple_arm/CMakeFiles/look_away.dir/clean

simple_arm/CMakeFiles/look_away.dir/depend:
	cd /home/cristian/Documents/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cristian/Documents/catkin_ws/src /home/cristian/Documents/catkin_ws/src/simple_arm /home/cristian/Documents/catkin_ws/build /home/cristian/Documents/catkin_ws/build/simple_arm /home/cristian/Documents/catkin_ws/build/simple_arm/CMakeFiles/look_away.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : simple_arm/CMakeFiles/look_away.dir/depend

