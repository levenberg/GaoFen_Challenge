# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /root/Documents/roswork/DJI2016_Challenge/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/Documents/roswork/DJI2016_Challenge/build

# Include any dependencies generated for this target.
include dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/depend.make

# Include the progress variables for this target.
include dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/progress.make

# Include the compile flags for this target's objects.
include dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/flags.make

dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/src/client.cpp.o: dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/flags.make
dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/src/client.cpp.o: /root/Documents/roswork/DJI2016_Challenge/src/dji_sdk_demo/src/client.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /root/Documents/roswork/DJI2016_Challenge/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/src/client.cpp.o"
	cd /root/Documents/roswork/DJI2016_Challenge/build/dji_sdk_demo && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/dji_sdk_client.dir/src/client.cpp.o -c /root/Documents/roswork/DJI2016_Challenge/src/dji_sdk_demo/src/client.cpp

dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/src/client.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dji_sdk_client.dir/src/client.cpp.i"
	cd /root/Documents/roswork/DJI2016_Challenge/build/dji_sdk_demo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /root/Documents/roswork/DJI2016_Challenge/src/dji_sdk_demo/src/client.cpp > CMakeFiles/dji_sdk_client.dir/src/client.cpp.i

dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/src/client.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dji_sdk_client.dir/src/client.cpp.s"
	cd /root/Documents/roswork/DJI2016_Challenge/build/dji_sdk_demo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /root/Documents/roswork/DJI2016_Challenge/src/dji_sdk_demo/src/client.cpp -o CMakeFiles/dji_sdk_client.dir/src/client.cpp.s

dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/src/client.cpp.o.requires:
.PHONY : dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/src/client.cpp.o.requires

dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/src/client.cpp.o.provides: dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/src/client.cpp.o.requires
	$(MAKE) -f dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/build.make dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/src/client.cpp.o.provides.build
.PHONY : dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/src/client.cpp.o.provides

dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/src/client.cpp.o.provides.build: dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/src/client.cpp.o

# Object files for target dji_sdk_client
dji_sdk_client_OBJECTS = \
"CMakeFiles/dji_sdk_client.dir/src/client.cpp.o"

# External object files for target dji_sdk_client
dji_sdk_client_EXTERNAL_OBJECTS =

/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/src/client.cpp.o
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/build.make
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: /opt/ros/indigo/lib/libactionlib.so
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: /opt/ros/indigo/lib/libroscpp.so
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: /usr/lib/arm-linux-gnueabihf/libboost_signals.so
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: /opt/ros/indigo/lib/librosconsole.so
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: /usr/lib/liblog4cxx.so
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: /opt/ros/indigo/lib/libxmlrpcpp.so
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: /opt/ros/indigo/lib/libroscpp_serialization.so
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: /opt/ros/indigo/lib/librostime.so
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: /opt/ros/indigo/lib/libcpp_common.so
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: /usr/lib/arm-linux-gnueabihf/libboost_system.so
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: /usr/lib/arm-linux-gnueabihf/libpthread.so
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: /root/Documents/roswork/DJI2016_Challenge/devel/lib/libdji_sdk_lib.a
/root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client: dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client"
	cd /root/Documents/roswork/DJI2016_Challenge/build/dji_sdk_demo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dji_sdk_client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/build: /root/Documents/roswork/DJI2016_Challenge/devel/lib/dji_sdk_demo/dji_sdk_client
.PHONY : dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/build

dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/requires: dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/src/client.cpp.o.requires
.PHONY : dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/requires

dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/clean:
	cd /root/Documents/roswork/DJI2016_Challenge/build/dji_sdk_demo && $(CMAKE_COMMAND) -P CMakeFiles/dji_sdk_client.dir/cmake_clean.cmake
.PHONY : dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/clean

dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/depend:
	cd /root/Documents/roswork/DJI2016_Challenge/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/Documents/roswork/DJI2016_Challenge/src /root/Documents/roswork/DJI2016_Challenge/src/dji_sdk_demo /root/Documents/roswork/DJI2016_Challenge/build /root/Documents/roswork/DJI2016_Challenge/build/dji_sdk_demo /root/Documents/roswork/DJI2016_Challenge/build/dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : dji_sdk_demo/CMakeFiles/dji_sdk_client.dir/depend
