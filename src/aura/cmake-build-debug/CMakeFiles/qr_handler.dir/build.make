# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_COMMAND = /home/ashkan/clion-2018.2.5/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/ashkan/clion-2018.2.5/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ashkan/Aura_hex/src/aura

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ashkan/Aura_hex/src/aura/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/qr_handler.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/qr_handler.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/qr_handler.dir/flags.make

CMakeFiles/qr_handler.dir/src/qr_handler.cpp.o: CMakeFiles/qr_handler.dir/flags.make
CMakeFiles/qr_handler.dir/src/qr_handler.cpp.o: ../src/qr_handler.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ashkan/Aura_hex/src/aura/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/qr_handler.dir/src/qr_handler.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/qr_handler.dir/src/qr_handler.cpp.o -c /home/ashkan/Aura_hex/src/aura/src/qr_handler.cpp

CMakeFiles/qr_handler.dir/src/qr_handler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/qr_handler.dir/src/qr_handler.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ashkan/Aura_hex/src/aura/src/qr_handler.cpp > CMakeFiles/qr_handler.dir/src/qr_handler.cpp.i

CMakeFiles/qr_handler.dir/src/qr_handler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/qr_handler.dir/src/qr_handler.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ashkan/Aura_hex/src/aura/src/qr_handler.cpp -o CMakeFiles/qr_handler.dir/src/qr_handler.cpp.s

# Object files for target qr_handler
qr_handler_OBJECTS = \
"CMakeFiles/qr_handler.dir/src/qr_handler.cpp.o"

# External object files for target qr_handler
qr_handler_EXTERNAL_OBJECTS =

qr_handler: CMakeFiles/qr_handler.dir/src/qr_handler.cpp.o
qr_handler: CMakeFiles/qr_handler.dir/build.make
qr_handler: /opt/ros/kinetic/lib/libroscpp.so
qr_handler: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
qr_handler: /usr/lib/x86_64-linux-gnu/libboost_signals.so
qr_handler: /opt/ros/kinetic/lib/libxmlrpcpp.so
qr_handler: /opt/ros/kinetic/lib/libcv_bridge.so
qr_handler: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
qr_handler: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
qr_handler: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
qr_handler: /opt/ros/kinetic/lib/librosconsole.so
qr_handler: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
qr_handler: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
qr_handler: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
qr_handler: /usr/lib/x86_64-linux-gnu/libboost_regex.so
qr_handler: /opt/ros/kinetic/lib/libroscpp_serialization.so
qr_handler: /opt/ros/kinetic/lib/librostime.so
qr_handler: /opt/ros/kinetic/lib/libcpp_common.so
qr_handler: /usr/lib/x86_64-linux-gnu/libboost_system.so
qr_handler: /usr/lib/x86_64-linux-gnu/libboost_thread.so
qr_handler: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
qr_handler: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
qr_handler: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
qr_handler: /usr/lib/x86_64-linux-gnu/libpthread.so
qr_handler: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
qr_handler: CMakeFiles/qr_handler.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ashkan/Aura_hex/src/aura/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable qr_handler"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/qr_handler.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/qr_handler.dir/build: qr_handler

.PHONY : CMakeFiles/qr_handler.dir/build

CMakeFiles/qr_handler.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/qr_handler.dir/cmake_clean.cmake
.PHONY : CMakeFiles/qr_handler.dir/clean

CMakeFiles/qr_handler.dir/depend:
	cd /home/ashkan/Aura_hex/src/aura/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ashkan/Aura_hex/src/aura /home/ashkan/Aura_hex/src/aura /home/ashkan/Aura_hex/src/aura/cmake-build-debug /home/ashkan/Aura_hex/src/aura/cmake-build-debug /home/ashkan/Aura_hex/src/aura/cmake-build-debug/CMakeFiles/qr_handler.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/qr_handler.dir/depend

