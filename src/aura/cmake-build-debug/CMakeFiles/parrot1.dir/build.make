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
include CMakeFiles/parrot1.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/parrot1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/parrot1.dir/flags.make

CMakeFiles/parrot1.dir/src/controller.cpp.o: CMakeFiles/parrot1.dir/flags.make
CMakeFiles/parrot1.dir/src/controller.cpp.o: ../src/controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ashkan/Aura_hex/src/aura/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/parrot1.dir/src/controller.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/parrot1.dir/src/controller.cpp.o -c /home/ashkan/Aura_hex/src/aura/src/controller.cpp

CMakeFiles/parrot1.dir/src/controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/parrot1.dir/src/controller.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ashkan/Aura_hex/src/aura/src/controller.cpp > CMakeFiles/parrot1.dir/src/controller.cpp.i

CMakeFiles/parrot1.dir/src/controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/parrot1.dir/src/controller.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ashkan/Aura_hex/src/aura/src/controller.cpp -o CMakeFiles/parrot1.dir/src/controller.cpp.s

CMakeFiles/parrot1.dir/src/trajectory.cpp.o: CMakeFiles/parrot1.dir/flags.make
CMakeFiles/parrot1.dir/src/trajectory.cpp.o: ../src/trajectory.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ashkan/Aura_hex/src/aura/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/parrot1.dir/src/trajectory.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/parrot1.dir/src/trajectory.cpp.o -c /home/ashkan/Aura_hex/src/aura/src/trajectory.cpp

CMakeFiles/parrot1.dir/src/trajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/parrot1.dir/src/trajectory.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ashkan/Aura_hex/src/aura/src/trajectory.cpp > CMakeFiles/parrot1.dir/src/trajectory.cpp.i

CMakeFiles/parrot1.dir/src/trajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/parrot1.dir/src/trajectory.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ashkan/Aura_hex/src/aura/src/trajectory.cpp -o CMakeFiles/parrot1.dir/src/trajectory.cpp.s

CMakeFiles/parrot1.dir/src/test1.cpp.o: CMakeFiles/parrot1.dir/flags.make
CMakeFiles/parrot1.dir/src/test1.cpp.o: ../src/test1.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ashkan/Aura_hex/src/aura/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/parrot1.dir/src/test1.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/parrot1.dir/src/test1.cpp.o -c /home/ashkan/Aura_hex/src/aura/src/test1.cpp

CMakeFiles/parrot1.dir/src/test1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/parrot1.dir/src/test1.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ashkan/Aura_hex/src/aura/src/test1.cpp > CMakeFiles/parrot1.dir/src/test1.cpp.i

CMakeFiles/parrot1.dir/src/test1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/parrot1.dir/src/test1.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ashkan/Aura_hex/src/aura/src/test1.cpp -o CMakeFiles/parrot1.dir/src/test1.cpp.s

# Object files for target parrot1
parrot1_OBJECTS = \
"CMakeFiles/parrot1.dir/src/controller.cpp.o" \
"CMakeFiles/parrot1.dir/src/trajectory.cpp.o" \
"CMakeFiles/parrot1.dir/src/test1.cpp.o"

# External object files for target parrot1
parrot1_EXTERNAL_OBJECTS =

parrot1: CMakeFiles/parrot1.dir/src/controller.cpp.o
parrot1: CMakeFiles/parrot1.dir/src/trajectory.cpp.o
parrot1: CMakeFiles/parrot1.dir/src/test1.cpp.o
parrot1: CMakeFiles/parrot1.dir/build.make
parrot1: /opt/ros/kinetic/lib/libroscpp.so
parrot1: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
parrot1: /usr/lib/x86_64-linux-gnu/libboost_signals.so
parrot1: /opt/ros/kinetic/lib/libxmlrpcpp.so
parrot1: /opt/ros/kinetic/lib/libcv_bridge.so
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/librosconsole.so
parrot1: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
parrot1: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
parrot1: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
parrot1: /usr/lib/x86_64-linux-gnu/libboost_regex.so
parrot1: /opt/ros/kinetic/lib/libroscpp_serialization.so
parrot1: /opt/ros/kinetic/lib/librostime.so
parrot1: /opt/ros/kinetic/lib/libcpp_common.so
parrot1: /usr/lib/x86_64-linux-gnu/libboost_system.so
parrot1: /usr/lib/x86_64-linux-gnu/libboost_thread.so
parrot1: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
parrot1: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
parrot1: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
parrot1: /usr/lib/x86_64-linux-gnu/libpthread.so
parrot1: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
parrot1: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
parrot1: CMakeFiles/parrot1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ashkan/Aura_hex/src/aura/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable parrot1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/parrot1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/parrot1.dir/build: parrot1

.PHONY : CMakeFiles/parrot1.dir/build

CMakeFiles/parrot1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/parrot1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/parrot1.dir/clean

CMakeFiles/parrot1.dir/depend:
	cd /home/ashkan/Aura_hex/src/aura/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ashkan/Aura_hex/src/aura /home/ashkan/Aura_hex/src/aura /home/ashkan/Aura_hex/src/aura/cmake-build-debug /home/ashkan/Aura_hex/src/aura/cmake-build-debug /home/ashkan/Aura_hex/src/aura/cmake-build-debug/CMakeFiles/parrot1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/parrot1.dir/depend

