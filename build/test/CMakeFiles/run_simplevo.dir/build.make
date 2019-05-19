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
CMAKE_SOURCE_DIR = /home/ljf/code/MonoVO

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ljf/code/MonoVO/build

# Include any dependencies generated for this target.
include test/CMakeFiles/run_simplevo.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/run_simplevo.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/run_simplevo.dir/flags.make

test/CMakeFiles/run_simplevo.dir/run_simplevo.cpp.o: test/CMakeFiles/run_simplevo.dir/flags.make
test/CMakeFiles/run_simplevo.dir/run_simplevo.cpp.o: ../test/run_simplevo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ljf/code/MonoVO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/run_simplevo.dir/run_simplevo.cpp.o"
	cd /home/ljf/code/MonoVO/build/test && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run_simplevo.dir/run_simplevo.cpp.o -c /home/ljf/code/MonoVO/test/run_simplevo.cpp

test/CMakeFiles/run_simplevo.dir/run_simplevo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run_simplevo.dir/run_simplevo.cpp.i"
	cd /home/ljf/code/MonoVO/build/test && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ljf/code/MonoVO/test/run_simplevo.cpp > CMakeFiles/run_simplevo.dir/run_simplevo.cpp.i

test/CMakeFiles/run_simplevo.dir/run_simplevo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run_simplevo.dir/run_simplevo.cpp.s"
	cd /home/ljf/code/MonoVO/build/test && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ljf/code/MonoVO/test/run_simplevo.cpp -o CMakeFiles/run_simplevo.dir/run_simplevo.cpp.s

test/CMakeFiles/run_simplevo.dir/run_simplevo.cpp.o.requires:

.PHONY : test/CMakeFiles/run_simplevo.dir/run_simplevo.cpp.o.requires

test/CMakeFiles/run_simplevo.dir/run_simplevo.cpp.o.provides: test/CMakeFiles/run_simplevo.dir/run_simplevo.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/run_simplevo.dir/build.make test/CMakeFiles/run_simplevo.dir/run_simplevo.cpp.o.provides.build
.PHONY : test/CMakeFiles/run_simplevo.dir/run_simplevo.cpp.o.provides

test/CMakeFiles/run_simplevo.dir/run_simplevo.cpp.o.provides.build: test/CMakeFiles/run_simplevo.dir/run_simplevo.cpp.o


# Object files for target run_simplevo
run_simplevo_OBJECTS = \
"CMakeFiles/run_simplevo.dir/run_simplevo.cpp.o"

# External object files for target run_simplevo
run_simplevo_EXTERNAL_OBJECTS =

../bin/run_simplevo: test/CMakeFiles/run_simplevo.dir/run_simplevo.cpp.o
../bin/run_simplevo: test/CMakeFiles/run_simplevo.dir/build.make
../bin/run_simplevo: ../lib/libmonovo.so
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
../bin/run_simplevo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
../bin/run_simplevo: /home/ljf/slam14/3rdparty/Sophus/build/libSophus.so
../bin/run_simplevo: test/CMakeFiles/run_simplevo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ljf/code/MonoVO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/run_simplevo"
	cd /home/ljf/code/MonoVO/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/run_simplevo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/run_simplevo.dir/build: ../bin/run_simplevo

.PHONY : test/CMakeFiles/run_simplevo.dir/build

test/CMakeFiles/run_simplevo.dir/requires: test/CMakeFiles/run_simplevo.dir/run_simplevo.cpp.o.requires

.PHONY : test/CMakeFiles/run_simplevo.dir/requires

test/CMakeFiles/run_simplevo.dir/clean:
	cd /home/ljf/code/MonoVO/build/test && $(CMAKE_COMMAND) -P CMakeFiles/run_simplevo.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/run_simplevo.dir/clean

test/CMakeFiles/run_simplevo.dir/depend:
	cd /home/ljf/code/MonoVO/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ljf/code/MonoVO /home/ljf/code/MonoVO/test /home/ljf/code/MonoVO/build /home/ljf/code/MonoVO/build/test /home/ljf/code/MonoVO/build/test/CMakeFiles/run_simplevo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/run_simplevo.dir/depend

