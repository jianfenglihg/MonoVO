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
CMAKE_SOURCE_DIR = /home/ljf/slam14/test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ljf/slam14/test/build

# Include any dependencies generated for this target.
include CMakeFiles/run_vo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/run_vo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/run_vo.dir/flags.make

CMakeFiles/run_vo.dir/run_vo.o: CMakeFiles/run_vo.dir/flags.make
CMakeFiles/run_vo.dir/run_vo.o: ../run_vo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ljf/slam14/test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/run_vo.dir/run_vo.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run_vo.dir/run_vo.o -c /home/ljf/slam14/test/run_vo.cpp

CMakeFiles/run_vo.dir/run_vo.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run_vo.dir/run_vo.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ljf/slam14/test/run_vo.cpp > CMakeFiles/run_vo.dir/run_vo.i

CMakeFiles/run_vo.dir/run_vo.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run_vo.dir/run_vo.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ljf/slam14/test/run_vo.cpp -o CMakeFiles/run_vo.dir/run_vo.s

CMakeFiles/run_vo.dir/run_vo.o.requires:

.PHONY : CMakeFiles/run_vo.dir/run_vo.o.requires

CMakeFiles/run_vo.dir/run_vo.o.provides: CMakeFiles/run_vo.dir/run_vo.o.requires
	$(MAKE) -f CMakeFiles/run_vo.dir/build.make CMakeFiles/run_vo.dir/run_vo.o.provides.build
.PHONY : CMakeFiles/run_vo.dir/run_vo.o.provides

CMakeFiles/run_vo.dir/run_vo.o.provides.build: CMakeFiles/run_vo.dir/run_vo.o


# Object files for target run_vo
run_vo_OBJECTS = \
"CMakeFiles/run_vo.dir/run_vo.o"

# External object files for target run_vo
run_vo_EXTERNAL_OBJECTS =

run_vo: CMakeFiles/run_vo.dir/run_vo.o
run_vo: CMakeFiles/run_vo.dir/build.make
run_vo: CMakeFiles/run_vo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ljf/slam14/test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable run_vo"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/run_vo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/run_vo.dir/build: run_vo

.PHONY : CMakeFiles/run_vo.dir/build

CMakeFiles/run_vo.dir/requires: CMakeFiles/run_vo.dir/run_vo.o.requires

.PHONY : CMakeFiles/run_vo.dir/requires

CMakeFiles/run_vo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/run_vo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/run_vo.dir/clean

CMakeFiles/run_vo.dir/depend:
	cd /home/ljf/slam14/test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ljf/slam14/test /home/ljf/slam14/test /home/ljf/slam14/test/build /home/ljf/slam14/test/build /home/ljf/slam14/test/build/CMakeFiles/run_vo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/run_vo.dir/depend

