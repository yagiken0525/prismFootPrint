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


# Produce verbose output by default.
VERBOSE = 1

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
CMAKE_SOURCE_DIR = /home/yagi/CLionProjects/prismFootPrint

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yagi/CLionProjects/prismFootPrint/build

# Include any dependencies generated for this target.
include src/openPose/CMakeFiles/Openpose.dir/depend.make

# Include the progress variables for this target.
include src/openPose/CMakeFiles/Openpose.dir/progress.make

# Include the compile flags for this target's objects.
include src/openPose/CMakeFiles/Openpose.dir/flags.make

src/openPose/CMakeFiles/Openpose.dir/OpenPosePerson.cpp.o: src/openPose/CMakeFiles/Openpose.dir/flags.make
src/openPose/CMakeFiles/Openpose.dir/OpenPosePerson.cpp.o: ../src/openPose/OpenPosePerson.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yagi/CLionProjects/prismFootPrint/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/openPose/CMakeFiles/Openpose.dir/OpenPosePerson.cpp.o"
	cd /home/yagi/CLionProjects/prismFootPrint/build/src/openPose && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Openpose.dir/OpenPosePerson.cpp.o -c /home/yagi/CLionProjects/prismFootPrint/src/openPose/OpenPosePerson.cpp

src/openPose/CMakeFiles/Openpose.dir/OpenPosePerson.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Openpose.dir/OpenPosePerson.cpp.i"
	cd /home/yagi/CLionProjects/prismFootPrint/build/src/openPose && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yagi/CLionProjects/prismFootPrint/src/openPose/OpenPosePerson.cpp > CMakeFiles/Openpose.dir/OpenPosePerson.cpp.i

src/openPose/CMakeFiles/Openpose.dir/OpenPosePerson.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Openpose.dir/OpenPosePerson.cpp.s"
	cd /home/yagi/CLionProjects/prismFootPrint/build/src/openPose && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yagi/CLionProjects/prismFootPrint/src/openPose/OpenPosePerson.cpp -o CMakeFiles/Openpose.dir/OpenPosePerson.cpp.s

src/openPose/CMakeFiles/Openpose.dir/OpenPosePerson.cpp.o.requires:

.PHONY : src/openPose/CMakeFiles/Openpose.dir/OpenPosePerson.cpp.o.requires

src/openPose/CMakeFiles/Openpose.dir/OpenPosePerson.cpp.o.provides: src/openPose/CMakeFiles/Openpose.dir/OpenPosePerson.cpp.o.requires
	$(MAKE) -f src/openPose/CMakeFiles/Openpose.dir/build.make src/openPose/CMakeFiles/Openpose.dir/OpenPosePerson.cpp.o.provides.build
.PHONY : src/openPose/CMakeFiles/Openpose.dir/OpenPosePerson.cpp.o.provides

src/openPose/CMakeFiles/Openpose.dir/OpenPosePerson.cpp.o.provides.build: src/openPose/CMakeFiles/Openpose.dir/OpenPosePerson.cpp.o


src/openPose/CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.o: src/openPose/CMakeFiles/Openpose.dir/flags.make
src/openPose/CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.o: ../src/openPose/exportHumanPoseTxt.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yagi/CLionProjects/prismFootPrint/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/openPose/CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.o"
	cd /home/yagi/CLionProjects/prismFootPrint/build/src/openPose && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.o -c /home/yagi/CLionProjects/prismFootPrint/src/openPose/exportHumanPoseTxt.cpp

src/openPose/CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.i"
	cd /home/yagi/CLionProjects/prismFootPrint/build/src/openPose && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yagi/CLionProjects/prismFootPrint/src/openPose/exportHumanPoseTxt.cpp > CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.i

src/openPose/CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.s"
	cd /home/yagi/CLionProjects/prismFootPrint/build/src/openPose && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yagi/CLionProjects/prismFootPrint/src/openPose/exportHumanPoseTxt.cpp -o CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.s

src/openPose/CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.o.requires:

.PHONY : src/openPose/CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.o.requires

src/openPose/CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.o.provides: src/openPose/CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.o.requires
	$(MAKE) -f src/openPose/CMakeFiles/Openpose.dir/build.make src/openPose/CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.o.provides.build
.PHONY : src/openPose/CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.o.provides

src/openPose/CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.o.provides.build: src/openPose/CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.o


# Object files for target Openpose
Openpose_OBJECTS = \
"CMakeFiles/Openpose.dir/OpenPosePerson.cpp.o" \
"CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.o"

# External object files for target Openpose
Openpose_EXTERNAL_OBJECTS =

src/openPose/libOpenpose.a: src/openPose/CMakeFiles/Openpose.dir/OpenPosePerson.cpp.o
src/openPose/libOpenpose.a: src/openPose/CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.o
src/openPose/libOpenpose.a: src/openPose/CMakeFiles/Openpose.dir/build.make
src/openPose/libOpenpose.a: src/openPose/CMakeFiles/Openpose.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yagi/CLionProjects/prismFootPrint/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libOpenpose.a"
	cd /home/yagi/CLionProjects/prismFootPrint/build/src/openPose && $(CMAKE_COMMAND) -P CMakeFiles/Openpose.dir/cmake_clean_target.cmake
	cd /home/yagi/CLionProjects/prismFootPrint/build/src/openPose && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Openpose.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/openPose/CMakeFiles/Openpose.dir/build: src/openPose/libOpenpose.a

.PHONY : src/openPose/CMakeFiles/Openpose.dir/build

src/openPose/CMakeFiles/Openpose.dir/requires: src/openPose/CMakeFiles/Openpose.dir/OpenPosePerson.cpp.o.requires
src/openPose/CMakeFiles/Openpose.dir/requires: src/openPose/CMakeFiles/Openpose.dir/exportHumanPoseTxt.cpp.o.requires

.PHONY : src/openPose/CMakeFiles/Openpose.dir/requires

src/openPose/CMakeFiles/Openpose.dir/clean:
	cd /home/yagi/CLionProjects/prismFootPrint/build/src/openPose && $(CMAKE_COMMAND) -P CMakeFiles/Openpose.dir/cmake_clean.cmake
.PHONY : src/openPose/CMakeFiles/Openpose.dir/clean

src/openPose/CMakeFiles/Openpose.dir/depend:
	cd /home/yagi/CLionProjects/prismFootPrint/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yagi/CLionProjects/prismFootPrint /home/yagi/CLionProjects/prismFootPrint/src/openPose /home/yagi/CLionProjects/prismFootPrint/build /home/yagi/CLionProjects/prismFootPrint/build/src/openPose /home/yagi/CLionProjects/prismFootPrint/build/src/openPose/CMakeFiles/Openpose.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/openPose/CMakeFiles/Openpose.dir/depend

