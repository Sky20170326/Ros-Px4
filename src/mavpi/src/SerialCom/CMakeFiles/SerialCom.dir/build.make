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
CMAKE_SOURCE_DIR = /home/feilong/Ros-Cat/src/mavpi/src/SerialCom

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/feilong/Ros-Cat/src/mavpi/src/SerialCom

# Include any dependencies generated for this target.
include CMakeFiles/SerialCom.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/SerialCom.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SerialCom.dir/flags.make

CMakeFiles/SerialCom.dir/test.cpp.o: CMakeFiles/SerialCom.dir/flags.make
CMakeFiles/SerialCom.dir/test.cpp.o: test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/feilong/Ros-Cat/src/mavpi/src/SerialCom/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SerialCom.dir/test.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SerialCom.dir/test.cpp.o -c /home/feilong/Ros-Cat/src/mavpi/src/SerialCom/test.cpp

CMakeFiles/SerialCom.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SerialCom.dir/test.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/feilong/Ros-Cat/src/mavpi/src/SerialCom/test.cpp > CMakeFiles/SerialCom.dir/test.cpp.i

CMakeFiles/SerialCom.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SerialCom.dir/test.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/feilong/Ros-Cat/src/mavpi/src/SerialCom/test.cpp -o CMakeFiles/SerialCom.dir/test.cpp.s

CMakeFiles/SerialCom.dir/test.cpp.o.requires:

.PHONY : CMakeFiles/SerialCom.dir/test.cpp.o.requires

CMakeFiles/SerialCom.dir/test.cpp.o.provides: CMakeFiles/SerialCom.dir/test.cpp.o.requires
	$(MAKE) -f CMakeFiles/SerialCom.dir/build.make CMakeFiles/SerialCom.dir/test.cpp.o.provides.build
.PHONY : CMakeFiles/SerialCom.dir/test.cpp.o.provides

CMakeFiles/SerialCom.dir/test.cpp.o.provides.build: CMakeFiles/SerialCom.dir/test.cpp.o


CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.o: CMakeFiles/SerialCom.dir/flags.make
CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.o: /home/feilong/Ros-Cat/src/mavpi/src/protocol.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/feilong/Ros-Cat/src/mavpi/src/SerialCom/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.o   -c /home/feilong/Ros-Cat/src/mavpi/src/protocol.c

CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/feilong/Ros-Cat/src/mavpi/src/protocol.c > CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.i

CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/feilong/Ros-Cat/src/mavpi/src/protocol.c -o CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.s

CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.o.requires:

.PHONY : CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.o.requires

CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.o.provides: CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.o.requires
	$(MAKE) -f CMakeFiles/SerialCom.dir/build.make CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.o.provides.build
.PHONY : CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.o.provides

CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.o.provides.build: CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.o


CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.o: CMakeFiles/SerialCom.dir/flags.make
CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.o: /home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/feilong/Ros-Cat/src/mavpi/src/SerialCom/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.o -c /home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp

CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp > CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.i

CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp -o CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.s

CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.o.requires:

.PHONY : CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.o.requires

CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.o.provides: CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.o.requires
	$(MAKE) -f CMakeFiles/SerialCom.dir/build.make CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.o.provides.build
.PHONY : CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.o.provides

CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.o.provides.build: CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.o


# Object files for target SerialCom
SerialCom_OBJECTS = \
"CMakeFiles/SerialCom.dir/test.cpp.o" \
"CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.o" \
"CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.o"

# External object files for target SerialCom
SerialCom_EXTERNAL_OBJECTS =

SerialCom: CMakeFiles/SerialCom.dir/test.cpp.o
SerialCom: CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.o
SerialCom: CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.o
SerialCom: CMakeFiles/SerialCom.dir/build.make
SerialCom: /usr/lib/x86_64-linux-gnu/libboost_system.so
SerialCom: CMakeFiles/SerialCom.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/feilong/Ros-Cat/src/mavpi/src/SerialCom/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable SerialCom"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SerialCom.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SerialCom.dir/build: SerialCom

.PHONY : CMakeFiles/SerialCom.dir/build

CMakeFiles/SerialCom.dir/requires: CMakeFiles/SerialCom.dir/test.cpp.o.requires
CMakeFiles/SerialCom.dir/requires: CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/protocol.c.o.requires
CMakeFiles/SerialCom.dir/requires: CMakeFiles/SerialCom.dir/home/feilong/Ros-Cat/src/mavpi/src/SerialCom.cpp.o.requires

.PHONY : CMakeFiles/SerialCom.dir/requires

CMakeFiles/SerialCom.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SerialCom.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SerialCom.dir/clean

CMakeFiles/SerialCom.dir/depend:
	cd /home/feilong/Ros-Cat/src/mavpi/src/SerialCom && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/feilong/Ros-Cat/src/mavpi/src/SerialCom /home/feilong/Ros-Cat/src/mavpi/src/SerialCom /home/feilong/Ros-Cat/src/mavpi/src/SerialCom /home/feilong/Ros-Cat/src/mavpi/src/SerialCom /home/feilong/Ros-Cat/src/mavpi/src/SerialCom/CMakeFiles/SerialCom.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SerialCom.dir/depend

