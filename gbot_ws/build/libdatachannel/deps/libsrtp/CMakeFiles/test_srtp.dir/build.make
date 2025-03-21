# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hady/teleop/gbot_ws/libdatachannel

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hady/teleop/gbot_ws/build/libdatachannel

# Include any dependencies generated for this target.
include deps/libsrtp/CMakeFiles/test_srtp.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include deps/libsrtp/CMakeFiles/test_srtp.dir/compiler_depend.make

# Include the progress variables for this target.
include deps/libsrtp/CMakeFiles/test_srtp.dir/progress.make

# Include the compile flags for this target's objects.
include deps/libsrtp/CMakeFiles/test_srtp.dir/flags.make

deps/libsrtp/CMakeFiles/test_srtp.dir/test/test_srtp.c.o: deps/libsrtp/CMakeFiles/test_srtp.dir/flags.make
deps/libsrtp/CMakeFiles/test_srtp.dir/test/test_srtp.c.o: /home/hady/teleop/gbot_ws/libdatachannel/deps/libsrtp/test/test_srtp.c
deps/libsrtp/CMakeFiles/test_srtp.dir/test/test_srtp.c.o: deps/libsrtp/CMakeFiles/test_srtp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hady/teleop/gbot_ws/build/libdatachannel/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object deps/libsrtp/CMakeFiles/test_srtp.dir/test/test_srtp.c.o"
	cd /home/hady/teleop/gbot_ws/build/libdatachannel/deps/libsrtp && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT deps/libsrtp/CMakeFiles/test_srtp.dir/test/test_srtp.c.o -MF CMakeFiles/test_srtp.dir/test/test_srtp.c.o.d -o CMakeFiles/test_srtp.dir/test/test_srtp.c.o -c /home/hady/teleop/gbot_ws/libdatachannel/deps/libsrtp/test/test_srtp.c

deps/libsrtp/CMakeFiles/test_srtp.dir/test/test_srtp.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/test_srtp.dir/test/test_srtp.c.i"
	cd /home/hady/teleop/gbot_ws/build/libdatachannel/deps/libsrtp && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/hady/teleop/gbot_ws/libdatachannel/deps/libsrtp/test/test_srtp.c > CMakeFiles/test_srtp.dir/test/test_srtp.c.i

deps/libsrtp/CMakeFiles/test_srtp.dir/test/test_srtp.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/test_srtp.dir/test/test_srtp.c.s"
	cd /home/hady/teleop/gbot_ws/build/libdatachannel/deps/libsrtp && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/hady/teleop/gbot_ws/libdatachannel/deps/libsrtp/test/test_srtp.c -o CMakeFiles/test_srtp.dir/test/test_srtp.c.s

# Object files for target test_srtp
test_srtp_OBJECTS = \
"CMakeFiles/test_srtp.dir/test/test_srtp.c.o"

# External object files for target test_srtp
test_srtp_EXTERNAL_OBJECTS =

deps/libsrtp/test_srtp: deps/libsrtp/CMakeFiles/test_srtp.dir/test/test_srtp.c.o
deps/libsrtp/test_srtp: deps/libsrtp/CMakeFiles/test_srtp.dir/build.make
deps/libsrtp/test_srtp: deps/libsrtp/libsrtp2.a
deps/libsrtp/test_srtp: /usr/lib/aarch64-linux-gnu/libcrypto.so
deps/libsrtp/test_srtp: deps/libsrtp/CMakeFiles/test_srtp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hady/teleop/gbot_ws/build/libdatachannel/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable test_srtp"
	cd /home/hady/teleop/gbot_ws/build/libdatachannel/deps/libsrtp && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_srtp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
deps/libsrtp/CMakeFiles/test_srtp.dir/build: deps/libsrtp/test_srtp
.PHONY : deps/libsrtp/CMakeFiles/test_srtp.dir/build

deps/libsrtp/CMakeFiles/test_srtp.dir/clean:
	cd /home/hady/teleop/gbot_ws/build/libdatachannel/deps/libsrtp && $(CMAKE_COMMAND) -P CMakeFiles/test_srtp.dir/cmake_clean.cmake
.PHONY : deps/libsrtp/CMakeFiles/test_srtp.dir/clean

deps/libsrtp/CMakeFiles/test_srtp.dir/depend:
	cd /home/hady/teleop/gbot_ws/build/libdatachannel && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hady/teleop/gbot_ws/libdatachannel /home/hady/teleop/gbot_ws/libdatachannel/deps/libsrtp /home/hady/teleop/gbot_ws/build/libdatachannel /home/hady/teleop/gbot_ws/build/libdatachannel/deps/libsrtp /home/hady/teleop/gbot_ws/build/libdatachannel/deps/libsrtp/CMakeFiles/test_srtp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : deps/libsrtp/CMakeFiles/test_srtp.dir/depend

