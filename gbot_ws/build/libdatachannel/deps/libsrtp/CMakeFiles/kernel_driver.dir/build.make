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
include deps/libsrtp/CMakeFiles/kernel_driver.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include deps/libsrtp/CMakeFiles/kernel_driver.dir/compiler_depend.make

# Include the progress variables for this target.
include deps/libsrtp/CMakeFiles/kernel_driver.dir/progress.make

# Include the compile flags for this target's objects.
include deps/libsrtp/CMakeFiles/kernel_driver.dir/flags.make

deps/libsrtp/CMakeFiles/kernel_driver.dir/crypto/test/kernel_driver.c.o: deps/libsrtp/CMakeFiles/kernel_driver.dir/flags.make
deps/libsrtp/CMakeFiles/kernel_driver.dir/crypto/test/kernel_driver.c.o: /home/hady/teleop/gbot_ws/libdatachannel/deps/libsrtp/crypto/test/kernel_driver.c
deps/libsrtp/CMakeFiles/kernel_driver.dir/crypto/test/kernel_driver.c.o: deps/libsrtp/CMakeFiles/kernel_driver.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hady/teleop/gbot_ws/build/libdatachannel/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object deps/libsrtp/CMakeFiles/kernel_driver.dir/crypto/test/kernel_driver.c.o"
	cd /home/hady/teleop/gbot_ws/build/libdatachannel/deps/libsrtp && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT deps/libsrtp/CMakeFiles/kernel_driver.dir/crypto/test/kernel_driver.c.o -MF CMakeFiles/kernel_driver.dir/crypto/test/kernel_driver.c.o.d -o CMakeFiles/kernel_driver.dir/crypto/test/kernel_driver.c.o -c /home/hady/teleop/gbot_ws/libdatachannel/deps/libsrtp/crypto/test/kernel_driver.c

deps/libsrtp/CMakeFiles/kernel_driver.dir/crypto/test/kernel_driver.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/kernel_driver.dir/crypto/test/kernel_driver.c.i"
	cd /home/hady/teleop/gbot_ws/build/libdatachannel/deps/libsrtp && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/hady/teleop/gbot_ws/libdatachannel/deps/libsrtp/crypto/test/kernel_driver.c > CMakeFiles/kernel_driver.dir/crypto/test/kernel_driver.c.i

deps/libsrtp/CMakeFiles/kernel_driver.dir/crypto/test/kernel_driver.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/kernel_driver.dir/crypto/test/kernel_driver.c.s"
	cd /home/hady/teleop/gbot_ws/build/libdatachannel/deps/libsrtp && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/hady/teleop/gbot_ws/libdatachannel/deps/libsrtp/crypto/test/kernel_driver.c -o CMakeFiles/kernel_driver.dir/crypto/test/kernel_driver.c.s

deps/libsrtp/CMakeFiles/kernel_driver.dir/test/getopt_s.c.o: deps/libsrtp/CMakeFiles/kernel_driver.dir/flags.make
deps/libsrtp/CMakeFiles/kernel_driver.dir/test/getopt_s.c.o: /home/hady/teleop/gbot_ws/libdatachannel/deps/libsrtp/test/getopt_s.c
deps/libsrtp/CMakeFiles/kernel_driver.dir/test/getopt_s.c.o: deps/libsrtp/CMakeFiles/kernel_driver.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hady/teleop/gbot_ws/build/libdatachannel/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object deps/libsrtp/CMakeFiles/kernel_driver.dir/test/getopt_s.c.o"
	cd /home/hady/teleop/gbot_ws/build/libdatachannel/deps/libsrtp && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT deps/libsrtp/CMakeFiles/kernel_driver.dir/test/getopt_s.c.o -MF CMakeFiles/kernel_driver.dir/test/getopt_s.c.o.d -o CMakeFiles/kernel_driver.dir/test/getopt_s.c.o -c /home/hady/teleop/gbot_ws/libdatachannel/deps/libsrtp/test/getopt_s.c

deps/libsrtp/CMakeFiles/kernel_driver.dir/test/getopt_s.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/kernel_driver.dir/test/getopt_s.c.i"
	cd /home/hady/teleop/gbot_ws/build/libdatachannel/deps/libsrtp && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/hady/teleop/gbot_ws/libdatachannel/deps/libsrtp/test/getopt_s.c > CMakeFiles/kernel_driver.dir/test/getopt_s.c.i

deps/libsrtp/CMakeFiles/kernel_driver.dir/test/getopt_s.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/kernel_driver.dir/test/getopt_s.c.s"
	cd /home/hady/teleop/gbot_ws/build/libdatachannel/deps/libsrtp && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/hady/teleop/gbot_ws/libdatachannel/deps/libsrtp/test/getopt_s.c -o CMakeFiles/kernel_driver.dir/test/getopt_s.c.s

# Object files for target kernel_driver
kernel_driver_OBJECTS = \
"CMakeFiles/kernel_driver.dir/crypto/test/kernel_driver.c.o" \
"CMakeFiles/kernel_driver.dir/test/getopt_s.c.o"

# External object files for target kernel_driver
kernel_driver_EXTERNAL_OBJECTS =

deps/libsrtp/kernel_driver: deps/libsrtp/CMakeFiles/kernel_driver.dir/crypto/test/kernel_driver.c.o
deps/libsrtp/kernel_driver: deps/libsrtp/CMakeFiles/kernel_driver.dir/test/getopt_s.c.o
deps/libsrtp/kernel_driver: deps/libsrtp/CMakeFiles/kernel_driver.dir/build.make
deps/libsrtp/kernel_driver: deps/libsrtp/libsrtp2.a
deps/libsrtp/kernel_driver: /usr/lib/aarch64-linux-gnu/libcrypto.so
deps/libsrtp/kernel_driver: deps/libsrtp/CMakeFiles/kernel_driver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hady/teleop/gbot_ws/build/libdatachannel/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable kernel_driver"
	cd /home/hady/teleop/gbot_ws/build/libdatachannel/deps/libsrtp && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kernel_driver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
deps/libsrtp/CMakeFiles/kernel_driver.dir/build: deps/libsrtp/kernel_driver
.PHONY : deps/libsrtp/CMakeFiles/kernel_driver.dir/build

deps/libsrtp/CMakeFiles/kernel_driver.dir/clean:
	cd /home/hady/teleop/gbot_ws/build/libdatachannel/deps/libsrtp && $(CMAKE_COMMAND) -P CMakeFiles/kernel_driver.dir/cmake_clean.cmake
.PHONY : deps/libsrtp/CMakeFiles/kernel_driver.dir/clean

deps/libsrtp/CMakeFiles/kernel_driver.dir/depend:
	cd /home/hady/teleop/gbot_ws/build/libdatachannel && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hady/teleop/gbot_ws/libdatachannel /home/hady/teleop/gbot_ws/libdatachannel/deps/libsrtp /home/hady/teleop/gbot_ws/build/libdatachannel /home/hady/teleop/gbot_ws/build/libdatachannel/deps/libsrtp /home/hady/teleop/gbot_ws/build/libdatachannel/deps/libsrtp/CMakeFiles/kernel_driver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : deps/libsrtp/CMakeFiles/kernel_driver.dir/depend

