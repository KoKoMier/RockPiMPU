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
CMAKE_SOURCE_DIR = /home/cyh/FlightCode/RockPiMPU

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cyh/FlightCode/RockPiMPU/build

# Include any dependencies generated for this target.
include src/CMakeFiles/RPIMPU.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/RPIMPU.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/RPIMPU.dir/flags.make

src/CMakeFiles/RPIMPU.dir/MPU6500/MPU6500.cpp.o: src/CMakeFiles/RPIMPU.dir/flags.make
src/CMakeFiles/RPIMPU.dir/MPU6500/MPU6500.cpp.o: ../src/MPU6500/MPU6500.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cyh/FlightCode/RockPiMPU/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/RPIMPU.dir/MPU6500/MPU6500.cpp.o"
	cd /home/cyh/FlightCode/RockPiMPU/build/src && /home/cyh/openwrt-SingleFlight/openwrt-sdk-rockchip-armv8_gcc-11.2.0_musl.Linux-x86_64/staging_dir/toolchain-aarch64_generic_gcc-11.2.0_musl/bin/aarch64-openwrt-linux-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RPIMPU.dir/MPU6500/MPU6500.cpp.o -c /home/cyh/FlightCode/RockPiMPU/src/MPU6500/MPU6500.cpp

src/CMakeFiles/RPIMPU.dir/MPU6500/MPU6500.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RPIMPU.dir/MPU6500/MPU6500.cpp.i"
	cd /home/cyh/FlightCode/RockPiMPU/build/src && /home/cyh/openwrt-SingleFlight/openwrt-sdk-rockchip-armv8_gcc-11.2.0_musl.Linux-x86_64/staging_dir/toolchain-aarch64_generic_gcc-11.2.0_musl/bin/aarch64-openwrt-linux-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cyh/FlightCode/RockPiMPU/src/MPU6500/MPU6500.cpp > CMakeFiles/RPIMPU.dir/MPU6500/MPU6500.cpp.i

src/CMakeFiles/RPIMPU.dir/MPU6500/MPU6500.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RPIMPU.dir/MPU6500/MPU6500.cpp.s"
	cd /home/cyh/FlightCode/RockPiMPU/build/src && /home/cyh/openwrt-SingleFlight/openwrt-sdk-rockchip-armv8_gcc-11.2.0_musl.Linux-x86_64/staging_dir/toolchain-aarch64_generic_gcc-11.2.0_musl/bin/aarch64-openwrt-linux-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cyh/FlightCode/RockPiMPU/src/MPU6500/MPU6500.cpp -o CMakeFiles/RPIMPU.dir/MPU6500/MPU6500.cpp.s

# Object files for target RPIMPU
RPIMPU_OBJECTS = \
"CMakeFiles/RPIMPU.dir/MPU6500/MPU6500.cpp.o"

# External object files for target RPIMPU
RPIMPU_EXTERNAL_OBJECTS =

src/libRPIMPU.a: src/CMakeFiles/RPIMPU.dir/MPU6500/MPU6500.cpp.o
src/libRPIMPU.a: src/CMakeFiles/RPIMPU.dir/build.make
src/libRPIMPU.a: src/CMakeFiles/RPIMPU.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cyh/FlightCode/RockPiMPU/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libRPIMPU.a"
	cd /home/cyh/FlightCode/RockPiMPU/build/src && $(CMAKE_COMMAND) -P CMakeFiles/RPIMPU.dir/cmake_clean_target.cmake
	cd /home/cyh/FlightCode/RockPiMPU/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RPIMPU.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/RPIMPU.dir/build: src/libRPIMPU.a

.PHONY : src/CMakeFiles/RPIMPU.dir/build

src/CMakeFiles/RPIMPU.dir/clean:
	cd /home/cyh/FlightCode/RockPiMPU/build/src && $(CMAKE_COMMAND) -P CMakeFiles/RPIMPU.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/RPIMPU.dir/clean

src/CMakeFiles/RPIMPU.dir/depend:
	cd /home/cyh/FlightCode/RockPiMPU/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cyh/FlightCode/RockPiMPU /home/cyh/FlightCode/RockPiMPU/src /home/cyh/FlightCode/RockPiMPU/build /home/cyh/FlightCode/RockPiMPU/build/src /home/cyh/FlightCode/RockPiMPU/build/src/CMakeFiles/RPIMPU.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/RPIMPU.dir/depend

