# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = "/home/himalaya/Desktop/space carving"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/himalaya/Desktop/space carving/cmake-build-debug"

# Include any dependencies generated for this target.
include CMakeFiles/space_carving.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/space_carving.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/space_carving.dir/flags.make

CMakeFiles/space_carving.dir/main.cpp.o: CMakeFiles/space_carving.dir/flags.make
CMakeFiles/space_carving.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/himalaya/Desktop/space carving/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/space_carving.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/space_carving.dir/main.cpp.o -c "/home/himalaya/Desktop/space carving/main.cpp"

CMakeFiles/space_carving.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/space_carving.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/himalaya/Desktop/space carving/main.cpp" > CMakeFiles/space_carving.dir/main.cpp.i

CMakeFiles/space_carving.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/space_carving.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/himalaya/Desktop/space carving/main.cpp" -o CMakeFiles/space_carving.dir/main.cpp.s

CMakeFiles/space_carving.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/space_carving.dir/main.cpp.o.requires

CMakeFiles/space_carving.dir/main.cpp.o.provides: CMakeFiles/space_carving.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/space_carving.dir/build.make CMakeFiles/space_carving.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/space_carving.dir/main.cpp.o.provides

CMakeFiles/space_carving.dir/main.cpp.o.provides.build: CMakeFiles/space_carving.dir/main.cpp.o


CMakeFiles/space_carving.dir/src/extract_contour.cpp.o: CMakeFiles/space_carving.dir/flags.make
CMakeFiles/space_carving.dir/src/extract_contour.cpp.o: ../src/extract_contour.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/himalaya/Desktop/space carving/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/space_carving.dir/src/extract_contour.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/space_carving.dir/src/extract_contour.cpp.o -c "/home/himalaya/Desktop/space carving/src/extract_contour.cpp"

CMakeFiles/space_carving.dir/src/extract_contour.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/space_carving.dir/src/extract_contour.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/himalaya/Desktop/space carving/src/extract_contour.cpp" > CMakeFiles/space_carving.dir/src/extract_contour.cpp.i

CMakeFiles/space_carving.dir/src/extract_contour.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/space_carving.dir/src/extract_contour.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/himalaya/Desktop/space carving/src/extract_contour.cpp" -o CMakeFiles/space_carving.dir/src/extract_contour.cpp.s

CMakeFiles/space_carving.dir/src/extract_contour.cpp.o.requires:

.PHONY : CMakeFiles/space_carving.dir/src/extract_contour.cpp.o.requires

CMakeFiles/space_carving.dir/src/extract_contour.cpp.o.provides: CMakeFiles/space_carving.dir/src/extract_contour.cpp.o.requires
	$(MAKE) -f CMakeFiles/space_carving.dir/build.make CMakeFiles/space_carving.dir/src/extract_contour.cpp.o.provides.build
.PHONY : CMakeFiles/space_carving.dir/src/extract_contour.cpp.o.provides

CMakeFiles/space_carving.dir/src/extract_contour.cpp.o.provides.build: CMakeFiles/space_carving.dir/src/extract_contour.cpp.o


CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.o: CMakeFiles/space_carving.dir/flags.make
CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.o: ../src/bg_constraints_carving.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/himalaya/Desktop/space carving/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.o -c "/home/himalaya/Desktop/space carving/src/bg_constraints_carving.cpp"

CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/himalaya/Desktop/space carving/src/bg_constraints_carving.cpp" > CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.i

CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/himalaya/Desktop/space carving/src/bg_constraints_carving.cpp" -o CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.s

CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.o.requires:

.PHONY : CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.o.requires

CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.o.provides: CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.o.requires
	$(MAKE) -f CMakeFiles/space_carving.dir/build.make CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.o.provides.build
.PHONY : CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.o.provides

CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.o.provides.build: CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.o


CMakeFiles/space_carving.dir/src/show3d.cpp.o: CMakeFiles/space_carving.dir/flags.make
CMakeFiles/space_carving.dir/src/show3d.cpp.o: ../src/show3d.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/himalaya/Desktop/space carving/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/space_carving.dir/src/show3d.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/space_carving.dir/src/show3d.cpp.o -c "/home/himalaya/Desktop/space carving/src/show3d.cpp"

CMakeFiles/space_carving.dir/src/show3d.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/space_carving.dir/src/show3d.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/himalaya/Desktop/space carving/src/show3d.cpp" > CMakeFiles/space_carving.dir/src/show3d.cpp.i

CMakeFiles/space_carving.dir/src/show3d.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/space_carving.dir/src/show3d.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/himalaya/Desktop/space carving/src/show3d.cpp" -o CMakeFiles/space_carving.dir/src/show3d.cpp.s

CMakeFiles/space_carving.dir/src/show3d.cpp.o.requires:

.PHONY : CMakeFiles/space_carving.dir/src/show3d.cpp.o.requires

CMakeFiles/space_carving.dir/src/show3d.cpp.o.provides: CMakeFiles/space_carving.dir/src/show3d.cpp.o.requires
	$(MAKE) -f CMakeFiles/space_carving.dir/build.make CMakeFiles/space_carving.dir/src/show3d.cpp.o.provides.build
.PHONY : CMakeFiles/space_carving.dir/src/show3d.cpp.o.provides

CMakeFiles/space_carving.dir/src/show3d.cpp.o.provides.build: CMakeFiles/space_carving.dir/src/show3d.cpp.o


CMakeFiles/space_carving.dir/src/carving.cpp.o: CMakeFiles/space_carving.dir/flags.make
CMakeFiles/space_carving.dir/src/carving.cpp.o: ../src/carving.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/himalaya/Desktop/space carving/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/space_carving.dir/src/carving.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/space_carving.dir/src/carving.cpp.o -c "/home/himalaya/Desktop/space carving/src/carving.cpp"

CMakeFiles/space_carving.dir/src/carving.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/space_carving.dir/src/carving.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/himalaya/Desktop/space carving/src/carving.cpp" > CMakeFiles/space_carving.dir/src/carving.cpp.i

CMakeFiles/space_carving.dir/src/carving.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/space_carving.dir/src/carving.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/himalaya/Desktop/space carving/src/carving.cpp" -o CMakeFiles/space_carving.dir/src/carving.cpp.s

CMakeFiles/space_carving.dir/src/carving.cpp.o.requires:

.PHONY : CMakeFiles/space_carving.dir/src/carving.cpp.o.requires

CMakeFiles/space_carving.dir/src/carving.cpp.o.provides: CMakeFiles/space_carving.dir/src/carving.cpp.o.requires
	$(MAKE) -f CMakeFiles/space_carving.dir/build.make CMakeFiles/space_carving.dir/src/carving.cpp.o.provides.build
.PHONY : CMakeFiles/space_carving.dir/src/carving.cpp.o.provides

CMakeFiles/space_carving.dir/src/carving.cpp.o.provides.build: CMakeFiles/space_carving.dir/src/carving.cpp.o


CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.o: CMakeFiles/space_carving.dir/flags.make
CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.o: ../src/pc_constraint_carving.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/himalaya/Desktop/space carving/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.o -c "/home/himalaya/Desktop/space carving/src/pc_constraint_carving.cpp"

CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/himalaya/Desktop/space carving/src/pc_constraint_carving.cpp" > CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.i

CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/himalaya/Desktop/space carving/src/pc_constraint_carving.cpp" -o CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.s

CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.o.requires:

.PHONY : CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.o.requires

CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.o.provides: CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.o.requires
	$(MAKE) -f CMakeFiles/space_carving.dir/build.make CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.o.provides.build
.PHONY : CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.o.provides

CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.o.provides.build: CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.o


CMakeFiles/space_carving.dir/src/fileio.cpp.o: CMakeFiles/space_carving.dir/flags.make
CMakeFiles/space_carving.dir/src/fileio.cpp.o: ../src/fileio.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/himalaya/Desktop/space carving/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/space_carving.dir/src/fileio.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/space_carving.dir/src/fileio.cpp.o -c "/home/himalaya/Desktop/space carving/src/fileio.cpp"

CMakeFiles/space_carving.dir/src/fileio.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/space_carving.dir/src/fileio.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/himalaya/Desktop/space carving/src/fileio.cpp" > CMakeFiles/space_carving.dir/src/fileio.cpp.i

CMakeFiles/space_carving.dir/src/fileio.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/space_carving.dir/src/fileio.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/himalaya/Desktop/space carving/src/fileio.cpp" -o CMakeFiles/space_carving.dir/src/fileio.cpp.s

CMakeFiles/space_carving.dir/src/fileio.cpp.o.requires:

.PHONY : CMakeFiles/space_carving.dir/src/fileio.cpp.o.requires

CMakeFiles/space_carving.dir/src/fileio.cpp.o.provides: CMakeFiles/space_carving.dir/src/fileio.cpp.o.requires
	$(MAKE) -f CMakeFiles/space_carving.dir/build.make CMakeFiles/space_carving.dir/src/fileio.cpp.o.provides.build
.PHONY : CMakeFiles/space_carving.dir/src/fileio.cpp.o.provides

CMakeFiles/space_carving.dir/src/fileio.cpp.o.provides.build: CMakeFiles/space_carving.dir/src/fileio.cpp.o


# Object files for target space_carving
space_carving_OBJECTS = \
"CMakeFiles/space_carving.dir/main.cpp.o" \
"CMakeFiles/space_carving.dir/src/extract_contour.cpp.o" \
"CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.o" \
"CMakeFiles/space_carving.dir/src/show3d.cpp.o" \
"CMakeFiles/space_carving.dir/src/carving.cpp.o" \
"CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.o" \
"CMakeFiles/space_carving.dir/src/fileio.cpp.o"

# External object files for target space_carving
space_carving_EXTERNAL_OBJECTS =

space_carving: CMakeFiles/space_carving.dir/main.cpp.o
space_carving: CMakeFiles/space_carving.dir/src/extract_contour.cpp.o
space_carving: CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.o
space_carving: CMakeFiles/space_carving.dir/src/show3d.cpp.o
space_carving: CMakeFiles/space_carving.dir/src/carving.cpp.o
space_carving: CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.o
space_carving: CMakeFiles/space_carving.dir/src/fileio.cpp.o
space_carving: CMakeFiles/space_carving.dir/build.make
space_carving: /usr/local/lib/libopencv_gapi.so.4.0.1
space_carving: /usr/local/lib/libopencv_stitching.so.4.0.1
space_carving: /usr/local/lib/libopencv_aruco.so.4.0.1
space_carving: /usr/local/lib/libopencv_bgsegm.so.4.0.1
space_carving: /usr/local/lib/libopencv_bioinspired.so.4.0.1
space_carving: /usr/local/lib/libopencv_ccalib.so.4.0.1
space_carving: /usr/local/lib/libopencv_dnn_objdetect.so.4.0.1
space_carving: /usr/local/lib/libopencv_dpm.so.4.0.1
space_carving: /usr/local/lib/libopencv_face.so.4.0.1
space_carving: /usr/local/lib/libopencv_freetype.so.4.0.1
space_carving: /usr/local/lib/libopencv_fuzzy.so.4.0.1
space_carving: /usr/local/lib/libopencv_hdf.so.4.0.1
space_carving: /usr/local/lib/libopencv_hfs.so.4.0.1
space_carving: /usr/local/lib/libopencv_img_hash.so.4.0.1
space_carving: /usr/local/lib/libopencv_line_descriptor.so.4.0.1
space_carving: /usr/local/lib/libopencv_reg.so.4.0.1
space_carving: /usr/local/lib/libopencv_rgbd.so.4.0.1
space_carving: /usr/local/lib/libopencv_saliency.so.4.0.1
space_carving: /usr/local/lib/libopencv_sfm.so.4.0.1
space_carving: /usr/local/lib/libopencv_stereo.so.4.0.1
space_carving: /usr/local/lib/libopencv_structured_light.so.4.0.1
space_carving: /usr/local/lib/libopencv_superres.so.4.0.1
space_carving: /usr/local/lib/libopencv_surface_matching.so.4.0.1
space_carving: /usr/local/lib/libopencv_tracking.so.4.0.1
space_carving: /usr/local/lib/libopencv_videostab.so.4.0.1
space_carving: /usr/local/lib/libopencv_viz.so.4.0.1
space_carving: /usr/local/lib/libopencv_xfeatures2d.so.4.0.1
space_carving: /usr/local/lib/libopencv_xobjdetect.so.4.0.1
space_carving: /usr/local/lib/libopencv_xphoto.so.4.0.1
space_carving: /usr/local/lib/libopencv_shape.so.4.0.1
space_carving: /usr/local/lib/libopencv_datasets.so.4.0.1
space_carving: /usr/local/lib/libopencv_plot.so.4.0.1
space_carving: /usr/local/lib/libopencv_text.so.4.0.1
space_carving: /usr/local/lib/libopencv_dnn.so.4.0.1
space_carving: /usr/local/lib/libopencv_ml.so.4.0.1
space_carving: /usr/local/lib/libopencv_phase_unwrapping.so.4.0.1
space_carving: /usr/local/lib/libopencv_optflow.so.4.0.1
space_carving: /usr/local/lib/libopencv_ximgproc.so.4.0.1
space_carving: /usr/local/lib/libopencv_video.so.4.0.1
space_carving: /usr/local/lib/libopencv_objdetect.so.4.0.1
space_carving: /usr/local/lib/libopencv_calib3d.so.4.0.1
space_carving: /usr/local/lib/libopencv_features2d.so.4.0.1
space_carving: /usr/local/lib/libopencv_flann.so.4.0.1
space_carving: /usr/local/lib/libopencv_highgui.so.4.0.1
space_carving: /usr/local/lib/libopencv_videoio.so.4.0.1
space_carving: /usr/local/lib/libopencv_imgcodecs.so.4.0.1
space_carving: /usr/local/lib/libopencv_photo.so.4.0.1
space_carving: /usr/local/lib/libopencv_imgproc.so.4.0.1
space_carving: /usr/local/lib/libopencv_core.so.4.0.1
space_carving: CMakeFiles/space_carving.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/himalaya/Desktop/space carving/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX executable space_carving"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/space_carving.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/space_carving.dir/build: space_carving

.PHONY : CMakeFiles/space_carving.dir/build

CMakeFiles/space_carving.dir/requires: CMakeFiles/space_carving.dir/main.cpp.o.requires
CMakeFiles/space_carving.dir/requires: CMakeFiles/space_carving.dir/src/extract_contour.cpp.o.requires
CMakeFiles/space_carving.dir/requires: CMakeFiles/space_carving.dir/src/bg_constraints_carving.cpp.o.requires
CMakeFiles/space_carving.dir/requires: CMakeFiles/space_carving.dir/src/show3d.cpp.o.requires
CMakeFiles/space_carving.dir/requires: CMakeFiles/space_carving.dir/src/carving.cpp.o.requires
CMakeFiles/space_carving.dir/requires: CMakeFiles/space_carving.dir/src/pc_constraint_carving.cpp.o.requires
CMakeFiles/space_carving.dir/requires: CMakeFiles/space_carving.dir/src/fileio.cpp.o.requires

.PHONY : CMakeFiles/space_carving.dir/requires

CMakeFiles/space_carving.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/space_carving.dir/cmake_clean.cmake
.PHONY : CMakeFiles/space_carving.dir/clean

CMakeFiles/space_carving.dir/depend:
	cd "/home/himalaya/Desktop/space carving/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/himalaya/Desktop/space carving" "/home/himalaya/Desktop/space carving" "/home/himalaya/Desktop/space carving/cmake-build-debug" "/home/himalaya/Desktop/space carving/cmake-build-debug" "/home/himalaya/Desktop/space carving/cmake-build-debug/CMakeFiles/space_carving.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/space_carving.dir/depend

