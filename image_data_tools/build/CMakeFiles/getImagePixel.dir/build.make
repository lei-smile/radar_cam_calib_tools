# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /opt/cmake-3.14.5-Linux-x86_64/bin/cmake

# The command to remove a file.
RM = /opt/cmake-3.14.5-Linux-x86_64/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/wulei/calib/calib_tools/image_data_tools

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wulei/calib/calib_tools/image_data_tools/build

# Include any dependencies generated for this target.
include CMakeFiles/getImagePixel.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/getImagePixel.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/getImagePixel.dir/flags.make

CMakeFiles/getImagePixel.dir/src/getImagePixel.cpp.o: CMakeFiles/getImagePixel.dir/flags.make
CMakeFiles/getImagePixel.dir/src/getImagePixel.cpp.o: ../src/getImagePixel.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wulei/calib/calib_tools/image_data_tools/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/getImagePixel.dir/src/getImagePixel.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/getImagePixel.dir/src/getImagePixel.cpp.o -c /home/wulei/calib/calib_tools/image_data_tools/src/getImagePixel.cpp

CMakeFiles/getImagePixel.dir/src/getImagePixel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/getImagePixel.dir/src/getImagePixel.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wulei/calib/calib_tools/image_data_tools/src/getImagePixel.cpp > CMakeFiles/getImagePixel.dir/src/getImagePixel.cpp.i

CMakeFiles/getImagePixel.dir/src/getImagePixel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/getImagePixel.dir/src/getImagePixel.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wulei/calib/calib_tools/image_data_tools/src/getImagePixel.cpp -o CMakeFiles/getImagePixel.dir/src/getImagePixel.cpp.s

# Object files for target getImagePixel
getImagePixel_OBJECTS = \
"CMakeFiles/getImagePixel.dir/src/getImagePixel.cpp.o"

# External object files for target getImagePixel
getImagePixel_EXTERNAL_OBJECTS =

getImagePixel: CMakeFiles/getImagePixel.dir/src/getImagePixel.cpp.o
getImagePixel: CMakeFiles/getImagePixel.dir/build.make
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
getImagePixel: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
getImagePixel: CMakeFiles/getImagePixel.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wulei/calib/calib_tools/image_data_tools/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable getImagePixel"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/getImagePixel.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/getImagePixel.dir/build: getImagePixel

.PHONY : CMakeFiles/getImagePixel.dir/build

CMakeFiles/getImagePixel.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/getImagePixel.dir/cmake_clean.cmake
.PHONY : CMakeFiles/getImagePixel.dir/clean

CMakeFiles/getImagePixel.dir/depend:
	cd /home/wulei/calib/calib_tools/image_data_tools/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wulei/calib/calib_tools/image_data_tools /home/wulei/calib/calib_tools/image_data_tools /home/wulei/calib/calib_tools/image_data_tools/build /home/wulei/calib/calib_tools/image_data_tools/build /home/wulei/calib/calib_tools/image_data_tools/build/CMakeFiles/getImagePixel.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/getImagePixel.dir/depend

