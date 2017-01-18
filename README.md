# Transform360

Transform360 is a video filter that transforms 360 video in equirectangular projection into a cubemap projection. 
We also keep the previous version of the transform, Transform_V1, in the file vf_transform_v1.c

## Advantages of Transform360
1. Transform360 achieves better performance in memory usage and visual quality. For many different algorithm and parameter settings in real-world applications, it also achieves better processing speed.
2. Transform360 allows people to have more control on the quality of the output video frames, and even the quality of different regions within a frame.
3. Transform360 separates the computation and transform logic from ffmpeg. Thus, people have much more flexibility to use different resources to develop the transforms, but do not need to be concerned with the details of the ffmpeg implementation/integration.

## To Build And Use Transform360

### Building

Transform360 is implemented in C++ and is invoked by an ffmpeg video filter. To build and use Transform360, follow these steps:

1. Checkout the source for the Transform360, openCV and ffmpeg.
2. Build .cpp and .h files in Transform360, together with openCV, as a library, where these files are dependent on openCV.
3. Add the Transform360 library file to the extra-libs of ffmpeg.
4. Copy `vf_transform360.c` to the libavfilter subdirectory in ffmpeg source.
5. Edit `libavfilter/allfilters.c` and register the filter by adding the
   line: `REGISTER_FILTER(TRANSFORM360, transform360, vf);` in the video filter registration section.
6. Edit `libavfilter/Makefile` and add the filter to adding the
   line: `OBJS-$(CONFIG_TRANSFORM360_FILTER) += vf_transform360.o` in the filter section.
7. Configure and build ffmpeg as usual.

#### More details about building Transform360 using CMake

1. CMakeLists.txt:
cmake_minimum_required(VERSION 2.8)

project (libtransform360)
set(CMAKE_BUILD_TYPE Release)

set (SOURCE_DIR "${PROJECT_SOURCE_DIR}/")

SET(SOURCES
${SOURCE_DIR}/VideoFrameTransform.cpp
${SOURCE_DIR}/VideoFrameTransformHandler.cpp)

SET(HEADERS
${SOURCE_DIR}VideoFrameTransform.h
${SOURCE_DIR}VideoFrameTransformHandler.h
${SOURCE_DIR}VideoFrameTransformHelper.h)

list(APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_library(Transform360 STATIC ${SOURCES} ${HEADERS})
target_link_libraries(Transform360 ${PROJECT_LINK_LIBS} )

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(Transform360 ${OpenCV_LIBS})

install(TARGETS Transform360
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
COMPONENT library
)
install(DIRECTORY . DESTINATION include/Transform360 FILES_MATCHING PATTERN "*.h")

2. Configuration for ffmpeg
./configure --prefix=/usr/local --enable-gpl --enable-nonfree --enable-libass --enable-libfdk-aac --enable-libfreetype --enable-libmp3lame --enable-libopus --enable-libtheora --enable-libvorbis --enable-libvpx --enable-libx264 --enable-libxvid --enable-libopencv --extra-libs='-lTransform360 -lstdc++'

### Running

Check out the options for the filter by running `ffmpeg -h filter=transform360`.

A typical example looks something like:

``` sh
ffmpeg -i input.mp4 
    -vf transform360="input_stereo_format=MONO
    :cube_edge_length=512
    :interpolation_alg=cubic
    :enable_low_pass_filter=1
    :enable_multi_threading=1
    :num_horizontal_segments=32
    :num_vertical_segments=15
    :adjust_kernel=1"
    output.mp4
```

## To Build And Use Transform_V1

### Building

Transform is implemented as an ffmpeg video filter. To build Transform, follow these steps:

1. Checkout the source for ffmpeg.
2. Copy `vf_transform_v1.c` to the libavfilter subdirectory in ffmpeg source.
3. Edit `libavfilter/allfilters.c` and register the filter by adding the
   line: `REGISTER_FILTER(TRANSFORM_V1, transform_v1, vf);` in the video filter registration section.
4. Edit `libavfilter/Makefile` and add the filter to adding the
   line: `OBJS-$(CONFIG_TRANSFORM_V1_FILTER) += vf_transform_v1.o` in the filter section.
5. Configure and build ffmpeg as usual.

### Running

Check out the options for the filter by running `ffmpeg -h filter=transform_v1`.

A typical example looks something like:

``` sh
ffmpeg -i input.mp4 
    -vf transform_v1="input_stereo_format=MONO
    :w_subdivisions=4
    :h_subdivisions=4
    :max_cube_edge_length=512" 
    output.mp4
```

