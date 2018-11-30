# Transform360

Transform360 is a video/image filter that transforms a 360 video from one projection to another. Usually, the input projection is equirectangular and the output projection is cubemap.
We also keep the previous version of the transform, Transform_V1, in the file vf_transform_v1.c

## Advantages of Transform360
1. Transform360 achieves better performance in memory usage and visual quality. For many different algorithm and parameter settings in real-world applications, it also achieves better processing speed.
2. Transform360 allows people to have more control on the quality of the output video frames, and even the quality of different regions within a frame.
3. Transform360 separates the computation and transform logic from ffmpeg. Thus, people have much more flexibility to use different resources to develop the transforms, but do not need to be concerned with the details of the ffmpeg implementation/integration.

## To Build And Use Transform360

### Building on Ubuntu

Transform360 is implemented in C++ and is invoked by ffmpeg video filter. To build and use Transform360, follow these steps (special thanks to https://github.com/danrossi):

1. Checkout `transform360`
2. Checkout `ffmpeg` source
3. Install ffmpeg, dev versions of openCV and codec libraries that you need, e.g.
```
sudo apt-get install ffmpeg
sudo apt-get install libopencv-dev
sudo apt-get install nasm libxvidcore-dev libass-dev libfdk-aac-dev libvpx-dev libx264-dev
```
4. Build and install transform360 in `Transform360` folder:
```
cmake ./
make
sudo make install
```
5. Copy `vf_transform360.c` to the `libavfilter` subdirectory in ffmpeg source.
6. Edit `libavfilter/allfilters.c` and register the filter by adding the following line in the video filter registration section:

```
extern AVFilter ff_vf_transform360;
```

For older ffmpeg versions (i.e., if you see existing filters are registered with REGISTER_FILTER), please instead add

```
REGISTER_FILTER(TRANSFORM360, transform360, vf);
```

7. Edit `libavfilter/Makefile` and add the filter to adding the following line in the filter section:

```
OBJS-$(CONFIG_TRANSFORM360_FILTER) += vf_transform360.o
```

8. Edit `vf_transform360.c` in `libavfilter` folder

Change the include path from
```
#include "transform360/VideoFrameTransformHandler.h"
#include "transform360/VideoFrameTransformHelper.h"
```

to
```
#include "Transform360/Library/VideoFrameTransformHandler.h"
#include "Transform360/Library/VideoFrameTransformHelper.h"
```

9. Configure ffmpeg in the source folder:

```
./configure --prefix=/usr/local/transform/ffmpeg --enable-gpl --enable-nonfree --enable-libass --enable-libfdk-aac --enable-libfreetype --enable-libvpx --enable-libx264 --enable-libxvid --enable-libopencv --extra-libs='-lTransform360 -lstdc++'
```

10. make ffmpeg

```
make
```

11. use local binary with `./ffmpeg` or by installing it with `make install`

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
   line: `extern AVFilter ff_vf_transform_v1;` For older ffmpeg versions
   (i.e., if you see existing filters are registered with REGISTER_FILTER),
   please instead add `REGISTER_FILTER(TRANSFORM_V1, transform_v1, vf);`
   in the video filter registration section.
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

### License

Transform360 and Transform_V1 are BSD licensed, as found in the LICENSE file.
