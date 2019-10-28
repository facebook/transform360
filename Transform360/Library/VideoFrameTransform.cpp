/**
 * Copyright (c) 2015-present, Facebook, Inc.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

/**
 * This software contains some transform algorithms to transform
 * equirectangular panorama video frames to cubemap images
 */

#include "VideoFrameTransform.h"

#include <algorithm>
#include <assert.h>
#include <array>
#include <cmath>
#include <math.h>
#include <memory>
#include <set>
#include <stdio.h>

#ifdef FACEBOOK_LAYOUT
#include "FBVideoFrameTransformUtilities.h"
#endif

using namespace cv;
using namespace std;

constexpr float kCubemapSideDistance = 0.5f;
static const float kXHalf = 0.5;
static const float kYHalf = 0.5;
static double kEpsilon = 1e-9;
static double kSphereArea = 4 * M_PI;
static double kFov = 0.5333 * M_PI;

// cube transform parameters
static const array<float, 3> P0 = {-0.5f,-0.5f,-0.5f };
static const array<float, 3> P1 = { 0.5f,-0.5f,-0.5f };
static const array<float, 3> P3 = { 0.5f, 0.5f,-0.5f };
static const array<float, 3> P4 = {-0.5f,-0.5f, 0.5f };
static const array<float, 3> P5 = { 0.5f,-0.5f, 0.5f };
static const array<float, 3> P6 = {-0.5f, 0.5f, 0.5f };

static const array<float, 3> PX = { 1.0f, 0.0f, 0.0f};
static const array<float, 3> PY = { 0.0f, 1.0f, 0.0f};
static const array<float, 3> PZ = { 0.0f, 0.0f, 1.0f};
static const array<float, 3> NX = {-1.0f, 0.0f, 0.0f};
static const array<float, 3> NZ = { 0.0f, 0.0f,-1.0f};

// Input normalized ray direction and offset vector,
// return distance to intersection
static float intersectSphereOffset(
  float x,
  float y,
  float z,
  float ox,
  float oy,
  float oz) {
  float loc;
  float odot;
  float root;

  loc = x * -ox + y * -oy + z * -oz ;
  odot = ox * ox + oy * oy + oz * oz;

  root = loc * loc - odot + 1.0;

  if ( root <= 0.0f ) {
    return 0.0f;
  }

  root = sqrtf( root );

  if ( root < loc ) {
    return 0.0f;
  }

  return root - loc;
}

// Calculate 1D kernel
static Mat calculateKernel(float sigma) {
  // kernel box length is 2x the sigma
  int boxHalfLength = sigma * 2;
  Mat kernel = Mat::zeros(1, boxHalfLength * 2 + 1, CV_32F);

  float sum = 0;
  float sigmaComponent = std::abs(sigma) < kEpsilon ?
    0 : 0.5 / (sigma * sigma);
  for (int u = -boxHalfLength; u <= boxHalfLength; ++u) {
    float value = expf(-(u * u * sigmaComponent));
    kernel.at<float>(0, u + boxHalfLength) = value;
    sum += value;
  }

  // Normalize
  kernel /= sum;
  return kernel;
}

// We need to end up with X and Y coordinates in the range [0..1).
// Horizontally wrapping is easy: 1.25 becomes 0.25, -0.25 becomes 0.75.
// Vertically, if we pass through the north pole, we start coming back 'down'
// in the Y direction (ie, a reflection from the boundary) but we also are
// on the opposite side of the sphere so the X value changes by 0.5.
static void normalize_equirectangular(
    float x,
    float y,
    float *xout,
    float *yout) {
    if (y >= 1.0f) {
        // Example: y = 1.25 ; 2.0 - 1.25 = 0.75.
        y = 2.0f - y;
        x += 0.5f;
    } else if (y < 0.0f) {
        y = -y;
        x += 0.5f;
    }

    if (x >= 1.0f) {
        int ipart = (int) x;
        x -= ipart;
    } else if (x < 0.0f) {
        // Example: x = -1.25.  ipart = 1. x += 2 so x = 0.25.
        int ipart = (int) (-x);
        x += (ipart + 1);
    }

    *xout = x;
    *yout = y;
}

// Calculate the distance, in terms of radians, between two points on the sphere
static double angularDistance(
  double yaw1,
  double pitch1,
  double yaw2,
  double pitch2) {
  return acos(sin(pitch1) * sin(pitch2) +
    cos(pitch1) * cos(pitch2) * cos(yaw1 - yaw2));
}

static double samplingArc(double offset, double renderArc) {
  return M_PI - 2 * atan2(cos(0.5 * renderArc) - offset, sin(0.5 * renderArc));
}

static double sphericalArea(double angle) {
  return (1 - cos(0.5 * angle)) * 2 * M_PI;
}

static double getEffectiveRatio(double angularDist, double offset, double fov) {
  double majorAxisScaling;
  if (angularDist - kEpsilon > fov / 2) {
    if (angularDist + fov / 2 > M_PI) {
      double edge1 = samplingArc(offset,
        (2 * M_PI - angularDist - fov / 2) * 2) / 2;
      double edge2 = samplingArc(offset, (angularDist - fov / 2) * 2) / 2;
      majorAxisScaling = (2 * M_PI - edge1 - edge2) / fov;
    } else {
      majorAxisScaling = (samplingArc(offset, 2 * angularDist + fov) -
      samplingArc(offset, 2 * angularDist - fov)) / 2 / fov;
    }
  } else {
    majorAxisScaling = (samplingArc(offset, 2 * angularDist + fov) +
      samplingArc(offset, fov - 2 * angularDist)) / 2 / fov;
  }

  double distToCoVertex = angularDistance(
    angularDist,
    0.5 * fov,
    0.0,
    0.0);
  double minorAxisScaling = samplingArc(
    offset, distToCoVertex * 2) / (distToCoVertex * 2);

  return min(
    majorAxisScaling * minorAxisScaling * sphericalArea(fov) / kSphereArea,
    1.0);
}

static double getEffectiveRatio(double angularDist, double offset) {
  return getEffectiveRatio(angularDist, offset, kFov);
}

// Filter frame plane using the kernels
void VideoFrameTransform::filterSegment(
  const Mat& inputMat,
  Mat& outputMat,
  const Mat& kernelX,
  const Mat& kernelY,
  int left,
  int top,
  int width,
  int height,
  int imagePlaneIndex) {
  try {
    Rect segmentRect(left, top, width, height);
    Mat inputSegment(inputMat(segmentRect));
    Mat outputSegment(outputMat(segmentRect));

    // Filtering using two 1D kernels
    sepFilter2D(
      inputSegment,
      outputSegment,
      -1,
      kernelX,
      kernelY,
      Point(-1,-1)  /* anchor */,
      0  /* delta */,
      BORDER_REPLICATE);
  } catch (const exception& ex) {
    printf(
      "Could not filter segment for the plane %d. Error: %s\n",
      imagePlaneIndex,
      ex.what());
  }
}

VideoFrameTransform::VideoFrameTransform(
  FrameTransformContext* ctx) {
  memcpy(&ctx_, ctx,  sizeof(*ctx));
}

void VideoFrameTransform::generateKernelAndFilteringConfig(
  int top,
  int bottom,
  float angle,
  float sigmaY,
  const Mat& kernelY,
  int inputWidth,
  int inputHeight,
  int transformMatPlaneIndex) {
  float sigmaX = min(
    0.5 * inputWidth, sigmaY / (
      cos(angle) + kEpsilon));

  // Build a basic 1D kernel along X direction
  Mat kernelX = calculateKernel(sigmaX);

  int numHorizontalSegments = ctx_.adjust_kernel ?
    ctx_.num_horizontal_segments : 1;
  vector<SegmentFilteringConfig> configs;
  int segmentWidth = ceil(1.0 * inputWidth / numHorizontalSegments);

  // Will be used to normalize the kernel scaling factor
  double baseEffectiveRatio = getEffectiveRatio(0.0, 0.0);

  // Calculate the kernel and filtering config for each of the horizontal
  // tile/segment. These tiles are all in the same vertical segment which is
  // created in generateKernelsAndFilteringConfigs.
  for (int i = 0; i <  numHorizontalSegments &&
      i * segmentWidth < inputWidth; ++i) {
    segmentFilteringConfigs_[transformMatPlaneIndex].emplace_back(
      SegmentFilteringConfig(
        i * segmentWidth,
        top,
        min(segmentWidth, inputWidth - i * segmentWidth),
        bottom - top + 1));

    if (ctx_.adjust_kernel) {
      // Calculate the average yaw and pitch values of a tile, from which
      // we calcuate the "distance" (in radians) between the point
      // (average yaw, average pitch) and the input point
      // (fixed_yaw, fixex_pitch).
      // Here, the origins of the yaw and pitch are at the center of the frame.
      // inputWidth corresponds to 2 * M_PI in radians for yaw, and inputHeight
      // corresponds to M_PI in radians for pitch. Yaw is changed from 0 to
      // PI when moving from center to left, and from 0 to -PI when moving from
      // center to right. Pitch is changed from 0 to 0.5 * PI when moving from
      // center to top, and from 0 to -0.5 * PI when moving from center to
      // bottom.
      float avgYaw = 2 * M_PI * (
        (i * segmentWidth +
         0.5 * min(segmentWidth, inputWidth - i * segmentWidth))
        - 0.5 * inputWidth) / inputWidth;
      float avgPitch = 0.5 * M_PI * (inputHeight - top - bottom) / inputHeight;

      float yaw = ctx_.fixed_yaw * M_PI / 180.0f;
      float pitch = ctx_.fixed_pitch * M_PI / 180.0f;
      float offset = std::abs(ctx_.fixed_cube_offcenter_z);

      // Check if yaw and pitch are both 0 and we should use reverse direction
      // of the offset instead.
      if (std::abs(yaw) < kEpsilon && std::abs(pitch) < kEpsilon && (
          std::abs(ctx_.fixed_cube_offcenter_x) > kEpsilon ||
          std::abs(ctx_.fixed_cube_offcenter_y) > kEpsilon ||
          ctx_.fixed_cube_offcenter_z > kEpsilon)) {  // check Z > 0 explicitly
        offset = sqrtf(
            ctx_.fixed_cube_offcenter_x * ctx_.fixed_cube_offcenter_x +
            ctx_.fixed_cube_offcenter_y * ctx_.fixed_cube_offcenter_y +
            ctx_.fixed_cube_offcenter_z * ctx_.fixed_cube_offcenter_z);
        yaw = atan2f(
            -ctx_.fixed_cube_offcenter_x / offset,
            -ctx_.fixed_cube_offcenter_z / offset);
        pitch = asinf(-ctx_.fixed_cube_offcenter_y / offset);
      }

      double dist = angularDistance(
        yaw,
        pitch,
        avgYaw,
        avgPitch);

      double effectiveRatio = getEffectiveRatio(dist, offset);
      double kernelScalingFactor =
        ctx_.kernel_adjust_factor * baseEffectiveRatio / effectiveRatio;
      Mat adjustedKernelX = calculateKernel(kernelScalingFactor * sigmaX);
      Mat adjustedKernelY = calculateKernel(kernelScalingFactor * sigmaY);
      filterKernelsX_[transformMatPlaneIndex].emplace_back(adjustedKernelX);
      filterKernelsY_[transformMatPlaneIndex].emplace_back(adjustedKernelY);
    } else {
      filterKernelsX_[transformMatPlaneIndex].emplace_back(kernelX);
      filterKernelsY_[transformMatPlaneIndex].emplace_back(kernelY);
    }
  }
}

// Split a frame plane into top half and bottom half, and calculate
// the filtering configs for them, respectively.
// "top" and "bottom" are the Y (vertical) coordinates of a segment.
// "angle" is the radians between the center of a segment (Y direction) and
// equator, where the equator has the coordinate of 0.5 * inputHeight in Y
// direction. (Since two segments, which are symmetric with respective to
// equator, have the same kernel, we only consider the absolute value of the
// "angle", which is a representative of the "distance" between the center and
// equator.) The "distance" is calculated by finding the difference between
// 0.5 * inputHeight and the average value of "top" and "bottom". This is
// because openCV's coordinate system is from 0 to inputHeight - 1 in a top-down
// manner. The "angle" is thereafter calculated by measuring the ratio between
// the above "distance" and inputHeight which is corresponding to M_PI in
// radians.
// If adjust_kernel is True, we further break the above segment into
// horizontal segments. Thus, a frame is finally divided into tiles. We
// calculate the distance (in radians) between the center of the tile and
// the input point (yaw, pitch), from which we adjust the kernel for that tile
// accordingly. Please see generateKernelAndFilteringConfig for more details.
void VideoFrameTransform::generateKernelsAndFilteringConfigs(
    int startTop,
    int startBottom,
    float sigmaY,
    const Mat& kernelY,
    int baseSegmentHeight,
    int inputWidth,
    int inputHeight,
    int transformMatPlaneIndex) {
  // Top half
  // "bottom" is used to calculate the position of a vertical segment
  for (int bottom = startBottom; bottom >= 0; bottom -= baseSegmentHeight) {
    int top = max(bottom - baseSegmentHeight + 1, 0);
    // Here, both "top" and "bottom" are small than 0.5 * inputHeight.
    // The "angle", or "distance" of the center, of the segment to equator is
    // equal to 0.5 * inputHeight - 0.5 * (top + bottom)
    float angle  = 0.5 * M_PI * (inputHeight - top - bottom) / inputHeight;
    generateKernelAndFilteringConfig(
      top,
      bottom,
      angle,
      sigmaY,
      kernelY,
      inputWidth,
      inputHeight,
      transformMatPlaneIndex);
  }

  // Bottom half
  // "top" is used to calculate the position of a vertical segment
  for (int top = startTop; top < inputHeight; top += baseSegmentHeight) {
    int bottom = min(top + baseSegmentHeight - 1, inputHeight - 1);
    // Here, both "top" and "bottom" are larger than or equal to
    // 0.5 * inputHeight. The "angle" of the segment is equal to
    // 0.5 * (top + bottom) - 0.5 * inputHeight
    float angle = 0.5 * M_PI * (top + bottom - inputHeight) / inputHeight;
    generateKernelAndFilteringConfig(
      top,
      bottom,
      angle,
      sigmaY,
      kernelY,
      inputWidth,
      inputHeight,
      transformMatPlaneIndex);
  }
}

// Calculate variable kernels for segments
void VideoFrameTransform::calcualteFilteringConfig(
    int inputWidth,
    int inputHeight,
    int outputWidth,
    int outputHeight,
    int transformMatPlaneIndex) {
  // For stereo videos, we calculate the filtering config
  // only for one single view because the configs are the same
  // for both views. During the final filtering process, we apply
  // the config on both views, respectively.
  switch (ctx_.input_stereo_format) {
    case STEREO_FORMAT_LR:
      inputWidth *= 0.5;
      break;
    case STEREO_FORMAT_TB:
      inputHeight *= 0.5;
      break;
    case STEREO_FORMAT_MONO:
    case STEREO_FORMAT_GUESS:
    case STEREO_FORMAT_N:
      break;
  }

  switch (ctx_.output_stereo_format) {
    case STEREO_FORMAT_LR:
      outputWidth *= 0.5;
      break;
    case STEREO_FORMAT_TB:
      outputHeight *= 0.5;
      break;
    case STEREO_FORMAT_MONO:
    case STEREO_FORMAT_GUESS:
    case STEREO_FORMAT_N:
      break;
  }

  // Calculate the size of the basic 1D filter along Y direction
  float hFov, vFov;
  switch (ctx_.output_layout) {
    case LAYOUT_CUBEMAP_32:
    case LAYOUT_TB_ONLY:
      {
        hFov = 270.0;
        vFov = 180.0;
        break;
      }
    case LAYOUT_CUBEMAP_23_OFFCENTER:
      {
        hFov = 180.0;
        vFov = 270.0;
        break;
      }
#ifdef FACEBOOK_LAYOUT
    case LAYOUT_FB:
      calculateFov(ctx_.fixed_hfov, ctx_.fixed_vfov, hFov, vFov);
      break;
#endif
    case LAYOUT_FLAT_FIXED:
      {
        hFov = ctx_.fixed_hfov;
        vFov = ctx_.fixed_vfov;
        break;
      }
    case LAYOUT_EQUIRECT:
      {
        hFov = 360.0;
        vFov = 180.0;
        break;
      }
    case LAYOUT_BARREL:
    case LAYOUT_BARREL_SPLIT:
    case LAYOUT_TB_BARREL_ONLY:
      {
        hFov = 450.0;
        vFov = 90.0;
        break;
      }
    case LAYOUT_EAC_32:
      {
        hFov = 270.0;
        vFov = 180.0;
        break;
      }
    case LAYOUT_N:
      {
        printf(
          "Invalid layout type for plane %d.\n", transformMatPlaneIndex);
        return;
      }
  }

  float sigmaY = 0.5f * std::min(
      ctx_.max_kernel_half_height,
      std::max(
        ctx_.min_kernel_half_height,
        ctx_.kernel_height_scale_factor *
          std::min(inputWidth / 360.0f, inputHeight / 180.0f) /
            std::max(outputWidth / hFov, outputHeight / vFov)));

  // Build a basic 1D filter along Y direction
  Mat kernelY = calculateKernel(sigmaY);

  // The maximum height of a segment
  int baseSegmentHeight = ceil(1.0 * inputHeight / ctx_.num_vertical_segments);

  if (ctx_.num_vertical_segments % 2 == 0) {
    // Both top and bottom halves of the frame plane are divided into
    // 0.5 * ctx_.num_vertical_segments segments
    generateKernelsAndFilteringConfigs(
      0.5 * inputHeight,
      0.5 * inputHeight - 1,
      sigmaY,
      kernelY,
      baseSegmentHeight,
      inputWidth,
      inputHeight,
      transformMatPlaneIndex);
  } else {
    // One segment is centered at equator and occupies
    // 0.5 * baseSegmentHeight in top and bottom halves, respectively.
    int top = 0.5 * (inputHeight - baseSegmentHeight);
    int bottom = top + baseSegmentHeight - 1;
    generateKernelAndFilteringConfig(
      top,
      bottom,
      0,
      sigmaY,
      kernelY,
      inputWidth,
      inputHeight,
      transformMatPlaneIndex);

    // Divid the rest of the frame plane into
    // ctx_.num_vertical_segments - 1 segments
    generateKernelsAndFilteringConfigs(
      bottom + 1,
      top - 1,
      sigmaY,
      kernelY,
      baseSegmentHeight,
      inputWidth,
      inputHeight,
      transformMatPlaneIndex);
  }
}

// Calculate the transform maps
bool VideoFrameTransform::generateMapForPlane(
  int inputWidth,
  int inputHeight,
  int outputWidth,
  int outputHeight,
  int transformMatPlaneIndex) {
  try {
    assert(
      inputWidth > 0 && inputHeight > 0 &&
      outputWidth > 0 && outputHeight > 0 &&
      ctx_.width_scale_factor > 0 &&
      ctx_.height_scale_factor > 0 &&
      ctx_.kernel_height_scale_factor > 0 &&
      ctx_.num_vertical_segments >= 2 &&
      ctx_.num_vertical_segments <= inputHeight &&
      ctx_.num_horizontal_segments >= 1 &&
      ctx_.num_horizontal_segments <= inputWidth &&
      ctx_.min_kernel_half_height >= 0.5 &&
      ctx_.max_kernel_half_height >= 0.5);

    // Both scaling and low pass filtering processes are for antialiasing
    // purpose
    int scaledOutputWidth =
      (int) (ctx_.width_scale_factor * outputWidth + 0.5);
    int scaledOutputHeight =
      (int) (ctx_.height_scale_factor * outputHeight + 0.5);

    float inputPixelWidth = 1.0f / inputWidth;
    if (ctx_.input_stereo_format == STEREO_FORMAT_LR) {
      inputPixelWidth *= 2;
    }

    Mat warpMat = Mat(scaledOutputHeight, scaledOutputWidth, CV_32FC2);
    for (int i = 0; i < scaledOutputHeight; ++i) {
      for (int j = 0; j < scaledOutputWidth; ++j) {
        float outX, outY;
        float y = (i + 0.5f) / scaledOutputHeight;
        float x = (j + 0.5f) / scaledOutputWidth;
        if (transformPos(
            x, y, &outX, &outY, transformMatPlaneIndex, inputPixelWidth)) {
          // OpenCV uses coordinates system. Where (0, 0) is the center of
          // top-left pixel instead of a corner. In this case we have to
          // shift output (x, y) by (-0.5f, -0.5f) to account for this.
          warpMat.at<Point2f>(i, j) =
            Point2f(outX * inputWidth - 0.5f, outY * inputHeight - 0.5f);
        } else {
          printf(
            "Failed to find the mapping coordinate for point (%d, %d)\n",
            i, j);
          return false;
        }
      }
    }

    warpMats_[transformMatPlaneIndex] = warpMat;

    if (ctx_.enable_low_pass_filter) {
      // Calculate variable kernels for segments of the frame plane
      calcualteFilteringConfig(
        inputWidth,
        inputHeight,
        scaledOutputWidth,
        scaledOutputHeight,
        transformMatPlaneIndex);
    }

    return true;
  } catch (const exception& ex) {
    printf("Could not generate map for plane %d. Error: %s\n",
      transformMatPlaneIndex,
      ex.what());
    return false;
  }
}

// Run filtering
void VideoFrameTransform::runFiltering(
    const Mat& inputMat,
    Mat& blurredPlane,
    int transformMatPlaneIndex,
    int imagePlaneIndex,
    int leftOffset,
    int topOffset,
    vector<thread>& segmentFilteringThreads) {
  for (int i = 0;
    i < segmentFilteringConfigs_[transformMatPlaneIndex].size(); ++i) {
    SegmentFilteringConfig config =
      segmentFilteringConfigs_[transformMatPlaneIndex][i];

    if (ctx_.enable_multi_threading) {
      segmentFilteringThreads.emplace_back(
        &VideoFrameTransform::filterSegment,
        this,
        cref(inputMat),
        ref(blurredPlane),
        cref(filterKernelsX_[transformMatPlaneIndex][i]),
        cref(filterKernelsY_[transformMatPlaneIndex][i]),
        config.left + leftOffset,
        config.top + topOffset,
        config.width,
        config.height,
        imagePlaneIndex);
    } else {
      filterSegment(
        inputMat,
        blurredPlane,
        filterKernelsX_[transformMatPlaneIndex][i],
        filterKernelsY_[transformMatPlaneIndex][i],
        config.left + leftOffset,
        config.top + topOffset,
        config.width,
        config.height,
        imagePlaneIndex);
    }
  }
}

// Filter the plane using the caculated kernels
Mat VideoFrameTransform::filterPlane(
    const Mat& inputMat,
    int transformMatPlaneIndex,
    int imagePlaneIndex) {
  Mat blurredPlane = Mat::zeros(inputMat.size(), inputMat.type());
  try {
    // Vector of threads to filter segments in parallel
    vector<thread> segmentFilteringThreads;

    switch (ctx_.input_stereo_format) {
      case STEREO_FORMAT_LR:
        {
          // Fiter left half plane
          runFiltering(
            inputMat,
            blurredPlane,
            transformMatPlaneIndex,
            imagePlaneIndex,
            0,
            0,
            segmentFilteringThreads);

          // Filter right half plane
          runFiltering(
            inputMat,
            blurredPlane,
            transformMatPlaneIndex,
            imagePlaneIndex,
            0.5 * inputMat.cols,
            0,
            segmentFilteringThreads);

          break;
        }
      case STEREO_FORMAT_TB:
        {
          // Fiter top half plane
          runFiltering(
            inputMat,
            blurredPlane,
            transformMatPlaneIndex,
            imagePlaneIndex,
            0,
            0,
            segmentFilteringThreads);

          // Filter bottom half plane
          runFiltering(
            inputMat,
            blurredPlane,
            transformMatPlaneIndex,
            imagePlaneIndex,
            0,
            0.5 * inputMat.rows,
            segmentFilteringThreads);

          break;
        }
      case STEREO_FORMAT_MONO:
      case STEREO_FORMAT_GUESS:
      case STEREO_FORMAT_N:
        // Fiter entire frame plane
        runFiltering(
          inputMat,
          blurredPlane,
          transformMatPlaneIndex,
          imagePlaneIndex,
          0,
          0,
          segmentFilteringThreads);

        break;
    }

    if (ctx_.enable_multi_threading) {
      for (int i = 0; i < segmentFilteringThreads.size(); ++i) {
        segmentFilteringThreads[i].join();
      }
    }
  } catch (const exception& ex) {
    printf(
      "Could not filter plane %d. Error: %s\n",
      imagePlaneIndex,
      ex.what());
  }

  return blurredPlane;
}

// Find mapping between the input and output pixel coordinates
bool VideoFrameTransform::transformPlane(
  const Mat& inputMat,
  Mat& outputMat,
  int outputWidth,
  int outputHeight,
  int transformMatPlaneIndex,
  int imagePlaneIndex) {
  // For Barrel layout we want to have some black spots on the video frame,
  // so we need to change border mode to transparent, to avoid overwriting them.
  int borderMode =
      (ctx_.output_layout == LAYOUT_BARREL ||
       ctx_.output_layout == LAYOUT_BARREL_SPLIT) ?
      BORDER_TRANSPARENT :
      BORDER_WRAP;
  try {
    switch (ctx_.interpolation_alg) {
      case NEAREST:
      case LINEAR:
      case CUBIC:
      case LANCZOS4:
        {
          Mat tempMat;
          if (ctx_.enable_low_pass_filter) {
            // Filter frame plane with filters
            tempMat = filterPlane(
              inputMat,
              transformMatPlaneIndex,
              imagePlaneIndex);
          } else {
            tempMat = inputMat;
          }

          bool needResize =
            (outputHeight != warpMats_[transformMatPlaneIndex].rows ||
            outputWidth != warpMats_[transformMatPlaneIndex].cols);

          if (!needResize) {
            // We want to set default YUV values to 0.
            // UV (plane index > 0) planes are scaled from [-1, 1],
            // so we set it to 128.
            if (transformMatPlaneIndex &&
              (ctx_.output_layout == LAYOUT_BARREL ||
              ctx_.output_layout == LAYOUT_BARREL_SPLIT)) {
              outputMat.setTo(Scalar(128));
            }
            remap(
              tempMat,
              outputMat,
              warpMats_[transformMatPlaneIndex],
              cv::Mat(),
              ctx_.interpolation_alg,
              borderMode);
          } else {
            // We want to set default YUV values to 0.
            // UV (plane index > 0) planes are scaled from [-1, 1],
            // so we set it to 128.
            Mat scaledWarpedImage(
              warpMats_[transformMatPlaneIndex].size(),
              tempMat.type(),
              Scalar(transformMatPlaneIndex ? 128 : 0));
            remap(
              tempMat,
              scaledWarpedImage,
              warpMats_[transformMatPlaneIndex],
              cv::Mat(),
              ctx_.interpolation_alg,
              borderMode);
            resize(
              scaledWarpedImage,
              outputMat,
              Size(outputWidth, outputHeight),
              0,
              0,
              INTER_AREA);
          }
          break;
        }
      default:
        printf(
          "Could not find interpolation algorithm for plane %d",
          imagePlaneIndex);
    }
  } catch (const exception& ex) {
    printf(
      "Could not transform the plane %d. Error: %s\n",
      imagePlaneIndex,
      ex.what());
      return false;
  }

  return true;
}

void VideoFrameTransform::transformCubeFacePos(
  float tx,
  float ty,
  float tz,
  float *outX,
  float *outY
) {
  float x, y;

  if (tz <= -kCubemapSideDistance) {
    x = tx / tz;
    y = ty / tz;
    if (x >= -1.0 && x <= 1.0 && y >= -1.0 && y <= 1.0) {
      *outX = (5.0f + x / ctx_.input_expand_coef) / 6.0f;
      *outY = (3.0f + y / ctx_.input_expand_coef) / 4.0f;
      return;
    }
  }
  if (tz >= kCubemapSideDistance) {
    x = tx / tz;
    y = ty / tz;
    if (x >= -1.0 && x <= 1.0 && y >= -1.0 && y <= 1.0) {
      *outX = (3.0f + x / ctx_.input_expand_coef) / 6.0f;
      *outY = (3.0f - y / ctx_.input_expand_coef) / 4.0f;
      return;
    }
  }
  if (tx <= -kCubemapSideDistance) {
    x = tz / tx;
    y = ty / tx;
    if (x >= -1.0 && x <= 1.0 && y >= -1.0 && y <= 1.0) {
      *outX = (3.0f - x / ctx_.input_expand_coef) / 6.0f;
      *outY = (1.0f + y / ctx_.input_expand_coef) / 4.0f;
      return;
    }
  }
  if (tx >= kCubemapSideDistance) {
    x = tz / tx;
    y = ty / tx;
    if (x >= -1.0 && x <= 1.0 && y >= -1.0 && y <= 1.0) {
      *outX = (1.0f - x / ctx_.input_expand_coef) / 6.0f;
      *outY = (1.0f - y / ctx_.input_expand_coef) / 4.0f;
      return;
    }
  }
  if (ty <= -kCubemapSideDistance) {
    x = tx / ty;
    y = tz / ty;
    if (x >= -1.0 && x <= 1.0 && y >= -1.0 && y <= 1.0) {
      *outX = (1.0f - x / ctx_.input_expand_coef) / 6.0f;
      *outY = (3.0f + y / ctx_.input_expand_coef) / 4.0f;
      return;
    }
  }
  if (ty >= kCubemapSideDistance) {
    x = tx / ty;
    y = tz / ty;
    if (x >= -1.0 && x <= 1.0 && y >= -1.0 && y <= 1.0) {
      *outX = (5.0f + x / ctx_.input_expand_coef) / 6.0f;
      *outY = (1.0f + y / ctx_.input_expand_coef) / 4.0f;
      return;
    }
  }
  // Return outside coordinates.
  *outX = -1.0f;
  *outY = 0.0f;
}

void VideoFrameTransform::transformInputPos(
    float tx,
    float ty,
    float tz,
    float inputPixelWidth,
    float* outX,
    float* outY) {
  switch (ctx_.input_layout) {
    case LAYOUT_CUBEMAP_32:
    {
      float d = sqrtf(tx * tx + ty * ty + tz * tz);
      transformCubeFacePos(tx / d, ty / d, tz / d, outX, outY);
      break;
    }
    default:
    {
      // Assumming equirect
      float d = sqrtf(tx * tx + ty * ty + tz * tz);

      *outX = -atan2f (-tx / d, tz / d) / (M_PI * 2.0f) + 0.5f;
      if (ctx_.output_layout == LAYOUT_BARREL ||
          ctx_.output_layout == LAYOUT_BARREL_SPLIT) {
        // Clamp pixels on the right, since we might have padding from ffmpeg.
        *outX = std::min(*outX, 1.0f - inputPixelWidth * 0.5f);
        *outX = std::max(*outX, inputPixelWidth * 0.5f);
      }
      *outY = asinf (-ty / d) / M_PI + 0.5f;
      break;
    }
  }
}

bool VideoFrameTransform::transformPos(
    float x,
    float y,
    float* outX,
    float* outY,
    int transformMatPlaneIndex,
    float inputPixelWidth) {
  try {
    int isRight = 0;

    if (ctx_.input_stereo_format != STEREO_FORMAT_MONO) {
      switch (ctx_.output_stereo_format) {
        case STEREO_FORMAT_LR:
          {
            if (x > kXHalf) {
              x = (x - kXHalf) / kXHalf;
              isRight = 1;
            } else {
              x = x / kXHalf;
            }
            break;
          }
        case STEREO_FORMAT_TB:
          {
            if (y > kYHalf) {
              y = (y - kYHalf) / kYHalf;
              if (ctx_.vflip) {
                y = 1.0f - y;
              }
              isRight = 1;
              } else {
                y = y / kYHalf;
              }
              break;
          }
        case STEREO_FORMAT_MONO:
        case STEREO_FORMAT_GUESS:
        case STEREO_FORMAT_N:
          break;
      }
    }

    float qx, qy, qz, tx, ty, tz, d;
    float yaw, pitch;
    bool hasMapping = true;
    if (ctx_.output_layout != LAYOUT_FLAT_FIXED) {
      y = 1.0f - y;
    }
    array<float, 3> vx, vy, p;
    int face = 0, vFace, hFace;

    switch (ctx_.output_layout) {
      case LAYOUT_CUBEMAP_32:
        {
          vFace = (int) (y * 2);
          hFace = (int) (x * 3);
          x = x * 3.0f - hFace;
          y = y * 2.0f - vFace;
          face = hFace + (1 - vFace) * 3;
          break;
        }
      case LAYOUT_CUBEMAP_23_OFFCENTER:
        {
          vFace = (int) (y * 3);
          hFace = (int) (x * 2);
          x = x * 2.0f - hFace;
          y = y * 3.0f - vFace;
          face = hFace + (2 - vFace) * 2;
          break;
        }
      case LAYOUT_TB_ONLY:
      case LAYOUT_TB_BARREL_ONLY:
        {
          vFace = (int) (y * 2);
          face = (vFace == 1) ? TOP : BOTTOM;
          y = y * 2.0f - vFace;
          break;
        }
#ifdef FACEBOOK_LAYOUT
      case LAYOUT_FB:
        break;
#endif
      case LAYOUT_FLAT_FIXED:
        break;
      case LAYOUT_EQUIRECT:
        {
          yaw = (2.0f * x - 1.0f) * M_PI;
          pitch = (y - 0.5f) * M_PI;
          break;
        }
      case LAYOUT_BARREL:
        {
          if (x <= 0.8f) {
            yaw = (2.5f * x - 1.0f) * ctx_.expand_coef * M_PI;
            pitch = (y * 0.5f  - 0.25f) * ctx_.expand_coef * M_PI;
            face = -1;
          } else {
            vFace = (int) (y * 2);
            face = (vFace == 1) ? TOP : BOTTOM;
            x = x * 5.0f - 4.0f;
            y = y * 2.0f - vFace;
          }
          break;
        }
      case LAYOUT_BARREL_SPLIT:
        //  For LAYOUT_BARREL_SPLIT, we separate the front view (-90~+90) and
        //  back view (-180~-90 & +90~+180) of the sphere into top & bottom rows
        //  of the projection. The top row holds the front half with its top and
        //  bottom part in the top row, and the back half with its top & bottom
        //  part in the bottom row.
        //
        //  Illustration of Equirectangular:
        //
        //   +---+---+---+---+---+---+---+---+
        //   |   1   |       3       |   1   |
        //   +---+---+---+---+---+---+---+---+
        //   |       |               |       |     Front half: 3, 4, 2
        //   +   5   +       4       +   5   +
        //   |       |               |       |     Back half: 1, 5, 0
        //   +---+---+---+---+---+---+---+---+
        //   |   0   |       2       |   0   |
        //   +---+---+---+---+---+---+---+---+
        //   <-Back-><---- Front ----><-Back->
        //
        //  Projection to LAYOUT_BARREL_SPLIT:
        //
        //   +---+---+---+---+---+---+
        //   |               |\  3  /|     3: Front Top
        //   +       4       +  - -  +
        //   |               |/  2  \|     2: Front Bottom
        //   +---+---+---+---+---+---+
        //   |               |\  1  /|     1: Back Top
        //   +       5       +  - -  +
        //   |               |/  0  \|     0: Back Bottom
        //   +---+---+---+---+---+---+
        //
        //  - Area 0, 1, 2, 3 are all half circles.
        //
        {
          if (3.0f * x <= 2.0f) {
            vFace = (int)(y * 2);
            yaw =
                ((3.0f / 2.0f * x - 0.5f) * ctx_.expand_coef - vFace + 1.0f) *
                M_PI;
            pitch = (y - 0.25f - 0.5f * vFace) * ctx_.expand_coef * M_PI;
            face = -1;
          } else {
            //  Index which half circle the area is rendering from
            //  - Area 0: Back Bottom.
            //    * face: BOTTOM
            //    * Needs to rotate 180 degrees to match shape.
            //    * Rendering Range: [0, 0.5] -> [0.5, 0]
            //  - Area 1: Back Top.
            //    * face: TOP
            //    * Needs to rotate 180 degrees to match shape.
            //    * Rendering Range: [0.5, 1] -> [1, 0.5]
            //  - Area 2: Front Bottom.
            //    * face: BOTTOM
            //    * Rendering Range: [0.5, 1]
            //  - Area 3: Front Top.
            //    * face: TOP
            //    * Rendering Range: [0, 0.5]
            //
            int halfVFace = (int)(y * 4);
            face = (halfVFace == 1 || halfVFace == 3) ? TOP : BOTTOM;
            x = x * 3.0f - 2.0f;
            switch (halfVFace) {
              case 0:
                y = y * 2.0f;
                // Rotate 180 degrees
                x = 1.0f - x;
                y = (0.5f - y) * ctx_.expand_coef;
                break;
              case 1:
                y = y * 2.0f;
                // Rotate 180 degrees
                x = 1.0f - x;
                y = 1.0f - ctx_.expand_coef * (y - 0.5f);
                break;
              case 2:
                y = y * 2.0f - 0.5f;
                y = 1.0f - ctx_.expand_coef * (1.0f - y);
                break;
              case 3:
                y = y * 2.0f - 1.5f;
                y = y * ctx_.expand_coef;
                break;
            }
          }
          break;
        }
      case LAYOUT_EAC_32:
        {
          vFace = (int) (y * 2);
          hFace = (int) (x * 3);
          x = x * 3.0f - hFace;
          y = y * 2.0f - vFace;
          x = tan((x - 0.5f) * M_PI * 0.5f) * 0.5f + 0.5f;
          y = tan((y - 0.5f) * M_PI * 0.5f) * 0.5f + 0.5f;
          face = hFace + (1 - vFace) * 3;
          break;
        }
      case LAYOUT_N:
        {
          printf(
            "Invalid layout type for plane %d.\n", transformMatPlaneIndex);
          return false;
        }
    }

    switch (ctx_.output_layout) {
      case LAYOUT_CUBEMAP_32:
      case LAYOUT_CUBEMAP_23_OFFCENTER:
      case LAYOUT_EQUIRECT:
      case LAYOUT_BARREL:
      case LAYOUT_BARREL_SPLIT:
      case LAYOUT_EAC_32:
      case LAYOUT_TB_ONLY:
      case LAYOUT_TB_BARREL_ONLY:
      {
        if (ctx_.output_layout == LAYOUT_EQUIRECT ||
          (ctx_.output_layout == LAYOUT_BARREL && face < 0) ||
          (ctx_.output_layout == LAYOUT_BARREL_SPLIT && face < 0)) {
          float sin_yaw = sin(yaw);
          float sin_pitch = sin(pitch);
          float cos_yaw = cos(yaw);
          float cos_pitch = cos(pitch);
          qx = sin_yaw * cos_pitch;
          qy = sin_pitch;
          qz = cos_yaw * cos_pitch;
        } else {
          assert(x >= 0 && x <= 1);
          assert(y >= 0 && y <= 1);
          assert(face >= 0 && face < 6);
          if (ctx_.output_layout == LAYOUT_BARREL ||
            ctx_.output_layout == LAYOUT_BARREL_SPLIT ||
            ctx_.output_layout == LAYOUT_TB_BARREL_ONLY) {
            float radius = (x - 0.5f) * (x - 0.5f) + (y - 0.5f) * (y - 0.5f);
            if (radius > 0.25f * ctx_.expand_coef * ctx_.expand_coef) {
              hasMapping = false;
              break;
            }
          }

          x = (x - 0.5f) * ctx_.expand_coef + 0.5f;
          y = (y - 0.5f) * ctx_.expand_coef + 0.5f;

          TransformFaceType enumFace = static_cast<TransformFaceType>(face);
          if (ctx_.output_layout == LAYOUT_CUBEMAP_23_OFFCENTER) {
            switch (enumFace) {
              case RIGHT:  p = P4; vx = PY; vy = NZ; break;
              case LEFT:   p = P3; vx = NX; vy = PZ; break;
              case TOP:    p = P5; vx = PY; vy = NX; break;
              case BOTTOM: p = P1; vx = NX; vy = PY; break;
              case FRONT:  p = P1; vx = PY; vy = PZ; break;
              case BACK:   p = P5; vx = NX; vy = NZ; break;
            }
          } else {
            switch (enumFace) {
              case RIGHT:   p = P5; vx = NZ; vy = PY; break;
              case LEFT:    p = P0; vx = PZ; vy = PY; break;
              case TOP:     p = P6; vx = PX; vy = NZ; break;
              case BOTTOM:  p = P0; vx = PX; vy = PZ; break;
              case FRONT:   p = P4; vx = PX; vy = PY; break;
              case BACK:    p = P1; vx = NX; vy = PY; break;
            }
          }

          qx = p [0] + vx [0] * x + vy [0] * y;
          qy = p [1] + vx [1] * x + vy [1] * y;
          qz = p [2] + vx [2] * x + vy [2] * y;
        }

        if (std::abs(ctx_.fixed_cube_offcenter_x) > kEpsilon ||
          std::abs(ctx_.fixed_cube_offcenter_y) > kEpsilon ||
          std::abs(ctx_.fixed_cube_offcenter_z) > kEpsilon) {
          float dist;
          d = sqrtf(qx * qx + qy * qy + qz * qz);
          qx = qx / d;
          qy = qy / d;
          qz = qz / d;
          if (ctx_.is_horizontal_offset) {
            d = sqrtf(qx * qx + qz * qz);
            qx = qx / d;
            qy = qy / d;
            qz = qz / d;
            dist = intersectSphereOffset(
              qx, 0, qz,
              ctx_.fixed_cube_offcenter_x, 0, ctx_.fixed_cube_offcenter_z);
            if (dist > 0.0f) {
              qx = qx * dist - ctx_.fixed_cube_offcenter_x;
              qz = qz * dist - ctx_.fixed_cube_offcenter_z;
            }
          } else {
            dist = intersectSphereOffset(
              qx, qy, qz, ctx_.fixed_cube_offcenter_x,
              ctx_.fixed_cube_offcenter_y, ctx_.fixed_cube_offcenter_z);
            if (dist > 0.0f) {
              qx = qx * dist - ctx_.fixed_cube_offcenter_x;
              qy = qy * dist - ctx_.fixed_cube_offcenter_y;
              qz = qz * dist - ctx_.fixed_cube_offcenter_z;
            }
          }
        }

        // rotation
        float s1 = sin(ctx_.fixed_yaw * M_PI / 180.0f);
        float s2 = sin(ctx_.fixed_pitch * M_PI / 180.0f);
        float s3 = sin(ctx_.fixed_roll * M_PI / 180.0f);
        float c1 = cos(ctx_.fixed_yaw * M_PI / 180.0f);
        float c2 = cos(ctx_.fixed_pitch * M_PI / 180.0f);
        float c3 = cos(ctx_.fixed_roll * M_PI / 180.0f);

        tx = qx * (c1 * c3 + s1 * s2 * s3)
          - qy * (c3 * s1 * s2 - c1 * s3)
          + qz * (c2 * s1);
        ty = qx * (c2 * s3) - qy * (c2 * c3)
          + qz * (-s2);
        tz = qx * (c1 * s2 * s3 - c3 * s1)
          - qy * (c1 * c3 * s2 + s1 * s3)
          + qz * (c1 * c2);

        ty = -ty;

        transformInputPos(tx, ty, tz, inputPixelWidth, outX, outY);
        break;
      }
 #ifdef FACEBOOK_LAYOUT
      case LAYOUT_FB:
      {
        calculateTranspos(
          ctx_.fixed_yaw,
          ctx_.fixed_pitch,
          ctx_.fixed_hfov,
          ctx_.fixed_vfov,
          x,
          y,
          outX,
          outY);
        break;
      }
#endif
      case LAYOUT_FLAT_FIXED:
      {
        *outX = ((x - 0.5f) * ctx_.fixed_hfov + ctx_.fixed_yaw) / 360.0f + 0.5f;
        *outY = ((y - 0.5f) * ctx_.fixed_vfov - ctx_.fixed_pitch) / 180.0f + 0.5f;
        normalize_equirectangular(*outX, *outY, outX, outY);
        break;
      }
      case LAYOUT_N:
        {
          printf(
            "Invalid layout type for plane %d.\n", transformMatPlaneIndex);
          return false;
        }
    }

    if (hasMapping) {
      switch (ctx_.input_stereo_format) {
        case STEREO_FORMAT_TB:
          {
            if (isRight) {
              *outY = *outY * kYHalf + kYHalf;
            } else {
              *outY = *outY * kYHalf;
          }
          break;
        }
        case STEREO_FORMAT_LR:
          {
            if (isRight) {
              *outX = *outX * kXHalf + kXHalf;
            } else {
              *outX = *outX * kXHalf;
            }
            break;
          }
        case STEREO_FORMAT_MONO:
        case STEREO_FORMAT_GUESS:
        case STEREO_FORMAT_N:
          break;
      }

      assert(*outX >= 0 && *outX <= 1);
      assert(*outY >= 0 && *outY <= 1);
    } else {
      *outX = -1;
      *outY = 0;
    }
    return true;
  } catch (const exception& ex) {
    printf(
      "Could not transform the plane %d. Error: %s\n",
       transformMatPlaneIndex,
       ex.what());
    return false;
  }
}

// Transform each frame plane
bool VideoFrameTransform::transformFramePlane(
  uint8_t* inputArray,
  uint8_t* outputArray,
  int inputWidth,
  int inputHeight,
  int inputWidthWithPadding,
  int outputWidth,
  int outputHeight,
  int outputWidthWithPadding,
  int transformMatPlaneIndex,
  int imagePlaneIndex) {
  try {
    Mat inputMat(
      inputHeight,
      inputWidth,
      CV_8U,
      inputArray,
      inputWidthWithPadding);

    Mat outputMat(
      outputHeight,
      outputWidth,
      CV_8U,
      outputArray,
      outputWidthWithPadding);

    return transformPlane(
      inputMat,
      outputMat,
      outputWidth,
      outputHeight,
      transformMatPlaneIndex,
      imagePlaneIndex);
  } catch (const exception& ex) {
    printf(
      "Could not transform the plane %d. Error: %s\n",
      imagePlaneIndex,
      ex.what());
    return false;
  }
}
