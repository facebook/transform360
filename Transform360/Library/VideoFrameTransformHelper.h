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

#pragma once

#include <stdint.h>

typedef enum TransformFaceType {
  RIGHT = 0,
  LEFT,
  TOP,
  BOTTOM,
  FRONT,
  BACK
} TransformFaceType;

typedef enum Layout {
  LAYOUT_CUBEMAP_32 = 0,
  LAYOUT_CUBEMAP_23_OFFCENTER,
#ifdef FACEBOOK_LAYOUT
  LAYOUT_FB,
#endif
  LAYOUT_FLAT_FIXED,
  LAYOUT_TB_ONLY,
  LAYOUT_TB_BARREL_ONLY,
  LAYOUT_EQUIRECT,
  LAYOUT_BARREL,
  LAYOUT_BARREL_SPLIT,
  LAYOUT_EAC_32,
  LAYOUT_N
} Layout;

typedef enum StereoFormat {
  STEREO_FORMAT_TB = 0,
  STEREO_FORMAT_LR,
  STEREO_FORMAT_MONO,
  STEREO_FORMAT_GUESS,
  STEREO_FORMAT_N
} StereoFormat;

typedef enum InterpolationAlg {
  NEAREST = 0,
  LINEAR = 1,
  CUBIC = 2,
  LANCZOS4 = 4
} InterpolationAlg;

typedef struct FrameTransformContext {
  Layout input_layout;
  Layout output_layout;
  StereoFormat input_stereo_format;
  StereoFormat output_stereo_format;
  int vflip;
  float input_expand_coef;  /// Expansion coefficient for the input
  float expand_coef;  /// Expansion coefficient for the output
  InterpolationAlg interpolation_alg;
  float width_scale_factor; /// Width scale factor for antialiasing purpose
  float height_scale_factor; /// Height scale factor for antialiasing purpose
  float fixed_yaw;    ///< Yaw (asimuth) angle, degrees
  float fixed_pitch;  ///< Pitch (elevation) angle, degrees
  float fixed_roll;   ///< Roll (tilt) angle, degrees
  float fixed_hfov;   ///< Horizontal field of view, degrees
  float fixed_vfov;   ///< Vertical field of view, degrees
  float fixed_cube_offcenter_x; /// offcenter projection x
  float fixed_cube_offcenter_y; /// offcenter projection y
  float fixed_cube_offcenter_z; /// offcenter projection z
  int is_horizontal_offset;  /// Whether horizontal plane offset is enabled
  int enable_low_pass_filter;
  float kernel_height_scale_factor; /// Factor to scale the calculated kernel
                                    /// height for low pass filtering
  float min_kernel_half_height; /// Half of the mininum kernel height which is
                              /// usually applied to areas with small pitch
                              /// values
  float max_kernel_half_height; /// Maximum value of the kernel height
  int enable_multi_threading; /// Use multi-threading to filter segments
                              /// in parallel
  int num_vertical_segments; /// Number of vertical segments in a plane
  int num_horizontal_segments; /// Number of horizontal segments in a plane
  int adjust_kernel; /// Adjust kernels bsed on the "distance" (in radians)
                     /// from the input point (yaw, pitch)
  float kernel_adjust_factor; /// Factor to further adjust the kernel size
} FrameTransformContext;
