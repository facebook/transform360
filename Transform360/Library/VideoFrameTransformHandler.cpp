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

#include "VideoFrameTransformHandler.h"

#include "VideoFrameTransform.h"

VideoFrameTransform* VideoFrameTransform_new(
    FrameTransformContext* ctx) {
  return new VideoFrameTransform(ctx);
}

void VideoFrameTransform_delete(VideoFrameTransform* transform) {
  delete transform;
}

int VideoFrameTransform_generateMapForPlane(
  VideoFrameTransform* transform,
  int inputWidth,
  int inputHeight,
  int outputWidth,
  int outputHeight,
  int transformMatPlaneIndex) {
  return transform->generateMapForPlane(
    inputWidth,
    inputHeight,
    outputWidth,
    outputHeight,
    transformMatPlaneIndex);
}

int VideoFrameTransform_transformFramePlane(
  VideoFrameTransform* transform,
  uint8_t* inputData,
  uint8_t* outputData,
  int inputWidth,
  int inputHeight,
  int inputWidthWithPadding,
  int outputWidth,
  int outputHeight,
  int outputWidthWithPadding,
  int transformMatPlaneIndex,
  int imagePlaneIndex) {
  return transform->transformFramePlane(
    inputData,
    outputData,
    inputWidth,
    inputHeight,
    inputWidthWithPadding,
    outputWidth,
    outputHeight,
    outputWidthWithPadding,
    transformMatPlaneIndex,
    imagePlaneIndex);
}
