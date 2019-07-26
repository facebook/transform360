/**
 * Copyright (c) 2015-present, Facebook, Inc.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

/**
 * @file
 * transform360 video filter
 */

#include "libavutil/avassert.h"
#include "libavutil/avstring.h"
#include "libavutil/eval.h"
#include "libavutil/imgutils.h"
#include "libavutil/internal.h"
#include "libavutil/opt.h"
#include "libavutil/mem.h"
#include "libavutil/parseutils.h"
#include "avfilter.h"
#include "internal.h"
#include "video.h"
#include <stdio.h>

#include "transform360/VideoFrameTransformHandler.h"
#include "transform360/VideoFrameTransformHelper.h"

static const char *const var_names[] = {
    "out_w",  "ow",
    "out_h",  "oh",
    NULL
};

enum var_name {
    VAR_OUT_W, VAR_OW,
    VAR_OUT_H, VAR_OH,
    VARS_NB
};

/*
   MMMMT
   MMMMB
   */

typedef struct TransformContext {
    const AVClass *class;
    int w, h;
    int out_map_planes;

    AVDictionary *opts;
    char *w_expr;               ///< width  expression string
    char *h_expr;               ///< height expression string
    char *size_str;
    int cube_edge_length;
    int max_cube_edge_length;
    int max_output_h;
    int max_output_w;
    int input_layout;
    int output_layout;
    int input_stereo_format;
    int output_stereo_format;
    int vflip;
    int planes;
    float input_expand_coef;
    float expand_coef;
    int is_horizontal_offset;
    float fixed_yaw;    ///< Yaw (asimuth) angle, degrees
    float fixed_pitch;  ///< Pitch (elevation) angle, degrees
    float fixed_roll;   ///< Roll (tilt) angle, degrees
    float fixed_hfov;   ///< Horizontal field of view, degrees
    float fixed_vfov;   ///< Vertical field of view, degrees
    float fixed_cube_offcenter_x; // offcenter projection x
    float fixed_cube_offcenter_y; // offcenter projection y
    float fixed_cube_offcenter_z; // offcenter projection z

    // openCV-based transform parameters
    VideoFrameTransform* transform;
    int interpolation_alg;
    float width_scale_factor;
    float height_scale_factor;
    int enable_low_pass_filter;
    float kernel_height_scale_factor;
    float min_kernel_half_height;
    float max_kernel_half_height;
    int enable_multi_threading;
    int num_vertical_segments;
    int num_horizontal_segments;
    int adjust_kernel;
    float kernel_adjust_factor;

} TransformContext;

static inline void update_plane_sizes(
    AVPixFmtDescriptor* desc,
    int* in_w, int* in_h, int* out_w, int* out_h) {
    *in_w = FF_CEIL_RSHIFT(*in_w, desc->log2_chroma_w);
    *in_h = FF_CEIL_RSHIFT(*in_h, desc->log2_chroma_h);
    *out_w = FF_CEIL_RSHIFT(*out_w, desc->log2_chroma_w);
    *out_h = FF_CEIL_RSHIFT(*out_h, desc->log2_chroma_h);
}

static inline int generate_map(
    TransformContext *s, AVFilterLink *inlink,
    AVFilterLink *outlink, AVFrame *in) {
    AVFilterContext *ctx = outlink->src;
    int ret = 0;

    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(outlink->format);
    s->planes = av_pix_fmt_count_planes(outlink->format);
    s->out_map_planes = 2;

    FrameTransformContext frame_transform_ctx = (FrameTransformContext) {
      .input_layout = s->input_layout,
      .output_layout = s->output_layout,
      .input_stereo_format = s->input_stereo_format,
      .output_stereo_format = s->output_stereo_format,
      .vflip = s->vflip,
      .input_expand_coef = s->input_expand_coef,
      .expand_coef = s->expand_coef,
      .interpolation_alg = s->interpolation_alg,
      .width_scale_factor = s->width_scale_factor,
      .height_scale_factor = s->height_scale_factor,
      .fixed_yaw = s->fixed_yaw,
      .fixed_pitch = s->fixed_pitch,
      .fixed_roll = s->fixed_roll,
      .fixed_hfov = s->fixed_hfov,
      .fixed_vfov = s->fixed_vfov,
      .fixed_cube_offcenter_x = s->fixed_cube_offcenter_x,
      .fixed_cube_offcenter_y = s->fixed_cube_offcenter_y,
      .fixed_cube_offcenter_z = s->fixed_cube_offcenter_z,
      .is_horizontal_offset = s->is_horizontal_offset,
      .enable_low_pass_filter = s->enable_low_pass_filter,
      .kernel_height_scale_factor = s->kernel_height_scale_factor,
      .min_kernel_half_height = s->min_kernel_half_height,
      .max_kernel_half_height = s->max_kernel_half_height,
      .enable_multi_threading = s->enable_multi_threading,
      .num_vertical_segments = s->num_vertical_segments,
      .num_horizontal_segments = s->num_horizontal_segments,
      .adjust_kernel = s->adjust_kernel,
      .kernel_adjust_factor = s->kernel_adjust_factor};

    s->transform = VideoFrameTransform_new(&frame_transform_ctx);
    if (!s->transform) {
      return AVERROR(ENOMEM);
    }

    int in_w, in_h, out_w, out_h;
    for (int plane = 0; plane < s->out_map_planes; ++plane) {
      out_w = outlink->w;
      out_h = outlink->h;
      in_w = inlink->w;
      in_h = inlink->h;

      if (plane == 1) {
        update_plane_sizes(desc, &in_w, &in_h, &out_w, &out_h);
      }

      if (!VideoFrameTransform_generateMapForPlane(
            s->transform, in_w, in_h, out_w, out_h, plane)) {
        av_log(ctx, AV_LOG_INFO, "Failed to generate map for plane %d\n", plane);
        return AVERROR(EINVAL);
      }
    }

    return 0;
}

static int config_output(AVFilterLink *outlink) {
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = outlink->src->inputs[0];
    TransformContext *s = ctx->priv;
    double var_values[VARS_NB], res;
    char *expr;
    int ret;

    var_values[VAR_OUT_W] = var_values[VAR_OW] = NAN;
    var_values[VAR_OUT_H] = var_values[VAR_OH] = NAN;

    if (s->input_stereo_format == STEREO_FORMAT_GUESS) {
        int aspect_ratio = inlink->w / inlink->h;
        if (aspect_ratio == 1)
            s->input_stereo_format = STEREO_FORMAT_TB;
        else if (aspect_ratio == 4)
            s->input_stereo_format = STEREO_FORMAT_LR;
        else
            s->input_stereo_format = STEREO_FORMAT_MONO;
    }

    if (s->output_stereo_format == STEREO_FORMAT_GUESS) {
        if (s->input_stereo_format == STEREO_FORMAT_MONO) {
            s->output_stereo_format = STEREO_FORMAT_MONO;
        } else {
            s->output_stereo_format =
                (s->output_layout == LAYOUT_CUBEMAP_23_OFFCENTER)
                    ? STEREO_FORMAT_LR
                    : STEREO_FORMAT_TB;
        }
    }

    if (s->max_cube_edge_length > 0) {
        if (s->input_stereo_format == STEREO_FORMAT_LR) {
            s->cube_edge_length = inlink->w / 8;
        } else {
            s->cube_edge_length = inlink->w / 4;
        }

        // do not exceed the max length supplied
        if (s->cube_edge_length > s->max_cube_edge_length) {
            s->cube_edge_length = s->max_cube_edge_length;
        }
    }

    // ensure cube edge length is a multiple of 16 by rounding down
    // so that macroblocks do not cross cube edge boundaries
    s->cube_edge_length = s->cube_edge_length - (s->cube_edge_length % 16);

    if (s->cube_edge_length > 0) {
         if (s->output_layout == LAYOUT_CUBEMAP_32) {
            outlink->w = s->cube_edge_length * 3;
            outlink->h = s->cube_edge_length * 2;

        } else if (s->output_layout == LAYOUT_CUBEMAP_23_OFFCENTER) {
            outlink->w = s->cube_edge_length * 2;
            outlink->h = s->cube_edge_length * 3;
        }
    } else {
        var_values[VAR_OUT_W] = var_values[VAR_OW] = NAN;
        var_values[VAR_OUT_H] = var_values[VAR_OH] = NAN;

        av_expr_parse_and_eval(&res, (expr = s->w_expr),
                var_names, var_values,
                NULL, NULL, NULL, NULL, NULL, 0, ctx);
        s->w = var_values[VAR_OUT_W] = var_values[VAR_OW] = res;
        if ((ret = av_expr_parse_and_eval(&res, (expr = s->h_expr),
                        var_names, var_values,
                        NULL, NULL, NULL, NULL, NULL, 0, ctx)) < 0) {
            av_log(NULL, AV_LOG_ERROR,
                    "Error when evaluating the expression '%s'.\n"
                    "Maybe the expression for out_w:'%s' or for out_h:'%s' is self-referencing.\n",
                    expr, s->w_expr, s->h_expr);
            return ret;
        }
        s->h = var_values[VAR_OUT_H] = var_values[VAR_OH] = res;
        /* evaluate again the width, as it may depend on the output height */
        if ((ret = av_expr_parse_and_eval(&res, (expr = s->w_expr),
                        var_names, var_values,
                        NULL, NULL, NULL, NULL, NULL, 0, ctx)) < 0) {
            av_log(NULL, AV_LOG_ERROR,
                    "Error when evaluating the expression '%s'.\n"
                    "Maybe the expression for out_w:'%s' or for out_h:'%s' is self-referencing.\n",
                    expr, s->w_expr, s->h_expr);
            return ret;
        }
        s->w = res;

        outlink->w = s->w;
        outlink->h = s->h;
    }

    if (s->output_stereo_format == STEREO_FORMAT_TB) {
        outlink->h *= 2;
        s->h *= 2;
    } else if (s->output_stereo_format == STEREO_FORMAT_LR) {
        outlink->w *= 2;
        s->w *= 2;
    }

    av_log(ctx, AV_LOG_VERBOSE, "out_w:%d out_h:%d\n",
            outlink->w, outlink->h);

    return 0;
}

static av_cold int init_dict(AVFilterContext *ctx, AVDictionary **opts) {
    TransformContext *s = ctx->priv;

    if (s->size_str && (s->w_expr || s->h_expr)) {
        av_log(ctx, AV_LOG_ERROR,
                "Size and width/height expressions cannot be set at the same time.\n");
        return AVERROR(EINVAL);
    }

    if (s->w_expr && !s->h_expr)
        FFSWAP(char *, s->w_expr, s->size_str);

    av_log(ctx, AV_LOG_VERBOSE, "w:%s h:%s\n",
            s->w_expr, s->h_expr);

    s->opts = *opts;
    *opts = NULL;

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx) {
    TransformContext *s = ctx->priv;

    av_dict_free(&s->opts);
    s->opts = NULL;

    VideoFrameTransform_delete(s->transform);
    s->transform = NULL;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in) {
    AVFilterContext *ctx = inlink->dst;
    TransformContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *out;
    av_log(ctx, AV_LOG_VERBOSE, "Frame\n");

    // map not yet set
    if (s->out_map_planes != 2) {
      int result = generate_map(s, inlink, outlink, in);
      if (result != 0) {
          av_frame_free(&in);
          return result;
      }
    }

    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    av_log(ctx, AV_LOG_VERBOSE, "Got Frame %dx%d\n", outlink->w, outlink->h);

    if (!out) {
      av_frame_free(&in);
      return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);
    av_log(ctx, AV_LOG_VERBOSE, "Copied props \n");

    uint8_t *in_data, *out_data;
    int out_map_plane;
    int in_w, in_h, out_w, out_h;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(outlink->format);
    for (int plane = 0; plane < s->planes; ++plane) {
      in_data = in->data[plane];
      av_assert1(in_data);
      out_data = out->data[plane];
      out_map_plane = (plane == 1 || plane == 2) ? 1 : 0;

      out_w = outlink->w;
      out_h = outlink->h;
      in_w = inlink->w;
      in_h = inlink->h;

      if (plane >= 1) {
        update_plane_sizes(desc, &in_w, &in_h, &out_w, &out_h);
      }

      if (!VideoFrameTransform_transformFramePlane(
        s->transform,
        in_data,
        out_data,
        in_w,
        in_h,
        in->linesize[plane],
        out_w,
        out_h,
        out->linesize[plane],
        out_map_plane,
        plane)) {
        return AVERROR(EINVAL);
      }
    }

    av_frame_free(&in);
    av_log(ctx, AV_LOG_VERBOSE, "Done freeing in \n");
    return ff_filter_frame(outlink, out);
}

#define OFFSET(x) offsetof(TransformContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption transform360_options[] = {
    { "w",             "Output video width",          OFFSET(w_expr),    AV_OPT_TYPE_STRING,        .flags = FLAGS },
    { "width",         "Output video width",          OFFSET(w_expr),    AV_OPT_TYPE_STRING,        .flags = FLAGS },
    { "h",             "Output video height",         OFFSET(h_expr),    AV_OPT_TYPE_STRING,        .flags = FLAGS },
    { "height",        "Output video height",         OFFSET(h_expr),    AV_OPT_TYPE_STRING,        .flags = FLAGS },
    { "size",          "set video size",              OFFSET(size_str), AV_OPT_TYPE_STRING, {.str = NULL}, 0, FLAGS },
    { "s",             "set video size",              OFFSET(size_str), AV_OPT_TYPE_STRING, {.str = NULL}, 0, FLAGS },
    { "is_horizontal_offset", "Whether to use offset on the horizontal plane only. It only affects yaw.", OFFSET(is_horizontal_offset), AV_OPT_TYPE_BOOL, {.i64 =   0},     0, 1,  .flags = FLAGS },
    { "cube_edge_length", "Length of a cube edge (for cubic transform, overrides w and h, default 0 for off)",         OFFSET(cube_edge_length),    AV_OPT_TYPE_INT,  {.i64 = 0}, 0, 16384,  .flags = FLAGS },
    { "max_cube_edge_length", "Max length of a cube edge (for cubic transform, overrides w, h, and cube_edge_length, default 0 for off)",   OFFSET(max_cube_edge_length),    AV_OPT_TYPE_INT,  {.i64 = 0}, 0, 16384,  .flags = FLAGS },
    { "max_output_h", "Max height of the output video (for pyramid/cone transform, overrides pyramid_height, default 0 for off)",         OFFSET(max_output_h),    AV_OPT_TYPE_INT,  {.i64 = 0}, 0, 16384,  .flags = FLAGS },
    { "max_output_w", "Max width of the output video (for pyramid/cone transform, overrides pyramid_height, default 0 for off)",         OFFSET(max_output_w),    AV_OPT_TYPE_INT,  {.i64 = 0}, 0, 16384,  .flags = FLAGS },
    { "input_stereo_format", "Input video stereo format",         OFFSET(input_stereo_format),    AV_OPT_TYPE_INT,  {.i64 = STEREO_FORMAT_GUESS }, 0, STEREO_FORMAT_N - 1,  .flags = FLAGS, "stereo_format" },
    { "output_stereo_format", "Output video stereo format",         OFFSET(output_stereo_format),    AV_OPT_TYPE_INT,  {.i64 = STEREO_FORMAT_GUESS }, 0, STEREO_FORMAT_N - 1,  .flags = FLAGS, "stereo_format" },
    { "TB",      NULL, 0, AV_OPT_TYPE_CONST, {.i64 = STEREO_FORMAT_TB },      0, 0, FLAGS, "stereo_format" },
    { "LR",      NULL, 0, AV_OPT_TYPE_CONST, {.i64 = STEREO_FORMAT_LR },      0, 0, FLAGS, "stereo_format" },
    { "MONO",    NULL, 0, AV_OPT_TYPE_CONST, {.i64 = STEREO_FORMAT_MONO },    0, 0, FLAGS, "stereo_format" },
    { "GUESS",   NULL, 0, AV_OPT_TYPE_CONST, {.i64 = STEREO_FORMAT_GUESS },   0, 0, FLAGS, "stereo_format" },
    { "tb",      NULL, 0, AV_OPT_TYPE_CONST, {.i64 = STEREO_FORMAT_TB },      0, 0, FLAGS, "stereo_format" },
    { "lr",      NULL, 0, AV_OPT_TYPE_CONST, {.i64 = STEREO_FORMAT_LR },      0, 0, FLAGS, "stereo_format" },
    { "mono",    NULL, 0, AV_OPT_TYPE_CONST, {.i64 = STEREO_FORMAT_MONO },    0, 0, FLAGS, "stereo_format" },
    { "guess",   NULL, 0, AV_OPT_TYPE_CONST, {.i64 = STEREO_FORMAT_GUESS },   0, 0, FLAGS, "stereo_format" },
    { "input_layout", "Input video layout format",          OFFSET(input_layout),     AV_OPT_TYPE_INT,  {.i64 = LAYOUT_EQUIRECT },   0, LAYOUT_N - 1,  .flags = FLAGS, "layout" },
    { "output_layout", "Output video layout format",         OFFSET(output_layout),    AV_OPT_TYPE_INT,  {.i64 = LAYOUT_CUBEMAP_32 }, 0, LAYOUT_N - 1,  .flags = FLAGS, "layout" },
    { "CUBEMAP_32",          NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_CUBEMAP_32 },          0, 0, FLAGS, "layout" },
    { "CUBEMAP_23_OFFCENTER",NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_CUBEMAP_23_OFFCENTER },0, 0, FLAGS, "layout" },
    { "EQUIRECT",NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_EQUIRECT },0, 0, FLAGS, "layout" },
    { "BARREL",  NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_BARREL },  0, 0, FLAGS, "layout" },
    { "BARREL_SPLIT",  NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_BARREL_SPLIT },  0, 0, FLAGS, "layout" },
    { "EAC_32",  NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_EAC_32 },  0, 0, FLAGS, "layout" },
    { "FLAT_FIXED",          NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_FLAT_FIXED },          0, 0, FLAGS, "layout" },
    { "TB_ONLY",             NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_TB_ONLY },             0, 0, FLAGS, "layout" },
    { "TB_BARREL_ONLY",      NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_TB_BARREL_ONLY },      0, 0, FLAGS, "layout" },
    { "cubemap_32",          NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_CUBEMAP_32 },          0, 0, FLAGS, "layout" },
    { "cubemap_23_offcenter",NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_CUBEMAP_23_OFFCENTER },0, 0, FLAGS, "layout" },
    { "equirect",NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_EQUIRECT },0, 0, FLAGS, "layout" },
    { "flat_fixed",          NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_FLAT_FIXED },          0, 0, FLAGS, "layout" },
    { "tb_only",             NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_TB_ONLY },             0, 0, FLAGS, "layout" },
    { "tb_barrel_only",      NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_TB_BARREL_ONLY },      0, 0, FLAGS, "layout" },
    { "barrel",  NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_BARREL },  0, 0, FLAGS, "layout" },
    { "barrel_split",  NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_BARREL_SPLIT },  0, 0, FLAGS, "layout" },
    { "eac_32",  NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_EAC_32 },  0, 0, FLAGS, "layout" },
    { "vflip", "Output video 2nd eye vertical flip (true, false)",         OFFSET(vflip),    AV_OPT_TYPE_INT, {.i64 = 0 }, 0, 1,     .flags = FLAGS, "vflip" },
    { "false",  NULL, 0, AV_OPT_TYPE_CONST, {.i64 = 0 }, 0, 0, FLAGS, "vflip" },
    { "true",   NULL, 0, AV_OPT_TYPE_CONST, {.i64 = 1 }, 0, 0, FLAGS, "vflip" },
    { "input_expand_coef", "Expansion coeffiecient of the input",         OFFSET(input_expand_coef),    AV_OPT_TYPE_FLOAT,  {.dbl=1.01f}, 0, 10,  .flags = FLAGS },
    { "expand_coef", "Expansion coeffiecient for each face in cubemap (default 1.01)",         OFFSET(expand_coef),    AV_OPT_TYPE_FLOAT,  {.dbl=1.01f}, 0, 10,  .flags = FLAGS },
    { "yaw", "View orientation for flat_fixed projection, degrees",   OFFSET(fixed_yaw),          AV_OPT_TYPE_FLOAT,   {.dbl =   0.0}, -360, 360,  .flags = FLAGS },
    { "pitch", "View orientation for flat_fixed projection, degrees", OFFSET(fixed_pitch),        AV_OPT_TYPE_FLOAT,   {.dbl =   0.0}, -180, 180,  .flags = FLAGS },
    { "roll", "View orientation for flat_fixed projection, degrees",  OFFSET(fixed_roll),         AV_OPT_TYPE_FLOAT,   {.dbl =   0.0}, -180, 180,  .flags = FLAGS },
    { "hfov", "Horizontal field of view for flat_fixed projection, degrees (default 120)",  OFFSET(fixed_hfov), AV_OPT_TYPE_FLOAT,   {.dbl = 120.0}, -360, 360,  .flags = FLAGS },
    { "vfov", "Vertical field of view for flat_fixed projection, degrees (default 110)",     OFFSET(fixed_vfov), AV_OPT_TYPE_FLOAT,   {.dbl = 110.0}, -180, 180,  .flags = FLAGS },
    { "cube_offcenter_x", "Offcenter cube displacement x",   OFFSET(fixed_cube_offcenter_x),          AV_OPT_TYPE_FLOAT,   {.dbl =   0.0}, -1, 1,  .flags = FLAGS },
    { "cube_offcenter_y", "Offcenter cube displacement y",   OFFSET(fixed_cube_offcenter_y),          AV_OPT_TYPE_FLOAT,   {.dbl =   0.0}, -1, 1,  .flags = FLAGS },
    { "cube_offcenter_z", "Offcenter cube displacement z",   OFFSET(fixed_cube_offcenter_z),          AV_OPT_TYPE_FLOAT,   {.dbl =   0.0}, -1, 1,  .flags = FLAGS },
    { "NEAREST",  NULL, 0, AV_OPT_TYPE_CONST, {.i64 = NEAREST },  0, 0, FLAGS, "interpolation_alg" },
    { "LINEAR",   NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LINEAR },   0, 0, FLAGS, "interpolation_alg" },
    { "CUBIC",    NULL, 0, AV_OPT_TYPE_CONST, {.i64 = CUBIC },    0, 0, FLAGS, "interpolation_alg" },
    { "LANCZOS4", NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LANCZOS4 }, 0, 0, FLAGS, "interpolation_alg" },
    { "nearest",  NULL, 0, AV_OPT_TYPE_CONST, {.i64 = NEAREST },  0, 0, FLAGS, "interpolation_alg" },
    { "linear",   NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LINEAR },   0, 0, FLAGS, "interpolation_alg" },
    { "cubic",    NULL, 0, AV_OPT_TYPE_CONST, {.i64 = CUBIC },    0, 0, FLAGS, "interpolation_alg" },
    { "lanczos4", NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LANCZOS4 }, 0, 0, FLAGS, "interpolation_alg" },
    { "interpolation_alg", "Interpolation algorithm", OFFSET(interpolation_alg), AV_OPT_TYPE_INT, {.i64 = 2}, 0, 4, .flags = FLAGS, "interpolation_alg"},
    { "width_scale_factor", "Scale factor of width for antialiasing", OFFSET(width_scale_factor), AV_OPT_TYPE_FLOAT, {.dbl = 1.0}, 0, 10, .flags = FLAGS, "width_scale_factor" },
    { "height_scale_factor", "Scale factor of height for antialiasing", OFFSET(height_scale_factor), AV_OPT_TYPE_FLOAT, {.dbl = 1.0}, 0, 10, .flags = FLAGS, "height_scale_factor" },
    { "enable_low_pass_filter", "Enable low pass filter-based antialiasing", OFFSET(enable_low_pass_filter), AV_OPT_TYPE_INT, {.i64 = 1}, 0, 1, .flags = FLAGS, "enable_low_pass_filter" },
    { "enable_multi_threading", "Enable multi-threading to speed up low pass filter-based antialiasing", OFFSET(enable_multi_threading), AV_OPT_TYPE_INT, {.i64 = 1}, 0, 1, .flags = FLAGS, "enable_multi_threading" },
    { "num_vertical_segments" , "Number of vertical segments per frame plane", OFFSET(num_vertical_segments), AV_OPT_TYPE_INT, {.i64 = 5}, 2, 500, .flags = FLAGS, "num_vertical_segments" },
    { "num_horizontal_segments" , "Number of horizontal segments per frame plane", OFFSET(num_horizontal_segments), AV_OPT_TYPE_INT, {.i64 = 1}, 1, 500, .flags = FLAGS, "num_horizontal_segments" },
    { "kernel_height_scale_factor", "Factor to scale the calculated kernel height for low pass filtering", OFFSET(kernel_height_scale_factor), AV_OPT_TYPE_FLOAT, {.dbl = 1.0}, 0.1, 100.0, .flags = FLAGS, "kernel_height_scale_factor" },
    { "min_kernel_half_height", "Half of the mininum kernel height", OFFSET(min_kernel_half_height), AV_OPT_TYPE_FLOAT, {.dbl = 1.0}, 0.5, 200, .flags = FLAGS, "min_kernel_half_height" },
    { "max_kernel_half_height", "The maximum value of the kernel height", OFFSET(max_kernel_half_height), AV_OPT_TYPE_FLOAT, {.dbl = 10000.0}, 0.5, 100000, .flags = FLAGS, "max_kernel_half_height" },
    { "adjust_kernel", "Enable adjustment of kernel", OFFSET(adjust_kernel), AV_OPT_TYPE_INT, {.i64 = 1}, 0, 1, .flags = FLAGS, "adjust_kernel" },
    { "kernel_adjust_factor", "Factor to further adjust the kernel size", OFFSET(kernel_adjust_factor), AV_OPT_TYPE_FLOAT, {.dbl = 1.0}, 0.1, 100.0, .flags = FLAGS, "kernel_adjust_factor" },
    { NULL }
};

static const AVClass transform360_class = {
    .class_name       = "transform360",
    .item_name        = av_default_item_name,
    .option           = transform360_options,
    .version          = LIBAVUTIL_VERSION_INT,
    .category         = AV_CLASS_CATEGORY_FILTER,
};

static const AVFilterPad avfilter_vf_transform_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
    },
    { NULL }
};

static const AVFilterPad avfilter_vf_transform_outputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
        .config_props = config_output,
    },
    { NULL }
};

AVFilter ff_vf_transform360 = {
    .name        = "transform360",
    .description = NULL_IF_CONFIG_SMALL("Transforms equirectangular input video to the other format."),
    .init_dict   = init_dict,
    .uninit      = uninit,
    .priv_size   = sizeof(TransformContext),
    .priv_class  = &transform360_class,
    .inputs      = avfilter_vf_transform_inputs,
    .outputs     = avfilter_vf_transform_outputs,
};
