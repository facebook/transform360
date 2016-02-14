/**
 * Copyright (c) 2015-present, Facebook, Inc.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

/**
 * @file
 * transform video filter
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
#define MAIN_PLANE_WIDTH (8.0f / 9.0f)

#define RIGHT   0
#define LEFT    1
#define TOP     2
#define BOTTOM  3
#define FRONT   4
#define BACK    5

#define UNPACK_ID(pair) ((pair) >> 6)
#define UNPACK_COUNT(pair) ((uint8_t) (pair) & 0x3F)
#define PACK_PAIR(id, count) (((id) << 6) ^ (count))

#define SOFTWARE_PREFETCH_OPT

static const int INITIAL_PAIR_SIZE = 2;

static const float PH = 0.25f;

static const float YC_TOP = 0.75f;
static const float YC_BOTTOM = 0.25f;

static const float Y_HALF = 1.0f / 2.0f;
static const float X_HALF = 1.0f / 2.0f;

// cube transform parameters
static const float P0[] = {-0.5f,-0.5f,-0.5f };
static const float P1[] = { 0.5f,-0.5f,-0.5f };
static const float P4[] = {-0.5f,-0.5f, 0.5f };
static const float P5[] = { 0.5f,-0.5f, 0.5f };
static const float P6[] = {-0.5f, 0.5f, 0.5f };

static const float PX[] = { 1.0f, 0.0f, 0.0f};
static const float PY[] = { 0.0f, 1.0f, 0.0f};
static const float PZ[] = { 0.0f, 0.0f, 1.0f};
static const float NX[] = {-1.0f, 0.0f, 0.0f};
static const float NZ[] = { 0.0f, 0.0f,-1.0f};

static const int PLANE_POLES_FACE_MAP[] = {1, 4, 0, 5, 2, 3};
static const int PLANE_CUBEMAP_FACE_MAP[] = {1, 4, 0, 5, 2, 3};
static const int PLANE_CUBEMAP_32_FACE_MAP[] = {1, 4, 0, 5, 3, 2};

typedef enum StereoFormat {
    STEREO_FORMAT_TB,
    STEREO_FORMAT_LR,
    STEREO_FORMAT_MONO,
    STEREO_FORMAT_GUESS,

    STEREO_FORMAT_N
} StereoFormat;

typedef enum Layout {
    LAYOUT_CUBEMAP,
    LAYOUT_CUBEMAP_32,
    LAYOUT_CUBEMAP_180,
    LAYOUT_PLANE_POLES,
    LAYOUT_PLANE_POLES_6,
    LAYOUT_PLANE_POLES_CUBEMAP,
    LAYOUT_PLANE_CUBEMAP,
    LAYOUT_PLANE_CUBEMAP_32,
    LAYOUT_FLAT_FIXED,

    LAYOUT_N
} Layout;

typedef struct TransformPixelWeights {
    uint32_t *pairs;
    uint8_t n;
} TransformPixelWeights;

typedef struct TransformPlaneMap {
    int w, h;
    TransformPixelWeights *weights;
} TransformPlaneMap;

typedef struct TransformContext {
    const AVClass *class;
    int w, h;
    TransformPlaneMap *out_map;
    int out_map_planes;

    AVDictionary *opts;
    char *w_expr;               ///< width  expression string
    char *h_expr;               ///< height expression string
    char *size_str;
    int cube_edge_length;
    int max_cube_edge_length;
    int output_layout;
    int input_stereo_format;
    int vflip;
    int planes;
    int w_subdivisons, h_subdivisons;
    float main_plane_ratio;
    float expand_coef;
    float fixed_yaw;    ///< Yaw (asimuth) angle, degrees
    float fixed_pitch;  ///< Pitch (elevation) angle, degrees
    float fixed_hfov;   ///< Horizontal field of view, degrees
    float fixed_vfov;   ///< Vertical field of view, degrees
} TransformContext;

typedef struct ThreadData {
    TransformPlaneMap *p;
    int subs;
    int linesize;
    int num_tiles;
    int num_tiles_col;
    uint8_t* in_data;
    uint8_t* out_data;
} ThreadData;

static int query_formats(AVFilterContext *ctx)
{
    static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_GBRP,
    AV_PIX_FMT_YUV410P,
    AV_PIX_FMT_YUV411P,
    AV_PIX_FMT_YUV420P,
    AV_PIX_FMT_YUV422P,
    AV_PIX_FMT_YUV440P,
    AV_PIX_FMT_YUV444P,
    AV_PIX_FMT_NONE
    };
    AVFilterFormats *fmts_list = ff_make_format_list(pix_fmts);
    if (!fmts_list)
        return AVERROR(ENOMEM);
    return ff_set_common_formats(ctx, fmts_list);
}

// We need to end up with X and Y coordinates in the range [0..1).
// Horizontally wrapping is easy: 1.25 becomes 0.25, -0.25 becomes 0.75.
// Vertically, if we pass through the north pole, we start coming back 'down'
// in the Y direction (ie, a reflection from the boundary) but we also are
// on the opposite side of the sphere so the X value changes by 0.5.
static inline void normalize_equirectangular(float x, float y, float *xout, float *yout) {
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

static inline void transform_pos(TransformContext *ctx, float x, float y, float *outX, float *outY) {
    int is_right = 0;
    if (ctx->input_stereo_format != STEREO_FORMAT_MONO) {
        if (y > Y_HALF) {
            y = (y - Y_HALF) / Y_HALF;
            if (ctx->vflip) {
                y = 1.0f - y;
            }
            is_right = 1;
        } else {
            y = y / Y_HALF;
        }
    }

    if (ctx->output_layout == LAYOUT_PLANE_POLES) {
        if (x >= ctx->main_plane_ratio) {
            float dx = (x * 2 - 1 - ctx->main_plane_ratio) / (1 - ctx->main_plane_ratio);
            if (y < Y_HALF) {
                // Bottom
                float dy = (y - YC_BOTTOM) / PH;
                *outX = (atan2f(dy, dx)) / (M_PI * 2.0f) + 0.75f;
                *outY = sqrtf(dy * dy + dx * dx) * 0.25f;
            } else {
                // Top
                float dy = (y - YC_TOP) / PH;
                *outX = (atan2f(dy, dx)) / (M_PI * 2.0f) + 0.75f;
                *outY = 1.0f - sqrtf(dy * dy + dx * dx) * 0.25f;
            }
            if (*outX > 1.0f) {
                *outX -= 1.0f;
            }
        } else {
            // Main
            *outX = x / ctx->main_plane_ratio;
            *outY = y * 0.5f + 0.25f;
        }
    } else if (ctx->output_layout == LAYOUT_PLANE_POLES_6) {
        int face = (int) (x * 6);
        if (face < 4) {
            // Main
            *outX = x * 6.0f / 4.0f;
            *outY = y * 0.5f + 0.25f;
        } else {
            float dx, dy;
            x = x * 6.0f - face;
            dx = x * 2 - 1;
            dy = y * 2 - 1;
            if (face == 4) {
                // Top
                *outX = (atan2f(dy, dx)) / (M_PI * 2.0f) + 0.75f;
                *outY = 1.0f - sqrtf(dy * dy + dx * dx) * 0.25f;
            } else {
                // Bottom
                *outX = (atan2f(dy, dx)) / (M_PI * 2.0f) + 0.75f;
                *outY = sqrtf(dy * dy + dx * dx) * 0.25f;
            }
            if (*outX > 1.0f) {
                *outX -= 1.0f;
            }
        }
    } else if (ctx->output_layout == LAYOUT_FLAT_FIXED) {
        // Per the Metadata RFC for orienting the equirectangular coords:
        //                           Heading
        //         -180           0           180
        //       90 +-------------+-------------+   0.0
        //          |             |             |
        //    P     |             |      o      |
        //    i     |             ^             |
        //    t   0 +-------------X-------------+   0.5
        //    c     |             |             |
        //    h     |             |             |
        //          |             |             |
        //      -90 +-------------+-------------+   1.0
        //          0.0          0.5          1.0
        //    X  - the default camera center
        //    ^  - the default up vector
        //    o  - the image center for a pitch of 45 and a heading of 90
        //    Coords on left and top sides are degrees
        //    Coords on right and bottom axes are our X/Y in range [0..1)
        //  Note: Negative field of view can be supplied to flip the image.
        *outX = ((x - 0.5f) * ctx->fixed_hfov + ctx->fixed_yaw)   / 360.0f
            + 0.5f;
        *outY = ((y - 0.5f) * ctx->fixed_vfov - ctx->fixed_pitch) / 180.0f
            + 0.5f;

        normalize_equirectangular(*outX, *outY, outX, outY);
    } else if (ctx->output_layout == LAYOUT_CUBEMAP ||
            ctx->output_layout == LAYOUT_PLANE_POLES_CUBEMAP ||
            ctx->output_layout == LAYOUT_CUBEMAP_32 ||
            ctx->output_layout == LAYOUT_CUBEMAP_180 ||
            ctx->output_layout == LAYOUT_PLANE_CUBEMAP_32 ||
            ctx->output_layout == LAYOUT_PLANE_CUBEMAP) {
        float qx, qy, qz;
        float cos_y, cos_p, sin_y, sin_p;
        float tx, ty, tz;
        float d;
        y = 1.0f - y;

        const float *vx, *vy, *p;
        int face = 0;
        if (ctx->output_layout == LAYOUT_CUBEMAP) {
            face = (int) (x * 6);
            x = x * 6.0f - face;
        } else if (ctx->output_layout == LAYOUT_CUBEMAP_32) {
            int vface = (int) (y * 2);
            int hface = (int) (x * 3);
            x = x * 3.0f - hface;
            y = y * 2.0f - vface;
            face = hface + (1 - vface) * 3;
        } else if (ctx->output_layout == LAYOUT_CUBEMAP_180) {
            // LAYOUT_CUBEMAP_180: layout for spatial resolution downsampling with 180 degree viewport size
            //
            // - Given a view (yaw,pitch) we can create a customized cube mapping to make the view center at the front cube face.
            // - A 180 degree viewport cut the cube into 2 equal-sized halves: front half and back half.
            // - The front half contains these faces of the cube: front, half of right, half of left, half of top, half of bottom.
            //   The back half contains these faces of the cube: back, half of right, half of left, half of top, half of bottom.
            //   Illutrasion on LAYOUT_CUBEMAP_32 (mono):
            //
            //   +---+---+---+---+---+---+
            //   |   |   |   |   |   5   |
            //   + 1 | 2 + 3 | 4 +-------+     Area 1, 4, 6, 7, 9 are in the front half
            //   |   |   |   |   |   6   |
            //   +---+---+---+---+---+---+     Area 2, 3, 5, 8, 0 are in the back half
            //   |   7   |       |       |
            //   +-------+   9   +   0   +
            //   |   8   |       |       |
            //   +---+---+---+---+---+---+
            //
            // - LAYOUT_CUBEMAP_180 reduces the spatial resolution of the back half to 25% (1/2 height, 1/2 width makes 1/4 size)
            //   and then re-pack the cube map like this:
            //
            //   +---+---+---+---+---+      Front half   Back half (1/4 size)
            //   |       |   |   c   |      ----------   --------------------
            //   +   a   + b +---+----      Area a = 9   Area f = 0
            //   |       |   | f |   |      Area b = 4   Area g = 3
            //   +---+---+---+---+ d +      Area c = 6   Area h = 2
            //   |g|h|-i-|   e   |   |      Area d = 1   Area i1(top) = 5
            //   +---+---+---+---+---+      Area e = 7   Area i2(bottom) = 8
            //
            if (0.0f <= y && y < 1.0f/3 && 0.0f <= x && x < 0.8f) { // Area g, h, i1, i2, e
                if (0.0f <= x && x < 0.1f) { // g
                    face = LEFT;
                    x = x/0.2f;
                    y = y/(1.0f/3);
                }
                else if (0.1f <= x && x < 0.2f) { // h
                    face = RIGHT;
                    x = (x-0.1f)/0.2f + 0.5f;
                    y = y/(1.0f/3);
                }
                else if (0.2f <= x && x < 0.4f) {
                    if (y >= 1.0f/6){ //i1
                        face = TOP;
                        x = (x-0.2f)/0.2f;
                        y  =(y-1.0f/6)/(1.0f/3) + 0.5f;
                    }
                    else { // i2
                        face = BOTTOM;
                        x = (x-0.2f)/0.2f;
                        y = y/(1.0f/3);
                    }
                }
                else if (0.4f <= x && x < 0.8f){ // e
                    face = BOTTOM;
                    x = (x-0.4f)/0.4f;
                    y = y/(2.0f/3) + 0.5f;
                }
            }
            else if (2.0f/3 <= y && y < 1.0f && 0.6f <= x && x < 1.0f) { // Area c
                face = TOP;
                x = (x-0.6f)/0.4f;
                y = (y-2.0f/3)/(2.0f/3);
            }
            else { // Area a, b, f, d
                if (0.0f <= x && x < 0.4f) { // a
                    face = FRONT;
                    x = x/0.4f;
                    y = (y-1.0/3)/(2.0f/3);
                }
                else if (0.4f <= x && x < 0.6f) { // b
                    face = LEFT;
                    x = (x-0.4f)/0.4f + 0.5f;
                    y = (y-1.0f/3)/(2.0f/3);
                }
                else if (0.6f <= x && x < 0.8f) { // f
                    face = BACK;
                    x = (x-0.6f)/0.2f;
                    y = (y-1.0f/3)/(1.0f/3);
                }
                else if (0.8f <= x && x < 1.0f) { // d
                    face = RIGHT;
                    x = (x-0.8f)/0.4f;
                    y = y/(2.0f/3);
                }
            }
        } else if (ctx->output_layout == LAYOUT_PLANE_CUBEMAP_32) {
            int vface = (int) (y * 2);
            int hface = (int) (x * 3);
            x = x * 3.0f - hface;
            y = y * 2.0f - vface;
            face = hface + (1 - vface) * 3;
            face = PLANE_CUBEMAP_32_FACE_MAP[face];
        } else if (ctx->output_layout == LAYOUT_PLANE_POLES_CUBEMAP) {
            face = (int) (x * 4.5f);
            x = x * 4.5f - face;
            if (face == 4) {
                x *= 2.0f;
                y *= 2.0f;
                if (y >= 1.0f) {
                    y -= 1.0f;
                } else {
                    face = 5; // bottom
                }
            }
            face = PLANE_POLES_FACE_MAP[face];
        } else if (ctx->output_layout == LAYOUT_PLANE_CUBEMAP) {
            face = (int) (x * 6);
            x = x * 6.0f - face;
            face = PLANE_CUBEMAP_FACE_MAP[face];
        } else {
            av_assert0(0);
        }
        av_assert1(x >= 0 && x <= 1);
        av_assert1(y >= 0 && y <= 1);
        av_assert1(face >= 0 && face < 6);
        x = (x - 0.5f) * ctx->expand_coef + 0.5f;
        y = (y - 0.5f) * ctx->expand_coef + 0.5f;

        switch (face) {
            case RIGHT:   p = P5; vx = NZ; vy = PY; break;
            case LEFT:    p = P0; vx = PZ; vy = PY; break;
            case TOP:     p = P6; vx = PX; vy = NZ; break;
            case BOTTOM:  p = P0; vx = PX; vy = PZ; break;
            case FRONT:   p = P4; vx = PX; vy = PY; break;
            case BACK:    p = P1; vx = NX; vy = PY; break;
        }
        qx = p [0] + vx [0] * x + vy [0] * y;
        qy = p [1] + vx [1] * x + vy [1] * y;
        qz = p [2] + vx [2] * x + vy [2] * y;

        // rotation
        sin_y = sin(ctx->fixed_yaw*M_PI/180.0f);
        sin_p = sin(ctx->fixed_pitch*M_PI/180.0f);
        cos_y = cos(ctx->fixed_yaw*M_PI/180.0f);
        cos_p = cos(ctx->fixed_pitch*M_PI/180.0f);
        tx = qx * cos_y   - qy * sin_y*sin_p  + qz * sin_y*cos_p;
        ty =                qy * cos_p        + qz * sin_p;
        tz = qx* (-sin_y) - qy * cos_y*sin_p  + qz * cos_y*cos_p;

        d = sqrtf(tx * tx + ty * ty + tz * tz);
        *outX = -atan2f (-tx / d, tz / d) / (M_PI * 2.0f) + 0.5f;
        *outY = asinf (-ty / d) / M_PI + 0.5f;
    }

    if (ctx->input_stereo_format == STEREO_FORMAT_TB) {
        if (is_right) {
            *outY = *outY * Y_HALF + Y_HALF;
        } else {
            *outY = *outY * Y_HALF;
        }
    } else if (ctx->input_stereo_format == STEREO_FORMAT_LR) {
        if (is_right) {
            *outX = *outX * X_HALF + X_HALF;
        } else {
            *outX = *outX * X_HALF;
        }
    } else {
        // mono no steps needed.
    }
    av_assert1(*outX >= 0 && *outX <= 1);
    av_assert1(*outY >= 0 && *outY <= 1);
}

static inline int increase_pixel_weight(TransformPixelWeights *ws, uint32_t id) {
    if (ws->n == 0) {
        ws->pairs = av_malloc_array(INITIAL_PAIR_SIZE, sizeof(*ws->pairs));
        if (!ws->pairs) {
            return AVERROR(ENOMEM);
        }
        *ws->pairs = PACK_PAIR(id, 1);
        ++ws->n;
        return 0;
    }

    // Looking for existing id
    for (int i = 0; i < ws->n; ++i) {
        if (UNPACK_ID(ws->pairs[i]) == id) {
            ++ws->pairs[i]; // since weight is packed in the lower bits, it works
            return 0;
        }
    }

    // if n is a power of 2, then we need to grow the array
    // grow array by power of 2, copy elements over
    if ((ws->n >= INITIAL_PAIR_SIZE) && !(ws->n & (ws->n - 1))) {
        uint32_t *new_pairs = av_malloc_array(ws->n * 2, sizeof(*ws->pairs));
        if (!new_pairs) {
            return AVERROR(ENOMEM);
        }
        memcpy(new_pairs, ws->pairs, sizeof(*ws->pairs) * ws->n);
        av_freep(&ws->pairs);
        ws->pairs = new_pairs;
    }

    ws->pairs[ws->n] = PACK_PAIR(id, 1);
    ++ws->n;

    return 0;
}

static inline int generate_map(TransformContext *s,
        AVFilterLink *inlink, AVFilterLink *outlink, AVFrame *in) {
    AVFilterContext *ctx = outlink->src;

    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(outlink->format);
    s->planes = av_pix_fmt_count_planes(outlink->format);
    s->out_map_planes = 2;
    s->out_map = av_malloc_array(s->out_map_planes, sizeof(*s->out_map));
    if (!s->out_map) {
        return AVERROR(ENOMEM);
    }

    for (int plane = 0; plane < s->out_map_planes; ++plane) {
        int out_w, out_h, in_w, in_h;
        TransformPlaneMap *p;
        av_log(ctx, AV_LOG_VERBOSE, "processing plane #%d\n",
                plane);
        out_w = outlink->w;
        out_h = outlink->h;
        in_w = inlink->w;
        in_h = inlink->h;

        if (plane == 1) {
            out_w = FF_CEIL_RSHIFT(out_w, desc->log2_chroma_w);
            out_h = FF_CEIL_RSHIFT(out_h, desc->log2_chroma_h);
            in_w = FF_CEIL_RSHIFT(in_w, desc->log2_chroma_w);
            in_h = FF_CEIL_RSHIFT(in_h, desc->log2_chroma_h);
        }
        p = &s->out_map[plane];
        p->w = out_w;
        p->h = out_h;
        p->weights = av_malloc_array(out_w * out_h, sizeof(*p->weights));
        if (!p->weights) {
            return AVERROR(ENOMEM);
        }
        for (int i = 0; i < out_h; ++i) {
            for (int j = 0; j < out_w; ++j) {
                int id = i * out_w + j;
                float out_x, out_y;
                TransformPixelWeights *ws = &p->weights[id];
                ws->n = 0;
                for (int suby = 0; suby < s->h_subdivisons; ++suby) {
                    for (int subx = 0; subx < s->w_subdivisons; ++subx) {
                        float y = (i + (suby + 0.5f) / s->h_subdivisons) / out_h;
                        float x = (j + (subx + 0.5f) / s->w_subdivisons) / out_w;
                        int in_x, in_y;
                        uint32_t in_id;
                        int result;
                        transform_pos(s, x, y, &out_x, &out_y);

                        in_y = (int) (out_y * in_h);
                        in_x = (int) (out_x * in_w);

                        in_id = in_y * in->linesize[plane] + in_x;
                        result = increase_pixel_weight(ws, in_id);
                        if (result != 0) {
                            return result;
                        }
                    }
                }
            }
        }
    }
    return 0;
}

static int config_output(AVFilterLink *outlink)
{
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
        if (s->output_layout == LAYOUT_CUBEMAP || s->output_layout == LAYOUT_PLANE_CUBEMAP) {
            outlink->w = s->cube_edge_length * 6;
            outlink->h = s->cube_edge_length;

            if (s->input_stereo_format == STEREO_FORMAT_TB || s->input_stereo_format == STEREO_FORMAT_LR)
                outlink->h = outlink->h * 2;
        } else if (s->output_layout == LAYOUT_CUBEMAP_32 || s->output_layout == LAYOUT_PLANE_CUBEMAP_32) {
            outlink->w = s->cube_edge_length * 3;
            outlink->h = s->cube_edge_length * 2;

            if (s->input_stereo_format == STEREO_FORMAT_TB || s->input_stereo_format == STEREO_FORMAT_LR)
                outlink->h = outlink->h * 2;
        } else if (s->output_layout == LAYOUT_CUBEMAP_180) {
            outlink->w = s->cube_edge_length * 2.5;
            outlink->h = s->cube_edge_length * 1.5;

            if (s->input_stereo_format == STEREO_FORMAT_TB || s->input_stereo_format == STEREO_FORMAT_LR)
                outlink->h = outlink->h * 2;
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

    av_log(ctx, AV_LOG_VERBOSE, "out_w:%d out_h:%d\n",
            outlink->w, outlink->h);

    return 0;
}

static av_cold int init_dict(AVFilterContext *ctx, AVDictionary **opts)
{
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

static av_cold void uninit(AVFilterContext *ctx)
{
    TransformContext *s = ctx->priv;

    for (int plane = 0; plane < s->out_map_planes; ++plane) {
        TransformPlaneMap *p = &s->out_map[plane];
        for (int i = 0; i < p->h * p->w; ++i) {
            av_freep(&p->weights[i].pairs);
            p->weights[i].pairs = NULL;
        }
        av_freep(&p->weights);
        p->weights = NULL;
    }
    av_freep(&s->out_map);
    s->out_map = NULL;

    av_dict_free(&s->opts);
    s->opts = NULL;
}

static void filter_slice_boundcheck(
        const int tile_i,
        const int tile_j,
        const int linesize,
        const int subs,
        const TransformPlaneMap *p,
        const uint8_t* in_data, uint8_t* out_data
        )
{
    for (int i = 0; i < 16; ++i) {
        if (tile_i + i >= p->h) {
            break;
        }

        int out_line = linesize * (tile_i + i);
        int map_line = p->w * (tile_i + i);
        for (int j = 0; j < 16; ++j) {
            if (tile_j + j >= p->w) {
                break;
            }

            int out_sample = out_line + tile_j + j;
            int id = map_line + tile_j + j;
            TransformPixelWeights *ws = &p->weights[id];
            if (ws->n == 1) {
                out_data[out_sample] = in_data[UNPACK_ID(ws->pairs[0])];
            } else {
                int color_sum = 0;
                for (int k = 0; k < ws->n; ++k) {
                    color_sum += ((int) in_data[UNPACK_ID(ws->pairs[k])]) *
                        UNPACK_COUNT(ws->pairs[k]);
                }
                // Round to nearest
                out_data[out_sample] = (uint8_t) ((color_sum + (subs >> 1)) / subs);
            }
        }
    }
}

static int filter_slice(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    const ThreadData *td = arg;
    const TransformPlaneMap *p = td->p;
    const int linesize = td->linesize;
    const int subs = td->subs;
    const int num_tiles = td->num_tiles;
    const int num_tiles_col = td->num_tiles_col;
    const uint8_t *in_data = td->in_data;
    uint8_t *out_data = td->out_data;

    const int tile_start = (num_tiles * jobnr) / nb_jobs;
    const int tile_end = (num_tiles * (jobnr+1)) / nb_jobs;

    for (int tile = tile_start ; tile < tile_end ; ++tile) {
        int tile_i = (tile / num_tiles_col) * 16;
        int tile_j = (tile % num_tiles_col) * 16;

#ifdef SOFTWARE_PREFETCH_OPT
        TransformPixelWeights *ws_prefetch;
        const uint8_t prefetch_lookahead = 8;
        int id_prefetch = p->w * tile_i + tile_j;

        // The loop below prefetches all the weights from array "p" and
        // associated pairs array. The prefetch is only done for the initial
        // lookahead for the first iteration of tile processing loop below so
        // that they are ready to be consumed in the inner loop. In the tile
        // processing loop, we are prefetching addresses that are after the
        // lookahead (i.e in the same iteration and also the next
        // ietartion of the loop).
        for(int k = 0; k < prefetch_lookahead; ++k){
           ws_prefetch = &p->weights[id_prefetch+k];
           __builtin_prefetch (ws_prefetch, 0, 0);
           __builtin_prefetch (ws_prefetch->pairs, 0, 0);
        }
        // Prefetch the cacheline for out_data for writes
        int out_sample_prefetch = linesize * (tile_i + 2) + tile_j;
        __builtin_prefetch (&out_data[out_sample_prefetch], 1, 0);
#endif

        if ((tile_i + 15) >= p->h || (tile_j + 15) >= p->w) {
            filter_slice_boundcheck(tile_i, tile_j, linesize, subs, p, in_data, out_data);
            continue;
        }

        for (int i = 0; i < 16; ++i) {
            int out_line = linesize * (tile_i + i);
            int map_line = p->w * (tile_i + i);

#ifdef SOFTWARE_PREFETCH_OPT
            // Prefetch the cacheline for out_data for writes
            __builtin_prefetch (&out_data[out_line+tile_j], 1, 0);
#endif

            for (int j = 0; j < 16; ++j) {
                int out_sample = out_line + tile_j + j;
                int id = map_line + tile_j + j;
                TransformPixelWeights *ws = &p->weights[id];

#ifdef SOFTWARE_PREFETCH_OPT
                // In this inner loop, we prefech the weight from array "p" after the
                // prefetch_lookahead iteration. We also prefetch the weight pairs
                // along with weight address as we found that we were getting
                // datacache (L1 and LLC) and DTLB misses for both the address
                // and the pair.
                if (j <  prefetch_lookahead) {
                   ws_prefetch = &p->weights[id+prefetch_lookahead];
                   __builtin_prefetch (ws_prefetch->pairs, 0, 0);
                   __builtin_prefetch (ws_prefetch+prefetch_lookahead, 0, 0);
                }
                else if (i < 15) {
                   // Here we are prefetching the address for the next iteration of outer loop
                   // so that we have the data avaialble in the next loop when it starts.
                   id_prefetch = p->w + id - prefetch_lookahead;
                   ws_prefetch = &p->weights[id_prefetch];
                   __builtin_prefetch (ws_prefetch->pairs, 0, 0);
                   __builtin_prefetch (ws_prefetch+prefetch_lookahead, 0, 0);
                }
                // Prefetch the cacheline for out_data for writes
                __builtin_prefetch (&out_data[out_sample], 1, 0);
#endif

                if (ws->n == 1) {
                    out_data[out_sample] = in_data[UNPACK_ID(ws->pairs[0])];
                } else {
                    int color_sum = 0;
                    for (int k = 0; k < ws->n; ++k) {
                        color_sum += ((int) in_data[UNPACK_ID(ws->pairs[k])]) *
                            UNPACK_COUNT(ws->pairs[k]);
                    }
                    // Round to nearest
                    out_data[out_sample] = (uint8_t) ((color_sum + (subs >> 1)) / subs);
                }
            }
        }
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    TransformContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *out;
    int subs;
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
    subs = s->w_subdivisons * s->h_subdivisons;

    for (int plane = 0; plane < s->planes; ++plane) {
        uint8_t *in_data, *out_data;
        int out_map_plane;
        TransformPlaneMap *p;

        in_data = in->data[plane];
        av_assert1(in_data);
        out_map_plane = (plane == 1 || plane == 2) ? 1 : 0;
        p = &s->out_map[out_map_plane];
        out_data = out->data[plane];

        int num_tiles_row = 1 + ((p->h - 1) / 16); // ceiling operation
        int num_tiles_col = 1 + ((p->w - 1) / 16); // ceiling operation
        int num_tiles = num_tiles_row * num_tiles_col;

        ThreadData td;
        td.p = p;
        td.subs = subs;
        td.linesize = out->linesize[plane];
        td.num_tiles = num_tiles;
        td.num_tiles_col = num_tiles_col;
        td.in_data = in_data;
        td.out_data = out_data;
        ctx->internal->execute(ctx, filter_slice, &td, NULL, FFMIN(num_tiles, ctx->graph->nb_threads));
    }

    av_log(ctx, AV_LOG_VERBOSE, "Done with byte copy \n");

    av_frame_free(&in);
    av_log(ctx, AV_LOG_VERBOSE, "Done freeing in \n");
    return ff_filter_frame(outlink, out);
}

#define OFFSET(x) offsetof(TransformContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption transform_options[] = {
    { "w",             "Output video width",          OFFSET(w_expr),    AV_OPT_TYPE_STRING, {.str = "0"}, CHAR_MIN, CHAR_MAX, FLAGS },
    { "width",         "Output video width",          OFFSET(w_expr),    AV_OPT_TYPE_STRING, {.str = "0"}, CHAR_MIN, CHAR_MAX, FLAGS },
    { "h",             "Output video height",         OFFSET(h_expr),    AV_OPT_TYPE_STRING, {.str = "0"}, CHAR_MIN, CHAR_MAX, FLAGS },
    { "height",        "Output video height",         OFFSET(h_expr),    AV_OPT_TYPE_STRING, {.str = "0"}, CHAR_MIN, CHAR_MAX, FLAGS },
    { "size",          "set video size",              OFFSET(size_str), AV_OPT_TYPE_STRING, {.str = NULL}, 0, FLAGS },
    { "s",             "set video size",              OFFSET(size_str), AV_OPT_TYPE_STRING, {.str = NULL}, 0, FLAGS },
    { "cube_edge_length", "Length of a cube edge (for cubic transform, overrides w and h, default 0 for off)",         OFFSET(cube_edge_length),    AV_OPT_TYPE_INT,  {.i64 = 0}, 0, 16384,  .flags = FLAGS },
    { "max_cube_edge_length", "Max length of a cube edge (for cubic transform, overrides w, h, and cube_edge_length, default 0 for off)",   OFFSET(max_cube_edge_length),    AV_OPT_TYPE_INT,  {.i64 = 0}, 0, 16384,  .flags = FLAGS },
    { "input_stereo_format", "Input video stereo format",         OFFSET(input_stereo_format),    AV_OPT_TYPE_INT,  {.i64 = STEREO_FORMAT_GUESS }, 0, STEREO_FORMAT_N - 1,  .flags = FLAGS, "stereo_format" },
    { "TB",      NULL, 0, AV_OPT_TYPE_CONST, {.i64 = STEREO_FORMAT_TB },      0, 0, FLAGS, "stereo_format" },
    { "LR",      NULL, 0, AV_OPT_TYPE_CONST, {.i64 = STEREO_FORMAT_LR },      0, 0, FLAGS, "stereo_format" },
    { "MONO",    NULL, 0, AV_OPT_TYPE_CONST, {.i64 = STEREO_FORMAT_MONO },    0, 0, FLAGS, "stereo_format" },
    { "GUESS",   NULL, 0, AV_OPT_TYPE_CONST, {.i64 = STEREO_FORMAT_GUESS },   0, 0, FLAGS, "stereo_format" },
    { "tb",      NULL, 0, AV_OPT_TYPE_CONST, {.i64 = STEREO_FORMAT_TB },      0, 0, FLAGS, "stereo_format" },
    { "lr",      NULL, 0, AV_OPT_TYPE_CONST, {.i64 = STEREO_FORMAT_LR },      0, 0, FLAGS, "stereo_format" },
    { "mono",    NULL, 0, AV_OPT_TYPE_CONST, {.i64 = STEREO_FORMAT_MONO },    0, 0, FLAGS, "stereo_format" },
    { "guess",   NULL, 0, AV_OPT_TYPE_CONST, {.i64 = STEREO_FORMAT_GUESS },   0, 0, FLAGS, "stereo_format" },
    { "output_layout", "Output video layout format",         OFFSET(output_layout),    AV_OPT_TYPE_INT,  {.i64 = LAYOUT_CUBEMAP_32 }, 0, LAYOUT_N - 1,  .flags = FLAGS, "layout" },
    { "CUBEMAP",             NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_CUBEMAP },             0, 0, FLAGS, "layout" },
    { "CUBEMAP_32",          NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_CUBEMAP_32 },          0, 0, FLAGS, "layout" },
    { "CUBEMAP_180",         NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_CUBEMAP_180 },         0, 0, FLAGS, "layout" },
    { "PLANE_POLES",         NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_PLANE_POLES },         0, 0, FLAGS, "layout" },
    { "PLANE_POLES_6",       NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_PLANE_POLES_6 },       0, 0, FLAGS, "layout" },
    { "PLANE_POLES_CUBEMAP", NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_PLANE_POLES_CUBEMAP }, 0, 0, FLAGS, "layout" },
    { "PLANE_CUBEMAP",       NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_PLANE_CUBEMAP },       0, 0, FLAGS, "layout" },
    { "PLANE_CUBEMAP_32",    NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_PLANE_CUBEMAP_32 },    0, 0, FLAGS, "layout" },
    { "FLAT_FIXED",          NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_FLAT_FIXED },          0, 0, FLAGS, "layout" },
    { "cubemap",             NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_CUBEMAP },             0, 0, FLAGS, "layout" },
    { "cubemap_32",          NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_CUBEMAP_32 },          0, 0, FLAGS, "layout" },
    { "cubemap_180",         NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_CUBEMAP_180 },         0, 0, FLAGS, "layout" },
    { "plane_poles",         NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_PLANE_POLES },         0, 0, FLAGS, "layout" },
    { "plane_poles_6",       NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_PLANE_POLES_6 },       0, 0, FLAGS, "layout" },
    { "plane_poles_cubemap", NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_PLANE_POLES_CUBEMAP }, 0, 0, FLAGS, "layout" },
    { "plane_cubemap",       NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_PLANE_CUBEMAP },       0, 0, FLAGS, "layout" },
    { "plane_cubemap_32",    NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_PLANE_CUBEMAP_32 },    0, 0, FLAGS, "layout" },
    { "flat_fixed",          NULL, 0, AV_OPT_TYPE_CONST, {.i64 = LAYOUT_FLAT_FIXED },          0, 0, FLAGS, "layout" },
    { "vflip", "Output video 2nd eye vertical flip (true, false)",         OFFSET(vflip),    AV_OPT_TYPE_INT, {.i64 = 0 }, 0, 1,     .flags = FLAGS, "vflip" },
    { "false",  NULL, 0, AV_OPT_TYPE_CONST, {.i64 = 0 }, 0, 0, FLAGS, "vflip" },
    { "true",   NULL, 0, AV_OPT_TYPE_CONST, {.i64 = 1 }, 0, 0, FLAGS, "vflip" },
    { "main_plane_ratio", "Output video main plain ratio for PLANE_POLES format (0.88888)",         OFFSET(main_plane_ratio),    AV_OPT_TYPE_FLOAT,  {.dbl=MAIN_PLANE_WIDTH}, 0, 1,  .flags = FLAGS },
    { "expand_coef", "Expansion coeffiecient for each face in cubemap (default 1.01)",         OFFSET(expand_coef),    AV_OPT_TYPE_FLOAT,  {.dbl=1.01f}, 0, 10,  .flags = FLAGS },
    { "w_subdivisons", "Number of horizontal per-pixel subdivisions for better downsampling (default 8)",         OFFSET(w_subdivisons),    AV_OPT_TYPE_INT,  {.i64 = 8}, 1, 8,  .flags = FLAGS },
    { "h_subdivisons", "Number of vertical per-pixel subdivisions for better downsampling (default 8)",         OFFSET(h_subdivisons),    AV_OPT_TYPE_INT,  {.i64 = 8}, 1, 8,  .flags = FLAGS },
    { "yaw", "View orientation for flat_fixed projection, degrees",   OFFSET(fixed_yaw),          AV_OPT_TYPE_FLOAT,   {.dbl =   0.0}, -360, 360,  .flags = FLAGS },
    { "pitch", "View orientation for flat_fixed projection, degrees", OFFSET(fixed_pitch),        AV_OPT_TYPE_FLOAT,   {.dbl =   0.0},  -90,  90,  .flags = FLAGS },
    { "hfov", "Horizontal field of view for flat_fixed projection, degrees (default 120)",  OFFSET(fixed_hfov), AV_OPT_TYPE_FLOAT,   {.dbl = 120.0}, -360, 360,  .flags = FLAGS },
    { "vfov", "Vertical field of view for flat_fixed projection, degrees (default 110)",     OFFSET(fixed_vfov), AV_OPT_TYPE_FLOAT,   {.dbl = 110.0}, -180, 180,  .flags = FLAGS },
    { NULL }
};

static const AVClass transform_class = {
    .class_name       = "transform",
    .item_name        = av_default_item_name,
    .option           = transform_options,
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

AVFilter ff_vf_transform = {
    .name           = "transform",
    .description    = NULL_IF_CONFIG_SMALL("Transforms equirectangular input video to the other format."),
    .init_dict      = init_dict,
    .uninit         = uninit,
    .priv_size      = sizeof(TransformContext),
    .priv_class     = &transform_class,
    .query_formats  = query_formats,
    .inputs         = avfilter_vf_transform_inputs,
    .outputs        = avfilter_vf_transform_outputs,
};
