/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/eval.h"
#include "libavutil/imgutils.h"
#include "libavutil/pixdesc.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "video.h"

enum Projections {
    EQUIRECTANGULAR,
    CUBEMAP_3_2,
    CUBEMAP_6_1,
    EQUIANGULAR,
    FLAT,
    NB_PROJECTIONS,
};

enum InterpMethod {
    NEAREST,
    BILINEAR,
    BICUBIC,
    LANCZOS,
    NB_INTERP_METHODS,
};

enum Faces {
    TOP_LEFT,
    TOP_MIDDLE,
    TOP_RIGHT,
    BOTTOM_LEFT,
    BOTTOM_MIDDLE,
    BOTTOM_RIGHT,
    NB_FACES,
};

enum Direction {
    RIGHT,
    LEFT,
    UP,
    DOWN,
    FRONT,
    BACK,
    NB_DIRECTIONS,
};

enum Rotation {
    ROT_0,
    ROT_90,
    ROT_180,
    ROT_270,
    NB_ROTATIONS,
};

typedef struct XYRemap {
    uint16_t u[4][4];
    uint16_t v[4][4];
    float ker[4][4];
} XYRemap;

typedef struct PanoramaContext {
    const AVClass *class;
    int in, out;
    int interp;
    int width, height;
    char* in_forder;
    char* out_forder;
    char* in_frot;
    char* out_frot;

    int in_cubemap_face_order[6];
    int out_cubemap_direction_order[6];
    int in_cubemap_face_rotation[6];
    int out_cubemap_face_rotation[6];

    float yaw, pitch, roll;

    int hflip, vflip, dflip;

    float h_fov, v_fov;
    float flat_range[3];

    int planewidth[4], planeheight[4];
    int inplanewidth[4], inplaneheight[4];
    int nb_planes;

    XYRemap *remap[4];

    int (*panorama_slice)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} PanoramaContext;

typedef struct ThreadData {
    PanoramaContext *s;
    AVFrame *in;
    AVFrame *out;
    int nb_planes;
} ThreadData;

#define OFFSET(x) offsetof(PanoramaContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM

static const AVOption panorama_options[] = {
    {     "input", "set input projection",              OFFSET(in), AV_OPT_TYPE_INT,    {.i64=EQUIRECTANGULAR}, 0,    NB_PROJECTIONS-1, FLAGS, "in" },
    {         "e", "equirectangular",                            0, AV_OPT_TYPE_CONST,  {.i64=EQUIRECTANGULAR}, 0,                   0, FLAGS, "in" },
    {      "c3x2", "cubemap3x2",                                 0, AV_OPT_TYPE_CONST,  {.i64=CUBEMAP_3_2},     0,                   0, FLAGS, "in" },
    {      "c6x1", "cubemap6x1",                                 0, AV_OPT_TYPE_CONST,  {.i64=CUBEMAP_6_1},     0,                   0, FLAGS, "in" },
    {       "eac", "equi-angular",                               0, AV_OPT_TYPE_CONST,  {.i64=EQUIANGULAR},     0,                   0, FLAGS, "in" },
    {    "output", "set output projection",            OFFSET(out), AV_OPT_TYPE_INT,    {.i64=CUBEMAP_3_2},     0,    NB_PROJECTIONS-1, FLAGS, "out" },
    {         "e", "equirectangular",                            0, AV_OPT_TYPE_CONST,  {.i64=EQUIRECTANGULAR}, 0,                   0, FLAGS, "out" },
    {      "c3x2", "cubemap3x2",                                 0, AV_OPT_TYPE_CONST,  {.i64=CUBEMAP_3_2},     0,                   0, FLAGS, "out" },
    {      "c6x1", "cubemap6x1",                                 0, AV_OPT_TYPE_CONST,  {.i64=CUBEMAP_6_1},     0,                   0, FLAGS, "out" },
    {       "eac", "equi-angular",                               0, AV_OPT_TYPE_CONST,  {.i64=EQUIANGULAR},     0,                   0, FLAGS, "out" },
    {      "flat", "regular video",                              0, AV_OPT_TYPE_CONST,  {.i64=FLAT},            0,                   0, FLAGS, "out" },
    {    "interp", "set interpolation method",      OFFSET(interp), AV_OPT_TYPE_INT,    {.i64=BILINEAR},        0, NB_INTERP_METHODS-1, FLAGS, "interp" },
    {      "near", "nearest neighbour",                          0, AV_OPT_TYPE_CONST,  {.i64=NEAREST},         0,                   0, FLAGS, "interp" },
    {      "line", "bilinear",                                   0, AV_OPT_TYPE_CONST,  {.i64=BILINEAR},        0,                   0, FLAGS, "interp" },
    {      "cube", "bicubic",                                    0, AV_OPT_TYPE_CONST,  {.i64=BICUBIC},         0,                   0, FLAGS, "interp" },
    {      "lanc", "lanczos",                                    0, AV_OPT_TYPE_CONST,  {.i64=LANCZOS},         0,                   0, FLAGS, "interp" },
    {         "w", "output width",                   OFFSET(width), AV_OPT_TYPE_INT,    {.i64=0},               0,           INT64_MAX, FLAGS, "w"},
    {         "h", "output height",                 OFFSET(height), AV_OPT_TYPE_INT,    {.i64=0},               0,           INT64_MAX, FLAGS, "h"},
    { "in_forder", "input cubemap face order",   OFFSET(in_forder), AV_OPT_TYPE_STRING, {.str="default"},       0,     NB_DIRECTIONS-1, FLAGS, "in_forder"},
    {"out_forder", "output cubemap face order", OFFSET(out_forder), AV_OPT_TYPE_STRING, {.str="default"},       0,     NB_DIRECTIONS-1, FLAGS, "out_forder"},
    {   "in_frot", "input cubemap face rotation",  OFFSET(in_frot), AV_OPT_TYPE_STRING, {.str="default"},       0,     NB_DIRECTIONS-1, FLAGS, "in_frot"},
    {  "out_frot", "output cubemap face rotation",OFFSET(out_frot), AV_OPT_TYPE_STRING, {.str="default"},       0,     NB_DIRECTIONS-1, FLAGS, "out_frot"},
    {       "yaw", "yaw rotation",                     OFFSET(yaw), AV_OPT_TYPE_FLOAT,  {.dbl=0.0},             -M_PI,            M_PI, FLAGS},
    {     "pitch", "pitch rotation",                 OFFSET(pitch), AV_OPT_TYPE_FLOAT,  {.dbl=0.0},             -M_PI,            M_PI, FLAGS},
    {      "roll", "roll rotation",                   OFFSET(roll), AV_OPT_TYPE_FLOAT,  {.dbl=0.0},             -M_PI,            M_PI, FLAGS},
    {     "h_fov", "horizontal field of view",       OFFSET(h_fov), AV_OPT_TYPE_FLOAT,  {.dbl=90.f},              0.f,           180.f, FLAGS},
    {     "v_fov", "vertical field of view",         OFFSET(v_fov), AV_OPT_TYPE_FLOAT,  {.dbl=45.f},              0.f,            90.f, FLAGS},
    {     "hflip", "flip video horizontally",        OFFSET(hflip), AV_OPT_TYPE_BOOL,   {.i64=0},                   0,               1, FLAGS},
    {     "vflip", "flip video vertically",          OFFSET(vflip), AV_OPT_TYPE_BOOL,   {.i64=0},                   0,               1, FLAGS},
    {     "dflip", "flip video indepth",             OFFSET(dflip), AV_OPT_TYPE_BOOL,   {.i64=0},                   0,               1, FLAGS},
    { NULL }
};

AVFILTER_DEFINE_CLASS(panorama);

static int query_formats(AVFilterContext *ctx)
{
    static const enum AVPixelFormat pix_fmts[] = {
        AV_PIX_FMT_YUVA444P, AV_PIX_FMT_YUVA422P, AV_PIX_FMT_YUVA420P,
        AV_PIX_FMT_YUVJ444P, AV_PIX_FMT_YUVJ440P, AV_PIX_FMT_YUVJ422P,AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVJ411P,
        AV_PIX_FMT_YUV444P, AV_PIX_FMT_YUV440P, AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV411P, AV_PIX_FMT_YUV410P,
        AV_PIX_FMT_GBRP, AV_PIX_FMT_GBRAP, AV_PIX_FMT_GRAY8, AV_PIX_FMT_NONE
    };

    AVFilterFormats *fmts_list = ff_make_format_list(pix_fmts);
    if (!fmts_list)
        return AVERROR(ENOMEM);
    return ff_set_common_formats(ctx, fmts_list);
}

static int nearest_slice(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ThreadData *td = (ThreadData*)arg;
    const PanoramaContext *s = td->s;
    const AVFrame *in = td->in;
    AVFrame *out = td->out;

    int plane, x, y;

    for (plane = 0; plane < td->nb_planes; plane++) {
        const int in_linesize  = in->linesize[plane];
        const int out_linesize = out->linesize[plane];
        const uint8_t *src = in->data[plane];
        uint8_t *dst = out->data[plane];
        const XYRemap *remap = s->remap[plane];
        const int width = s->planewidth[plane];
        const int height = s->planeheight[plane];

        const int slice_start = (height *  jobnr     ) / nb_jobs;
        const int slice_end   = (height * (jobnr + 1)) / nb_jobs;

        for (y = slice_start; y < slice_end; y++) {
            uint8_t *d = dst + y * out_linesize;
            for (x = 0; x < width; x++) {
                const XYRemap *r = &remap[y * width + x];

                *d++ = src[r->v[1][1] * in_linesize + r->u[1][1]];
            }
        }
    }

    return 0;
}

static void nearest_kernel(float mu, float nu, XYRemap *r)
{
    // Store coordinates of nearest pixel in [1][1]
    // to speed up nearest_slice
    const int i = roundf(nu) + 1;
    const int j = roundf(mu) + 1;

    r->u[1][1] = r->u[i][j];
    r->v[1][1] = r->v[i][j];
}

static int bilinear_slice(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ThreadData *td = (ThreadData*)arg;
    const PanoramaContext *s = td->s;
    const AVFrame *in = td->in;
    AVFrame *out = td->out;

    int plane, x, y, i, j;

    for (plane = 0; plane < td->nb_planes; plane++) {
        const int in_linesize  = in->linesize[plane];
        const int out_linesize = out->linesize[plane];
        const uint8_t *src = in->data[plane];
        uint8_t *dst = out->data[plane];
        const XYRemap *remap = s->remap[plane];
        const int width = s->planewidth[plane];
        const int height = s->planeheight[plane];

        const int slice_start = (height *  jobnr     ) / nb_jobs;
        const int slice_end   = (height * (jobnr + 1)) / nb_jobs;

        for (y = slice_start; y < slice_end; y++) {
            uint8_t *d = dst + y * out_linesize;
            for (x = 0; x < width; x++) {
                const XYRemap *r = &remap[y * width + x];
                float tmp = 0.f;

                for (i = 1; i < 3; i++) {
                    for (j = 1; j < 3; j++) {
                        tmp += r->ker[i][j] * src[r->v[i][j] * in_linesize + r->u[i][j]];
                    }
                }

                *d++ = roundf(tmp);
            }
        }
    }

    return 0;
}

static void bilinear_kernel(float mu, float nu, XYRemap *r)
{
    r->ker[1][1] = (1.f - mu) * (1.f - nu);
    r->ker[1][2] =        mu  * (1.f - nu);
    r->ker[2][1] = (1.f - mu) *        nu;
    r->ker[2][2] =        mu  *        nu;
}

static int bicubic_slice(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ThreadData *td = (ThreadData*)arg;
    const PanoramaContext *s = td->s;
    const AVFrame *in = td->in;
    AVFrame *out = td->out;

    int plane, x, y, i, j;

    for (plane = 0; plane < td->nb_planes; plane++) {
        const int in_linesize  = in->linesize[plane];
        const int out_linesize = out->linesize[plane];
        const uint8_t *src = in->data[plane];
        uint8_t *dst = out->data[plane];
        const XYRemap *remap = s->remap[plane];
        const int width = s->planewidth[plane];
        const int height = s->planeheight[plane];

        const int slice_start = (height *  jobnr     ) / nb_jobs;
        const int slice_end   = (height * (jobnr + 1)) / nb_jobs;

        for (y = slice_start; y < slice_end; y++) {
            uint8_t *d = dst + y * out_linesize;
            for (x = 0; x < width; x++) {
                const XYRemap *r = &remap[y * width + x];
                float tmp = 0.f;

                for (i = 0; i < 4; i++) {
                    for (j = 0; j < 4; j++) {
                        tmp += r->ker[i][j] * src[r->v[i][j] * in_linesize + r->u[i][j]];
                    }
                }

                *d++ = av_clip(tmp, 0, 255);
            }
        }
    }

    return 0;
}

static inline void calculate_bicubic_coeffs(float t, float *coeffs)
{
    const float tt  = t * t;
    const float ttt = t * t * t;

    coeffs[0] =     - t / 3.f + tt / 2.f - ttt / 6.f;
    coeffs[1] = 1.f - t / 2.f - tt       + ttt / 2.f;
    coeffs[2] =       t       + tt / 2.f - ttt / 2.f;
    coeffs[3] =     - t / 6.f            + ttt / 6.f;
}

static void bicubic_kernel(float mu, float nu, XYRemap *r)
{
    int i, j;
    float mu_coeffs[4];
    float nu_coeffs[4];

    calculate_bicubic_coeffs(mu, mu_coeffs);
    calculate_bicubic_coeffs(nu, nu_coeffs);

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            r->ker[i][j] = mu_coeffs[j] * nu_coeffs[i];
        }
    }
}

static int lanczos_slice(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ThreadData *td = (ThreadData*)arg;
    const PanoramaContext *s = td->s;
    const AVFrame *in = td->in;
    AVFrame *out = td->out;

    int plane, x, y, i, j;

    for (plane = 0; plane < td->nb_planes; plane++) {
        const int in_linesize  = in->linesize[plane];
        const int out_linesize = out->linesize[plane];
        const uint8_t *src = in->data[plane];
        uint8_t *dst = out->data[plane];
        const XYRemap *remap = s->remap[plane];
        const int width = s->planewidth[plane];
        const int height = s->planeheight[plane];

        const int slice_start = (height *  jobnr     ) / nb_jobs;
        const int slice_end   = (height * (jobnr + 1)) / nb_jobs;

        for (y = slice_start; y < slice_end; y++) {
            uint8_t *d = dst + y * out_linesize;
            for (x = 0; x < width; x++) {
                const XYRemap *r = &remap[y * width + x];
                float tmp = 0.f;

                for (i = 0; i < 4; i++) {
                    for (j = 0; j < 4; j++) {
                        tmp += r->ker[i][j] * src[r->v[i][j] * in_linesize + r->u[i][j]];
                    }
                }

                *d++ = av_clip(tmp, 0, 255);
            }
        }
    }

    return 0;

}

static inline void calculate_lanczos_coeffs(float t, float *coeffs)
{
    int i;
    float sum = 0.f;

    for (i = 0; i < 4; i++) {
        const float x = M_PI * (t - i + 1);
        if (x == 0.f) {
            coeffs[i] = 1.f;
        } else {
            coeffs[i] = sinf(x) * sinf(x / 2.f) / (x * x / 2.f);
        }
        sum += coeffs[i];
    }

    for (int i = 0; i < 4; i++) {
        coeffs[i] /= sum;
    }
}

static void lanczos_kernel(float mu, float nu, XYRemap *r)
{
    int i, j;
    float mu_coeffs[4];
    float nu_coeffs[4];

    calculate_lanczos_coeffs(mu, mu_coeffs);
    calculate_lanczos_coeffs(nu, nu_coeffs);

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            r->ker[i][j] = mu_coeffs[j] * nu_coeffs[i];
        }
    }
}

static inline int equal(float a, float b, float epsilon)
{
    return fabsf(a - b) < epsilon;
}

static inline int smaller(float a, float b, float epsilon)
{
    return ((a - b) < 0.f) && (!equal(a, b, epsilon));
}

static inline int in_range(float rd, float small, float large, float res)
{
   return    !smaller(rd, small, res)
          &&  smaller(rd, large, res);
}

static inline int mod(int a, int b)
{
    const int res = a % b;
    if (res < 0) {
        return res + b;
    } else {
        return res;
    }
}

static int get_direction(char c)
{
    switch (c) {
    case 'r':
        return RIGHT;
    case 'l':
        return LEFT;
    case 'u':
        return UP;
    case 'd':
        return DOWN;
    case 'f':
        return FRONT;
    case 'b':
        return BACK;
    default:
        return -1;
    }
}

static int get_rotation(char c)
{
    switch (c) {
        case '0':
            return ROT_0;
        case '1':
            return ROT_90;
        case '2':
            return ROT_180;
        case '3':
            return ROT_270;
        default:
            return -1;
    }
}

static int prepare_cube_in(PanoramaContext *s)
{
    if (strcmp(s->in_forder, "default") == 0) {
        s->in_cubemap_face_order[RIGHT] = TOP_LEFT;
        s->in_cubemap_face_order[LEFT] = TOP_MIDDLE;
        s->in_cubemap_face_order[UP] = TOP_RIGHT;
        s->in_cubemap_face_order[DOWN] = BOTTOM_LEFT;
        s->in_cubemap_face_order[FRONT] = BOTTOM_MIDDLE;
        s->in_cubemap_face_order[BACK] = BOTTOM_RIGHT;
    } else {
        for (int face = 0; face < NB_FACES; face++) {
            const char c = s->in_forder[face];
            int direction;
            if (c == '\0') {
                return AVERROR(EINVAL);
            }

            direction = get_direction(c);
            if (direction == -1) {
                return AVERROR(EINVAL);
            }

            s->in_cubemap_face_order[direction] = face;
        }
    }

    if (strcmp(s->in_frot, "default") == 0) {
        s->in_cubemap_face_rotation[TOP_LEFT] = ROT_0;
        s->in_cubemap_face_rotation[TOP_MIDDLE] = ROT_0;
        s->in_cubemap_face_rotation[TOP_RIGHT] = ROT_0;
        s->in_cubemap_face_rotation[BOTTOM_LEFT] = ROT_0;
        s->in_cubemap_face_rotation[BOTTOM_MIDDLE] = ROT_0;
        s->in_cubemap_face_rotation[BOTTOM_RIGHT] = ROT_0;
    } else {
        for (int face = 0; face < NB_FACES; face++) {
            const char c = s->in_frot[face];
            int rotation;

            if (c == '\0') {
                return AVERROR(EINVAL);
            }

            rotation = get_rotation(c);
            if (rotation == -1) {
                return AVERROR(EINVAL);
            }

            s->in_cubemap_face_rotation[face] = rotation;
        }
    }

    return 0;
}

static int prepare_cube_out(PanoramaContext *s)
{
    if (strcmp(s->out_forder, "default") == 0) {
        s->out_cubemap_direction_order[TOP_LEFT] = RIGHT;
        s->out_cubemap_direction_order[TOP_MIDDLE] = LEFT;
        s->out_cubemap_direction_order[TOP_RIGHT] = UP;
        s->out_cubemap_direction_order[BOTTOM_LEFT] = DOWN;
        s->out_cubemap_direction_order[BOTTOM_MIDDLE] = FRONT;
        s->out_cubemap_direction_order[BOTTOM_RIGHT] = BACK;
    } else {
        for (int face = 0; face < NB_FACES; face++) {
            const char c = s->out_forder[face];
            int direction;

            if (c == '\0') {
                return AVERROR(EINVAL);
            }

            direction = get_direction(c);
            if (direction == -1) {
                return AVERROR(EINVAL);
            }

            s->out_cubemap_direction_order[face] = direction;
        }
    }

    if (strcmp(s->out_frot, "default") == 0) {
        s->out_cubemap_face_rotation[TOP_LEFT] = ROT_0;
        s->out_cubemap_face_rotation[TOP_MIDDLE] = ROT_0;
        s->out_cubemap_face_rotation[TOP_RIGHT] = ROT_0;
        s->out_cubemap_face_rotation[BOTTOM_LEFT] = ROT_0;
        s->out_cubemap_face_rotation[BOTTOM_MIDDLE] = ROT_0;
        s->out_cubemap_face_rotation[BOTTOM_RIGHT] = ROT_0;
    } else {
        for (int face = 0; face < NB_FACES; face++) {
            const char c = s->out_frot[face];
            int rotation;

            if (c == '\0') {
                return AVERROR(EINVAL);
            }

            rotation = get_rotation(c);
            if (rotation == -1) {
                return AVERROR(EINVAL);
            }

            s->out_cubemap_face_rotation[face] = rotation;
        }
    }

    return 0;
}

static inline void rotate_cube_face(float *uf, float *vf, int rotation)
{
    float tmp;

    switch (rotation) {
    case ROT_0:
        break;
    case ROT_90:
        tmp = *uf;
        *uf = -*vf;
        *vf = tmp;
        break;
    case ROT_180:
        *uf = -*uf;
        *vf = -*vf;
        break;
    case ROT_270:
        tmp = -*uf;
        *uf = *vf;
        *vf = tmp;
        break;
    }
}

static inline void rotate_cube_face_inverse(float *uf, float *vf, int rotation)
{
    float tmp;

    switch (rotation) {
    case ROT_0:
        break;
    case ROT_90:
        tmp = -*uf;
        *uf = *vf;
        *vf = tmp;
        break;
    case ROT_180:
        *uf = -*uf;
        *vf = -*vf;
        break;
    case ROT_270:
        tmp = *uf;
        *uf = -*vf;
        *vf = tmp;
        break;
    }
}

static void cube_to_xyz(const PanoramaContext *s,
                        float uf, float vf, int face,
                        float *vec)
{
    const int direction = s->out_cubemap_direction_order[face];
    float norm;
    float l_x, l_y, l_z;

    rotate_cube_face_inverse(&uf, &vf, s->out_cubemap_face_rotation[face]);

    switch (direction) {
    case RIGHT:
        l_x =  1.f;
        l_y = -vf;
        l_z =  uf;
        break;
    case LEFT:
        l_x = -1.f;
        l_y = -vf;
        l_z = -uf;
        break;
    case UP:
        l_x =  uf;
        l_y =  1.f;
        l_z = -vf;
        break;
    case DOWN:
        l_x =  uf;
        l_y = -1.f;
        l_z =  vf;
        break;
    case FRONT:
        l_x =  uf;
        l_y = -vf;
        l_z = -1.f;
        break;
    case BACK:
        l_x = -uf;
        l_y = -vf;
        l_z =  1.f;
        break;
    }

    norm = sqrtf(l_x * l_x + l_y * l_y + l_z * l_z);
    vec[0] = l_x / norm;
    vec[1] = l_y / norm;
    vec[2] = l_z / norm;
}

static void xyz_to_cube(const PanoramaContext *s,
                        const float *vec, float res,
                        float *uf, float *vf, int *direction)
{
    const float phi   = atan2f(vec[0], -vec[2]);
    const float theta = asinf(-vec[1]);
    float phi_norm, theta_threshold;
    int face;

    if (in_range(phi, -M_PI_4, M_PI_4, res)) {
        *direction = FRONT;
        phi_norm = phi;
    } else if (in_range(phi, -(M_PI_2 + M_PI_4), -M_PI_4, res)) {
        *direction = LEFT;
        phi_norm = phi + M_PI_2;
    } else if (in_range(phi, M_PI_4, M_PI_2 + M_PI_4, res)) {
        *direction = RIGHT;
        phi_norm = phi - M_PI_2;
    } else {
        *direction = BACK;
        phi_norm = phi + ((phi > 0.f) ? -M_PI : M_PI);
    }

    theta_threshold = atanf(cosf(phi_norm));
    if (theta > theta_threshold) {
        *direction = DOWN;
    } else if (theta < -theta_threshold) {
        *direction = UP;
    }

    switch (*direction) {
    case RIGHT:
        *uf =  vec[2] / vec[0];
        *vf = -vec[1] / vec[0];
        break;
    case LEFT:
        *uf =  vec[2] / vec[0];
        *vf =  vec[1] / vec[0];
        break;
    case UP:
        *uf =  vec[0] / vec[1];
        *vf = -vec[2] / vec[1];
        break;
    case DOWN:
        *uf = -vec[0] / vec[1];
        *vf = -vec[2] / vec[1];
        break;
    case FRONT:
        *uf = -vec[0] / vec[2];
        *vf =  vec[1] / vec[2];
        break;
    case BACK:
        *uf = -vec[0] / vec[2];
        *vf = -vec[1] / vec[2];
        break;
    }

    face = s->in_cubemap_face_order[*direction];
    rotate_cube_face(uf, vf, s->in_cubemap_face_rotation[face]);
}

/*
 *  Default cubemap
 *
 *           width
 *         <------->
 *         +-------+
 *         |       |                              U
 *         | up    |                   h       ------->
 * +-------+-------+-------+-------+ ^ e      |
 * |       |       |       |       | | i    V |
 * | left  | front | right | back  | | g      |
 * +-------+-------+-------+-------+ v h      v
 *         |       |                   t
 *         | down  |
 *         +-------+
 */
static void process_cube_coordinates(const PanoramaContext *s,
                                float uf, float vf, int direction,
                                float *new_uf, float *new_vf, int *face)
{
    *face = s->in_cubemap_face_order[direction];
    rotate_cube_face_inverse(&uf, &vf, s->in_cubemap_face_rotation[*face]);

    if ((uf < -1.f || uf >= 1.f) && (vf < -1.f || vf >= 1.f)) {
        // There are no pixels to use in this case
        *new_uf = uf;
        *new_vf = vf;
    } else if (uf < -1.f) {
        uf += 2.f;
        switch (direction) {
        case RIGHT:
            direction = FRONT;
            *new_uf = uf;
            *new_vf = vf;
            break;
        case LEFT:
            direction = BACK;
            *new_uf = uf;
            *new_vf = vf;
            break;
        case UP:
            direction = LEFT;
            *new_uf = vf;
            *new_vf = -uf;
            break;
        case DOWN:
            direction = LEFT;
            *new_uf = -vf;
            *new_vf = uf;
            break;
        case FRONT:
            direction = LEFT;
            *new_uf = uf;
            *new_vf = vf;
            break;
        case BACK:
            direction = RIGHT;
            *new_uf = uf;
            *new_vf = vf;
            break;
        }
    } else if (uf >= 1.f) {
        uf -= 2.f;
        switch (direction) {
        case RIGHT:
            direction = BACK;
            *new_uf = uf;
            *new_vf = vf;
            break;
        case LEFT:
            direction = FRONT;
            *new_uf = uf;
            *new_vf = vf;
            break;
        case UP:
            direction = RIGHT;
            *new_uf = -vf;
            *new_vf = uf;
            break;
        case DOWN:
            direction = RIGHT;
            *new_uf = vf;
            *new_vf = -uf;
            break;
        case FRONT:
            direction = RIGHT;
            *new_uf = uf;
            *new_vf = vf;
            break;
        case BACK:
            direction = LEFT;
            *new_uf = uf;
            *new_vf = vf;
            break;
        }
    } else if (vf < -1.f) {
        vf += 2.f;
        switch (direction) {
        case RIGHT:
            direction = UP;
            *new_uf = vf;
            *new_vf = -uf;
            break;
        case LEFT:
            direction = UP;
            *new_uf = -vf;
            *new_vf = uf;
            break;
        case UP:
            direction = BACK;
            *new_uf = -uf;
            *new_vf = -vf;
            break;
        case DOWN:
            direction = FRONT;
            *new_uf = uf;
            *new_vf = vf;
            break;
        case FRONT:
            direction = UP;
            *new_uf = uf;
            *new_vf = vf;
            break;
        case BACK:
            direction = UP;
            *new_uf = -uf;
            *new_vf = -vf;
            break;
        }
    } else if (vf >= 1.f) {
        vf -= 2.f;
        switch (direction) {
        case RIGHT:
            direction = DOWN;
            *new_uf = -vf;
            *new_vf = uf;
            break;
        case LEFT:
            direction = DOWN;
            *new_uf = vf;
            *new_vf = -uf;
            break;
        case UP:
            direction = FRONT;
            *new_uf = uf;
            *new_vf = vf;
            break;
        case DOWN:
            direction = BACK;
            *new_uf = -uf;
            *new_vf = -vf;
            break;
        case FRONT:
            direction = DOWN;
            *new_uf = uf;
            *new_vf = vf;
            break;
        case BACK:
            direction = DOWN;
            *new_uf = -uf;
            *new_vf = -vf;
            break;
        }
    } else {
        // Inside cube face
        *new_uf = uf;
        *new_vf = vf;
    }

    *face = s->in_cubemap_face_order[direction];
    rotate_cube_face(new_uf, new_vf, s->in_cubemap_face_rotation[*face]);
}

static void cube3x2_to_xyz(const PanoramaContext *s,
                           int i, int j, int width, int height,
                           float *vec)
{
    const float ew = width  / 3.f;
    const float eh = height / 2.f;

    const int u_face = floorf(i / ew);
    const int v_face = floorf(j / eh);
    const int face = u_face + 3 * v_face;

    const int ushift = ceilf(ew * u_face);
    const int vshift = ceilf(eh * v_face);
    const int ewi = ceilf(ew * (u_face + 1)) - ushift;
    const int ehi = ceilf(eh * (v_face + 1)) - vshift;

    const float uf = 2.f * (i - ushift) / ewi - 1.f;
    const float vf = 2.f * (j - vshift) / ehi - 1.f;

    cube_to_xyz(s, uf, vf, face, vec);
}

static void xyz_to_cube3x2(const PanoramaContext *s,
                           const float *vec, int width, int height,
                           uint16_t us[4][4], uint16_t vs[4][4], float *mu, float *nu)
{
    const float res = M_PI_4 / (width / 3) / 10.f;
    const float ew = width  / 3.f;
    const float eh = height / 2.f;
    float uf, vf;
    int ui, vi;
    int ewi, ehi;
    int i, j;
    int direction, face;
    int u_face, v_face;

    xyz_to_cube(s, vec, res, &uf, &vf, &direction);

    face = s->in_cubemap_face_order[direction];
    u_face = face % 3;
    v_face = face / 3;
    ewi = ceilf(ew * (u_face + 1)) - ceilf(ew * u_face);
    ehi = ceilf(eh * (v_face + 1)) - ceilf(eh * v_face);

    uf = 0.5f * ewi * (uf + 1.f);
    vf = 0.5f * ehi * (vf + 1.f);

    ui = floorf(uf);
    vi = floorf(vf);

    *mu = uf - ui;
    *nu = vf - vi;

    for (i = -1; i < 3; i++) {
        for (j = -1; j < 3; j++) {
            float u, v;
            int u_shift, v_shift;
            int new_ewi, new_ehi;

            process_cube_coordinates(s, 2.f * (ui + j) / ewi - 1.f,
                                        2.f * (vi + i) / ehi - 1.f,
                                        direction, &u, &v, &face);
            u_face = face % 3;
            v_face = face / 3;
            u_shift = ceilf(ew * u_face);
            v_shift = ceilf(eh * v_face);
            new_ewi = ceilf(ew * (u_face + 1)) - u_shift;
            new_ehi = ceilf(eh * (v_face + 1)) - v_shift;

            us[i + 1][j + 1] = u_shift + av_clip(roundf(0.5f * new_ewi * (u + 1.f)), 0, new_ewi - 1);
            vs[i + 1][j + 1] = v_shift + av_clip(roundf(0.5f * new_ehi * (v + 1.f)), 0, new_ehi - 1);
        }
    }
}

static void cube6x1_to_xyz(const PanoramaContext *s,
                           int i, int j, int width, int height,
                           float *vec)
{
    const float ew = width / 6.f;
    const float eh = height;

    const int face = floorf(i / ew);

    const int u_shift = ceilf(ew * face);
    const int ewi = ceilf(ew * (face + 1)) - u_shift;

    const float uf = 2.f * (i - u_shift) / ewi - 1.f;
    const float vf = 2.f *  j            / eh  - 1.f;

    cube_to_xyz(s, uf, vf, face, vec);
}

static void xyz_to_cube6x1(const PanoramaContext *s,
                           const float *vec, int width, int height,
                           uint16_t us[4][4], uint16_t vs[4][4], float *mu, float *nu)
{
    const float res = M_PI_4 / (width / 6) / 10.f;
    const float ew = width / 6.f;
    const float eh = height;
    float uf, vf;
    int ui, vi;
    int ewi;
    int i, j;
    int direction, face;

    xyz_to_cube(s, vec, res, &uf, &vf, &direction);

    face = s->in_cubemap_face_order[direction];
    ewi = ceilf(ew * (face + 1)) - ceilf(ew * face);

    uf = 0.5f * ewi * (uf + 1.f);
    vf = 0.5f * eh  * (vf + 1.f);

    ui = floorf(uf);
    vi = floorf(vf);

    *mu = uf - ui;
    *nu = vf - vi;

    for (i = -1; i < 3; i++) {
        for (j = -1; j < 3; j++) {
            float u, v;
            int u_shift;
            int new_ewi;

            process_cube_coordinates(s, 2.f * (ui + j) / ewi - 1.f,
                                        2.f * (vi + i) / eh  - 1.f,
                                        direction, &u, &v, &face);
            u_shift = ceilf(ew * face);
            new_ewi = ceilf(ew * (face + 1)) - u_shift;

            us[i + 1][j + 1] = u_shift + av_clip(roundf(0.5f * new_ewi * (u + 1.f)), 0, new_ewi - 1);
            vs[i + 1][j + 1] =           av_clip(roundf(0.5f * eh      * (v + 1.f)), 0, eh      - 1);
        }
    }
}

static void equirect_to_xyz(const PanoramaContext *s,
                            int i, int j, int width, int height,
                            float *vec)
{
    const float phi   = ((2.f * i) / width  - 1.f) * M_PI;
    const float theta = ((2.f * j) / height - 1.f) * M_PI_2;

    const float sin_phi   = sinf(phi);
    const float cos_phi   = cosf(phi);
    const float sin_theta = sinf(theta);
    const float cos_theta = cosf(theta);

    vec[0] =  cos_theta * sin_phi;
    vec[1] = -sin_theta;
    vec[2] = -cos_theta * cos_phi;
}

static void xyz_to_equirect(const PanoramaContext *s,
                            const float *vec, int width, int height,
                            uint16_t us[4][4], uint16_t vs[4][4], float *mu, float *nu)
{
    const float phi   = atan2f(vec[0], -vec[2]);
    const float theta = asinf(-vec[1]);
    float uf, vf;
    int ui, vi;
    int i, j;

    uf = (phi   / M_PI   + 1.f) * width  / 2.f;
    vf = (theta / M_PI_2 + 1.f) * height / 2.f;
    ui = floorf(uf);
    vi = floorf(vf);

    *mu = uf - ui;
    *nu = vf - vi;

    for (i = -1; i < 3; i++) {
        for (j = -1; j < 3; j++) {
            us[i + 1][j + 1] = mod(ui + j, width);
            vs[i + 1][j + 1] = av_clip(vi + i, 0, height - 1);
        }
    }
}

static int prepare_eac_in(PanoramaContext *s)
{
    s->in_cubemap_face_order[RIGHT] = TOP_RIGHT;
    s->in_cubemap_face_order[LEFT] = TOP_LEFT;
    s->in_cubemap_face_order[UP] = BOTTOM_RIGHT;
    s->in_cubemap_face_order[DOWN] = BOTTOM_LEFT;
    s->in_cubemap_face_order[FRONT] = TOP_MIDDLE;
    s->in_cubemap_face_order[BACK] = BOTTOM_MIDDLE;

    s->in_cubemap_face_rotation[TOP_LEFT] = ROT_0;
    s->in_cubemap_face_rotation[TOP_MIDDLE] = ROT_0;
    s->in_cubemap_face_rotation[TOP_RIGHT] = ROT_0;
    s->in_cubemap_face_rotation[BOTTOM_LEFT] = ROT_270;
    s->in_cubemap_face_rotation[BOTTOM_MIDDLE] = ROT_90;
    s->in_cubemap_face_rotation[BOTTOM_RIGHT] = ROT_270;

    return 0;
}

static int prepare_eac_out(PanoramaContext *s)
{
    s->out_cubemap_direction_order[TOP_LEFT] = LEFT;
    s->out_cubemap_direction_order[TOP_MIDDLE] = FRONT;
    s->out_cubemap_direction_order[TOP_RIGHT] = RIGHT;
    s->out_cubemap_direction_order[BOTTOM_LEFT] = DOWN;
    s->out_cubemap_direction_order[BOTTOM_MIDDLE] = BACK;
    s->out_cubemap_direction_order[BOTTOM_RIGHT] = UP;

    s->out_cubemap_face_rotation[TOP_LEFT] = ROT_0;
    s->out_cubemap_face_rotation[TOP_MIDDLE] = ROT_0;
    s->out_cubemap_face_rotation[TOP_RIGHT] = ROT_0;
    s->out_cubemap_face_rotation[BOTTOM_LEFT] = ROT_270;
    s->out_cubemap_face_rotation[BOTTOM_MIDDLE] = ROT_90;
    s->out_cubemap_face_rotation[BOTTOM_RIGHT] = ROT_270;

    return 0;
}

static void eac_to_xyz(const PanoramaContext *s,
                       int i, int j, int width, int height,
                       float *vec)
{
    const float ew = width  / 3.f;
    const float eh = height / 2.f;

    const int u_face = floorf(i / ew);
    const int v_face = floorf(j / eh);
    const int face = u_face + 3 * v_face;

    const int u_shift = ceilf(ew * u_face);
    const int v_shift = ceilf(eh * v_face);
    const int ewi = ceilf(ew * (u_face + 1)) - u_shift;
    const int ehi = ceilf(eh * (v_face + 1)) - v_shift;

    const float upad = 3.f / ewi;
    const float vpad = 3.f / ehi;

    float uf = tanf(M_PI_2 * ((0.5f + i - u_shift) / ewi - 0.5f));
    float vf = tanf(M_PI_2 * ((0.5f + j - v_shift) / ehi - 0.5f));


    // Process padding
    switch (face) {
        case TOP_LEFT:
        case BOTTOM_LEFT:
            uf = (1.f + upad) * (uf + 1.f) - 1.f - 2 * upad;
            break;
        case TOP_RIGHT:
        case BOTTOM_RIGHT:
            uf = (1.f + upad) * (uf + 1.f) - 1.f;
            break;
    }
    vf = (1.f + 2 * vpad) * (vf + 1.f) - 1.f - 2 * vpad;

    cube_to_xyz(s, uf, vf, face, vec);
}

static void xyz_to_eac(const PanoramaContext *s,
                       const float *vec, int width, int height,
                       uint16_t us[4][4], uint16_t vs[4][4], float *mu, float *nu)
{
    const float res = M_PI_4 / (width / 3) / 10.f;
    const float ew = width  / 3.f;
    const float eh = height / 2.f;
    const float upad = 3.f / ew;
    const float vpad = 3.f / eh;
    float uf, vf;
    int ui, vi;
    int ewi, ehi;
    int i, j;
    int direction, face;
    int u_face, v_face;
    int u_shift, v_shift;

    xyz_to_cube(s, vec, res, &uf, &vf, &direction);

    face = s->in_cubemap_face_order[direction];
    switch (face) {
        case TOP_LEFT:
        case BOTTOM_LEFT:
            uf = (uf + 1.f + 2 * upad) / (1.f + upad) - 1.f;
            break;
        case TOP_RIGHT:
        case BOTTOM_RIGHT:
            uf = (uf + 1.f) / (1.f + upad) - 1.f;
            break;
    }
    vf = (vf + 1.f + 2 * vpad) / (1.f + 2 * vpad) - 1.f;

    u_face = face % 3;
    v_face = face / 3;
    u_shift = ceilf(ew * u_face);
    v_shift = ceilf(eh * v_face);
    ewi = ceilf(ew * (u_face + 1)) - u_shift;
    ehi = ceilf(eh * (v_face + 1)) - v_shift;

    uf = ewi * (M_2_PI * atanf(uf) + 0.5f) - 0.5f;
    vf = ehi * (M_2_PI * atanf(vf) + 0.5f) - 0.5f;

    ui = floorf(uf);
    vi = floorf(vf);

    *mu = uf - ui;
    *nu = vf - vi;

    for (i = -1; i < 3; i++) {
        for (j = -1; j < 3; j++) {
            us[i + 1][j + 1] = u_shift + ui + j;
            vs[i + 1][j + 1] = v_shift + vi + i;
        }
    }
}

static int prepare_flat_out(PanoramaContext *s)
{
    const float h_angle = s->h_fov * M_PI / 360.f;
    const float v_angle = s->v_fov * M_PI / 360.f;

    const float sin_phi   = sinf(h_angle);
    const float cos_phi   = cosf(h_angle);
    const float sin_theta = sinf(v_angle);
    const float cos_theta = cosf(v_angle);

    s->flat_range[0] =  cos_theta * sin_phi;
    s->flat_range[1] =  sin_theta;
    s->flat_range[2] = -cos_theta * cos_phi;

    return 0;
}

static void flat_to_xyz(const PanoramaContext *s,
                        int i, int j, int width, int height,
                        float *vec)
{
    const float l_x =  s->flat_range[0] * (2.f * i / width  - 1.f);
    const float l_y = -s->flat_range[1] * (2.f * j / height - 1.f);
    const float l_z =  s->flat_range[2];

    const float norm = sqrtf(l_x * l_x + l_y * l_y + l_z * l_z);

    vec[0] = l_x / norm;
    vec[1] = l_y / norm;
    vec[2] = l_z / norm;
}

static inline void calculate_rotation_matrix(float yaw, float pitch, float roll,
                                             float rot_mat[3][3])
{
    const float sin_yaw = sinf(-yaw);
    const float cos_yaw = cosf(-yaw);
    const float sin_pitch = sinf(pitch);
    const float cos_pitch = cosf(pitch);
    const float sin_roll = sinf(roll);
    const float cos_roll = cosf(roll);

    rot_mat[0][0] = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    rot_mat[0][1] = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
    rot_mat[0][2] = sin_yaw * cos_pitch;

    rot_mat[1][0] = cos_pitch * sin_roll;
    rot_mat[1][1] = cos_pitch * cos_roll;
    rot_mat[1][2] = -sin_pitch;

    rot_mat[2][0] = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    rot_mat[2][1] = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
    rot_mat[2][2] = cos_yaw * cos_pitch;
}

static inline void rotate(const float rot_mat[3][3],
                          float *vec)
{
    const float x_tmp = vec[0] * rot_mat[0][0] + vec[1] * rot_mat[0][1] + vec[2] * rot_mat[0][2];
    const float y_tmp = vec[0] * rot_mat[1][0] + vec[1] * rot_mat[1][1] + vec[2] * rot_mat[1][2];
    const float z_tmp = vec[0] * rot_mat[2][0] + vec[1] * rot_mat[2][1] + vec[2] * rot_mat[2][2];

    vec[0] = x_tmp;
    vec[1] = y_tmp;
    vec[2] = z_tmp;
}

static inline void set_mirror_modifier(int hflip, int vflip, int dflip,
                                       float *modifier)
{
    modifier[0] = hflip ? -1.f : 1.f;
    modifier[1] = vflip ? -1.f : 1.f;
    modifier[2] = dflip ? -1.f : 1.f;
}

static inline void mirror(const float *modifier,
                          float *vec)
{
    vec[0] *= modifier[0];
    vec[1] *= modifier[1];
    vec[2] *= modifier[2];
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    PanoramaContext *s = ctx->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    int err;
    int p, h, w;
    float hf, wf;
    float mirror_modifier[3];
    void (*in_transform)(const PanoramaContext *s,
                         const float *vec, int width, int height,
                         uint16_t us[4][4], uint16_t vs[4][4], float *mu, float *nu);
    void (*out_transform)(const PanoramaContext *s,
                          int i, int j, int width, int height,
                          float *vec);
    void (*calculate_kernel)(float mu, float nu, XYRemap *r);
    float rot_mat[3][3];

    switch (s->interp) {
    case NEAREST:
        s->panorama_slice = nearest_slice;
        calculate_kernel = nearest_kernel;
        break;
    case BILINEAR:
        s->panorama_slice = bilinear_slice;
        calculate_kernel = bilinear_kernel;
        break;
    case BICUBIC:
        s->panorama_slice = bicubic_slice;
        calculate_kernel = bicubic_kernel;
        break;
    case LANCZOS:
        s->panorama_slice = lanczos_slice;
        calculate_kernel = lanczos_kernel;
        break;
    }

    switch (s->in) {
    case EQUIRECTANGULAR:
        in_transform = xyz_to_equirect;
        err = 0;
        wf = inlink->w;
        hf = inlink->h;
        break;
    case CUBEMAP_3_2:
        in_transform = xyz_to_cube3x2;
        err = prepare_cube_in(s);
        wf = inlink->w / 3.f * 4.f;
        hf = inlink->h;
        break;
    case CUBEMAP_6_1:
        in_transform = xyz_to_cube6x1;
        err = prepare_cube_in(s);
        wf = inlink->w / 3.f * 2.f;
        hf = inlink->h * 2.f;
        break;
    case EQUIANGULAR:
        in_transform = xyz_to_eac;
        err = prepare_eac_in(s);
        wf = inlink->w;
        hf = inlink->h / 9.f * 8.f;
        break;
    }

    if (err != 0) {
        return err;
    }

    switch (s->out) {
    case EQUIRECTANGULAR:
        out_transform = equirect_to_xyz;
        err = 0;
        w = wf;
        h = hf;
        break;
    case CUBEMAP_3_2:
        out_transform = cube3x2_to_xyz;
        err = prepare_cube_out(s);
        w = wf / 4.f * 3.f;
        h = hf;
        break;
    case CUBEMAP_6_1:
        out_transform = cube6x1_to_xyz;
        err = prepare_cube_out(s);
        w = wf / 2.f * 3.f;
        h = hf / 2.f;
        break;
    case EQUIANGULAR:
        out_transform = eac_to_xyz;
        err = prepare_eac_out(s);
        w = wf;
        h = hf / 8.f * 9.f;
        break;
    case FLAT:
        out_transform = flat_to_xyz;
        err = prepare_flat_out(s);
        w = wf * s->flat_range[0] / s->flat_range[1] / 2.f;
        h = hf;
        break;
    }

    if (err != 0) {
        return err;
    }

    if (s->width > 0 && s->height > 0) {
        w = s->width;
        h = s->height;
    }

    s->planeheight[1] = s->planeheight[2] = FF_CEIL_RSHIFT(h, desc->log2_chroma_h);
    s->planeheight[0] = s->planeheight[3] = h;
    s->planewidth[1] = s->planewidth[2] = FF_CEIL_RSHIFT(w, desc->log2_chroma_w);
    s->planewidth[0] = s->planewidth[3] = w;

    outlink->h = h;
    outlink->w = w;

    s->inplaneheight[1] = s->inplaneheight[2] = FF_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);
    s->inplaneheight[0] = s->inplaneheight[3] = inlink->h;
    s->inplanewidth[1]  = s->inplanewidth[2]  = FF_CEIL_RSHIFT(inlink->w, desc->log2_chroma_w);
    s->inplanewidth[0]  = s->inplanewidth[3]  = inlink->w;
    s->nb_planes = av_pix_fmt_count_planes(inlink->format);

    for (p = 0; p < s->nb_planes; p++) {
        s->remap[p] = av_calloc(s->planewidth[p] * s->planeheight[p], sizeof(XYRemap));
        if (!s->remap[p])
            return AVERROR(ENOMEM);
    }

    calculate_rotation_matrix(s->yaw, s->pitch, s->roll, rot_mat);
    set_mirror_modifier(s->hflip, s->vflip, s->dflip, mirror_modifier);

    for (p = 0; p < s->nb_planes; p++) {
        const int width = s->planewidth[p];
        const int height = s->planeheight[p];
        const int in_width = s->inplanewidth[p];
        const int in_height = s->inplaneheight[p];
        float mu, nu;
        float vec[3];
        int i, j;

        for (i = 0; i < width; i++) {
            for (j = 0; j < height; j++) {
                XYRemap *r = &s->remap[p][j * width + i];

                out_transform(s, i, j, width, height, vec);
                rotate(rot_mat, vec);
                mirror(mirror_modifier, vec);
                in_transform(s, vec, in_width, in_height, r->u, r->v, &mu, &nu);
                calculate_kernel(mu, nu, r);
            }
        }
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    PanoramaContext *s = ctx->priv;
    AVFrame *out;
    ThreadData td;

    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);

    td.s = s;
    td.in = in;
    td.out = out;
    td.nb_planes = s->nb_planes;

    ctx->internal->execute(ctx, s->panorama_slice, &td, NULL, FFMIN(outlink->h, ff_filter_get_nb_threads(ctx)));

    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    PanoramaContext *s = ctx->priv;
    int p;

    for (p = 0; p < s->nb_planes; p++)
        av_freep(&s->remap[p]);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
    },
    { NULL }
};

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_output,
    },
    { NULL }
};

AVFilter ff_vf_panorama = {
    .name          = "panorama",
    .description   = NULL_IF_CONFIG_SMALL("Convert panorama projection of video."),
    .priv_size     = sizeof(PanoramaContext),
    .uninit        = uninit,
    .query_formats = query_formats,
    .inputs        = inputs,
    .outputs       = outputs,
    .priv_class    = &panorama_class,
    .flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC | AVFILTER_FLAG_SLICE_THREADS,
};
