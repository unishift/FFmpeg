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

#include "libavutil/avassert.h"
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
    NB_PROJECTIONS,
};

enum InterpMethod {
    NEAREST,
    BILINEAR,
    BICUBIC,
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

struct XYRemap {
    int u[4];
    int v[4];
    float ker[4][4];
};

typedef struct PanoramaContext {
    const AVClass *class;
    int in, out;
    int interp;
    char* in_forder;
    char* out_forder;
    char* in_frot;
    char* out_frot;

    int planewidth[4], planeheight[4];
    int inplanewidth[4], inplaneheight[4];
    int nb_planes;

    struct XYRemap *remap[4];

    int (*panorama)(struct PanoramaContext *s,
                    const uint8_t *src, uint8_t *dst,
                    int width, int height,
                    int in_linesize, int out_linesize,
                    const struct XYRemap *remap);
} PanoramaContext;

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
    {    "interp", "set interpolation method",      OFFSET(interp), AV_OPT_TYPE_INT,    {.i64=BILINEAR},        0, NB_INTERP_METHODS-1, FLAGS, "interp" },
    {      "near", "nearest neighbour",                          0, AV_OPT_TYPE_CONST,  {.i64=NEAREST},         0,                   0, FLAGS, "interp" },
    {      "line", "bilinear",                                   0, AV_OPT_TYPE_CONST,  {.i64=BILINEAR},        0,                   0, FLAGS, "interp" },
    {      "cube", "bicubic",                                    0, AV_OPT_TYPE_CONST,  {.i64=BICUBIC},         0,                   0, FLAGS, "interp" },
    { "in_forder", "input cubemap face order",   OFFSET(in_forder), AV_OPT_TYPE_STRING, {.str="default"},       0,     NB_DIRECTIONS-1, FLAGS, "in_forder"},
    {"out_forder", "output cubemap face order", OFFSET(out_forder), AV_OPT_TYPE_STRING, {.str="default"},       0,     NB_DIRECTIONS-1, FLAGS, "out_forder"},
    {   "in_frot", "input cubemap face rotation",  OFFSET(in_frot), AV_OPT_TYPE_STRING, {.str="default"},       0,     NB_DIRECTIONS-1, FLAGS, "in_frot"},
    {  "out_frot", "output cubemap face rotation",OFFSET(out_frot), AV_OPT_TYPE_STRING, {.str="default"},       0,     NB_DIRECTIONS-1, FLAGS, "out_frot"},
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

static int nearest(PanoramaContext *s,
                   const uint8_t *src, uint8_t *dst,
                   int width, int height,
                   int in_linesize, int out_linesize,
                   const struct XYRemap *remap)
{
    int x, y;

    for (y = 0; y < height; y++) {
        uint8_t *d = dst + y * out_linesize;
        for (x = 0; x < width; x++) {
            const struct XYRemap *r = &remap[y * width + x];

            *d++ = src[r->v[1] * in_linesize + r->u[1]];
        }
    }

    return 0;
}

static void nearest_kernel(float mu, float nu, float kernel[4][4])
{
    return;
}

static int bilinear(PanoramaContext *s,
                    const uint8_t *src, uint8_t *dst,
                    int width, int height,
                    int in_linesize, int out_linesize,
                    const struct XYRemap *remap)
{
    int x, y, i, j;

    for (y = 0; y < height; y++) {
        uint8_t *d = dst + y * out_linesize;
        for (x = 0; x < width; x++) {
            const struct XYRemap *r = &remap[y * width + x];
            float tmp = 0.f;

            for (i = 1; i < 3; i++) {
                for (j = 1; j < 3; j++) {
                    tmp += r->ker[i][j] * src[r->v[i] * in_linesize + r->u[j]];
                }
            }

            *d++ = roundf(tmp);
        }
    }

    return 0;
}

static void bilinear_kernel(float mu, float nu, float kernel[4][4])
{
    kernel[1][1] = (1.f - mu) * (1.f - nu);
    kernel[1][2] =        mu  * (1.f - nu);
    kernel[2][1] = (1.f - mu) *        nu;
    kernel[2][2] =        mu  *        nu;
}

static int bicubic(PanoramaContext *s,
                   const uint8_t *src, uint8_t *dst,
                   int width, int height,
                   int in_linesize, int out_linesize,
                   const struct XYRemap *remap)
{
    int x, y, i, j;

    for (y = 0; y < height; y++) {
        uint8_t *d = dst + y * out_linesize;
        for (x = 0; x < width; x++) {
            const struct XYRemap *r = &remap[y * width + x];
            float tmp = 0.f;

            for (i = 0; i < 4; i++) {
                for (j = 0; j < 4; j++) {
                    tmp += r->ker[i][j] * src[r->v[i] * in_linesize + r->u[j]];
                }
            }

            *d++ = av_clip(tmp, 0, 255);
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

static void bicubic_kernel(float mu, float nu, float kernel[4][4])
{
    int i, j;
    float mu_coeffs[4];
    float nu_coeffs[4];

    calculate_bicubic_coeffs(mu, mu_coeffs);
    calculate_bicubic_coeffs(nu, nu_coeffs);

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            kernel[i][j] = mu_coeffs[j] * nu_coeffs[i];
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

static int in_cubemap_face_order[6] = {
    TOP_LEFT,     TOP_MIDDLE,    TOP_RIGHT,
    BOTTOM_LEFT,  BOTTOM_MIDDLE, BOTTOM_RIGHT,
};

static int out_cubemap_direction_order[6] = {
    RIGHT, LEFT,  UP,
    DOWN,  FRONT, BACK,
};

static int in_cubemap_face_rotation[6] = {
    ROT_0, ROT_0, ROT_0,
    ROT_0, ROT_0, ROT_0,
};

static int out_cubemap_face_rotation[6] = {
    ROT_0, ROT_0, ROT_0,
    ROT_0, ROT_0, ROT_0,
};

static void cube_to_xyz(float uf, float vf, int face,
                        float *x, float *y, float *z)
{
    int direction = out_cubemap_direction_order[face];
    float norm, tmp;
    float l_x, l_y, l_z;

    switch (out_cubemap_face_rotation[face]) {
    case ROT_0:
        break;
    case ROT_90:
        tmp = -uf;
        uf = vf;
        vf = tmp;
        break;
    case ROT_180:
        uf = -uf;
        vf = -vf;
        break;
    case ROT_270:
        tmp = uf;
        uf = -vf;
        vf = tmp;
        break;
    }

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
    *x = l_x / norm;
    *y = l_y / norm;
    *z = l_z / norm;
}

static void xyz_to_cube(float x, float y, float z, float res,
                        float *uf, float *vf, int *face)
{
    float phi   = atan2f(x, -z);
    float theta = asinf(-y);
    float phi_norm, theta_threshold;
    float tmp;
    int direction;

    if (in_range(phi, -M_PI_4, M_PI_4, res)) {
        direction = FRONT;
        phi_norm = phi;
    } else if (in_range(phi, -(M_PI_2 + M_PI_4), -M_PI_4, res)) {
        direction = LEFT;
        phi_norm = phi + M_PI_2;
    } else if (in_range(phi, M_PI_4, M_PI_2 + M_PI_4, res)) {
        direction = RIGHT;
        phi_norm = phi - M_PI_2;
    } else {
        direction = BACK;
        phi_norm = phi + ((phi > 0.f) ? -M_PI : M_PI);
    }

    theta_threshold = atan2f(1.f, 1.f / cosf(phi_norm));
    if (theta > theta_threshold) {
        direction = DOWN;
    } else if (theta < -theta_threshold) {
        direction = UP;
    }

    *face = in_cubemap_face_order[direction];
    switch (direction) {
    case RIGHT:
        *uf =  z / x;
        *vf = -y / x;
        break;
    case LEFT:
        *uf =  z / x;
        *vf =  y / x;
        break;
    case UP:
        *uf =  x / y;
        *vf = -z / y;
        break;
    case DOWN:
        *uf = -x / y;
        *vf = -z / y;
        break;
    case FRONT:
        *uf = -x / z;
        *vf =  y / z;
        break;
    case BACK:
        *uf = -x / z;
        *vf = -y / z;
        break;
    }

    switch (in_cubemap_face_rotation[*face]) {
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

static void cube3x2_to_xyz(int i, int j, int width, int height,
                           float *x, float *y, float *z)
{
    int ew = width / 3;
    int eh = height / 2;
    int face = (i / ew) + 3 * (j / eh);
    float uf = 2.f * (i % ew) / ew - 1.f;
    float vf = 2.f * (j % eh) / eh - 1.f;

    cube_to_xyz(uf, vf, face, x, y, z);
}

static void xyz_to_cube3x2(float x, float y, float z, int width, int height,
                           int *us, int *vs, float *mu, float *nu)
{
    float res = M_PI_4 / (width / 3) / 10.f;
    float uf, vf;
    float rh = height / 4.f;
    float rw = width / 6.f;
    int ui, vi;
    int u_shift, v_shift;
    int i;
    int face;

    xyz_to_cube(x, y, z, res, &uf, &vf, &face);
    uf = rw * (uf + 1.f);
    vf = rh * (vf + 1.f);

    ui = floorf(uf);
    vi = floorf(vf);

    *mu = uf - ui;
    *nu = vf - vi;

    u_shift = (width  / 3.f) * (face % 3);
    v_shift = (height / 2.f) * (face / 3);
    for (i = -1; i < 3; i++) {
        us[i + 1] = u_shift + av_clip(ui + i, 0, 2 * rw - 1);
        vs[i + 1] = v_shift + av_clip(vi + i, 0, 2 * rh - 1);
    }
}

static void cube6x1_to_xyz(int i, int j, int width, int height,
                           float *x, float *y, float *z)
{
    int ew = width / 6;
    int eh = height;
    int face = i / ew;
    float uf = 2.f * (i % ew) / ew - 1.f;
    float vf = 2.f * (j % eh) / eh - 1.f;

    cube_to_xyz(uf, vf, face, x, y, z);
}

static void xyz_to_cube6x1(float x, float y, float z, int width, int height,
                           int *us, int *vs, float *mu, float *nu)
{
    float res = M_PI_4 / (width / 6) / 10.f;
    float uf, vf;
    float rh = height / 2;
    float rw = width / 12;
    int ui, vi;
    int u_shift;
    int i;
    int face;

    xyz_to_cube(x, y, z, res, &uf, &vf, &face);
    uf = rw * (uf + 1.f);
    vf = rh * (vf + 1.f);

    ui = floorf(uf);
    vi = floorf(vf);

    *mu = uf - ui;
    *nu = vf - vi;

    u_shift = (width / 6.f) * face;
    for (i = -1; i < 3; i++) {
        us[i + 1] = u_shift + av_clip(ui + i, 0, 2 * rw - 1);
        vs[i + 1] =           av_clip(vi + i, 0, 2 * rh - 1);

    }
}

static void equirect_to_xyz(int i, int j, int width, int height,
                            float *x, float *y, float *z)
{
    float phi   = ((2.f * i) / width  - 1.f) * M_PI;
    float theta = ((2.f * j) / height - 1.f) * M_PI_2;

    const float sin_phi   = sinf(phi);
    const float cos_phi   = cosf(phi);
    const float sin_theta = sinf(theta);
    const float cos_theta = cosf(theta);

    *x =  cos_theta * sin_phi;
    *y = -sin_theta;
    *z = -cos_theta * cos_phi;
}

static void xyz_to_equirect(float x, float y, float z, int width, int height,
                            int *us, int *vs, float *mu, float *nu)
{
    float uf, vf;
    int ui, vi;
    int i;
    float phi   = atan2f(x, -z);
    float theta = asinf(-y);

    uf = (phi   / M_PI   + 1.f) * width  / 2.f;
    vf = (theta / M_PI_2 + 1.f) * height / 2.f;
    ui = floorf(uf);
    vi = floorf(vf);

    *mu = uf - ui;
    *nu = vf - vi;

    for (i = -1; i < 3; i++) {
        us[i + 1] = (ui + i) % width;
        vs[i + 1] = av_clip(vi + i, 0, height - 1);

    }
}

static void eac_to_xyz(int i, int j, int width, int height,
                       float *x, float *y, float *z)
{
    int ew = width / 3;
    int eh = height / 2;
    int face = (i / ew) + 3 * (j / eh);
    float uf = tanf(M_PI_2 * ((float)(i % ew) / ew - 0.5f));
    float vf = tanf(M_PI_2 * ((float)(j % eh) / eh - 0.5f));

    cube_to_xyz(uf, vf, face, x, y, z);
}

static void xyz_to_eac(float x, float y, float z, int width, int height,
                       int *us, int *vs, float *mu, float *nu)
{
    float res = M_PI_4 / (width / 3) / 10.f;
    float uf, vf;
    float rh = height / 2.f;
    float rw = width / 3.f;
    int ui, vi;
    int i;
    int u_shift, v_shift;
    int face;

    xyz_to_cube(x, y, z, res, &uf, &vf, &face);
    uf = rw * (M_2_PI * atanf(uf) + 0.5f);
    vf = rh * (M_2_PI * atanf(vf) + 0.5f);

    ui = floorf(uf);
    vi = floorf(vf);

    *mu = uf - ui;
    *nu = vf - vi;

    u_shift = (width  / 3.f) * (face % 3);
    v_shift = (height / 2.f) * (face / 3);
    for (i = -1; i < 3; i++) {
        us[i + 1] = u_shift + av_clip(ui + i, 0, rw - 1);
        vs[i + 1] = v_shift + av_clip(vi + i, 0, rh - 1);
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


static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    PanoramaContext *s = ctx->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    int p, h, w;
    float hf, wf;
    void (*in_transform)(float x, float y, float z, int width, int height,
                         int *us, int *vs, float *mu, float *nu);
    void (*out_transform)(int i, int j, int width, int height,
                          float *x, float *y, float *z);
    void (*calculate_kernel)(float mu, float nu, float kernel[4][4]);

    switch (s->interp) {
    case NEAREST:
        s->panorama = nearest;
        calculate_kernel = nearest_kernel;
        break;
    case BILINEAR:
        s->panorama = bilinear;
        calculate_kernel = bilinear_kernel;
        break;
    case BICUBIC:
        s->panorama = bicubic;
        calculate_kernel = bicubic_kernel;
        break;
    default:
        av_assert0(0);
    }

    switch (s->in) {
    case EQUIRECTANGULAR:
    case EQUIANGULAR:
        wf = inlink->w;
        hf = inlink->h;
        break;
    case CUBEMAP_3_2:
        wf = inlink->w / 3.f * 4.f;
        hf = inlink->h;
        break;
    case CUBEMAP_6_1:
        wf = inlink->w / 3.f * 2.f;
        hf = inlink->h * 2.f;
        break;
    default:
        av_assert0(0);
    }

    switch (s->out) {
    case EQUIRECTANGULAR:
    case EQUIANGULAR:
        w = wf;
        h = hf;
        break;
    case CUBEMAP_3_2:
        w = wf / 4.f * 3.f;
        h = hf;
        break;
    case CUBEMAP_6_1:
        w = wf / 2.f * 3.f;
        h = hf / 2.f;
        break;
    default:
        av_assert0(0);
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
        s->remap[p] = av_calloc(s->planewidth[p] * s->planeheight[p], sizeof(struct XYRemap));
        if (!s->remap[p])
            return AVERROR(ENOMEM);
    }

    switch (s->in) {
    case EQUIRECTANGULAR:
        in_transform = xyz_to_equirect;
        break;
    case CUBEMAP_3_2:
        in_transform = xyz_to_cube3x2;
        break;
    case CUBEMAP_6_1:
        in_transform = xyz_to_cube6x1;
        break;
    case EQUIANGULAR:
        in_transform = xyz_to_eac;
        break;
    }

    switch (s->out) {
    case EQUIRECTANGULAR:
        out_transform = equirect_to_xyz;
        break;
    case CUBEMAP_3_2:
        out_transform = cube3x2_to_xyz;
        break;
    case CUBEMAP_6_1:
        out_transform = cube6x1_to_xyz;
        break;
    case EQUIANGULAR:
        out_transform = eac_to_xyz;
        break;
    }

    if (strcmp(s->in_forder, "default") == 0) {
        switch (s->in) {
            case CUBEMAP_3_2:
            case CUBEMAP_6_1:
                in_cubemap_face_order[RIGHT] = TOP_LEFT;
                in_cubemap_face_order[LEFT] = TOP_MIDDLE;
                in_cubemap_face_order[UP] = TOP_RIGHT;
                in_cubemap_face_order[DOWN] = BOTTOM_LEFT;
                in_cubemap_face_order[FRONT] = BOTTOM_MIDDLE;
                in_cubemap_face_order[BACK] = BOTTOM_RIGHT;
                break;
            case EQUIANGULAR:
                in_cubemap_face_order[RIGHT] = TOP_RIGHT;
                in_cubemap_face_order[LEFT] = TOP_LEFT;
                in_cubemap_face_order[UP] = BOTTOM_RIGHT;
                in_cubemap_face_order[DOWN] = BOTTOM_LEFT;
                in_cubemap_face_order[FRONT] = TOP_MIDDLE;
                in_cubemap_face_order[BACK] = BOTTOM_MIDDLE;
                break;
            default:
                break;
        }
    } else {
        for (int face = 0; face < NB_FACES; face++) {
            const char c = s->in_forder[face];
            int direction;
            if (c == '\0') {
                av_assert0(0);
            }

            direction = get_direction(c);
            if (direction == -1) {
                av_assert0(0);
            }

            in_cubemap_face_order[direction] = face;
        }
    }

    if (strcmp(s->out_forder, "default") == 0) {
        switch (s->out) {
            case CUBEMAP_3_2:
            case CUBEMAP_6_1:
                out_cubemap_direction_order[TOP_LEFT] = RIGHT;
                out_cubemap_direction_order[TOP_MIDDLE] = LEFT;
                out_cubemap_direction_order[TOP_RIGHT] = UP;
                out_cubemap_direction_order[BOTTOM_LEFT] = DOWN;
                out_cubemap_direction_order[BOTTOM_MIDDLE] = FRONT;
                out_cubemap_direction_order[BOTTOM_RIGHT] = BACK;
                break;
            case EQUIANGULAR:
                out_cubemap_direction_order[TOP_LEFT] = LEFT;
                out_cubemap_direction_order[TOP_MIDDLE] = FRONT;
                out_cubemap_direction_order[TOP_RIGHT] = RIGHT;
                out_cubemap_direction_order[BOTTOM_LEFT] = DOWN;
                out_cubemap_direction_order[BOTTOM_MIDDLE] = BACK;
                out_cubemap_direction_order[BOTTOM_RIGHT] = UP;
                break;
            default:
                break;
        }
    } else {
        for (int face = 0; face < NB_FACES; face++) {
            const char c = s->out_forder[face];
            int direction;

            if (c == '\0') {
                av_assert0(0);
            }

            direction = get_direction(c);
            if (direction == -1) {
                av_assert0(0);
            }

            out_cubemap_direction_order[face] = direction;
        }
    }

    if (strcmp(s->in_frot, "default") == 0) {
        switch (s->in) {
            case CUBEMAP_3_2:
            case CUBEMAP_6_1:
                in_cubemap_face_rotation[TOP_LEFT] = ROT_0;
                in_cubemap_face_rotation[TOP_MIDDLE] = ROT_0;
                in_cubemap_face_rotation[TOP_RIGHT] = ROT_0;
                in_cubemap_face_rotation[BOTTOM_LEFT] = ROT_0;
                in_cubemap_face_rotation[BOTTOM_MIDDLE] = ROT_0;
                in_cubemap_face_rotation[BOTTOM_RIGHT] = ROT_0;
                break;
            case EQUIANGULAR:
                in_cubemap_face_rotation[TOP_LEFT] = ROT_0;
                in_cubemap_face_rotation[TOP_MIDDLE] = ROT_0;
                in_cubemap_face_rotation[TOP_RIGHT] = ROT_0;
                in_cubemap_face_rotation[BOTTOM_LEFT] = ROT_270;
                in_cubemap_face_rotation[BOTTOM_MIDDLE] = ROT_90;
                in_cubemap_face_rotation[BOTTOM_RIGHT] = ROT_270;
                break;
            default:
                break;
        }
    } else {
        for (int face = 0; face < NB_FACES; face++) {
            const char c = s->in_frot[face];
            int rotation;

            if (c == '\0') {
                av_assert0(0);
            }

            rotation = get_rotation(c);
            if (rotation == -1) {
                av_assert0(0);
            }

            in_cubemap_face_rotation[face] = rotation;
        }
    }

    if (strcmp(s->out_frot, "default") == 0) {
        switch (s->out) {
            case CUBEMAP_3_2:
            case CUBEMAP_6_1:
                out_cubemap_face_rotation[TOP_LEFT] = ROT_0;
                out_cubemap_face_rotation[TOP_MIDDLE] = ROT_0;
                out_cubemap_face_rotation[TOP_RIGHT] = ROT_0;
                out_cubemap_face_rotation[BOTTOM_LEFT] = ROT_0;
                out_cubemap_face_rotation[BOTTOM_MIDDLE] = ROT_0;
                out_cubemap_face_rotation[BOTTOM_RIGHT] = ROT_0;
                break;
            case EQUIANGULAR:
                out_cubemap_face_rotation[TOP_LEFT] = ROT_0;
                out_cubemap_face_rotation[TOP_MIDDLE] = ROT_0;
                out_cubemap_face_rotation[TOP_RIGHT] = ROT_0;
                out_cubemap_face_rotation[BOTTOM_LEFT] = ROT_270;
                out_cubemap_face_rotation[BOTTOM_MIDDLE] = ROT_90;
                out_cubemap_face_rotation[BOTTOM_RIGHT] = ROT_270;
                break;
            default:
                break;
        }
    } else {
        for (int face = 0; face < NB_FACES; face++) {
            const char c = s->out_frot[face];
            int rotation;

            if (c == '\0') {
                av_assert0(0);
            }

            rotation = get_rotation(c);
            if (rotation == -1) {
                av_assert0(0);
            }

            out_cubemap_face_rotation[face] = rotation;
        }
    }


    for (p = 0; p < s->nb_planes; p++) {
        float mu, nu, x, y, z;
        int width = s->planewidth[p];
        int height = s->planeheight[p];
        int in_width = s->inplanewidth[p];
        int in_height = s->inplaneheight[p];
        int i, j;

        for (i = 0; i < width; i++) {
            for (j = 0; j < height; j++) {
                struct XYRemap *r = &s->remap[p][j * width + i];

                out_transform(i, j, width, height, &x, &y, &z);
                in_transform(x, y, z, in_width, in_height, r->u, r->v, &mu, &nu);
                calculate_kernel(mu, nu, r->ker);
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
    int plane;

    if (s->in == s->out)
        return ff_filter_frame(outlink, in);

    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);

    for (plane = 0; plane < s->nb_planes; plane++) {
        s->panorama(s, in->data[plane], out->data[plane],
                    s->planewidth[plane], s->planeheight[plane],
                    in->linesize[plane], out->linesize[plane],
                    s->remap[plane]);
    }

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
    .flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC,
};
