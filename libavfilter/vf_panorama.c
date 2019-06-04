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
    int vi, ui;
    int v2, u2;
    double a, b, c, d;
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

static int bilinear(PanoramaContext *s,
                    const uint8_t *src, uint8_t *dst,
                    int width, int height,
                    int in_linesize, int out_linesize,
                    const struct XYRemap *remap)
{
    double A, B, C, D;
    int x, y;

    for (y = 0; y < height; y++) {
        uint8_t *d = dst + y * out_linesize;
        for (x = 0; x < width; x++) {
            const struct XYRemap *r = &remap[y * width + x];

            A = src[r->vi * in_linesize + r->ui];
            B = src[r->vi * in_linesize + r->u2];
            C = src[r->v2 * in_linesize + r->ui];
            D = src[r->v2 * in_linesize + r->u2];
            *d++ = round(A * r->a + B * r->b + C * r->c + D * r->d);
        }
    }

    return 0;
}

static int nearest(PanoramaContext *s,
                   const uint8_t *src, uint8_t *dst,
                   int width, int height,
                   int in_linesize, int out_linesize,
                   const struct XYRemap *remap)
{
    double A;
    int x, y;

    for (y = 0; y < height; y++) {
        uint8_t *d = dst + y * out_linesize;
        for (x = 0; x < width; x++) {
            const struct XYRemap *r = &remap[y * width + x];

            A = src[r->vi * in_linesize + r->ui];
            *d++ = A;
        }
    }

    return 0;
}

static inline int equal(double a, double b, double epsilon)
{
    return fabs(a - b) < epsilon;
}

static inline int smaller(double a, double b, double epsilon)
{
    return ((a - b) < 0.0) && (!equal(a, b, epsilon));
}

static inline int in_range(double rd, double small, double large, double res)
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

static void cube_to_xyz(double uf, double vf, int face,
                        double *x, double *y, double *z)
{
    int direction = out_cubemap_direction_order[face];
    double norm, tmp;
    double l_x, l_y, l_z;

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
        l_x =  1.;
        l_y = -vf;
        l_z =  uf;
        break;
    case LEFT:
        l_x = -1.;
        l_y = -vf;
        l_z = -uf;
        break;
    case UP:
        l_x =  uf;
        l_y =  1.;
        l_z = -vf;
        break;
    case DOWN:
        l_x =  uf;
        l_y = -1.;
        l_z =  vf;
        break;
    case FRONT:
        l_x =  uf;
        l_y = -vf;
        l_z = -1.;
        break;
    case BACK:
        l_x = -uf;
        l_y = -vf;
        l_z =  1.;
        break;
    }

    norm = sqrt(l_x * l_x + l_y * l_y + l_z * l_z);
    *x = l_x / norm;
    *y = l_y / norm;
    *z = l_z / norm;
}

static void xyz_to_cube(double x, double y, double z, double res,
                       double *uf, double *vf, int *face)
{
    double phi   = atan2(x, -z);
    double theta = asin(-y);
    double phi_norm, theta_threshold;
    double tmp;
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
        phi_norm = phi + ((phi > 0) ? -M_PI : M_PI);
    }

    theta_threshold = atan2(1., 1. / cos(phi_norm));
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
                           double *x, double *y, double *z)
{
    int ew = width / 3;
    int eh = height / 2;
    int face = (i / ew) + 3 * (j / eh);
    double uf = 2. * (i % ew) / ew - 1.;
    double vf = 2. * (j % eh) / eh - 1.;

    cube_to_xyz(uf, vf, face, x, y, z);
}

static void xyz_to_cube3x2(double x, double y, double z, int width, int height,
                           int *i, int *j, int *i2, int *j2, double *mu, double *nu)
{
    double res = M_PI_4 / (width / 3) / 10.0;
    double uf, vf;
    double rh = height / 4.0;
    double rw = width / 6.0;
    int ui, vi, u2, v2;
    int face;

    xyz_to_cube(x, y, z, res, &uf, &vf, &face);
    uf = rw * (uf + 1.0);
    vf = rh * (vf + 1.0);

    ui = floor(uf);
    vi = floor(vf);
    u2 = ui + 1;
    v2 = vi + 1;
    *mu = uf - ui;
    *nu = vf - vi;
    vi = av_clip(vi, 0, 2 * rh - 1);
    ui = av_clip(ui, 0, 2 * rw - 1);
    v2 = av_clip(v2, 0, 2 * rh - 1);
    u2 = av_clip(u2, 0, 2 * rw - 1);

    *i  = ui + (width / 3.) * (face % 3);
    *i2 = u2 + (width / 3.) * (face % 3);
    *j  = vi + (height / 2.) * (face / 3);
    *j2 = v2 + (height / 2.) * (face / 3);
}

static void cube6x1_to_xyz(int i, int j, int width, int height,
                           double *x, double *y, double *z)
{
    int ew = width / 6;
    int eh = height;
    int face = i / ew;
    double uf = 2. * (i % ew) / ew - 1.;
    double vf = 2. * (j % eh) / eh - 1.;

    cube_to_xyz(uf, vf, face, x, y, z);
}

static void xyz_to_cube6x1(double x, double y, double z, int width, int height,
                           int *i, int *j, int *i2, int *j2, double *mu, double *nu)
{
    double res = M_PI_4 / (width / 6) / 10.0;
    double uf, vf;
    double rh = height / 2;
    double rw = width / 12;
    int ui, vi, u2, v2;
    int face;

    xyz_to_cube(x, y, z, res, &uf, &vf, &face);
    uf = rw * (uf + 1.0);
    vf = rh * (vf + 1.0);

    ui = floor(uf);
    vi = floor(vf);
    u2 = ui + 1;
    v2 = vi + 1;
    *mu = uf - ui;
    *nu = vf - vi;
    vi = av_clip(vi, 0, 2 * rh - 1);
    ui = av_clip(ui, 0, 2 * rw - 1);
    v2 = av_clip(v2, 0, 2 * rh - 1);
    u2 = av_clip(u2, 0, 2 * rw - 1);

    *i  = ui + (width / 6.) * face;
    *i2 = u2 + (width / 6.) * face;
    *j  = vi;
    *j2 = v2;
}

static void equirect_to_xyz(int i, int j, int width, int height,
                            double *x, double *y, double *z)
{
    double phi   = ((2. * i) / width  - 1.) * M_PI;
    double theta = ((2. * j) / height - 1.) * M_PI_2;

    const double sin_phi = sin(phi);
    const double cos_phi = cos(phi);
    const double sin_theta = sin(theta);
    const double cos_theta = cos(theta);

    *x =  cos_theta * sin_phi;
    *y = -sin_theta;
    *z = -cos_theta * cos_phi;
}

static void xyz_to_equirect(double x, double y, double z, int width, int height,
                            int *i, int *j, int *i2, int *j2, double *mu, double *nu)
{
    double uf, vf;
    int ui, vi, u2, v2;
    double phi   = atan2(x, -z);
    double theta = asin(-y);

    uf = (phi / M_PI + 1.) * width / 2.;
    vf = (theta / M_PI_2 + 1.) * height / 2.;
    ui = floor(uf);
    vi = floor(vf);
    u2 = ui + 1;
    v2 = vi + 1;

    *mu = uf - ui;
    *nu = vf - vi;
    *j  = av_clip(vi, 0, height - 1);
    *i  = ui % width;
    *j2 = av_clip(v2, 0, height - 1);
    *i2 = u2 % width;
}

static void eac_to_xyz(int i, int j, int width, int height,
                       double *x, double *y, double *z)
{
    int ew = width / 3;
    int eh = height / 2;
    int face = (i / ew) + 3 * (j / eh);
    double uf = tan(M_PI_2 * ((double)(i % ew) / ew - 0.5));
    double vf = tan(M_PI_2 * ((double)(j % eh) / eh - 0.5));

    cube_to_xyz(uf, vf, face, x, y, z);
}

static void xyz_to_eac(double x, double y, double z, int width, int height,
                       int *i, int *j, int *i2, int *j2, double *mu, double *nu)
{
    double res = M_PI_4 / (width / 3) / 10.0;
    double uf, vf;
    double rh = height / 2.0;
    double rw = width / 3.0;
    int ui, vi, u2, v2;
    int face;

    xyz_to_cube(x, y, z, res, &uf, &vf, &face);
    uf = rw * (M_2_PI * atan(uf) + 0.5);
    vf = rh * (M_2_PI * atan(vf) + 0.5);

    ui = floor(uf);
    vi = floor(vf);
    u2 = ui + 1;
    v2 = vi + 1;
    *mu = uf - ui;
    *nu = vf - vi;
    vi = av_clip(vi, 0, rh - 1);
    ui = av_clip(ui, 0, rw - 1);
    v2 = av_clip(v2, 0, rh - 1);
    u2 = av_clip(u2, 0, rw - 1);

    *i  = ui + (width / 3.) * (face % 3);
    *i2 = u2 + (width / 3.) * (face % 3);
    *j  = vi + (height / 2.) * (face / 3);
    *j2 = v2 + (height / 2.) * (face / 3);
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
    double hf, wf;
    void (*in_transform)(double x, double y, double z, int width, int height,
                         int *i, int *j, int *i2, int *j2, double *mu, double *nu);
    void (*out_transform)(int i, int j, int width, int height,
                          double *x, double *y, double *z);

    switch (s->interp) {
    case NEAREST:
        s->panorama = nearest;
        break;
    case BILINEAR:
        s->panorama = bilinear;
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
        wf = inlink->w / 3. * 4.;
        hf = inlink->h;
        break;
    case CUBEMAP_6_1:
        wf = inlink->w / 3. * 2.;
        hf = inlink->h * 2.;
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
        w = wf / 4. * 3.;
        h = hf;
        break;
    case CUBEMAP_6_1:
        w = wf / 2. * 3.;
        h = hf / 2.;
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
        int ui, vi, u2, v2;
        double mu, nu, x, y, z;
        int width = s->planewidth[p];
        int height = s->planeheight[p];
        int in_width = s->inplanewidth[p];
        int in_height = s->inplaneheight[p];
        int i, j;

        for (i = 0; i < width; i++) {
            for (j = 0; j < height; j++) {
                struct XYRemap *r = &s->remap[p][j * width + i];

                out_transform(i, j, width, height, &x, &y, &z);
                in_transform(x, y, z, in_width, in_height, &ui, &vi, &u2, &v2, &mu, &nu);

                r->vi = vi;
                r->ui = ui;
                r->v2 = v2;
                r->u2 = u2;
                r->a = (1 - mu) * (1 - nu);
                r->b =  mu * (1 - nu);
                r->c = (1 - mu) * nu;
                r->d = mu * nu;
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
