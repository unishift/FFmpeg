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
    {    "input", "set input projection",         OFFSET(in), AV_OPT_TYPE_INT,   {.i64=EQUIRECTANGULAR}, 0,    NB_PROJECTIONS-1, FLAGS, "in" },
    {        "e", "equirectangular",                       0, AV_OPT_TYPE_CONST, {.i64=EQUIRECTANGULAR}, 0,                   0, FLAGS, "in" },
    {     "c3x2", "cubemap3x2",                            0, AV_OPT_TYPE_CONST, {.i64=CUBEMAP_3_2},     0,                   0, FLAGS, "in" },
    {     "c6x1", "cubemap6x1",                            0, AV_OPT_TYPE_CONST, {.i64=CUBEMAP_6_1},     0,                   0, FLAGS, "in" },
    {      "eac", "equi-angular",                          0, AV_OPT_TYPE_CONST, {.i64=EQUIANGULAR},     0,                   0, FLAGS, "in" },
    {   "output", "set output projection",       OFFSET(out), AV_OPT_TYPE_INT,   {.i64=CUBEMAP_3_2},     0,    NB_PROJECTIONS-1, FLAGS, "out" },
    {        "e", "equirectangular",                       0, AV_OPT_TYPE_CONST, {.i64=EQUIRECTANGULAR}, 0,                   0, FLAGS, "out" },
    {     "c3x2", "cubemap3x2",                            0, AV_OPT_TYPE_CONST, {.i64=CUBEMAP_3_2},     0,                   0, FLAGS, "out" },
    {     "c6x1", "cubemap6x1",                            0, AV_OPT_TYPE_CONST, {.i64=CUBEMAP_6_1},     0,                   0, FLAGS, "out" },
    {      "eac", "equi-angular",                          0, AV_OPT_TYPE_CONST, {.i64=EQUIANGULAR},     0,                   0, FLAGS, "out" },
    {   "interp", "set interpolation method", OFFSET(interp), AV_OPT_TYPE_INT,   {.i64=BILINEAR},        0, NB_INTERP_METHODS-1, FLAGS, "interp" },
    {     "near", "nearest neighbour",                     0, AV_OPT_TYPE_CONST, {.i64=NEAREST},         0,                   0, FLAGS, "interp" },
    {     "line", "bilinear",                              0, AV_OPT_TYPE_CONST, {.i64=BILINEAR},        0,                   0, FLAGS, "interp" },
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

static void cube3x2_to_xyz(int i, int j, int width, int height,
                           double *x, double *y, double *z)
{
    int ew = width / 3;
    int eh = height / 2;
    int face = (i / ew) + 3 * (j / eh);
    double a = 2. * i / ew;
    double b = 2. * j / eh;
    double norm;
    double l_x, l_y, l_z;

    switch (face) {
    case TOP_LEFT:
        l_x =  1.    ;
        l_y =  1. - b;
        l_z = -1. + a;
        break;
    case TOP_MIDDLE:
        l_x = -1.    ;
        l_y =  1. - b;
        l_z = -a  + 3;
        break;
    case TOP_RIGHT:
        l_x =  a  - 5;
        l_y =  1     ;
        l_z = -b  + 1;
        break;
    case BOTTOM_LEFT:
        l_x =  a  - 1;
        l_y = -1     ;
        l_z =  b  - 3;
        break;
    case BOTTOM_MIDDLE:
        l_x =  a  - 3;
        l_y =  3. - b;
        l_z = -1.    ;
        break;
    case BOTTOM_RIGHT:
        l_x =  5. - a;
        l_y =  3. - b;
        l_z =  1.    ;
        break;
    }

    norm = sqrt(l_x * l_x + l_y * l_y + l_z * l_z);
    *x = l_x / norm;
    *y = l_y / norm;
    *z = l_z / norm;
}

static void xyz_to_cube3x2(double x, double y, double z, int width, int height,
                           int *i, int *j, int *i2, int *j2, double *mu, double *nu)
{
    double phi_norm, theta_threshold;
    double res = M_PI_4 / (width / 3) / 10.0;
    double uf, vf;
    double rh = height / 4.0;
    double rw = width / 6.0;
    int ui, vi, u2, v2;
    int face;
    double phi   = atan2(x, -z);
    double theta = asin(-y);

    if (in_range(phi, -M_PI_4, M_PI_4, res)) {
        face = BOTTOM_MIDDLE;
        phi_norm = phi;
    } else if (in_range(phi, -(M_PI_2 + M_PI_4), -M_PI_4, res)) {
        face = TOP_MIDDLE;
        phi_norm = phi + M_PI_2;
    } else if (in_range(phi, M_PI_4, M_PI_2 + M_PI_4, res)) {
        face = TOP_LEFT;
        phi_norm = phi - M_PI_2;
    } else {
        face = BOTTOM_RIGHT;
        phi_norm = phi + ((phi > 0) ? -M_PI : M_PI);
    }

    theta_threshold = atan2(1., 1. / cos(phi_norm));
    if (theta > theta_threshold) {
        face = BOTTOM_LEFT;
    } else if (theta < -theta_threshold) {
        face = TOP_RIGHT;
    }

    switch (face) {
    case TOP_LEFT:
        uf = rw * ( z / x + 1.0);
        vf = rh * (-y / x + 1.0);
        break;
    case TOP_MIDDLE:
        uf = rw * ( z / x + 1.0);
        vf = rh * ( y / x + 1.0);
        break;
    case TOP_RIGHT:
        uf = rw * ( x / y + 1.0);
        vf = rh * (-z / y + 1.0);
        break;
    case BOTTOM_LEFT:
        uf = rw * (-x / y + 1.0);
        vf = rh * (-z / y + 1.0);
        break;
    case BOTTOM_MIDDLE:
        uf = rw * (-x / z + 1.0);
        vf = rh * ( y / z + 1.0);
        break;
    case BOTTOM_RIGHT:
        uf = rw * (-x / z + 1.0);
        vf = rh * (-y / z + 1.0);
        break;
    }

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
    double a = 2. * i / ew;
    double b = 2. * j / eh;
    double norm;
    double l_x, l_y, l_z;

    switch (face) {
    case TOP_LEFT:
        l_x =  1     ;
        l_y =  1. - b;
        l_z = -1. + a;
        break;
    case TOP_MIDDLE:
        l_x = -1.    ;
        l_y =  1. - b;
        l_z = -a  + 3;
        break;
    case TOP_RIGHT:
        l_x =  a  - 5;
        l_y =  1.    ;
        l_z = -b  + 1;
        break;
    case BOTTOM_LEFT:
        l_x =  a  - 7;
        l_y = -1.    ;
        l_z =  b  - 1;
        break;
    case BOTTOM_MIDDLE:
        l_x =  a  - 9;
        l_y =  1. - b;
        l_z = -1.    ;
        break;
    case BOTTOM_RIGHT:
        l_x = 11. - a;
        l_y =  1. - b;
        l_z =  1.    ;
        break;
    }

    norm = sqrt(l_x * l_x + l_y * l_y + l_z * l_z);
    *x = l_x / norm;
    *y = l_y / norm;
    *z = l_z / norm;
}

static void xyz_to_cube6x1(double x, double y, double z, int width, int height,
                           int *i, int *j, int *i2, int *j2, double *mu, double *nu)
{
    double phi_norm, theta_threshold;
    double res = M_PI_4 / (width / 6) / 10.0;
    double uf, vf;
    double rh = height / 2;
    double rw = width / 12;
    int ui, vi, u2, v2;
    int face;
    double phi   = atan2(x, -z);
    double theta = asin(-y);

    if (in_range(phi, -M_PI_4, M_PI_4, res)) {
        face = BOTTOM_MIDDLE;
        phi_norm = phi;
    } else if (in_range(phi, -(M_PI_2 + M_PI_4), -M_PI_4, res)) {
        face = TOP_MIDDLE;
        phi_norm = phi + M_PI_2;
    } else if (in_range(phi, M_PI_4, M_PI_2 + M_PI_4, res)) {
        face = TOP_LEFT;
        phi_norm = phi - M_PI_2;
    } else {
        face = BOTTOM_RIGHT;
        phi_norm = phi + ((phi > 0) ? -M_PI : M_PI);
    }

    theta_threshold = atan2(1., 1. / cos(phi_norm));
    if (theta > theta_threshold) {
        face = BOTTOM_LEFT;
    } else if (theta < -theta_threshold) {
        face = TOP_RIGHT;
    }


    switch (face) {
    case TOP_LEFT:
        uf = rw * ( z / x + 1.0);
        vf = rh * (-y / x + 1.0);
        break;
    case TOP_MIDDLE:
        uf = rw * ( z / x + 1.0);
        vf = rh * ( y / x + 1.0);
        break;
    case TOP_RIGHT:
        uf = rw * ( x / y + 1.0);
        vf = rh * (-z / y + 1.0);
        break;
    case BOTTOM_LEFT:
        uf = rw * (-x / y + 1.0);
        vf = rh * (-z / y + 1.0);
        break;
    case BOTTOM_MIDDLE:
        uf = rw * (-x / z + 1.0);
        vf = rh * ( y / z + 1.0);
        break;
    case BOTTOM_RIGHT:
        uf = rw * (-x / z + 1.0);
        vf = rh * (-y / z + 1.0);
        break;
    }

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
    double a = 0.5 * tan(M_PI_2 * ((double)(i % ew) / ew - 0.5));
    double b = 0.5 * tan(M_PI_2 * ((double)(j % eh) / eh - 0.5));
    double norm;
    double l_x, l_y, l_z;

    switch (face) {
    case TOP_LEFT:      // left
        l_x = -0.5;
        l_y = -b;
        l_z = -a;
        break;
    case TOP_MIDDLE:    // forward
        l_x =  a;
        l_y = -b;
        l_z = -0.5;
        break;
    case TOP_RIGHT:     // right
        l_x =  0.5;
        l_y = -b;
        l_z =  a;
        break;
    case BOTTOM_LEFT:   // down
        l_x = -b;
        l_y = -0.5;
        l_z =  a;
        break;
    case BOTTOM_MIDDLE: // back
        l_x = -b;
        l_y =  a;
        l_z =  0.5    ;
        break;
    case BOTTOM_RIGHT:  // up
        l_x = -b;
        l_y =  0.5;
        l_z = -a;
        break;
    }

    norm = sqrt(l_x * l_x + l_y * l_y + l_z * l_z);
    *x = l_x / norm;
    *y = l_y / norm;
    *z = l_z / norm;
}

static void xyz_to_eac(double x, double y, double z, int width, int height,
                       int *i, int *j, int *i2, int *j2, double *mu, double *nu)
{
    double phi_norm, theta_threshold;
    double res = M_PI_4 / (width / 3) / 10.0;
    double uf, vf;
    double rh = height / 2.0;
    double rw = width / 3.0;
    int ui, vi, u2, v2;
    int face;
    double phi   = atan2(x, -z);
    double theta = asin(-y);

    if (in_range(phi, -M_PI_4, M_PI_4, res)) {                      // forward
        face = TOP_MIDDLE;
        phi_norm = phi;
    } else if (in_range(phi, -(M_PI_2 + M_PI_4), -M_PI_4, res)) {   // left
        face = TOP_LEFT;
        phi_norm = phi + M_PI_2;
    } else if (in_range(phi, M_PI_4, M_PI_2 + M_PI_4, res)) {       // right
        face = TOP_RIGHT;
        phi_norm = phi - M_PI_2;
    } else {                                                        // back
        face = BOTTOM_MIDDLE;
        phi_norm = phi + ((phi > 0) ? -M_PI : M_PI);
    }

    theta_threshold = atan2(1., 1. / cos(phi_norm));
    if (theta > theta_threshold) {                                  // down
        face = BOTTOM_LEFT;
    } else if (theta < -theta_threshold) {                          // up
        face = BOTTOM_RIGHT;
    }

    switch (face) {
    case TOP_LEFT:
        uf = rw * (M_2_PI * atan( z / x) + 0.5);
        vf = rh * (M_2_PI * atan( y / x) + 0.5);
        break;
    case TOP_MIDDLE:
        uf = rw * (M_2_PI * atan(-x / z) + 0.5);
        vf = rh * (M_2_PI * atan( y / z) + 0.5);
        break;
    case TOP_RIGHT:
        uf = rw * (M_2_PI * atan( z / x) + 0.5);
        vf = rh * (M_2_PI * atan(-y / x) + 0.5);
        break;
    case BOTTOM_LEFT:
        uf = rw * (M_2_PI * atan(-z / y) + 0.5);
        vf = rh * (M_2_PI * atan( x / y) + 0.5);
        break;
    case BOTTOM_MIDDLE:
        uf = rw * (M_2_PI * atan( y / z) + 0.5);
        vf = rh * (M_2_PI * atan(-x / z) + 0.5);
        break;
    case BOTTOM_RIGHT:
        uf = rw * (M_2_PI * atan(-z / y) + 0.5);
        vf = rh * (M_2_PI * atan(-x / y) + 0.5);
        break;
    }

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

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    PanoramaContext *s = ctx->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    int p, h, w;
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
        w = inlink->w;
        h = inlink->h;
        break;
    case CUBEMAP_3_2:
    case EQUIANGULAR:
        w = inlink->w / 3 * 4;
        h = inlink->h;
        break;
    case CUBEMAP_6_1:
        w = inlink->w / 3 * 2;
        h = inlink->h * 2;
        break;
    default:
        av_assert0(0);
    }

    switch (s->out) {
    case EQUIRECTANGULAR:
        break;
    case CUBEMAP_3_2:
    case EQUIANGULAR:
        w = w / 4 * 3;
        break;
    case CUBEMAP_6_1:
        w = w / 2 * 3;
        h = h / 2;
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
