//
//  mini3d-opengl.c
//
//  Created by yucong on 2018/6/26.
//  Copyright © 2018年 yucong. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <GL/glew.h>
#include <glfw3.h>
#include <FreeImage.h>
#include <assert.h>
#include "tinyobj_loader_c.h"


#define EPSILON_E3 (float)(1E-3)
#define EPSILON_E4 (float)(1E-4)
#define EPSILON_E5 (float)(1E-5)
#define EPSILON_E6 (float)(1E-6)

typedef struct { float m[4][4]; } Matrix_t;
typedef struct { float x, y, z, w; } Vector_t;
typedef Vector_t Point_t;

typedef struct {
    Matrix_t world;     // 世界坐标变换
    Matrix_t view;      // 摄像机坐标变换
    Matrix_t projection;    // 投影坐标变换
    Matrix_t transform;     // transform = world * view * projection
    float sw, sh;   // 屏幕大小
} Transform_t;

typedef struct { float r, g, b, a; } Color_t;
typedef struct { float u, v; } Texcoord_t;
typedef struct {
    Point_t pos;
    Point_t wpos;
    Texcoord_t tex;
    Color_t color;
    float rhw;
    Vector_t normal;
} Vertex_t;

typedef struct {
    Point_t pos;
    Texcoord_t tex;
    Color_t color;
    Vector_t normal;
} VShader_t;

typedef struct {
    Point_t pos;
    Texcoord_t tex;
    Color_t color;
    Vector_t normal;
} FShader_t;

typedef struct {
    int attr;
    Vertex_t vlist[3];
    Vertex_t tvlist[3];
} Poly_t;

typedef struct { Vertex_t v, v1, v2; } Edge_t;
typedef struct {
    float top, bottom;
    Edge_t left, right;
    
} Trapezoid_t;    // 梯形
typedef struct {
    Vertex_t v;     // 当前顶点
    Vertex_t step;  // 像素步进为 1 时，color、tex的步进
    int x, y;
    int width;
} Scanline_t;

typedef struct {
    u_int32_t* buffer;
    int tex_width;
    int tex_height;
    int tex_pitch;
    int alpha;
    int type;
    int bpp;
} Texture_t;

typedef struct {
    float aspect;
    float fovy;
    float zn, zf;
    float viewplane_width;
    float viewplane_height;
    Point_t eye;        // 摄像机位置
    Point_t up;
    Vector_t front;      // 面向
    Matrix_t world;     // 世界坐标变换
    Matrix_t view;      // 摄像机坐标变换
    Matrix_t projection;    // 投影坐标变换
    Matrix_t transform;     // transform = world * view * projection
    Matrix_t viewprojection;    // view * projection
    Matrix_t inversetranspose;  // 逆转置矩阵
    int update;
} Camera_t;

#define LIGHT_ATTR_AMBIENT      0X0001  // 环境光源
#define LIGHT_ATTR_INFINITE     0X0002  // 无穷远光源
#define LIGHT_ATTR_POINT        0X0004  // 点光源
#define LIGHT_ATTR_SPOTLIGHT    0X0008  // 聚光灯

typedef struct {
    int attr;
    Vector_t dir;
    Point_t pos;
    Color_t c;
    float kc, kl, kq;   // 衰减因子
    float pf;           // 聚光灯指数因子
} Light_t;

typedef struct {
    Poly_t* polys;
    int num_polys;
    int num_vertex;
    Vector_t pos;
    Vector_t scale;
    Vector_t rotate;
    float theta;
    Texture_t texture;
    int render_state;
} Object_t;


typedef struct {
    //    Transform_t transform;
    Camera_t* cams[2];
    Camera_t* cam;
    int num_cam;
    int cur_cam;
    int width, height;
    uint8_t* framebuffer;
    float* zbuffer;
    Texture_t* texture;
    u_int32_t background;
    int render_attr;
    Light_t light[5];
    int num_light;
    
} Device_t;

#define RENDER_STATE_WIREFRAME      1        // 渲染线框
#define RENDER_STATE_TEXTURE        2        // 渲染纹理
#define RENDER_STATE_COLOR          4        // 渲染颜色

#define RENDER_ATTR_BILERP          1       // 双线性滤波

#define POLY_ATTR_2SIDED            8

#define SWAP(x, y, type)    \
{    type temp = (x);        \
    x = y; y = temp;        \
}

#define COLOR24_R(color) ((color >> 16) & 0xff)
#define COLOR24_G(color) ((color >> 8) & 0xff)
#define COLOR24_B(color) (color & 0xff)
#define COLOR24(r, g, b)  ((r << 16) | (g << 8) | b)

#define COLOR32_R(color) ((color >> 24) & 0xff)
#define COLOR32_G(color) ((color >> 16) & 0xff)
#define COLOR32_B(color) ((color >> 8) & 0xff)
#define COLOR32_A(color) (color & 0xff)
#define COLOR32(r, g, b, a)  ((r << 24) | (g << 16) | (b << 8) | a)

#define COLOR_R(color, f) (((f) == 1) ? COLOR32_R(color) : COLOR24_R(color))
#define COLOR_G(color, f) (((f) == 1) ? COLOR32_G(color) : COLOR24_G(color))
#define COLOR_B(color, f) (((f) == 1) ? COLOR32_B(color) : COLOR24_B(color))
#define COLOR_A(color, f) (((f) == 1) ? COLOR32_A(color) : 1)
#define COLOR(r, g, b, a, f) (((f) == 1) ? COLOR32(r, g, b, a) : COLOR24(r, g, b))

#define CMID(x, min, max) ((x < min) ? min : ((x > max) ? max : x))

// 角度转弧度
#define ANG_TO_RAD(angle) ((angle) * M_PI / 180.0)
// 弧度转角度
#define RAD_TO_ANG(radian) ((radian) * 180.0 / M_PI)

float interp(float x1, float x2, float t) { return x1 + (x2 - x1) * t; }

void calc_normal(float N[3], float v0[3], float v1[3], float v2[3]) {
    float v10[3];
    float v20[3];
    float len2;
    
    v10[0] = v1[0] - v0[0];
    v10[1] = v1[1] - v0[1];
    v10[2] = v1[2] - v0[2];
    
    v20[0] = v2[0] - v0[0];
    v20[1] = v2[1] - v0[1];
    v20[2] = v2[2] - v0[2];
    
    N[0] = v20[1] * v10[2] - v20[2] * v10[1];
    N[1] = v20[2] * v10[0] - v20[0] * v10[2];
    N[2] = v20[0] * v10[1] - v20[1] * v10[0];
    
    len2 = N[0] * N[0] + N[1] * N[1] + N[2] * N[2];
    if (len2 > 0.0f) {
        float len = (float)sqrt((double)len2);
        
        N[0] /= len;
        N[1] /= len;
    }
}


#pragma mark -
#pragma mark matrix vector

float vector_length(const Vector_t* v) {
    return (float)sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

float vector_distance(const Vector_t* v1, const Vector_t* v2)
{
    return (float)sqrtf((v1->x - v2->x) * (v1->x - v2->x) + (v1->y - v2->y) * (v1->y - v2->y) + (v1->z - v2->z) * (v1->z - v2->z));
}

void vector_init(Vector_t* v) {
    v->x = v->y = v->z = 0.f;
    v->w = 1.f;
}

void vector_copy(Vector_t* x, const Vector_t* y) {
    x->x = y->x;
    x->y = y->y;
    x->z = y->z;
    x->w = y->w;
}

void vector_set(Vector_t* v, float x, float y, float z) {
    v->x = x;
    v->y = y;
    v->z = z;
    v->w = 1.f;
}

void vector_add(Vector_t* z, const Vector_t* x, const Vector_t* y) {
    z->x = x->x + y->x;
    z->y = x->y + y->y;
    z->z = x->z + y->z;
    z->w = 1.0;
}

void vector_sub(Vector_t *z, const Vector_t *x, const Vector_t *y) {
    z->x = x->x - y->x;
    z->y = x->y - y->y;
    z->z = x->z - y->z;
    z->w = 1.0;
}

// 矢量点乘
float vector_dotproduct(const Vector_t *x, const Vector_t *y) {
    return x->x * y->x + x->y * y->y + x->z * y->z;
}

// 矢量叉乘
void vector_crossproduct(Vector_t *z, const Vector_t *x, const Vector_t *y) {
    float m1, m2, m3;
    m1 = x->y * y->z - x->z * y->y;
    m2 = x->z * y->x - x->x * y->z;
    m3 = x->x * y->y - x->y * y->x;
    z->x = m1;
    z->y = m2;
    z->z = m3;
    z->w = 1.0f;
}

// 矢量插值，t取值 [0, 1]
void vector_interp(Vector_t *z, const Vector_t *x1, const Vector_t *x2, float t) {
    z->x = interp(x1->x, x2->x, t);
    z->y = interp(x1->y, x2->y, t);
    z->z = interp(x1->z, x2->z, t);
    z->w = 1.0f;
}

void vector_inverse(Vector_t *v) {
    v->x = -v->x;
    v->y = -v->y;
    v->z = -v->z;
}

// 矢量归一化
void vector_normalize(Vector_t *v) {
    float length = vector_length(v);
    if (length != 0.0f) {
        float inv = 1.0f / length;
        v->x *= inv;
        v->y *= inv;
        v->z *= inv;
    }
}

// 两点间的角度，X 正轴角度是 0
float vector_angle(const Point_t* p1, const Point_t* p2) {
    float angle;
    float x = p2->x - p1->x;
    float y = p2->y - p1->y;
    if (x == 0.0)
        angle = M_PI / 2.0;
    else
        angle = atan(fabs(y / x));
    
    if ((x < 0.0) && (y >= 0.0))
        angle = M_PI - angle;
    else if ((x < 0.0) && (y < 0.0))
        angle = M_PI + angle;
    else if ((x >= 0.0) && (y < 0.0))
        angle = M_PI * 2.0 - angle;
    
    return angle;
}

void vector_scale(Vector_t* v, float scale) {
    v->x *= scale;
    v->y *= scale;
    v->z *= scale;
}

// c = a + b
void matrix_add(Matrix_t *c, const Matrix_t *a, const Matrix_t *b) {
    int i, j;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++)
            c->m[i][j] = a->m[i][j] + b->m[i][j];
    }
}

// c = a - b
void matrix_sub(Matrix_t *c, const Matrix_t *a, const Matrix_t *b) {
    int i, j;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++)
            c->m[i][j] = a->m[i][j] - b->m[i][j];
    }
}

// c = a * b
void matrix_mul(Matrix_t *c, const Matrix_t *a, const Matrix_t *b) {
    Matrix_t z;
    int i, j;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            z.m[j][i] = (a->m[j][0] * b->m[0][i]) +
            (a->m[j][1] * b->m[1][i]) +
            (a->m[j][2] * b->m[2][i]) +
            (a->m[j][3] * b->m[3][i]);
        }
    }
    c[0] = z;
}

// c = a * f
void matrix_scale(Matrix_t *c, const Matrix_t *a, float f) {
    int i, j;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++)
            c->m[i][j] = a->m[i][j] * f;
    }
}

// y = x * m
void matrix_apply(Vector_t *y, const Vector_t *x, const Matrix_t *m) {
    float X = x->x, Y = x->y, Z = x->z, W = x->w;
    y->x = X * m->m[0][0] + Y * m->m[1][0] + Z * m->m[2][0] + W * m->m[3][0];
    y->y = X * m->m[0][1] + Y * m->m[1][1] + Z * m->m[2][1] + W * m->m[3][1];
    y->z = X * m->m[0][2] + Y * m->m[1][2] + Z * m->m[2][2] + W * m->m[3][2];
    y->w = X * m->m[0][3] + Y * m->m[1][3] + Z * m->m[2][3] + W * m->m[3][3];
}

void matrix_clone(Matrix_t *dest, const Matrix_t *src) {
    int i, j;
    for(i = 0; i < 4; i++)
        for(j = 0; j < 4; j++)
            dest->m[i][j] = src->m[i][j];
}

float matrix_det(Matrix_t* m) {
    return m->m[0][0] * m->m[1][1] * m->m[2][2] - m->m[1][2] * m->m[2][1] -
        m->m[0][1] * m->m[1][0] * m->m[2][2] - m->m[1][2] * m->m[2][0] +
        m->m[0][2] * m->m[1][0] * m->m[2][1] - m->m[1][1] * m->m[2][0];
}

// 转置矩阵
void matrix_transpose(Matrix_t* m) {
    Matrix_t mt;
    mt.m[0][0] = m->m[0][0]; mt.m[0][1] = m->m[1][0]; mt.m[0][2] = m->m[2][0]; mt.m[0][3] = m->m[3][0];
    mt.m[1][0] = m->m[0][1]; mt.m[1][1] = m->m[1][1]; mt.m[1][2] = m->m[2][1]; mt.m[1][3] = m->m[3][1];
    mt.m[2][0] = m->m[0][2]; mt.m[2][1] = m->m[1][2]; mt.m[2][2] = m->m[2][2]; mt.m[2][3] = m->m[3][2];
    mt.m[3][0] = m->m[0][3]; mt.m[3][1] = m->m[1][3]; mt.m[3][2] = m->m[2][3]; mt.m[3][3] = m->m[3][3];
    memcpy(m, &mt, sizeof(Matrix_t));
}

// 逆矩阵
void matrix_inverse(Matrix_t* m) {
    float t[3][6];
    int i, j, k;
    float f;
    
    for(i = 0; i < 3; i++)
        for(j = 0; j < 6; j++) {
            if(j < 3)
                t[i][j] = m->m[i][j];
            else if(j == i + 3)
                t[i][j] = 1;
            else
                t[i][j] = 0;
        }
    
    for(i = 0; i < 3; i++) {
        f = t[i][i];
        for(j = 0; j < 6; j++)
            t[i][j] /= f;
        for(j = 0; j < 3; j++) {
            if(j != i) {
                f = t[j][i];
                for(k = 0; k < 6; k++)
                    t[j][k] = t[j][k] - t[i][k] * f;
            }
        }
    }
    
    for(i = 0; i < 3; i++)
        for(j = 3; j < 6; j++)
            m->m[i][j-3] = t[i][j];
    
    m->m[3][0] = -m->m[3][0];
    m->m[3][1] = -m->m[3][1];
    m->m[3][2] = -m->m[3][2];
    m->m[3][3] = 1.f;
}

// 初始化
void matrix_set_identity(Matrix_t *m) {
    m->m[0][0] = m->m[1][1] = m->m[2][2] = m->m[3][3] = 1.0f;
    m->m[0][1] = m->m[0][2] = m->m[0][3] = 0.0f;
    m->m[1][0] = m->m[1][2] = m->m[1][3] = 0.0f;
    m->m[2][0] = m->m[2][1] = m->m[2][3] = 0.0f;
    m->m[3][0] = m->m[3][1] = m->m[3][2] = 0.0f;
}

void matrix_set_zero(Matrix_t *m) {
    m->m[0][0] = m->m[0][1] = m->m[0][2] = m->m[0][3] = 0.0f;
    m->m[1][0] = m->m[1][1] = m->m[1][2] = m->m[1][3] = 0.0f;
    m->m[2][0] = m->m[2][1] = m->m[2][2] = m->m[2][3] = 0.0f;
    m->m[3][0] = m->m[3][1] = m->m[3][2] = m->m[3][3] = 0.0f;
}

// 平移变换
void matrix_set_translate(Matrix_t *m, float x, float y, float z) {
    m->m[3][0] = x;
    m->m[3][1] = y;
    m->m[3][2] = z;
}

// 缩放变换
void matrix_set_scale(Matrix_t *m, float x, float y, float z) {
    m->m[0][0] = x;
    m->m[1][1] = y;
    m->m[2][2] = z;
}

// 旋转矩阵
void matrix_set_rotate(Matrix_t *m, float x, float y, float z, float theta) {
    float qsin = (float)sin(theta * 0.5f);
    float qcos = (float)cos(theta * 0.5f);
    Vector_t vec = { x, y, z, 1.0f };
    float w = qcos;
    vector_normalize(&vec);
    x = vec.x * qsin;
    y = vec.y * qsin;
    z = vec.z * qsin;
    m->m[0][0] = 1 - 2 * y * y - 2 * z * z;
    m->m[1][0] = 2 * x * y - 2 * w * z;
    m->m[2][0] = 2 * x * z + 2 * w * y;
    m->m[0][1] = 2 * x * y + 2 * w * z;
    m->m[1][1] = 1 - 2 * x * x - 2 * z * z;
    m->m[2][1] = 2 * y * z - 2 * w * x;
    m->m[0][2] = 2 * x * z - 2 * w * y;
    m->m[1][2] = 2 * y * z + 2 * w * x;
    m->m[2][2] = 1 - 2 * x * x - 2 * y * y;
    m->m[0][3] = m->m[1][3] = m->m[2][3] = 0.0f;
    m->m[3][0] = m->m[3][1] = m->m[3][2] = 0.0f;
    m->m[3][3] = 1.0f;
}

void matrix_set_rotate_translate_scale(Matrix_t *m, const Vector_t *axis, float theta, const Point_t *pos, const Vector_t *scale) {
    matrix_set_scale(m, scale->x, scale->y, scale->z);
    Matrix_t r, t = *m;
    matrix_set_rotate(&r, axis->x, axis->y, axis->z, theta);
    matrix_mul(m, &t, &r);
    m->m[3][0] = pos->x;
    m->m[3][1] = pos->y;
    m->m[3][2] = pos->z;
}

void matrix_set_perspective(Matrix_t *m, float fov, float aspect, float zn, float zf) {
    float fax = 1.0f / (float)tan(fov * 0.5f);
    matrix_set_zero(m);
    m->m[0][0] = (float)(fax / aspect);
    m->m[1][1] = (float)(fax);
    m->m[2][2] = (zf + zn) / (zf - zn);
    m->m[3][2] = - (2 * zn * zf) / (zf - zn);
    m->m[2][3] = 1;
}

#pragma mark -
#pragma mark transform

int transform_check_cvv(const Vector_t* v) {
    float w = v->w;
    int check = 0;
    if (v->z < 0.0f) check |= 1;
    if (v->z >  w) check |= 2;
    if (v->x < -w) check |= 4;
    if (v->x >  w) check |= 8;
    if (v->y < -w) check |= 16;
    if (v->y >  w) check |= 32;
    return check;
}

// 转屏幕坐标[-1, 1]
void transform_screen(float width, float height, Vector_t* y, const Vector_t* x) {
    // 因为透视矩阵变换完后，w 就是原 z 的值
    float rhw = 1.f / x->w;
    y->x = (x->x * rhw + 1.0f) * width * 0.5f;
    // OpenGL 的原点在左下角
    y->y = (x->y * rhw + 1.0f) * height * 0.5f;
    y->z = x->z * rhw;
//    y->w = 1.0f;
}


#pragma mark -
#pragma mark Vertex

void vertex_init_rhw(Vertex_t* v) {
    float rhw = 1.f / v->pos.w;
    v->wpos.x *= rhw;
    v->wpos.y *= rhw;
    v->wpos.z *= rhw;
    v->wpos.w *= rhw;
    v->normal.x *= rhw;
    v->normal.y *= rhw;
    v->normal.z *= rhw;
    v->normal.w *= rhw;
    v->tex.u *= rhw;
    v->tex.v *= rhw;
    v->color.r *= rhw;
    v->color.g *= rhw;
    v->color.b *= rhw;
    v->color.a *= rhw;
    v->rhw = rhw;
}

void vertex_interp(Vertex_t* y, const Vertex_t* x1, const Vertex_t* x2, float t) {
    vector_interp(&y->pos, &x1->pos, &x2->pos, t);
    vector_interp(&y->wpos, &x1->wpos, &x2->wpos, t);
    vector_interp(&y->normal, &x1->normal, &x2->normal, t);
    y->tex.u = interp(x1->tex.u, x2->tex.u, t);
    y->tex.v = interp(x1->tex.v, x2->tex.v, t);
    y->color.r = interp(x1->color.r, x2->color.r, t);
    y->color.g = interp(x1->color.g, x2->color.g, t);
    y->color.b = interp(x1->color.b, x2->color.b, t);
    y->color.a = interp(x1->color.a, x2->color.a, t);
    y->rhw = interp(x1->rhw, x2->rhw, t);
}

void vertex_division(Vertex_t* y, const Vertex_t* x1, const Vertex_t* x2, float w) {
    if (w == 0.f) w = 1.f;
    float inv = 1.0f / w;
    y->pos.x = (x2->pos.x - x1->pos.x) * inv;
    y->pos.y = (x2->pos.y - x1->pos.y) * inv;
    y->pos.z = (x2->pos.z - x1->pos.z) * inv;
    y->wpos.x = (x2->wpos.x - x1->wpos.x) * inv;
    y->wpos.y = (x2->wpos.y - x1->wpos.y) * inv;
    y->wpos.z = (x2->wpos.z - x1->wpos.z) * inv;
    y->normal.x = (x2->normal.x - x1->normal.x) * inv;
    y->normal.y = (x2->normal.y - x1->normal.y) * inv;
    y->normal.z = (x2->normal.z - x1->normal.z) * inv;
    y->tex.u = (x2->tex.u - x1->tex.u) * inv;
    y->tex.v = (x2->tex.v - x1->tex.v) * inv;
    y->color.r = (x2->color.r - x1->color.r) * inv;
    y->color.g = (x2->color.g - x1->color.g) * inv;
    y->color.b = (x2->color.b - x1->color.b) * inv;
    y->rhw = (x2->rhw - x1->rhw) * inv;
}

void vertex_add(Vertex_t* y, const Vertex_t* x) {
    y->pos.x += x->pos.x;
    y->pos.y += x->pos.y;
    y->pos.z += x->pos.z;
//    y->pos.w += x->pos.w;
    y->wpos.x += x->wpos.x;
    y->wpos.y += x->wpos.y;
    y->wpos.z += x->wpos.z;
    y->normal.x += x->normal.x;
    y->normal.y += x->normal.y;
    y->normal.z += x->normal.z;
    y->tex.u += x->tex.u;
    y->tex.v += x->tex.v;
    y->color.r += x->color.r;
    y->color.g += x->color.g;
    y->color.b += x->color.b;
    y->color.a += x->color.a;
    y->rhw += x->rhw;
}

void vertex_copy(Vertex_t* y, const Vertex_t* x) {
    y->pos.x = x->pos.x;
    y->pos.y = x->pos.y;
    y->pos.z = x->pos.z;
    y->pos.w = x->pos.w;
    y->wpos.x = x->wpos.x;
    y->wpos.y = x->wpos.y;
    y->wpos.z = x->wpos.z;
    y->wpos.w = x->wpos.w;
    y->normal.x = x->normal.x;
    y->normal.y = x->normal.y;
    y->normal.z = x->normal.z;
    y->normal.w = x->normal.w;
    y->rhw = x->rhw;
    y->tex.u = x->tex.u;
    y->tex.v = x->tex.v;
    y->color = x->color;
}

#pragma mark -
#pragma mark Color
void color_copy(Color_t* y, const Color_t* x) {
    y->r = x->r;
    y->g = x->g;
    y->b = x->b;
    y->a = x->a;
}

void color_set(Color_t* y, float r, float g, float b, float a) {
    y->r = r;
    y->g = g;
    y->b = b;
    y->a = a;
}

void color_add(Color_t* y, const Color_t* x1, const Color_t* x2) {
    y->r = x1->r + x2->r;
    y->g = x1->g + x2->g;
    y->b = x1->b + x2->b;
    y->a = x1->a + x2->a;
}

void color_sub(Color_t *c, const Color_t *a, const Color_t *b) {
    c->r = a->r - b->r;
    c->g = a->g - b->g;
    c->b = a->b - b->b;
    c->a = a->a - b->a;
}

void color_mul(Color_t* y, const Color_t* x1, const Color_t* x2) {
    y->r = x1->r * x2->r;
    y->g = x1->g * x2->g;
    y->b = x1->b * x2->b;
    y->a = x1->a * x2->a;
}

void color_scale(Color_t* y, float scale) {
    y->r *= scale;
    y->g *= scale;
    y->b *= scale;
    y->a *= scale;
}

// 插值，t取值 [0, 1]
void color_interp(Color_t *z, const Color_t *x1, const Color_t *x2, float t) {
    z->r = interp(x1->r, x2->r, t);
    z->g = interp(x1->g, x2->g, t);
    z->b = interp(x1->b, x2->b, t);
    z->a = interp(x1->a, x2->a, t);
}

#pragma mark -
#pragma mark FShader

void fshader_init(FShader_t* fs, const Vertex_t* v) {
    vector_copy(&fs->pos, &v->wpos);
    fs->pos.w = 1.f / v->rhw;
    vector_copy(&fs->normal, &v->normal);
    color_copy(&fs->color, &v->color);
    fs->tex.u = v->tex.u;
    fs->tex.v = v->tex.v;
}

#pragma mark -
#pragma mark Trapezoid

// 根据三角形生成 0-2 个梯形，并且返回合法梯形的数量
int trapezoid_init_triangle(Trapezoid_t *trap, const Vertex_t *p1,
                            const Vertex_t *p2, const Vertex_t *p3) {
    const Vertex_t *p;
    float k, x;
    
    // 排列大小顺序 p1.y < p2.y < p3.y
    if (p1->pos.y > p2->pos.y) p = p1, p1 = p2, p2 = p;
    if (p1->pos.y > p3->pos.y) p = p1, p1 = p3, p3 = p;
    if (p2->pos.y > p3->pos.y) p = p2, p2 = p3, p3 = p;
    // 直线判断
    if (p1->pos.y == p2->pos.y && p1->pos.y == p3->pos.y) return 0;
    if (p1->pos.x == p2->pos.x && p1->pos.x == p3->pos.x) return 0;
    
    // 正三角
    if (p1->pos.y == p2->pos.y) {    // triangle down
        if (p1->pos.x > p2->pos.x) p = p1, p1 = p2, p2 = p;
        trap[0].top = p1->pos.y;
        trap[0].bottom = p3->pos.y;
        trap[0].left.v1 = *p1;
        trap[0].left.v2 = *p3;
        trap[0].right.v1 = *p2;
        trap[0].right.v2 = *p3;
        return (trap[0].top < trap[0].bottom)? 1 : 0;
    }
    
    // 倒三角
    if (p2->pos.y == p3->pos.y) {    // triangle up
        if (p2->pos.x > p3->pos.x) p = p2, p2 = p3, p3 = p;
        trap[0].top = p1->pos.y;
        trap[0].bottom = p3->pos.y;
        trap[0].left.v1 = *p1;
        trap[0].left.v2 = *p2;
        trap[0].right.v1 = *p1;
        trap[0].right.v2 = *p3;
        return (trap[0].top < trap[0].bottom)? 1 : 0;
    }
    
    trap[0].top = p1->pos.y;
    trap[0].bottom = p2->pos.y;
    trap[1].top = p2->pos.y;
    trap[1].bottom = p3->pos.y;
    
    k = (p3->pos.y - p1->pos.y) / (p2->pos.y - p1->pos.y);
    x = p1->pos.x + (p2->pos.x - p1->pos.x) * k;
    
    if (x <= p3->pos.x) {        // triangle left
        trap[0].left.v1 = *p1;
        trap[0].left.v2 = *p2;
        trap[0].right.v1 = *p1;
        trap[0].right.v2 = *p3;
        trap[1].left.v1 = *p2;
        trap[1].left.v2 = *p3;
        trap[1].right.v1 = *p1;
        trap[1].right.v2 = *p3;
    }    else {                    // triangle right
        trap[0].left.v1 = *p1;
        trap[0].left.v2 = *p3;
        trap[0].right.v1 = *p1;
        trap[0].right.v2 = *p2;
        trap[1].left.v1 = *p1;
        trap[1].left.v2 = *p3;
        trap[1].right.v1 = *p2;
        trap[1].right.v2 = *p3;
    }
    
    return 2;
}

// 按照 Y 坐标计算出左右两条边纵坐标等于 Y 的顶点
void trapezoid_edge_interp(Trapezoid_t *trap, float y) {
    float perc_left = (y - trap->left.v1.pos.y) / (trap->left.v2.pos.y - trap->left.v1.pos.y);
    float perc_right = (y - trap->right.v1.pos.y) / (trap->right.v2.pos.y - trap->right.v1.pos.y);
    perc_left = CMID(perc_left, 0.f, 1.f);
    perc_right = CMID(perc_right, 0.f, 1.f);
    vertex_interp(&trap->left.v, &trap->left.v1, &trap->left.v2, perc_left);
    vertex_interp(&trap->right.v, &trap->right.v1, &trap->right.v2, perc_right);
}

// 根据左右两边的端点，初始化计算出扫描线的起点和步长

void trapezoid_init_scan_line(const Trapezoid_t *trap, Scanline_t *scanline, int y) {
    float width = trap->right.v.pos.x - trap->left.v.pos.x;
    scanline->x = (int)(trap->left.v.pos.x + 0.5);
    scanline->y = y;
    scanline->width = (int)(trap->right.v.pos.x + 0.5) - scanline->x;
    scanline->v = trap->left.v;
    if (trap->left.v.pos.x >= trap->right.v.pos.x) scanline->width = 0;
    // 步长
    vertex_division(&scanline->step, &trap->left.v, &trap->right.v, width);
}

#pragma mark -
#pragma mark Texture

void texture_init(Texture_t* texture, void* buffer, int pitch, int width, int height) {
    texture->tex_width = width;
    texture->tex_height = height;
    texture->tex_pitch = pitch;
    texture->buffer = (u_int32_t*)buffer;
}

void texture_load(Texture_t* texture, const char* filename) {
    FIBITMAP* fib = NULL;
    FREE_IMAGE_FORMAT format = FreeImage_GetFIFFromFilename(filename);
    if(format != FIF_UNKNOWN && FreeImage_FIFSupportsReading(format))
    {
        fib = FreeImage_Load(format, filename, 0);
    }
    texture->tex_width = FreeImage_GetWidth(fib);
    texture->tex_height = FreeImage_GetHeight(fib);
    texture->type = FreeImage_GetColorType(fib);
    texture->bpp = FreeImage_GetBPP(fib) / sizeof(long);
    texture->alpha = FreeImage_IsTransparent(fib);
    texture->tex_pitch = FreeImage_GetPitch(fib);
    
    if (texture->type == FIC_RGB || texture->type == FIC_RGBALPHA) {
        size_t size = texture->tex_width * texture->tex_height * sizeof(u_int32_t);
        texture->buffer = (u_int32_t*)malloc(size);
        memset(texture->buffer, 0, size);
        BYTE* bits = FreeImage_GetBits(fib);
        for (int y = 0; y < texture->tex_height; ++y) {
            for (int x = 0; x < texture->tex_width; ++x) {
                int r = bits[y * texture->tex_pitch + texture->bpp * x + 2];
                int g = bits[y * texture->tex_pitch + texture->bpp * x + 1];
                int b = bits[y * texture->tex_pitch + texture->bpp * x + 0];
                int a = 255;
                if (texture->alpha) a = bits[y * texture->tex_pitch + texture->bpp * x];
                texture->buffer[y * texture->tex_width + x] = COLOR(r, g, b, a, 1);
            }
        }
    }
    texture->tex_pitch = texture->tex_width;
    FreeImage_Unload(fib);
}

void texture_destroy(Texture_t* texture) {
    if (texture->buffer) free(texture->buffer);
    texture->buffer = 0;
}

#pragma mark -
#pragma mark Camera

// 设置摄像机
void camera_set_lookat(Camera_t* cam, const Vector_t* eye, const Vector_t* front, const Vector_t* up) {
    Vector_t xaxis, yaxis, zaxis;
//    vector_sub(&zaxis, &cam->at, &cam->eye);
    zaxis = *front;
//    vector_normalize(&zaxis);
    vector_crossproduct(&xaxis, up, &zaxis);
    vector_normalize(&xaxis);
    vector_crossproduct(&yaxis, &zaxis, &xaxis);
    cam->view.m[0][0] = xaxis.x;
    cam->view.m[1][0] = xaxis.y;
    cam->view.m[2][0] = xaxis.z;
    cam->view.m[3][0] = -vector_dotproduct(&xaxis, eye);
    
    cam->view.m[0][1] = yaxis.x;
    cam->view.m[1][1] = yaxis.y;
    cam->view.m[2][1] = yaxis.z;
    cam->view.m[3][1] = -vector_dotproduct(&yaxis, eye);
    
    cam->view.m[0][2] = zaxis.x;
    cam->view.m[1][2] = zaxis.y;
    cam->view.m[2][2] = zaxis.z;
    cam->view.m[3][2] = -vector_dotproduct(&zaxis, eye);
    
    cam->view.m[0][3] = cam->view.m[1][3] = cam->view.m[2][3] = 0.0f;
    cam->view.m[3][3] = 1.0f;
}

void camera_update(Camera_t* cam) {
    if (!cam->update) return;
    cam->update = 0;
    Vector_t right, up;
    vector_normalize(&cam->front);
    vector_crossproduct(&right, &cam->up, &cam->front);
    vector_normalize(&right);
    vector_crossproduct(&up, &cam->front, &right);
    vector_normalize(&up);
    camera_set_lookat(cam, &cam->eye, &cam->front, &up);
    
    Matrix_t m;
    matrix_mul(&m, &cam->world, &cam->view);
    matrix_mul(&cam->transform, &m, &cam->projection);
    matrix_mul(&cam->viewprojection, &cam->view, &cam->projection);
}

void camera_set_eye(Camera_t* cam, float x, float y, float z) {
    vector_set(&cam->eye, x, y, z);
    cam->update = 1;
}

void camera_set_front(Camera_t* cam, float x, float y, float z) {
    vector_set(&cam->front, x, y, z);
    cam->update = 1;
}

void camera_set_at(Camera_t* cam, float x, float y, float z) {
    camera_set_front(cam, x - cam->eye.x, y - cam->eye.y, z - cam->eye.z);
}

void camera_at_zero(Camera_t* cam, float x, float y, float z) {
    vector_set(&cam->eye, x, y, z);
    vector_set(&cam->up, 0, 1, 0);
    vector_set(&cam->front, -x, -y, -z);
    cam->update = 1;
}

void camera_init(Camera_t* cam, float fov, float aspect, float zn, float zf) {
    cam->aspect = aspect;
    cam->fovy = fov;
    cam->zn = zn;
    cam->zf = zf;
    cam->viewplane_width = 2 * tan(fov * 0.5f) * zn;
    cam->viewplane_height = cam->viewplane_width / aspect;
    cam->update = 1;
    vector_set(&cam->eye, 1, 0, 0);
    vector_set(&cam->up, 0, 1, 0);
    vector_set(&cam->front, -1, 0, 0);
    matrix_set_identity(&cam->world);
    matrix_set_identity(&cam->view);
    matrix_set_perspective(&cam->projection, fov, aspect, zn, zf);
}

// 背面剔除
int camera_remove_backface(Camera_t* cam, const Poly_t* poly) {
    if (poly->attr & POLY_ATTR_2SIDED) return 0;
    Vector_t line1, line2, n;
    vector_sub(&line1, &poly->tvlist[1].pos, &poly->tvlist[0].pos);
    vector_sub(&line2, &poly->tvlist[2].pos, &poly->tvlist[0].pos);
    // 计算平面的法线
    vector_crossproduct(&n, &line1, &line2);
    vector_normalize(&n);
    // 计算视线
    Vector_t view;
    vector_sub(&view, &cam->eye, &poly->tvlist[0].pos);
//    float dot = vector_dotproduct(&view, &n);
    float dot = vector_dotproduct(&view, &poly->tvlist[0].normal);
    if (dot < 0.0) return 1;
    return 0;
}

void camera_init_inverse_transpose_matrix(Camera_t* cam) {
    matrix_clone(&cam->inversetranspose, &cam->world);
    matrix_inverse(&cam->inversetranspose);
    matrix_transpose(&cam->inversetranspose);
}

#pragma mark -
#pragma mark Poly

void poly_transform_model_world(Poly_t* poly, Camera_t* cam) {
    matrix_apply(&poly->tvlist[0].pos, &poly->vlist[0].pos, &cam->world);
    matrix_apply(&poly->tvlist[1].pos, &poly->vlist[1].pos, &cam->world);
    matrix_apply(&poly->tvlist[2].pos, &poly->vlist[2].pos, &cam->world);
    
    vector_copy(&poly->tvlist[0].wpos, &poly->tvlist[0].pos);
    vector_copy(&poly->tvlist[1].wpos, &poly->tvlist[1].pos);
    vector_copy(&poly->tvlist[2].wpos, &poly->tvlist[2].pos);
}

void poly_transform_world_view(Poly_t* poly, Camera_t* cam) {
    matrix_apply(&poly->tvlist[0].pos, &poly->tvlist[0].pos, &cam->view);
    matrix_apply(&poly->tvlist[1].pos, &poly->tvlist[1].pos, &cam->view);
    matrix_apply(&poly->tvlist[2].pos, &poly->tvlist[2].pos, &cam->view);
}

void poly_transform_view_perspective(Poly_t* poly, Camera_t* cam) {
    matrix_apply(&poly->tvlist[0].pos, &poly->tvlist[0].pos, &cam->projection);
    matrix_apply(&poly->tvlist[1].pos, &poly->tvlist[1].pos, &cam->projection);
    matrix_apply(&poly->tvlist[2].pos, &poly->tvlist[2].pos, &cam->projection);
}

void poly_transform_world_perspective(Poly_t* poly, Camera_t* cam) {
    matrix_apply(&poly->tvlist[0].pos, &poly->tvlist[0].pos, &cam->viewprojection);
    matrix_apply(&poly->tvlist[1].pos, &poly->tvlist[1].pos, &cam->viewprojection);
    matrix_apply(&poly->tvlist[2].pos, &poly->tvlist[2].pos, &cam->viewprojection);
}

void poly_compute_vertex_normal(Poly_t* poly) {
    Vector_t u, v;
    vector_sub(&u, &poly->vlist[1].pos, &poly->vlist[0].pos);
    vector_sub(&v, &poly->vlist[2].pos, &poly->vlist[0].pos);
    vector_crossproduct(&poly->vlist[0].normal, &u, &v);
    vector_normalize(&poly->vlist[0].normal);
    
    if (poly->vlist[0].normal.x == 0 &&
        poly->vlist[0].normal.y == 0 &&
        poly->vlist[0].normal.z == 0) {
    }
    
    vector_sub(&u, &poly->vlist[2].pos, &poly->vlist[1].pos);
    vector_sub(&v, &poly->vlist[0].pos, &poly->vlist[1].pos);
    vector_crossproduct(&poly->vlist[1].normal, &u, &v);
    vector_normalize(&poly->vlist[1].normal);
    
    vector_sub(&u, &poly->vlist[0].pos, &poly->vlist[2].pos);
    vector_sub(&v, &poly->vlist[1].pos, &poly->vlist[2].pos);
    vector_crossproduct(&poly->vlist[2].normal, &u, &v);
    vector_normalize(&poly->vlist[2].normal);
}

void poly_transform_vertex_normal(Poly_t* poly, Camera_t* cam) {
    // 正规矩阵对法线变换
    for (int vertex = 0; vertex < 3; ++vertex) {
        matrix_apply(&poly->tvlist[vertex].normal, &poly->vlist[vertex].normal, &cam->inversetranspose);
        vector_normalize(&poly->tvlist[vertex].normal);
    }
}

int poly_clip(Poly_t* poly, Device_t* device, Poly_t poly_list[]) {
#define CLIP_CODE_GX    0x0001    /* x > x_max */
#define CLIP_CODE_LX    0x0002    /* x < x_min */
#define CLIP_CODE_IX    0x0004    /* x_min < x < x_max */
    
#define CLIP_CODE_GY    0x0001    /* y > y_max */
#define CLIP_CODE_LY    0x0002    /* y < y_min */
#define CLIP_CODE_IY    0x0004    /* y_min < y < y_max */
    
#define CLIP_CODE_GZ    0x0001    /* z > z_max */
#define CLIP_CODE_LZ    0x0002    /* z < z_min */
#define CLIP_CODE_IZ    0x0004    /* z_min < z < z_max */
    
    int vertex_state[3];
    int num_vertx_in = 0;
    Camera_t* cam = device->cam;
    float fov = tan(cam->fovy * 0.5);
    // 左右平面
    for (int i = 0; i < 3; ++i) {
        Vector_t* v = &poly->tvlist[i].pos;
        float x_test = v->z * fov * cam->aspect;//factor;
        if (v->x < -x_test) vertex_state[i] = CLIP_CODE_LX;
        else if (v->x > x_test) vertex_state[i] = CLIP_CODE_GX;
        else vertex_state[i] = CLIP_CODE_IX;
    }
    if ((vertex_state[0] == CLIP_CODE_LX && vertex_state[1] == CLIP_CODE_LX && vertex_state[2] == CLIP_CODE_LX) ||
        (vertex_state[0] == CLIP_CODE_GX && vertex_state[1] == CLIP_CODE_GX && vertex_state[2] == CLIP_CODE_GX))
        return 0;
    // 上下平面
    for (int i = 0; i < 3; ++i) {
        Vector_t* v = &poly->tvlist[i].pos;
        float y_test = v->z * fov;
        if (v->y < -y_test) vertex_state[i] = CLIP_CODE_LY;
        else if (v->y > y_test) vertex_state[i] = CLIP_CODE_GY;
        else vertex_state[i] = CLIP_CODE_IY;
    }
    if ((vertex_state[0] == CLIP_CODE_LY && vertex_state[1] == CLIP_CODE_LY && vertex_state[2] == CLIP_CODE_LY) ||
        (vertex_state[0] == CLIP_CODE_GY && vertex_state[1] == CLIP_CODE_GY && vertex_state[2] == CLIP_CODE_GY))
        return 0;
    // 前后平面
    for (int i = 0; i < 3; ++i) {
        Vector_t* v = &poly->tvlist[i].pos;
        if (v->z < cam->zn) vertex_state[i] = CLIP_CODE_LZ;
        else if (v->z > cam->zf) vertex_state[i] = CLIP_CODE_GZ;
        else ++num_vertx_in, vertex_state[i] = CLIP_CODE_IZ;
    }
    if ((vertex_state[0] == CLIP_CODE_LZ && vertex_state[1] == CLIP_CODE_LZ && vertex_state[2] == CLIP_CODE_LZ) ||
        (vertex_state[0] == CLIP_CODE_GZ && vertex_state[1] == CLIP_CODE_GZ && vertex_state[2] == CLIP_CODE_GZ))
        return 0;
    
    // clip near z
    if ((vertex_state[0] | vertex_state[1] | vertex_state[2]) & CLIP_CODE_LZ) {
        if (num_vertx_in == 1) {
            int v_in = 0, v_out_1 = 0, v_out_2 = 0;
            Vector_t line;
            if (vertex_state[0] == CLIP_CODE_IZ) v_in = 0, v_out_1 = 1, v_out_2 = 2;
            else if (vertex_state[1] == CLIP_CODE_IZ) v_in = 1, v_out_1 = 2, v_out_2 = 0;
            else if (vertex_state[2] == CLIP_CODE_IZ) v_in = 2, v_out_1 = 0, v_out_2 = 1;
            // calc point 1
            vector_sub(&line, &poly->tvlist[v_out_1].pos, &poly->tvlist[v_in].pos);
            float t1 = ((cam->zn - poly->tvlist[v_in].pos.z) / line.z);
            vertex_interp(&poly->tvlist[v_out_1], &poly->tvlist[v_in], &poly->tvlist[v_out_1], t1);
            // calc point 2
            vector_sub(&line, &poly->tvlist[v_out_2].pos, &poly->tvlist[v_in].pos);
            float t2 = ((cam->zn - poly->tvlist[v_in].pos.z) / line.z);
            vertex_interp(&poly->tvlist[v_out_2], &poly->tvlist[v_in], &poly->tvlist[v_out_2], t2);
            poly->tvlist[v_out_1].pos.z = cam->zn;
            poly->tvlist[v_out_2].pos.z = cam->zn;

            poly_list[0] = *poly;
            
        } else if (num_vertx_in == 2) {
            Poly_t npoly;
            memcpy(&npoly, poly, sizeof(Poly_t));
            int v_in_1 = 0, v_in_2 = 0, v_out_1 = 0;
            Vector_t line;
            if (!(vertex_state[0] == CLIP_CODE_IZ)) v_in_1 = 1, v_in_2 = 2, v_out_1 = 0;
            else if (!(vertex_state[1] == CLIP_CODE_IZ)) v_in_1 = 2, v_in_2 = 0, v_out_1 = 1;
            else if (!(vertex_state[2] == CLIP_CODE_IZ)) v_in_1 = 0, v_in_2 = 1, v_out_1 = 2;
            // calc point 1
            vector_sub(&line, &poly->tvlist[v_out_1].pos, &poly->tvlist[v_in_1].pos);
            float t1 = ((cam->zn - poly->tvlist[v_in_1].pos.z) / line.z);
            // calc point 2
            vector_sub(&line, &poly->tvlist[v_out_1].pos, &poly->tvlist[v_in_2].pos);
            float t2 = ((cam->zn - poly->tvlist[v_in_2].pos.z) / line.z);
            Vertex_t clone = poly->tvlist[v_out_1];
            // cur poly
            vertex_interp(&poly->tvlist[v_out_1], &poly->tvlist[v_in_1], &clone, t1);
            // new poly
            vertex_interp(&npoly.tvlist[v_out_1], &poly->tvlist[v_in_2], &clone, t2);
            vertex_interp(&npoly.tvlist[v_in_1], &poly->tvlist[v_in_1], &clone, t1);
            poly->tvlist[v_out_1].pos.z = cam->zn;
            npoly.tvlist[v_out_1].pos.z = cam->zn;
            npoly.tvlist[v_in_1].pos.z = cam->zn;
            
            poly_list[0] = *poly;
            poly_list[1] = npoly;
        }
    }
    // clip far z
    else if ((vertex_state[0] | vertex_state[1] | vertex_state[2]) & CLIP_CODE_GZ) {
        
    }
    else if ((vertex_state[0] & vertex_state[1] & vertex_state[2]) & CLIP_CODE_IZ) {
        num_vertx_in = 1;
        poly_list[0] = *poly;
    }
    return num_vertx_in;
}

#pragma mark -
#pragma mark Object

//typedef struct {
//    Poly_t* polys;
//    int num_polys;
//    int num_vertex;
//    Vector_t pos;
//    Vector_t scale;
//    Texture_t* texture;
//} Object_t;

void object_init(Object_t* object, int num_vertex, int num_polys) {
    int need = sizeof(Poly_t) * num_polys;
    char* ptr = (char*)malloc(need);
    object->polys = (Poly_t*)ptr;
    object->num_polys = num_polys;
    object->num_vertex = num_vertex;
    object->theta = 0.f;
    object->render_state = RENDER_STATE_TEXTURE;
    object->texture.buffer = NULL;
    object->texture.tex_width = 0.f;
    object->texture.tex_height = 0.f;
    object->texture.tex_pitch = 0.f;
    vector_set(&object->pos, 0, 0, 0);
    vector_set(&object->scale, 1, 1, 1);
}

// 重置物体
void object_reset(Object_t* object) {
    for (int i = 0; i < object->num_polys; ++i) {
        Poly_t* poly = &object->polys[i];
        for (int vertex = 0; vertex < 3; ++vertex) {
            vertex_copy(&poly->tvlist[vertex], &poly->vlist[vertex]);
        }
    }
}

void object_set_rotate(Object_t* object, float x, float y, float z, float theta) {
    vector_set(&object->rotate, x, y, z);
    object->theta = theta;
}

void object_set_scale(Object_t* object, float x, float y, float z) {
    vector_set(&object->scale, x, y, z);
}

void object_set_position(Object_t* object, float x, float y, float z) {
    vector_set(&object->pos, x, y, z);
}

void object_create_ball(Object_t* obj, float r) {
    float radius = 0.8 * r;
    int kRow = 100, kCol = 100;
    float k = 1.f / 1.f;  // 比例
    float step_z = M_PI / kRow ;  //z方向每次步进的角度(上下，纬度)
    float step_xy = 2 * M_PI / kCol;  //x,y平面每次步进的角度（经度）
    
    float angle_z = 0; //起始角度
    float angle_xy = 0;
    
    object_init(obj, kRow * kCol * 6, kRow * kCol * 2);
    
    int idx = 0;
    for(int i = 0; i < kRow; i++)
    {
        angle_z = i * step_z;  //每次步进step_z
        
        for(int j = 0; j < kCol ; j++)
        {
            Poly_t* poly1 = &obj->polys[idx++], *poly2 = &obj->polys[idx++];
            poly1->attr = RENDER_STATE_COLOR;// | POLY_ATTR_2SIDED;
            poly2->attr = RENDER_STATE_COLOR;// | POLY_ATTR_2SIDED;
            for (int k = 0; k < 3; ++k) {
                color_set(&poly1->vlist[k].color, 0.6, 0.6, 0.6, 1);
                color_set(&poly2->vlist[k].color, 0.6, 0.6, 0.6, 1);
                poly1->vlist[k].rhw = poly1->vlist[k].pos.w = 1;
                poly2->vlist[k].rhw = poly2->vlist[k].pos.w = 1;
            }
            angle_xy = j * step_xy;
            poly1->vlist[0].pos.x = k * radius * sin(angle_z) * cos(angle_xy);
            poly1->vlist[0].pos.y = radius * sin(angle_z) * sin(angle_xy);
            poly1->vlist[0].pos.z = radius * cos(angle_z);
            
            poly1->vlist[1].pos.x = k * radius * sin(angle_z + step_z) * cos(angle_xy);
            poly1->vlist[1].pos.y = radius * sin(angle_z + step_z) * sin(angle_xy);
            poly1->vlist[1].pos.z = radius * cos(angle_z + step_z);
            
            poly1->vlist[2].pos.x = k * radius * sin(angle_z + step_z) * cos(angle_xy + step_xy);
            poly1->vlist[2].pos.y = radius * sin(angle_z + step_z) * sin(angle_xy + step_xy);
            poly1->vlist[2].pos.z = radius * cos(angle_z + step_z);
            // 顶点法线
            poly_compute_vertex_normal(poly1);
            
            poly2->vlist[0].pos.x = k * radius * sin(angle_z) * cos(angle_xy);
            poly2->vlist[0].pos.y = radius * sin(angle_z) * sin(angle_xy);
            poly2->vlist[0].pos.z = radius * cos(angle_z);
            
            poly2->vlist[1].pos.x = k * radius * sin(angle_z + step_z) * cos(angle_xy + step_xy);
            poly2->vlist[1].pos.y = radius * sin(angle_z + step_z) * sin(angle_xy + step_xy);
            poly2->vlist[1].pos.z = radius * cos(angle_z + step_z);
            
            poly2->vlist[2].pos.x = k * radius * sin(angle_z) * cos(angle_xy + step_xy);
            poly2->vlist[2].pos.y = radius * sin(angle_z) * sin(angle_xy + step_xy);
            poly2->vlist[2].pos.z = radius * cos(angle_z);
            poly_compute_vertex_normal(poly2);
            
        } //循环画出这一层的平面，组成一个环
    }  //z轴++，画出剩余层
}

// 加载代码来源于 https://github.com/syoyo/tinyobjloader-c
int object_load_obj(Object_t* obj, const char* obj_file_name) {
    tinyobj_attrib_t attrib;
    tinyobj_shape_t* shapes = NULL;
    size_t num_shapes;
    tinyobj_material_t* tmaterials = NULL;
    size_t num_materials;
    
    FILE * pFile;
    long lSize;
    size_t data_len;
    char * buffer;
    size_t result;
    pFile = fopen(obj_file_name, "r");
    fseek(pFile, 0, SEEK_END);
    lSize = ftell (pFile);
    rewind (pFile);
    
    // allocate memory to contain the whole file:
    buffer = (char*) malloc (sizeof(char)*lSize);
    if (buffer == NULL) {fputs ("Memory error",stderr); exit (2);}
    
    // copy the file into the buffer:
    result = fread (buffer,1,lSize,pFile);
    if (result > lSize) {fputs ("Reading error",stderr); exit (3);}
    
    
    data_len = (size_t)lSize;
    if (buffer == NULL) {
        exit(-1);
        /* return 0; */
    }
    
    char prefix_path[1000];
    long len = strrchr(obj_file_name, '/') - obj_file_name+1;
    strncpy(prefix_path, obj_file_name, len);
    prefix_path[len] = '\0';
    unsigned int flags = TINYOBJ_FLAG_TRIANGULATE;
    int ret = tinyobj_parse_obj(&attrib, &shapes, &num_shapes, &tmaterials,
                                &num_materials, buffer, data_len, flags, prefix_path);
    if (ret != TINYOBJ_SUCCESS) {
        return 0;
    }
    object_init(obj, attrib.num_vertices, attrib.num_face_num_verts);
    size_t face_offset = 0;
    size_t i;
    for (i = 0; i < attrib.num_face_num_verts; i++) {
        size_t f = 0;
        assert(attrib.face_num_verts[i] % 3 == 0); /* assume all triangle faces. */
        
        // 暂时只支持三顶点的面
        size_t k;
        float v[3][3];
        float n[3][3];
        float c[3];
        float t[3][2];
        float len2;
        
        tinyobj_vertex_index_t idx0 = attrib.faces[face_offset + 3 * f + 0];
        tinyobj_vertex_index_t idx1 = attrib.faces[face_offset + 3 * f + 1];
        tinyobj_vertex_index_t idx2 = attrib.faces[face_offset + 3 * f + 2];
        
        for (k = 0; k < 3; k++) {
            int f0 = idx0.v_idx;
            int f1 = idx1.v_idx;
            int f2 = idx2.v_idx;
            assert(f0 >= 0);
            assert(f1 >= 0);
            assert(f2 >= 0);
            
            v[0][k] = attrib.vertices[3 * (size_t)f0 + k];
            v[1][k] = attrib.vertices[3 * (size_t)f1 + k];
            v[2][k] = attrib.vertices[3 * (size_t)f2 + k];
        }
        
        if (attrib.num_normals > 0) {
            int f0 = idx0.vn_idx;
            int f1 = idx1.vn_idx;
            int f2 = idx2.vn_idx;
            if (f0 >= 0 && f1 >= 0 && f2 >= 0) {
                assert(f0 < (int)attrib.num_normals);
                assert(f1 < (int)attrib.num_normals);
                assert(f2 < (int)attrib.num_normals);
                for (k = 0; k < 3; k++) {
                    n[0][k] = attrib.normals[3 * (size_t)f0 + k];
                    n[1][k] = attrib.normals[3 * (size_t)f1 + k];
                    n[2][k] = attrib.normals[3 * (size_t)f2 + k];
                }
            } else { /* normal index is not defined for this face */
                /* compute geometric normal */
                calc_normal(n[0], v[0], v[1], v[2]);
                n[1][0] = n[0][0];
                n[1][1] = n[0][1];
                n[1][2] = n[0][2];
                n[2][0] = n[0][0];
                n[2][1] = n[0][1];
                n[2][2] = n[0][2];
            }
        } else {
            /* compute geometric normal */
            calc_normal(n[0], v[0], v[1], v[2]);
            n[1][0] = n[0][0];
            n[1][1] = n[0][1];
            n[1][2] = n[0][2];
            n[2][0] = n[0][0];
            n[2][1] = n[0][1];
            n[2][2] = n[0][2];
        }
        
        if (attrib.num_texcoords > 0) {
            int t0 = idx0.vt_idx;
            int t1 = idx1.vt_idx;
            int t2 = idx2.vt_idx;
            if (t0 >= 0 && t1 >= 0 && t2 >= 0) {
                assert(t0 < (int)attrib.num_texcoords);
                assert(t1 < (int)attrib.num_texcoords);
                assert(t2 < (int)attrib.num_texcoords);
                for (k = 0; k < 2; k++) {
                    t[0][k] = attrib.texcoords[2 * (size_t)t0 + k];
                    t[1][k] = attrib.texcoords[2 * (size_t)t1 + k];
                    t[2][k] = attrib.texcoords[2 * (size_t)t2 + k];
                }
            } else {
            }
        }
        
        Poly_t* poly = &obj->polys[i];
        poly->attr |= RENDER_STATE_COLOR;
        for (k = 0; k < 3; k++) {
            vector_set(&poly->vlist[k].pos, v[k][0], v[k][1], v[k][2]);
            vector_set(&poly->vlist[k].normal, n[k][0], n[k][1], n[k][2]);
            poly->vlist[k].tex.u = t[k][0];
            poly->vlist[k].tex.v = t[k][1];
            /* Use normal as color. */
            c[0] = n[k][0];
            c[1] = n[k][1];
            c[2] = n[k][2];
            len2 = c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
            if (len2 > 0.0f) {
                float len = (float)sqrt((double)len2);
                c[0] /= len;
                c[1] /= len;
                c[2] /= len;
            }
            color_set(&poly->vlist[k].color, (c[0] * 0.5f + 0.5f), (c[1] * 0.5f + 0.5f), (c[2] * 0.5f + 0.5f), 1.0f);
            color_set(&poly->vlist[k].color, 1, 1, 1, 1);
        }
        face_offset += (size_t)attrib.face_num_verts[i];
    }
    tinyobj_attrib_free(&attrib);
    tinyobj_shapes_free(shapes, num_shapes);
    tinyobj_materials_free(tmaterials, num_materials);
    
    // terminate
    fclose (pFile);
    free (buffer);
    return 1;
}

void vertex_shader(Device_t* deivce, const VShader_t* vs, FShader_t* fs) {
    vector_copy(&fs->pos, &vs->pos);
    vector_copy(&fs->normal, &vs->normal);
    color_copy(&fs->color, &vs->color);
    fs->tex.u = vs->tex.u;
    fs->tex.v = vs->tex.v;
}

void frag_shader(Device_t* device, FShader_t* fs, u_int32_t* color) {
    Color_t c = {0, 0, 0, 0};
    float w = fs->pos.w;
    
//    Color_t tc = {fs->color.r * w, fs->color.g * w, fs->color.b * w, fs->color.a * w};
    int a = COLOR_A(*color, 1);
    Color_t tc = {COLOR_R(*color, 1) / 255.f, COLOR_G(*color, 1) / 255.f, COLOR_B(*color, 1) / 255.f, COLOR_A(*color, 1) / 255.f};
    Vector_t normal = {fs->normal.x * w, fs->normal.y * w, fs->normal.z * w, 1.f};
    Vector_t pos = {fs->pos.x * w, fs->pos.y * w, fs->pos.z * w, 1.f};
    float nl = vector_length(&normal);
    Vector_t l;
    
    for (int i = 0; i < device->num_light; ++i) {
        Light_t* light = &device->light[i];
        if (light->attr & LIGHT_ATTR_AMBIENT) {
            color_mul(&tc, &light->c, &tc);
            color_add(&c, &c, &tc);
            color_copy(&tc, &c);
        } else if (light->attr & LIGHT_ATTR_INFINITE) {
            // 光强
            float dp = vector_dotproduct(&normal, &light->dir);
            if (dp > EPSILON_E5) {
                float ii = dp / nl;
                for (int vertex = 0; vertex < 3; ++vertex) {
                    color_mul(&tc, &light->c, &tc);
                    color_scale(&tc, ii);
                    color_add(&c, &c, &tc);
                    color_copy(&tc, &c);
                }
            }
        } else if (light->attr & LIGHT_ATTR_POINT) {
            //计算从表面到光源的向量
            vector_sub(&l, &light->pos, &pos);
            //计算距离和衰减
            float distance = vector_length(&l);
            // 光强
            float dp = vector_dotproduct(&normal, &l);
            if (dp > EPSILON_E5)
            {
                float atten = (light->kc + light->kl * distance + light->kq * distance * distance);
                float ii = dp / (nl * distance * atten);
                color_mul(&tc, &light->c, &tc);
                color_scale(&tc, ii);
                color_add(&c, &c, &tc);
                color_copy(&tc, &c);
            }
        } else if (light->attr & LIGHT_ATTR_SPOTLIGHT) {
            float dp = vector_dotproduct(&normal, &light->dir);
            if (dp > EPSILON_E5)
            {
                vector_sub(&l, &light->pos, &pos);
                float distance = vector_length(&l);
                float dpsl = vector_dotproduct(&l, &light->dir) / distance;
                // proceed only if term is positive
                if (dpsl > EPSILON_E5)
                {
                    // compute attenuation
                    float atten = (light->kc + light->kl * distance + light->kq * distance * distance);
                    float dpsl_exp = dpsl;
                    // exponentiate for positive integral powers
                    for (int e_index = 1; e_index < (int)light->pf; e_index++)
                        dpsl_exp *= dpsl;
                    float ii = dp * dpsl_exp / (nl * atten);
                    color_mul(&tc, &light->c, &tc);
                    color_scale(&tc, ii);
                    color_add(&c, &c, &tc);
                    color_copy(&tc, &c);
                } // end if
            }
        }
    }
    
    int r = CMID(c.r * 255.f, 0.f, 255.f);
    int g = CMID(c.g * 255.f, 0.f, 255.f);
    int b = CMID(c.b * 255.f, 0.f, 255.f);
    *color = COLOR(r, g, b, a, 1);
}

#pragma mark -
#pragma mark Device


void device_init(Device_t* device, int width, int height) {
    int need = (width * height) * (sizeof(uint8_t) * 4 + sizeof(float));
    char* ptr = (char*)malloc(need);
    device->framebuffer = (uint8_t*)ptr;
    ptr += (width * height) * sizeof(uint8_t) * 4;
    device->zbuffer = (float*)ptr;
    device->texture = NULL;
    memset(device->framebuffer, 144, (width * height) * sizeof(uint8_t) * 4);
    memset(device->zbuffer, 0, (width * height) * sizeof(float));
    
    device->width = width;
    device->height = height;
    device->num_cam = 0;
    device->cur_cam = 0;
    device->num_light = 0;
    device->background = 0xc0c0c0;
}

void device_destroy(Device_t* device) {
    if (device->framebuffer)
        free(device->framebuffer);
    device->framebuffer = NULL;
    device->zbuffer = NULL;
    device->texture = NULL;
}

void device_clear(Device_t* device) {
    int n = device->width * device->height;
    memset(device->framebuffer, 77, n * sizeof(u_int8_t) * 4);
    memset(device->zbuffer, 0, n * sizeof(float));
}

void device_insert_camera(Device_t* device, Camera_t* cam) {
    device->cams[device->num_cam++] = cam;
}

void device_activate_camera(Device_t* device, int idx) {
    assert(idx < device->num_cam);
    device->cur_cam = idx;
    device->cam = device->cams[idx];
}

void device_new_light(Device_t* device, int attr, Point_t* pos, Vector_t* dir, Color_t* color, float kc, float kl, float kq, float pf) {
    Light_t* light = &device->light[device->num_light++];
    light->attr = attr;
    if (pos) vector_copy(&light->pos, pos);
    if (dir) vector_copy(&light->dir, dir);
    light->c.r = color->r;
    light->c.g = color->g;
    light->c.b = color->b;
    light->c.a = color->a;
    light->kc = kc;
    light->kl = kl;
    light->kq = kq;
    light->pf = pf;
}

void device_draw_pixel(Device_t* device, int x, int y, uint32_t color) {
    if (x < device->width && y < device->height && x >= 0 && y >= 0) {
//        device->framebuffer[y * device->width * 3 + x * 3] = COLOR_B(color, 0);
//        device->framebuffer[y * device->width * 3 + x * 3 + 1] = COLOR_G(color, 0);
//        device->framebuffer[y * device->width * 3 + x * 3 + 2] = COLOR_R(color, 0);
        device->framebuffer[y * device->width * 4 + x * 4] = COLOR_B(color, 1);
        device->framebuffer[y * device->width * 4 + x * 4 + 1] = COLOR_G(color, 1);
        device->framebuffer[y * device->width * 4 + x * 4 + 2] = COLOR_R(color, 1);
        device->framebuffer[y * device->width * 4 + x * 4 + 3] = COLOR_A(color, 1);
    }
}

void device_draw_line(Device_t* device, int x1, int y1, int x2, int y2, uint32_t color) {
    if ((x1 < 0 && x2 < 0) || (x1 >= device->width && x2 >= device->width)) return;
    if ((y1 < 0 && y2 < 0) || (y1 >= device->height && y2 >= device->height)) return;
    x1 = CMID(x1, 0, device->width - 1); x2 = CMID(x2, 0, device->width - 1);
    y1 = CMID(y1, 0, device->height - 1); y2 = CMID(y2, 0, device->height - 1);

    if (x1 == x2 && y1 == y2) {
        device_draw_pixel(device, x1, y1, color);
    } else if (x1 == x2) {
        int inc = y1 < y2 ? 1 : -1;
        for (int y = y1; y != y2; y += inc)
            device_draw_pixel(device, x1, y, color);
        device_draw_pixel(device, x2, y2, color);
    } else if (y1 == y2) {
        int inc = x1 < x2 ? 1 : -1;
        for (int x = x1; x != x2; x += inc)
            device_draw_pixel(device, x, y1, color);
        device_draw_pixel(device, x2, y2, color);
    } else {
        int dx = (x1 < x2) ? (x2 - x1) : (x1 - x2);
        int dy = (y1 < y2) ? (y2 - y1) : (y1 - y2);
        int tmp = 0;
        if (dx >= dy) {
            // swap
            if (x2 < x1) { SWAP(x1, x2, int); SWAP(y1, y2, int); }
            int inc = y1 < y2 ? 1 : -1;
            for (int x = x1, y = y1; x <= x2; ++x) {
                device_draw_pixel(device, x, y, color);
                tmp += dy;
                if (tmp >= dx) {
                    tmp -= dx;
                    y += inc;
                    device_draw_pixel(device, x, y, color);
                }
            }
            device_draw_pixel(device, x2, y2, color);
        } else {
            if (y2 < y1) { SWAP(x1, x2, int); SWAP(y1, y2, int); }
            int inc = x1 < x2 ? 1 : -1;
            for (int x = x1, y = y1; y <= y2; ++y) {
                device_draw_pixel(device, x, y, color);
                tmp += dx;
                if (tmp >= dy) {
                    tmp -= dy;
                    x += inc;
                    device_draw_pixel(device, x, y, color);
                }
            }
            device_draw_pixel(device, x2, y2, color);
        }
    }
}

// 读取纹理，u,v范围[0,1]
u_int32_t device_texture_read(const Device_t* device, float u, float v) {
    if (!device->texture) return 0;
    int tw = device->texture->tex_width;
    int th = device->texture->tex_height;
    int pitch = device->texture->tex_pitch;
    int isa = device->texture->alpha;
    u = CMID(u, 0.f, 1.f);
    v = CMID(v, 0.f, 1.f);
    u = tw * u;
    v = th * v;
    
    // 双线性滤波
    if (device->render_attr & RENDER_ATTR_BILERP) {
        // 整数和小数部分
        int ui = (int)u, vi = (int)v;
        float uf = u - ui, vf = v - vi;
        
        u_int32_t* texture = device->texture->buffer;
        // 右、下、右下三个顶点
        int p0 = ui + vi * pitch;
        int p1 = CMID((ui + 1) + vi * pitch, 0, tw * th - 1);
        int p2 = CMID((ui + 1) + (vi + 1) * pitch, 0, tw * th - 1);
        int p3 = CMID(ui + (vi + 1) * pitch, 0, tw * th - 1);
        u_int32_t c0 = texture[p0], c1 = texture[p1], c2 = texture[p2], c3 = texture[p3];
        int R = (1 - uf) * (1 - vf) * COLOR_R(c0, isa) + uf * (1 - vf) * COLOR_R(c1, isa) + uf * vf * COLOR_R(c2, isa) + (1 - uf) * vf * COLOR_R(c3, isa);
        int G = (1 - uf) * (1 - vf) * COLOR_G(c0, isa) + uf * (1 - vf) * COLOR_G(c1, isa) + uf * vf * COLOR_G(c2, isa) + (1 - uf) * vf * COLOR_G(c3, isa);
        int B = (1 - uf) * (1 - vf) * COLOR_B(c0, isa) + uf * (1 - vf) * COLOR_B(c1, isa) + uf * vf * COLOR_B(c2, isa) + (1 - uf) * vf * COLOR_B(c3, isa);
        int A = (1 - uf) * (1 - vf) * COLOR_A(c0, isa) + uf * (1 - vf) * COLOR_A(c1, isa) + uf * vf * COLOR_A(c2, isa) + (1 - uf) * vf * COLOR_A(c3, isa);
        return COLOR(R, G, B, A, isa);
    } else {
        int x = (int)(u + 0.5);
        int y = (int)(v + 0.5);
        x = CMID(x, 0, tw - 1);
        y = CMID(y, 0, th - 1);
        return device->texture->buffer[y * tw + x];
    }
}

void device_draw_scanline(Device_t* device, Scanline_t* scanline, int attr) {
    int width = device->width;
    int y = scanline->y;
    for (int x = scanline->x; x <= scanline->width + scanline->x; ++x) {
        if (x >= 0 && x < width) {
            float rhw = scanline->v.rhw;
            if (rhw >= device->zbuffer[y * device->width + x]) {
                float w = 1.f / rhw;
                device->zbuffer[y * device->width + x] = rhw;
                
                u_int32_t color = 0;
                if (attr & RENDER_STATE_COLOR) {
                    float r = scanline->v.color.r * w;
                    float g = scanline->v.color.g * w;
                    float b = scanline->v.color.b * w;
                    float a = scanline->v.color.a * w;
                    int R = CMID((int)(r * 255.f), 0, 255);
                    int G = CMID((int)(g * 255.f), 0, 255);
                    int B = CMID((int)(b * 255.f), 0, 255);
                    int A = CMID((int)(a * 255.f), 0, 255);
                    color = COLOR(R, G, B, A, 1);

                } else if (attr & RENDER_STATE_TEXTURE) {
                    float u = scanline->v.tex.u * w;
                    float v = scanline->v.tex.v * w;
                    color = device_texture_read(device, u, v);
                }
                FShader_t fs;
                fshader_init(&fs, &scanline->v);
                frag_shader(device, &fs, &color);
                device_draw_pixel(device, x, y, color);
            }
        }
        // color、tex 步进
        vertex_add(&scanline->v, &scanline->step);
    }
}

void device_render_trap(Device_t* device, Trapezoid_t* trap, int attr) {
    Scanline_t scanline;
    int top = (int)(trap->top + 0.5);
    int bottom = (int)(trap->bottom + 0.5);
    for (int y = top; y < bottom; ++y) {
        if (y >= 0 && y < device->height) {
            trapezoid_edge_interp(trap, y);
            trapezoid_init_scan_line(trap, &scanline, y);
            device_draw_scanline(device, &scanline, attr);
        }
    }
}

void device_draw_poly(Device_t* device, Poly_t* poly, const Matrix_t* it) {
    // 转世界坐标
    poly_transform_model_world(poly, device->cam);
    
    // 正规矩阵对法线变换
    poly_transform_vertex_normal(poly, device->cam);
    
    // 背面剔除
    if (camera_remove_backface(device->cam, poly)) return;
    
//    VShader_t avs[3];
//    FShader_t afs[3];
//    for (int vertex = 0; vertex < 3; ++vertex) {
//        VShader_t* vs = &avs[vertex];
//        vector_copy(&vs->pos, &poly->tvlist[vertex].pos);  // save world pos
//        vector_copy(&vs->normal, &poly->tvlist[vertex].normal);     // save transformed value
//        color_copy(&vs->color, &poly->tvlist[vertex].color);
//        vs->tex.u = poly->tvlist[vertex].tex.u;
//        vs->tex.v = poly->tvlist[vertex].tex.v;
//
//        vertex_shader(device, vs, &afs[vertex]);
//    }
    
    // 光照
    // Warning！光照处理放在这里有bug，3D裁剪生成的新 poly 无法执行，而且多耗费 CPU 对被裁剪掉的面执行光照
#if 0
    Color_t c[3];
    for (int vertex = 0; vertex < 3; ++vertex) {
        c[vertex].r = c[vertex].g = c[vertex].b = c[vertex].a = 0;
    }
    Vector_t l;
    Color_t tc = {0, 0, 0, 0};
    
    for (int i = 0; i < device->num_light; ++i) {
        Light_t* light = &device->light[i];
        if (light->attr & LIGHT_ATTR_AMBIENT) {
            for (int vertex = 0; vertex < 3; ++vertex) {
                color_mul(&tc, &light->c, &poly->vlist[vertex].color);
                color_add(&c[vertex], &c[vertex], &tc);
            }
        } else if (light->attr & LIGHT_ATTR_INFINITE) {
            for (int vertex = 0; vertex < 3; ++vertex) {
                // 光强
                float dp = vector_dotproduct(&poly->tvlist[vertex].normal, &light->dir);
                if (dp > EPSILON_E5) {
                    float nl = vector_length(&poly->tvlist[vertex].normal);
                    float ii = dp / nl;
                    color_mul(&tc, &light->c, &poly->vlist[vertex].color);
                    color_scale(&tc, ii);
                    color_add(&c[vertex], &c[vertex], &tc);
                }
            }
            
        } else if (light->attr & LIGHT_ATTR_POINT) {
            for (int vertex = 0; vertex < 3; ++vertex) {
                //计算从表面到光源的向量
                vector_sub(&l, &light->pos, &poly->tvlist[vertex].pos);
                //计算距离和衰减
                float distance = vector_length(&l);
                // 光强
                float dp = vector_dotproduct(&poly->tvlist[vertex].normal, &l);
                if (dp > EPSILON_E5)
                {
                    float nl = vector_length(&poly->tvlist[vertex].normal);
                    float atten = (light->kc + light->kl * distance + light->kq * distance * distance);
                    float ii = dp / (nl * distance * atten);
                    color_mul(&tc, &light->c, &poly->vlist[vertex].color);
                    color_scale(&tc, ii);
                    color_add(&c[vertex], &c[vertex], &tc);
                }
            }
        }
    }
    
    for (int vertex = 0; vertex < 3; ++vertex) {
        poly->tvlist[vertex].color.r = CMID(c[vertex].r, 0.f, 1.f);
        poly->tvlist[vertex].color.g = CMID(c[vertex].g, 0.f, 1.f);
        poly->tvlist[vertex].color.b = CMID(c[vertex].b, 0.f, 1.f);
    }
#else
    for (int vertex = 0; vertex < 3; ++vertex) {
        poly->tvlist[vertex].color.r = poly->vlist[vertex].color.r;
        poly->tvlist[vertex].color.g = poly->vlist[vertex].color.g;
        poly->tvlist[vertex].color.b = poly->vlist[vertex].color.b;
    }
#endif
    
    poly_transform_world_view(poly, device->cam);
    
    // 3D 裁剪
    Poly_t polys[2];
    int poly_num = poly_clip(poly, device, polys);
    if (poly_num == 0) return;
    
    for (int i = 0; i < poly_num; ++i) {
        Poly_t* poly = &polys[i];
        Point_t* p1 = &poly->tvlist[0].pos;
        Point_t* p2 = &poly->tvlist[1].pos;
        Point_t* p3 = &poly->tvlist[2].pos;
        
        // 投影转换后，w 是 z 的值
        poly_transform_view_perspective(poly, device->cam);
        
        transform_screen(device->width, device->height, p1, p1);
        transform_screen(device->width, device->height, p2, p2);
        transform_screen(device->width, device->height, p3, p3);
        
        if (poly->attr & (RENDER_STATE_TEXTURE | RENDER_STATE_COLOR)) {
            Trapezoid_t traps[2];
            Vertex_t* t1 = &poly->tvlist[0], *t2 = &poly->tvlist[1], *t3 = &poly->tvlist[2];
            
            //u,v,r,g,b 都乘以 1/z，以线性
            vertex_init_rhw(t1);
            vertex_init_rhw(t2);
            vertex_init_rhw(t3);
            
            int n = trapezoid_init_triangle(traps, t1, t2, t3);
            if (n >= 1) device_render_trap(device, &traps[0], poly->attr);
            if (n >= 2) device_render_trap(device, &traps[1], poly->attr);
        }
        
        if (poly->attr & RENDER_STATE_WIREFRAME) {        // 线框绘制
            device_draw_line(device, (int)p1->x, (int)p1->y, (int)p2->x, (int)p2->y, 0xff0000);
            device_draw_line(device, (int)p1->x, (int)p1->y, (int)p3->x, (int)p3->y, 0xff0000);
            device_draw_line(device, (int)p3->x, (int)p3->y, (int)p2->x, (int)p2->y, 0xff0000);
        }
    }
}


#pragma mark -
#pragma mark Window

static Vertex_t box_mesh[] = {
    { {  1,  1, 1, 1 }, {0,0,0,0}, { 0, 0 }, { 1.0f, 0.2f, 1.0f }, 1 },
    { { -1,  1, 1, 1 }, {0,0,0,0}, { 0, 1 }, { 1.0f, 0.2f, 0.2f }, 1 },
    { { -1, -1, 1, 1 }, {0,0,0,0}, { 1, 1 }, { 1.0f, 1.0f, 0.2f }, 1 },
    { { -1, -1, 1, 1 }, {0,0,0,0}, { 1, 1 }, { 1.0f, 1.0f, 0.2f }, 1 },
    { {  1, -1, 1, 1 }, {0,0,0,0}, { 1, 0 }, { 1.0f, 0.2f, 0.2f }, 1 },
    { {  1,  1, 1, 1 }, {0,0,0,0}, { 0, 0 }, { 1.0f, 0.2f, 1.0f }, 1 },

    { {  1, -1, -1, 1 }, {0,0,0,0}, { 0, 0 }, { 1.0f, 0.2f, 1.0f }, 1 },
    { { -1, -1, -1, 1 }, {0,0,0,0}, { 0, 1 }, { 1.0f, 0.2f, 0.2f }, 1 },
    { { -1,  1, -1, 1 }, {0,0,0,0}, { 1, 1 }, { 1.0f, 1.0f, 0.2f }, 1 },
    { { -1,  1, -1, 1 }, {0,0,0,0}, { 1, 1 }, { 1.0f, 1.0f, 0.2f }, 1 },
    { {  1,  1, -1, 1 }, {0,0,0,0}, { 1, 0 }, { 1.0f, 0.2f, 0.2f }, 1 },
    { {  1, -1, -1, 1 }, {0,0,0,0}, { 0, 0 }, { 1.0f, 0.2f, 1.0f }, 1 },

    { { -1, -1,  1, 1 }, {0,0,0,0}, { 0, 0 }, { 1.0f, 0.2f, 1.0f }, 1 },
    { { -1, -1, -1, 1 }, {0,0,0,0}, { 0, 1 }, { 1.0f, 0.2f, 0.2f }, 1 },
    { {  1, -1, -1, 1 }, {0,0,0,0}, { 1, 1 }, { 1.0f, 1.0f, 0.2f }, 1 },
    { {  1, -1, -1, 1 }, {0,0,0,0}, { 1, 1 }, { 1.0f, 1.0f, 0.2f }, 1 },
    { {  1, -1,  1, 1 }, {0,0,0,0}, { 1, 0 }, { 1.0f, 0.2f, 0.2f }, 1 },
    { { -1, -1,  1, 1 }, {0,0,0,0}, { 0, 0 }, { 1.0f, 0.2f, 1.0f }, 1 },

    { { -1, -1,  1, 1 }, {0,0,0,0}, { 0, 0 }, { 1.0f, 0.2f, 1.0f }, 1 },
    { { -1,  1,  1, 1 }, {0,0,0,0}, { 0, 1 }, { 1.0f, 0.2f, 0.2f }, 1 },
    { { -1,  1, -1, 1 }, {0,0,0,0}, { 1, 1 }, { 1.0f, 1.0f, 0.2f }, 1 },
    { { -1,  1, -1, 1 }, {0,0,0,0}, { 1, 1 }, { 1.0f, 1.0f, 0.2f }, 1 },
    { { -1, -1, -1, 1 }, {0,0,0,0}, { 1, 0 }, { 1.0f, 0.2f, 0.2f }, 1 },
    { { -1, -1,  1, 1 }, {0,0,0,0}, { 0, 0 }, { 1.0f, 0.2f, 1.0f }, 1 },

    { { -1,  1,  1, 1 }, {0,0,0,0}, { 0, 0 }, { 1.0f, 0.2f, 1.0f }, 1 },
    { {  1,  1,  1, 1 }, {0,0,0,0}, { 0, 1 }, { 1.0f, 0.2f, 0.2f }, 1 },
    { {  1,  1, -1, 1 }, {0,0,0,0}, { 1, 1 }, { 1.0f, 1.0f, 0.2f }, 1 },
    { {  1,  1, -1, 1 }, {0,0,0,0}, { 1, 1 }, { 1.0f, 1.0f, 0.2f }, 1 },
    { { -1,  1, -1, 1 }, {0,0,0,0}, { 1, 0 }, { 1.0f, 0.2f, 0.2f }, 1 },
    { { -1,  1,  1, 1 }, {0,0,0,0}, { 0, 0 }, { 1.0f, 0.2f, 1.0f }, 1 },

    { {  1,  1,  1, 1 }, {0,0,0,0}, { 0, 0 }, { 1.0f, 0.2f, 1.0f }, 1 },
    { {  1, -1,  1, 1 }, {0,0,0,0}, { 0, 1 }, { 1.0f, 0.2f, 0.2f }, 1 },
    { {  1, -1, -1, 1 }, {0,0,0,0}, { 1, 1 }, { 1.0f, 1.0f, 0.2f }, 1 },
    { {  1, -1, -1, 1 }, {0,0,0,0}, { 1, 1 }, { 1.0f, 1.0f, 0.2f }, 1 },
    { {  1,  1, -1, 1 }, {0,0,0,0}, { 1, 0 }, { 1.0f, 0.2f, 0.2f }, 1 },
    { {  1,  1,  1, 1 }, {0,0,0,0}, { 0, 0 }, { 1.0f, 0.2f, 1.0f }, 1 }
};

static Vertex_t ground_mesh[6] = {
    {{ -1,  0, -1, 1 }, {0,0,0,0}, { 0, 1 }, { 1.0f, 1.0f, 1.0f }, 1},
    {{ -1,  0,  1, 1 }, {0,0,0,0}, { 1, 1 }, { 1.0f, 1.0f, 1.0f }, 1},
    {{  1,  0,  1, 1 }, {0,0,0,0}, { 1, 0 }, { 1.0f, 1.0f, 1.0f }, 1},
    
    {{  1,  0,  1, 1 }, {0,0,0,0}, { 1, 0 }, { 1.0f, 1.0f, 1.0f }, 1},
    {{  1,  0, -1, 1 }, {0,0,0,0}, { 0, 0 }, { 1.0f, 1.0f, 1.0f }, 1},
    {{ -1,  0, -1, 1 }, {0,0,0,0}, { 0, 1 }, { 1.0f, 1.0f, 1.0f }, 1}
};


void screen_update(Device_t* device) {
    glDrawPixels(device->width, device->height, GL_BGRA_EXT, GL_UNSIGNED_BYTE, device->framebuffer);
}

void draw_object(Device_t *device, Object_t* obj) {
    matrix_set_identity(&device->cam->world);
    matrix_set_rotate_translate_scale(&device->cam->world, &obj->rotate, obj->theta, &obj->pos, &obj->scale);
    camera_update(device->cam);
    camera_init_inverse_transpose_matrix(device->cam);
    
    device->texture = &obj->texture;
    
    for (int i = 0; i < obj->num_polys; ++i) {
        device_draw_poly(device, &obj->polys[i], &device->cam->inversetranspose);
    }
}

void init_texture(Texture_t* texture) {
    static u_int32_t buffer[256][256];
    int i, j;
    for (j = 0; j < 256; j++) {
        for (i = 0; i < 256; i++) {
            int x = i / 16, y = j / 16;
            buffer[j][i] = ((x + y) & 1)? 0xffffffff : 0xa1a100ff;
        }
    }
    texture_init(texture, buffer, 256, 256, 256);
}


static GLFWwindow* screen_handle = 0x0;
static Device_t device;

static void screen_error(int errorCode, const char* msg)
{
    printf("error : %s\n", msg);
}


float angle(float x1, float y1, float x2, float y2) {
    float angle_temp;
    float xx = x2 - x1;
    float yy = y2 - y1;
    if (xx == 0.0) angle_temp = M_PI / 2.0;
    else angle_temp = atan(fabs(yy / xx));
    
    if ((xx < 0.0) && (yy >= 0.0))
        angle_temp = M_PI - angle_temp;
    else if ((xx < 0.0) && (yy < 0.0))
        angle_temp = M_PI + angle_temp;
    else if ((xx >= 0.0) && (yy < 0.0))
        angle_temp = M_PI * 2.0 - angle_temp;
    return angle_temp;
}

static float fyaw = 0.f, fpitch = 0.f;
static Point_t prev_cursor_pos;
static int is_init = 0;
void screen_cursor_move(GLFWwindow* window, double posx, double posy) {
    Camera_t* cam = device.cam;
    // 第一次进入这个方法前获取不到鼠标的位置
    // 所以添加初始化的代码，设置鼠标当前位置，相机在[世界坐标中]对[观察点]的 偏航角、俯仰角
    // 当切换相机时，默认重新初始化这些数据，以达到多个相机互相不扰乱观察位置（因为使用的是公共的 fyaw、fpitch）
    if (!is_init) {
        is_init = 1;
        prev_cursor_pos.x = posx;
        prev_cursor_pos.y = posy;
        Vector_t at;
        vector_add(&at, &cam->eye, &cam->front);
        fpitch = RAD_TO_ANG(asin(cam->front.y));
        fyaw = RAD_TO_ANG(angle(cam->eye.x, cam->eye.z, at.x, at.z));
        return;
    }
    
    // 左手坐标系，向左角度增，向下角度增
    float xoffset = prev_cursor_pos.x - posx;
    float yoffset = prev_cursor_pos.y - posy;
    
    prev_cursor_pos.x = posx;
    prev_cursor_pos.y = posy;
    
    float sensitivity = 0.3f;
    xoffset *= sensitivity;
    yoffset *= sensitivity;
    
    fyaw += xoffset;
    fpitch += yoffset;
    
    fpitch = CMID(fpitch, -89.0f, 89.0f);
    
    Vector_t front;
    front.x = cos(ANG_TO_RAD(fpitch)) * cos(ANG_TO_RAD(fyaw));
    front.y = sin(ANG_TO_RAD(fpitch));
    front.z = cos(ANG_TO_RAD(fpitch)) * sin(ANG_TO_RAD(fyaw));
    camera_set_front(cam, front.x, front.y, front.z);
//    printf("x %f y %f z %f\n", front.x, front.y, front.z);
}

void screen_init(int w, int h, const char* title) {
    glfwSetErrorCallback(screen_error);
    if(!glfwInit()) assert(0);
    
    screen_handle = glfwCreateWindow(w, h, title, NULL, NULL);
    if (!screen_handle) assert(0);
    
    glViewport(0, 0, w, h);
    glfwMakeContextCurrent(screen_handle);
    glfwSetInputMode(screen_handle, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSetCursorPosCallback(screen_handle, screen_cursor_move);
    glfwSetCursorPos(screen_handle, w/2.f, h/2.f);
    
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) assert(0);
    
    // Device init
    {
        device_init(&device, w, h);
        device.render_attr = 0;
        
        Point_t pos = {0, 5, 10, 1}, pos2 = {15, 5, 0, 1}, pos3 = {15, 5, 0, 1};
        Vector_t dir1 = {1, 0.5, 0, 1}, dir2 = {0.5, 0.5, 0, 1};
        Color_t c1 = {0.4, 0.4, 0.4, 1}, c2 = {0, 0, 1, 1}, c3 = {0, 1, 1, 1}, c4 = {0.8, 0.8, 0, 1}, c5 = {0.3, 0.5, 0.8};
        device_new_light(&device, LIGHT_ATTR_AMBIENT, NULL, NULL, &c1, 0, 0, 0, 0);
        device_new_light(&device, LIGHT_ATTR_SPOTLIGHT, &pos3, &dir1, &c5, 0, 0.03, 0, 50);
//        device_new_light(&device, LIGHT_ATTR_INFINITE, NULL, &dir1, &c2, 0, 0, 0);
        device_new_light(&device, LIGHT_ATTR_POINT, &pos, NULL, &c3, 0, 0.05, 0, 0);
        device_new_light(&device, LIGHT_ATTR_POINT, &pos2, NULL, &c4, 0, 0.05, 0, 0);
        
    }
    
    // Camera init
    Camera_t cam1, cam2;
    {
        camera_init(&cam1, M_PI * 0.3f, (float)w / h, 0.1f, 1000.f);
        camera_set_eye(&cam1, 8, 3.5, 0);
        device_insert_camera(&device, &cam1);
        
        camera_init(&cam2, M_PI * 0.3f, (float)w / h, 1.f, 1000.f);
        camera_set_eye(&cam2, 15.f, 10.f, 0.f);
        device_insert_camera(&device, &cam2);
        
        device_activate_camera(&device, 0);
    }
    
    // Object init
    Object_t obj_box;
    {
        object_init(&obj_box, 8, 12);
        obj_box.render_state = RENDER_STATE_TEXTURE;
        for (int i = 0; i < obj_box.num_polys; ++i) {
            Poly_t* poly = &obj_box.polys[i];
            poly->attr = obj_box.render_state;
            for (int vertex = 0; vertex < 3; ++vertex) {
                vertex_copy(&poly->vlist[vertex], &box_mesh[3 * i + vertex]);
            }
            poly_compute_vertex_normal(poly);
        }
        object_set_scale(&obj_box, 1, 1, 1);
        object_set_rotate(&obj_box, 0, 1, 0, ANG_TO_RAD(90.f));
        object_set_position(&obj_box, 0, 2, 0);
        init_texture(&obj_box.texture);
    }
    
    Object_t obj_ground;
    {
        object_init(&obj_ground, 6, 2);
        obj_ground.render_state = RENDER_STATE_TEXTURE;
        for (int i = 0; i < obj_ground.num_polys; ++i) {
            Poly_t* poly = &obj_ground.polys[i];
            poly->attr = obj_ground.render_state;
            for (int vertex = 0; vertex < 3; ++vertex) {
                vertex_copy(&poly->vlist[vertex], &ground_mesh[3 * i + vertex]);
            }
            poly_compute_vertex_normal(poly);
        }
        object_set_scale(&obj_ground, 15, 15, 15);
        object_set_rotate(&obj_ground, 0, 0, 0, ANG_TO_RAD(0.f));
        object_set_position(&obj_ground, 0, 0, 0);
        init_texture(&obj_ground.texture);
        texture_load(&obj_ground.texture, "/Users/yucong/Library/Developer/Xcode/DerivedData/XDoc-dzaryrigotkspgdukxoxekhmsknv/Build/Products/Debug/timg.jpg");
    }
    
    Object_t obj_ball;
    {
        object_create_ball(&obj_ball, 10);
        object_set_scale(&obj_ball, 1, 1, 1);
        object_set_rotate(&obj_ball, 0, 1, 0, ANG_TO_RAD(0.f));
        object_set_position(&obj_ball, -10, 7, 0);
    }
    
    Object_t obj_nanosuit;
    {
        object_load_obj(&obj_nanosuit, "/Users/yucong/Library/Developer/Xcode/DerivedData/XDoc-dzaryrigotkspgdukxoxekhmsknv/Build/Products/Debug/nanosuit.obj");
        object_set_scale(&obj_nanosuit, 0.3, 0.3, 0.3);
        object_set_rotate(&obj_nanosuit, 0, 1, 0, ANG_TO_RAD(90.f));
        object_set_position(&obj_nanosuit, 0, 0, 0);
    }
    
    
    float theta = 0.f;
    Vector_t tempv;
    float l = 0.001;
    while (!glfwWindowShouldClose(screen_handle)) {
        device_clear(&device);
        object_reset(&obj_box);
        object_reset(&obj_ground);
        object_reset(&obj_ball);
        object_reset(&obj_nanosuit);
        
        // 相机 2 绕0点旋转
        theta = theta + 0.1f;
        if (theta >= 360.f) theta = 0.f;
        float c2x = 10.f * cosf(theta);
        float c2z = 10.f * sinf(theta);
        camera_set_eye(&cam2, c2x, 20, c2z);
        camera_set_front(&cam2, -c2x, -3, -c2z);
        
//        vector_set(&obj_nanosuit.pos, c2x, 0, c2z);
//        obj_nanosuit.theta = angle(cam2.front.x, cam2.front.z, c2x, c2z);
        vector_copy(&device.light[1].pos, &cam1.eye);
        vector_set(&device.light[1].dir, -cam1.front.x, -cam1.front.y, -cam1.front.z);
        
        if (glfwGetKey(screen_handle, GLFW_KEY_W)) {
            vector_copy(&tempv, &cam1.front);
            vector_scale(&tempv, 0.3);
            vector_add(&tempv, &cam1.eye, &tempv);
            camera_set_eye(&cam1, tempv.x, tempv.y, tempv.z);
        }
        else if (glfwGetKey(screen_handle, GLFW_KEY_S)) {
            vector_copy(&tempv, &cam1.front);
            vector_scale(&tempv, 0.3);
            vector_sub(&tempv, &cam1.eye, &tempv);
            camera_set_eye(&cam1, tempv.x, tempv.y, tempv.z);
        }
        if (glfwGetKey(screen_handle, GLFW_KEY_A)) {
            vector_crossproduct(&tempv, &cam1.up, &cam1.front);
            vector_normalize(&tempv);
            vector_scale(&tempv, 0.3);
            vector_sub(&tempv, &cam1.eye, &tempv);
            camera_set_eye(&cam1, tempv.x, tempv.y, tempv.z);
        }
        else if (glfwGetKey(screen_handle, GLFW_KEY_D)) {
            vector_crossproduct(&tempv, &cam1.up, &cam1.front);
            vector_normalize(&tempv);
            vector_scale(&tempv, 0.3);
            vector_add(&tempv, &cam1.eye, &tempv);
            camera_set_eye(&cam1, tempv.x, tempv.y, tempv.z);
        }
        if (glfwGetKey(screen_handle, GLFW_KEY_Q)) {
            device_activate_camera(&device, 1 - device.cur_cam);
            is_init = 0;
        }
        
        for (int i = 0; i < device.num_cam; ++i) {
            camera_update(device.cams[i]);
        }
        
//        draw_object(&device, &obj_box);
        draw_object(&device, &obj_ground);
//        draw_object(&device, &obj_ball);
        draw_object(&device, &obj_nanosuit);
//        obj_box.theta = obj_box.theta + 0.05f;
//        obj_ball.theta = obj_ball.theta + 0.1f;
//        obj_nanosuit.theta = obj_nanosuit.theta + 0.1f;
//        draw_box(&device, alpha);
//        device_draw_line(&device, 100, 100, 200, 100, 0x00ff00);
        screen_update(&device);
        
        glfwSwapBuffers(screen_handle);
        glfwPollEvents();
        if(glfwGetKey(screen_handle, GLFW_KEY_ESCAPE))
            glfwSetWindowShouldClose(screen_handle, GL_TRUE);
        
        usleep(1.0 / 30 * 1000000);
    }
    
    device_destroy(&device);
//    texture_destroy(&obj_box.texture);
//    texture_destroy(&obj_ground.texture);
    
    glfwTerminate();
}

int main_mini3d() {
    screen_init(640, 480, "mini3d");
    return 0;
}

