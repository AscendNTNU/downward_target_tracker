// so_math.h - ver 27
// + Vector and matrix math.
// + Linear algebra.
// + GLSL like functions
// + Euclidean transformations.
// + Conversions between Euler angles, quaternions and matrices
// + Representations of SE(3) and SO(3)
// + Conversions between SO(3) and so(3)
// + Conversions between SE(3) and se(3)
//
// :::::::::::::::::::::::::Changelog::::::::::::::::::::::::::
//   5/4/17: m_solvespd without templates
//  20/3/17: m_intersect_ray_ray, m_intersect_ray_flat_cone
//  12/3/17: Removed projections.
//  11/3/17: m_orthogonalize. [] accessors.
//
//  26/2/17: Camera math, inverse and forward projections.
//           Got rid of T type template.
//
//  5/12/16: m_so3_log bounds check on acos
//
//  1/10/16: fixed m_ortho
//
//  15/9/16: removed undefined types (s32, r32)
//
//  24/7/16: m_floor, m_mod
//
//  17/7/16: m_normalize no longer defaults to using fast_inv_sqrt.
//
//  15/7/16: m_map: Now works with y1 < y0
//
//   8/7/16: m_so3_to_ypr: Rotation matrix to euler angles (YPR)
//
//   7/7/16: m_quat_to_ypr: Quaternion to euler angles (YPR)
//
//   1/7/16: m_solvespd: Solve Sx=b where S is symmetric positive definite
//           m_cholesky: Decompose a symmetric positive definite matrix
//                       S=U'U, where U is upper-triangular
//
//  29/6/16: Added ability to disable PI, TWO_PI define.
//
//   3/6/16: Circle-circle intersection test: m_is_circle_circle
//
//   4/1/16: m_smoothstep
//
// 11/12/15: Fixed sign convention on the rotation matrices in
//           mat_rotate_*. They now follow the right-hand rule:
//           Positive angle means counter-clockwise rotation
//           about the respective axis.
//
//  6/12/15: m_sign
//
// 26/11/15: Made it a template library after reading
//           http://www.reedbeta.com/blog/2013/12/28/on-vector-math-libraries/
//           I agree on a lot of the points. I do however not
//           like templates in general, due to poor compiler
//           error messages. Kind of unsure if this was a good
//           idea or not. I don't really use vectors or matrices
//           or dimension greater than 4x4; but I do sometimes
//           use matrices of odd dimensions.
//
// Some earlier entries have been omitted.

#ifndef SO_MATH_HEADER_INCLUDE
#define SO_MATH_HEADER_INCLUDE
#include "math.h"

#define SO_PI     3.14159265359f
#define SO_TWO_PI 6.28318530718f

#ifndef SO_MATH_NO_PI
#define PI     3.14159265359f
#define TWO_PI 6.28318530718f
#endif

template <int n>
struct m_vector_t
{
    float data[n];
    float &operator[](int i) { return data[i]; }
};

template <int rows, int cols>
struct m_matrix_t
{
    float data[rows*cols];
    float &operator[](int i) { return data[i]; }
    float &operator()(int row, int col) { return data[row + col*rows]; }
};

typedef m_vector_t<2> vec2;
typedef m_vector_t<3> vec3;
typedef m_vector_t<4> vec4;
typedef m_matrix_t<2, 2> mat2;
typedef m_matrix_t<3, 3> mat3;
typedef m_matrix_t<4, 4> mat4;

///////////////// m_vector_t Specializations /////////////////
// Specializations for n = 2, 3, 4 so that we can access each
// component of a vector through .a notation, as well as access
// truncated vectors.

template <> struct m_vector_t<2>
{
    union
    {
        float data[2];
        struct { float x, y; };
    };
    float &operator[](int i) { return data[i]; }
};

template <> struct m_vector_t<3>
{
    union
    {
        float data[3];
        struct { float x, y, z; };
        struct { float r, g, b; };
        m_vector_t<2> xy;
    };
    float &operator[](int i) { return data[i]; }
};

template <> struct m_vector_t<4>
{
    union
    {
        float data[4];
        struct { float x, y, z, w; };
        struct { float r, g, b, a; };
        m_vector_t<3> xyz;
        m_vector_t<3> rgb;
        m_vector_t<2> xy;
    };
    float &operator[](int i) { return data[i]; }
};

///////////////// m_matrix_t Specializations /////////////////
// Specializations for m = n = 2, 3, 4 so that we can access
// each column of a matrix through .a notation.

// The data layout for each matrix is interpreted as column
// order. For example
// | a11 a12 |
// | a21 a22 |
// is stored in memory as [a11, a21, a12, a22].
// The columns of the matrix can be accessed via .a1 or .a2.

template <> struct m_matrix_t<2, 2>
{
    union
    {
        float data[4];
        struct { float a11, a21, a12, a22; };
        struct { m_vector_t<2> a1, a2; };
    };
    float &operator[](int i) { return data[i]; }
};

template <> struct m_matrix_t<3, 3>
{
    union
    {
        float data[9];
        struct { float a11, a21, a31, a12, a22, a32, a13, a23, a33; };
        struct { m_vector_t<3> a1, a2, a3; };
    };
    float &operator[](int i) { return data[i]; }
};

template <> struct m_matrix_t<4, 4>
{
    union
    {
        float data[16];
        struct { float a11, a21, a31, a41, a12, a22, a32, a42, a13, a23, a33, a43, a14, a24, a34, a44; };
        struct { m_vector_t<4> a1, a2, a3, a4; };
    };
    float &operator[](int i) { return data[i]; }
};

//////////// m_matrix_t and vector constructors ////////////
// Convenience functions for constructing matrices and
// vectors, from components or other vectors.

vec2 m_vec2(float s)                            { vec2 result = { s, s       }; return result; }
vec3 m_vec3(float s)                            { vec3 result = { s, s, s    }; return result; }
vec4 m_vec4(float s)                            { vec4 result = { s, s, s, s }; return result; }
vec2 m_vec2(float x, float y)                   { vec2 result = { x, y       }; return result; }
vec3 m_vec3(float x, float y, float z)          { vec3 result = { x, y, z    }; return result; }
vec4 m_vec4(float x, float y, float z, float w) { vec4 result = { x, y, z, w }; return result; }
vec4 m_vec4(vec3 xyz, float w)                  { vec4 result = { xyz.x, xyz.y, xyz.z, w }; return result; }

template <int n>
m_matrix_t<n, n> m_identity_()
{
    m_matrix_t<n, n> result = { };
    for (unsigned int i = 0; i < n; i++)
        result.data[i*n+i] = 1;
    return result;
}

m_matrix_t<2, 2> m_mat2(m_vector_t<2> a1, m_vector_t<2> a2)
{
    mat2 result = { };
    result.a1 = a1;
    result.a2 = a2;
    return result;
}

m_matrix_t<3, 3> m_mat3(m_vector_t<3> a1, m_vector_t<3> a2, m_vector_t<3> a3)
{
    mat3 result = { };
    result.a1 = a1;
    result.a2 = a2;
    result.a3 = a3;
    return result;
}

m_matrix_t<3, 3> m_mat3(m_matrix_t<4, 4> m)
{
    mat3 result = { };
    result.a1 = m.a1.xyz;
    result.a2 = m.a2.xyz;
    result.a3 = m.a3.xyz;
    return result;
}

m_matrix_t<4, 4> m_mat4(m_vector_t<4> a1, m_vector_t<4> a2, m_vector_t<4> a3, m_vector_t<4> a4)
{
    mat4 result = { };
    result.a1 = a1;
    result.a2 = a2;
    result.a3 = a3;
    result.a4 = a4;
    return result;
}

m_matrix_t<4, 4> m_mat4(m_matrix_t<3, 3> m)
{
    mat4 result = m_identity_<4>();
    result.a1.xyz = m.a1;
    result.a2.xyz = m.a2;
    result.a3.xyz = m.a3;
    result.a4.xyz = m_vec3(0,0,0);
    return result;
}

#define m_id2 m_identity_<2>
#define m_id3 m_identity_<3>
#define m_id4 m_identity_<4>

///////////////// m_matrix_t functions /////////////////
template <int r, int c>
float *m_element(m_matrix_t<r, c> *m, int row, int column)
{
    return m->data + row + column * r;
}

template <int r, int c>
m_vector_t<r> m_column(m_matrix_t<r, c> m, int column)
{
    m_vector_t<r> result = {};
    for (int i = 0; i < r; i++)
        result.data[i] = m.data[i + column * r];
    return result;
}

template <int ra, int ca, int cb>
m_matrix_t<ra, cb> operator *(m_matrix_t<ra, ca> a, m_matrix_t<ca, cb> b)
{
    m_matrix_t<ra, cb> result = {};
    float *entry = (float*)result.data;
    for (int col = 0; col < cb; col++)
    for (int row = 0; row < ra; row++)
    {
        for (int i = 0; i < ca; i++)
        {
            float x = a.data[i * ra + row];
            float y = b.data[i + col * ca];
            *entry += x * y;
        }
        entry++;
    }
    return result;
}

template <int r, int c>
m_matrix_t<r, c> operator +(m_matrix_t<r, c> a, m_matrix_t<r, c> b)
{
    m_matrix_t<r, c> result = {};
    for (int i = 0; i < r*c; i++) result.data[i] = a.data[i] + b.data[i];
    return result;
}

template <int r, int c>
m_matrix_t<r, c> operator -(m_matrix_t<r, c> a, m_matrix_t<r, c> b)
{
    m_matrix_t<r, c> result = {};
    for (int i = 0; i < r*c; i++) result.data[i] = a.data[i] - b.data[i];
    return result;
}

template <int r, int c>
m_matrix_t<r, c> operator *(m_matrix_t<r, c> a, float s)
{
    m_matrix_t<r, c> result = {};
    for (int i = 0; i < r*c; i++) result.data[i] = a.data[i] * s;
    return result;
}

template <int r, int c>
m_matrix_t<r, c> operator -(m_matrix_t<r, c> a)
{
    m_matrix_t<r, c> result = {};
    for (int i = 0; i < r*c; i++) result.data[i] = -a.data[i];
    return result;
}

template <int r, int c>
m_matrix_t<r, c> operator *(float s, m_matrix_t<r, c> a)
{
    m_matrix_t<r, c> result = {};
    for (int i = 0; i < r*c; i++) result.data[i] = a.data[i] * s;
    return result;
}

template <int r, int c>
m_matrix_t<c, r> m_transpose(m_matrix_t<r, c> m)
{
    m_matrix_t<c, r> result = {};
    for (int row = 0; row < r; row++)
    for (int col = 0; col < c; col++)
    {
        float *a = m_element(&result, col, row);
        float *b = m_element(&m, row, col);
        *a = *b;
    }
    return result;
}

///////////////// m_vector_t functions /////////////////
// Let a := vector of dimension N and type T
//     b := another vector of dimension N and type T
//     s := scalar of same type T
//
// Then the following operators are defined
//
//     -A        :: Component-wise negation of A
//     A+B = B+A :: B added component-wise to A
//     A-B       :: B subtracted component-wise from A
//     A*s = s*A :: s multiplied each component of A
//     A*B = B*A :: A multiplied component-wise by B
//     A/B       :: A divided component-wise by B
//     dot(A, B) :: The inner product of A and B
#define vec_template template <int n>
#define tvec m_vector_t<n>

vec_template tvec  operator+(tvec a, tvec b)    { tvec r={0}; for (int i=0; i<n; i++) r.data[i] = a.data[i]+b.data[i]; return r; }
vec_template tvec  operator-(tvec a, tvec b)    { tvec r={0}; for (int i=0; i<n; i++) r.data[i] = a.data[i]-b.data[i]; return r; }
vec_template tvec  operator-(tvec a)            { tvec r={0}; for (int i=0; i<n; i++) r.data[i] = -a.data[i];          return r; }
vec_template tvec  operator*(tvec v, float s)   { tvec r={0}; for (int i=0; i<n; i++) r.data[i] = v.data[i]*s;         return r; }
vec_template tvec  operator*(float s, tvec v)   { tvec r={0}; for (int i=0; i<n; i++) r.data[i] = v.data[i]*s;         return r; }
vec_template tvec  operator*(tvec a, tvec b)    { tvec r={0}; for (int i=0; i<n; i++) r.data[i] = a.data[i]*b.data[i]; return r; }
vec_template tvec  operator/(tvec a, tvec b)    { tvec r={0}; for (int i=0; i<n; i++) r.data[i] = a.data[i]/b.data[i]; return r; }
vec_template tvec  operator/(tvec v, float s)   { tvec r={0}; for (int i=0; i<n; i++) r.data[i] = v.data[i]/s;         return r; }
vec_template tvec &operator+=(tvec &a, tvec b)  { for (int i = 0; i < n; i++) a.data[i] += b.data[i]; return a; }
vec_template tvec &operator-=(tvec &a, tvec b)  { for (int i = 0; i < n; i++) a.data[i] -= b.data[i]; return a; }
vec_template tvec &operator*=(tvec &v, float s) { for (int i = 0; i < n; i++) v.data[i] *= s;         return v; }
vec_template tvec &operator*=(tvec &a, tvec b)  { for (int i = 0; i < n; i++) a.data[i] *= b.data[i]; return a; }
vec_template tvec &operator/=(tvec &a, tvec b)  { for (int i = 0; i < n; i++) a.data[i] /= b.data[i]; return a; }
vec_template tvec &operator/=(tvec &v, float s) { for (int i = 0; i < n; i++) v.data[i] /= s;         return v; }

vec_template float m_dot(tvec a, tvec b)
{
    float result = 0;
    for (int i = 0; i < n; i++)
        result += a.data[i] * b.data[i];
    return result;
}

#undef vec_template
#undef tvec

///////////// m_vector_t matrix functions /////////////
template <int r, int c>
m_vector_t<r> operator *(m_matrix_t<r, c> m, m_vector_t<c> x)
{
    m_vector_t<r> result = {};
    for (int col = 0; col < c; col++)
    for (int row = 0; row < r; row++)
        result.data[row] += m.data[row + col * r] * x.data[col];
    return result;
}

mat3 m_outer_product(vec3 a, vec3 b)
{
    mat3 result;
    result.a11 = a.x*b.x; result.a12 = a.x*b.y; result.a13 = a.x*b.z;
    result.a21 = a.y*b.x; result.a22 = a.y*b.y; result.a23 = a.y*b.z;
    result.a31 = a.z*b.x; result.a32 = a.z*b.y; result.a33 = a.z*b.z;
    return result;
}

// Because I like readable error messages when there is a
// dimension mismatch in the most common operations.
vec2 operator *(mat2 m, vec2 b) { return m.a1*b.x + m.a2*b.y; }
vec3 operator *(mat3 m, vec3 b) { return m.a1*b.x + m.a2*b.y + m.a3*b.z; }
vec4 operator *(mat4 m, vec4 b) { return m.a1*b.x + m.a2*b.y + m.a3*b.z + m.a4*b.w; }

template <int n>
float m_length(m_vector_t<n> v)
{
    float result = sqrtf(m_dot(v, v));
    return result;
}

float m_fast_inv_sqrt(float x)
{
    float xhalf = 0.5f * x;
    int i = *(int*)&x;          // Integer representation of float
    i = 0x5f3759df - (i >> 1);  // Initial guess
    x = *(float*)&i;            // Converted to floating point
    x = x*(1.5f-(xhalf*x*x));   // One round of Newton-Raphson's method
    return x;
}

template <int n>
m_vector_t<n> m_normalize(m_vector_t<n> v)
{
    m_vector_t<n> result = v / m_length(v);
    return result;
}

///////////////// GLSL-like stuff ////////////////
int m_sign(int x)       { return x < 0 ? -1 : 1; }
int m_abs(int x)        { return x < 0 ? -x : x; }
int m_min(int x, int y) { return x < y ? x : y; }
int m_max(int x, int y) { return x > y ? x : y; }
int m_clamp(int x, int low, int high)
{
    return x < low ? low : (x > high ? high : x);
}

float m_sign(float x)         { return x < 0 ? -1.0f : +1.0f; }
float m_abs(float x)          { return x < 0 ? -x : x; }
float m_min(float x, float y) { return x < y ? x : y; }
float m_max(float x, float y) { return x > y ? x : y; }
float m_clamp(float x, float low, float high)
{
    return x < low ? low : (x > high ? high : x);
}

float m_square(float x) { return x*x; }

// return: Linear mapping from [t0, t1] to [y0, y1]
float m_map(float t0, float t1, float t, float y0, float y1)
{
    if (y0 > y1) return m_clamp(y0 + (y1 - y0) * (t - t0) / (t1 - t0), y1, y0);
    else         return m_clamp(y0 + (y1 - y0) * (t - t0) / (t1 - t0), y0, y1);
}

// return: Linear mapping from [0, 1] to [low, high]
float m_mix(float low, float high, float t)
{
    return low + (high - low) * t;
}

float m_smoothstep(float lo, float hi, float x)
{
    float t = m_clamp((x - lo) / (hi - lo), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

float m_floor(float x)
{
    return floor(x);
}

float m_mod(float x, float m)
{
    if (x < 0.0f)
    {
        int n = (int)(-x / m);
        float r = m - (-x - n*m);
        return r;
    }
    else
    {
        int n = (int)(x / m);
        float r = x - n*m;
        return r;
    }
}

///////////////// Linear algebra /////////////////
vec3 m_cross(vec3 a, vec3 b)
{
    return m_vec3(a.y*b.z-a.z*b.y,
                  a.z*b.x-a.x*b.z,
                  a.x*b.y-a.y*b.x);
}

// return: The skew-symmetric matrix form of the
//         cross product operator applied by v.
mat3 m_skew(vec3 v)
{
    mat3 result = {0, v.z, -v.y, -v.z, 0, v.x, v.y, -v.x, 0};
    return result;
}

// return: A vector that is orthogonal to v.
//         - Always works if the input is non-zero.
//         - Doesn’t require the input to be normalised.
//         - Doesn’t normalize the output.
// thanks: http://lolengine.net/blog/2013/09/21/picking-orthogonal-vector-combing-coconuts
vec3 m_orthogonal_vector(vec3 v)
{
    return m_abs(v.x) > m_abs(v.z) ? m_vec3(-v.y, v.x, 0.0)
                                   : m_vec3(0.0, -v.z, v.y);
}

// return: A transformation matrix in SE3 (rotation and translation)
mat4 m_se3(mat3 R, vec3 r)
{
    mat4 result = m_id4();
    result.a1.xyz = R.a1;
    result.a2.xyz = R.a2;
    result.a3.xyz = R.a3;
    result.a4.xyz = r;
    return result;
}

void m_se3_decompose(mat4 se3, mat3 *R, vec3 *p)
{
    *R = m_mat3(se3);
    *p = se3.a4.xyz;
}

// return: The inverse of a SO3 matrix
mat4 m_se3_inverse(mat4 m)
{
    mat3 R = m_transpose(m_mat3(m));
    vec3 r = -R * m.a4.xyz;
    return m_se3(R, r);
}

int m_cholesky(float *S, int n, float *b, float *U, float *y)
/*
Decompose the matrix S into S = U^t U, where U is upper triangular.
S must be symmetric positive definite.

Since this operation is often performed in conjunction with solving
the matrix equation Sx = b, you can specify vectors b and y satisfying

    Sx = U^t Ux = U^t y = b

and therefore

    Ux = y
    y = U^-t b

With y, you can easily solve for x by successively solving for the
unknown components of x, starting from the bottom row.

The function returns 0 if S is non-positive definite, and 1 with
successful completion.

Arguments:

S (input) nxn matrix to decompose
b (input) nx1 right-hand-side of the equation Sx = b
U (output) nxn upper triangular Cholesky decomposition matrix
y (output) nx1 solution of U^t y = b
*/
{
    int i, j, k;

    for (i=0; i<n; i++)
    {
        U[i*n+i] = S[i*n+i];
        for (k=0; k<i; k++)
            U[i*n+i] -= U[i*n+k]*U[i*n+k];

        if (U[i*n+i] <= 0) {
            return 0; // m_matrix_t was not symmetric-positive-definite
                      // Maybe consider a damped inverse?
        }

        U[i*n+i] = sqrt(U[i*n+i]);

        // This portion multiplies the extra matrix by C^-t
        y[i] = b[i];
        for (k=0; k<i; k++)
        {
            y[i] -= y[k]*U[i*n+k];
        }
        y[i] /= U[i*n+i];

        for (j=i+1; j<n; j++)
        {
            U[j*n+i] = S[j*n+i];
            for (k=0; k<i; k++)
                U[j*n+i] -= U[i*n+k]*U[j*n+k];
            U[j*n+i] /= U[i*n+i];
        }
    }
    return 1;
}

void m_solveut(float *U, float *x, float *y, int n)
// Solves Ux = y for x where U is upper triangular.
// U (input) nxn upper triangular matrix (1d column-order)
// y (input) nx1 vector of known right-hand-side values
// x (output) nx1 vector of unknowns
// n (input)
{
    for (int i = n-1; i >= 0; i--)
    {
        float r = 0.0f;
        for (int j = n-1; j > i; j--)
        {
            r += U[j*n+i] * x[j];
        }
        x[i] = (y[i] - r) / U[i*n+i];
    }
}

int m_solvespd(int n, float *S, float *b, float *x, float *U, float *y)
// Solves S x = b where S is symmetric positive definite.
// S (input) nxn symmetric positive definite matrix (1d column-order)
// b (input) nx1 right hand side
// x (output) nx1 solution
// U (buffer) nxn matrix (you allocate this)
// y (buffer) nx1 vector (you allocate this)
// Returns 1 with result on success
//         0 if S was not symmetric positive definite
{
    if (m_cholesky(S, n, b, U, y) == 1)
    {
        m_solveut(U, x, y, n);
        return 1;
    }
    else
    {
        return 0;
    }
}

template <int n>
int m_solvespd(m_matrix_t<n, n> S, m_vector_t<n> b, m_vector_t<n> *x)
// Solves S x = b where S is symmetric positive definite.
// S (input) nxn symmetric positive definite matrix (1d column-order)
// b (input) nx1 right hand side
// x (output) nx1 solution
// Returns 1 with result on success
//         0 if S was not symmetric positive definite
{
    m_matrix_t<n, n> U;
    m_vector_t<n> y;
    return m_solvespd(n, S.data, b.data, x->data, U.data, y.data);
}

bool m_so3_to_ypr(mat3 R, float *yaw, float *pitch, float *roll)
// Outputs the Euler angles (roll, pitch, yaw) that parametrize
// the zyx rotation matrix R = Rz(yaw)Ry(pitch)Rx(roll).
//
// RETURN VALUE
//   FALSE : If pitch is sufficently close to +-pi/2
//           (where "sufficiently close" is defined below)
//           If the quaternion is sufficently far from unit length
//    TRUE : Otherwise
//
// NOTES
// The factorization returned is not unique. If the pitch is close to +-pi/2,
// we choose an arbitrary factorization. Otherwise, there are two solutions
// involving two values of pitch. We always choose the one closest to zero.
//
// REFERENCES
// [1] Computing Euler angles from a rotation matrix, Slabaugh
//     http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf
{
    float R11 = R.a11;
    float R21 = R.a21;
    float R31 = R.a31;
    float R32 = R.a32;
    float R33 = R.a33;

    // The remaining procedure follows the paper [1] assuming
    // that we want the solution for pitch closest to zero.

    // R31 = sin(pitch). So we can compare R31 with a threshold
    // to determine if the pitch angle is close to pi/2. If angles
    // greater than X are to be considered "close to pi/2", then
    // the threshold is sin(X)
    if (R31 >= 0.9998f) // Pitch is 1 degree or less close to +pi/2
    {
        *yaw = 0.0f;
        *pitch = -SO_PI / 2.0f;
        *roll = atan2f(R32, R33);
        return false;
    }
    else if (R31 <= -0.9998f) // Pitch is 1 degree or less close to -pi/2
    {
        *yaw = 0.0f;
        *pitch = SO_PI / 2.0f;
        *roll = atan2f(R32, R33);
        return false;
    }
    else
    {
        *yaw = atan2f(R21, R11);
        *pitch = -asinf(R31);
        *roll = atan2f(R32, R33);
        return true;
    }
}

///////////////// Unit quaternions /////////////////
// Quaternions are represented as a vec4, with the
// xyz components representing the imaginary part
// and w component representing the real part.
typedef vec4 quat;

// return: The quaternion describing the rotation of
// _angle_ radians about a normalized axis _axis_.
quat m_quat_from_angle_axis(vec3 axis, float angle)
{
    quat result = {};
    float s = sinf(angle/2.0f);
    float c = cosf(angle/2.0f);
    result.x = axis.x*s;
    result.y = axis.y*s;
    result.z = axis.z*s;
    result.w = c;
    return result;
}

// return: A quaternion describing the rotation
// represented by the given Euler angles that
// parametrize the rotation matrix given by
//   R = Rz(ez)Ry(ey)Rx(ex)
quat m_quat_from_euler(float ex, float ey, float ez)
{
    // Implements Shepperd's method
    // See Handbook of Marine Craft Hydrodynamics and Motion Control, Fossen
    //     Page 32, section 2.2.3, Shepperd's method.
    float cz = cosf(ez); float sz = sinf(ez);
    float cy = cosf(ey); float sy = sinf(ey);
    float cx = cosf(ex); float sx = sinf(ex);

    float R11 = cy*cz;
    float R12 = cz*sx*sy - cx*sz;
    float R13 = sx*sz + cx*cz*sy;
    float R21 = cy*sz;
    float R22 = cx*cz + sx*sy*sz;
    float R23 = cx*sy*sz - cz*sx;
    float R31 = -sy;
    float R32 = cy*sx;
    float R33 = cx*cy;

    float T = R11 + R22 + R33;

    // Find maximum of T, R11, R22, and R33
    float rii; int i;
    {
        rii = T; i = 0;
        if (R11 > rii) { rii = R11; i = 1; }
        if (R22 > rii) { rii = R22; i = 2; }
        if (R33 > rii) { rii = R33; i = 3; }
    }

    float z0,z1,z2,z3;

    if (i == 0)
    {
        z0 = sqrtf(1.0f + 2.0f*rii - T);
        z1 = (R32 - R23) / z0;
        z2 = (R13 - R31) / z0;
        z3 = (R21 - R12) / z0;
    }
    else if (i == 1)
    {
        z1 = sqrtf(1.0f + 2.0f*rii - T);
        z0 = (R32 - R23) / z1;
        z2 = (R21 + R12) / z1;
        z3 = (R13 + R31) / z1;
    }
    else if (i == 2)
    {
        z2 = sqrtf(1.0f + 2.0f*rii - T);
        z0 = (R13 - R31) / z2;
        z1 = (R21 + R12) / z2;
        z3 = (R32 + R23) / z2;
    }
    else
    {
        z3 = sqrtf(1.0f + 2.0f*rii - T);
        z0 = (R21 - R12) / z3;
        z1 = (R13 + R31) / z3;
        z2 = (R32 + R23) / z3;
    }

    quat result;
    result.xyz = 0.5f * m_vec3(z1, z2, z3);
    result.w = 0.5f* z0;
    return result;
}

// return: The SO3 rotation matrix of q
mat3 m_quat_to_so3(quat q)
{
    mat3 e_skew = m_skew(q.xyz);
    mat3 result = m_id3() + 2.0f*q.w*e_skew + 2.0f*e_skew*e_skew;
    return result;
}

bool m_quat_to_ypr(float qx, float qy, float qz, float qw,
                   float *yaw, float *pitch, float *roll)
// Outputs the Euler angles (roll, pitch, yaw) that parametrize
// the zyx rotation matrix R = Rz(yaw)Ry(pitch)Rx(roll), where
// R is the rotation matrix form of q = (qx, qy, qz, qw).
//
// RETURN VALUE
//   FALSE : If pitch is sufficently close to +-pi/2
//           (where "sufficiently close" is defined below)
//           If the quaternion is sufficently far from unit length
//    TRUE : Otherwise
//
// NOTES
// The factorization returned is not unique. If the pitch is close to +-pi/2,
// we choose an arbitrary factorization. Otherwise, there are two solutions
// involving two values of pitch. We always choose the one closest to zero.
//
// REFERENCES
// [1] Computing Euler angles from a rotation matrix, Slabaugh
//     http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf
{
    // First: Convert quaternion to rotation matrix
    // We only care about a handful of the elements,
    // so we only compute those. We also orthogonalize
    // the matrix by normalizing the elements in the
    // process.
    //     R11 R12 R13
    // R = R21 R22 R23
    //     R31 R32 R33
    float d = qx*qx+qy*qy+qz*qz+qw*qw;
    if (d <= 0.00001f)
    {
        // Unit quaternions should be close to unit length.
        // If the length is this small, something has gone
        // wrong.
        return false;
    }
    float s   = 2.0f / d;
    float R11 = 1.0f - (qy*qy + qz*qz)*s;
    float R21 =        (qx*qy + qz*qw)*s;
    float R31 =        (qx*qz - qy*qw)*s;
    float R32 =        (qy*qz + qx*qw)*s;
    float R33 = 1.0f - (qx*qx + qy*qy)*s;

    // The remaining procedure follows the paper [1] assuming
    // that we want the solution for pitch closest to zero.

    // R31 = sin(pitch). So we can compare R31 with a threshold
    // to determine if the pitch angle is close to pi/2. If angles
    // greater than X are to be considered "close to pi/2", then
    // the threshold is sin(X)
    if (R31 >= 0.9998f) // Pitch is 1 degree or less close to +pi/2
    {
        *yaw = 0.0f;
        *pitch = -SO_PI / 2.0f;
        *roll = atan2(R32, R33);
        return false;
    }
    else if (R31 <= -0.9998f) // Pitch is 1 degree or less close to -pi/2
    {
        *yaw = 0.0f;
        *pitch = SO_PI / 2.0f;
        *roll = atan2(R32, R33);
        return false;
    }
    else
    {
        *yaw = atan2(R21, R11);
        *pitch = -asin(R31);
        *roll = atan2(R32, R33);
        return true;
    }
}

// return: The quaternion product q MUL r
quat m_quat_mul(quat q, quat r)
{
    quat result = {};
    result.w = q.w*r.w - m_dot(q.xyz, r.xyz);
    result.xyz = q.w*r.xyz + r.w*q.xyz + m_skew(q.xyz)*r.xyz;
    return result;
}

// return: The matrix which when left-multiplied by a vector v=(x,y,z)
// produces the same result as m_quat_mul(q, m_vec4(v, 0)).
m_matrix_t<4,3> m_quat_mul_matrix(quat q)
{
    m_matrix_t<4,3> result = {};
    *m_element(&result, 0, 0) = +q.w;
    *m_element(&result, 1, 0) = +q.z;
    *m_element(&result, 2, 0) = -q.y;
    *m_element(&result, 3, 0) = -q.x;

    *m_element(&result, 0, 1) = -q.z;
    *m_element(&result, 1, 1) = +q.w;
    *m_element(&result, 2, 1) = +q.x;
    *m_element(&result, 3, 1) = -q.y;

    *m_element(&result, 0, 2) = +q.y;
    *m_element(&result, 1, 2) = -q.x;
    *m_element(&result, 2, 2) = +q.w;
    *m_element(&result, 3, 2) = -q.z;
    return result;
}

///////////////// SO(3)/SE(3) maps /////////////////
// Define w to be a rotation axis, and t to be an
// angle about this axis. Then wt is angular twist.
// so3_exp(wt): Returns the rotation matrix describing
//              a rotation about w for t radians.
// so3_log(R):  Returns the angular twist vector
//              describing the rotation matrix R.
//
// Define v to be a linear velocity. Then (w, v)
// is an element of the se(3) Lie algebra, also
// called a twist vector, x.
//
// se3_exp(x): The matrix H such that
//               DH/Dt = x H
//             evaluated at t=1.
// se3_log(H): The vector x such that the solution
//             of DJ/Dt = x J at t=1 is H.

mat3 m_so3_exp(vec3 wt)
{
    float t = m_length(wt);
    if (t < 0.01f)
    {
        return m_id3() + m_skew(wt);
    }
    else
    {
        mat3 W = m_skew(wt/t);
        return m_id3() + sin(t)*W + (1.0f-cos(t))*W*W;
    }
}

// https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Log_map_from_SO.283.29_to_so.283.29
vec3 m_so3_log(mat3 R)
{
    float arg = (R.a11+R.a22+R.a33-1.0f)/2.0f;
    if (arg < -1.0f) arg = -1.0f;
    if (arg > +1.0f) arg = +1.0f;
    float theta = acos(arg);
    if (theta < 0.01f)
    {
        // For small angles
        // R = I + theta K = I + W
        vec3 w;
        w.x = R.a32;
        w.y = R.a13;
        w.z = R.a21;
        return w;
    }
    else
    {
        vec3 k;
        float s = sin(theta);
        k.x = (R.a32-R.a23)/(2.0f*s);
        k.y = (R.a13-R.a31)/(2.0f*s);
        k.z = (R.a21-R.a12)/(2.0f*s);
        vec3 w = k*theta;
        return w;
    }
}

// Motilal Agrawal.
// A Lie Algebraic Approach for Consistent Pose Registration for General Euclidean Motion.
// Proceedings of the 2006 IEEE/RSJ International Conference on Intelligent Robots and Systems
void m_se3_log(mat4 SE3, vec3 *out_w, vec3 *out_v)
{
    mat3 R;
    vec3 T;
    m_se3_decompose(SE3, &R, &T);
    vec3 w = m_so3_log(R);

    float t = m_length(w);
    if (t < 0.01f)
    {
        *out_v = T;
    }
    else
    {
        mat3 W = m_skew(w);
        float s = sin(t);
        float c = cos(t);
        mat3 M = m_id3() - 0.5f*W + W*W*((2.0f*s - t*(1.0f+c))/(2.0f*t*t*s));
        *out_v = M*T;
    }

    *out_w = w;
}

mat4 m_se3_exp(vec3 w, vec3 v)
{
    mat3 R = m_so3_exp(w);
    float t = m_length(w);
    vec3 T = v;
    if (t >= 0.01f)
    {
        vec3 a = w / t;
        mat3 A = m_skew(a);
        T = v + (A*(1.0f-cos(t)) + A*A*(t-sin(t)))*(v/t);

        // This is equivalent
        // T = m_outer_product(a, a)*v + ((m_id3()-R)*m_skew(a))*(v/t);
    }
    mat4 result = m_se3(R, T);
    return result;
}

mat3 m_rotx(float angle)
{
    mat3 m = m_id3();
    float c = cosf(angle);
    float s = sinf(angle);
    m.a2.y = c; m.a3.y = -s;
    m.a2.z = s; m.a3.z =  c;
    return m;
}

mat3 m_roty(float angle)
{
    mat3 m = m_id3();
    float c = cosf(angle);
    float s = sinf(angle);
    m.a1.x =  c; m.a3.x = s;
    m.a1.z = -s; m.a3.z = c;
    return m;
}

mat3 m_rotz(float angle)
{
    mat3 m = m_id3();
    float c = cosf(angle);
    float s = sinf(angle);
    m.a1.x = c; m.a2.x = -s;
    m.a1.y = s; m.a2.y =  c;
    return m;
}

mat3 m_orthogonalize(mat3 R)
{
    vec3 x = R.a1;
    vec3 y = R.a2;
    float e = m_dot(x, y);
    vec3 ny = (y - 0.5f*e*x)/(1.0f-0.25f*e*e);
    vec3 nx = (x - 0.5f*e*ny);
    nx = m_normalize(nx);
    ny = m_normalize(ny);
    vec3 nz = m_cross(nx, ny);
    mat3 result;
    result.a1 = nx;
    result.a2 = ny;
    result.a3 = nz;
    return result;
}

///////////////// SE(3) transformations /////////////////

mat4 mat_scale(float s)
{
    mat4 result = {};
    result.a1.x = s;
    result.a2.y = s;
    result.a3.z = s;
    result.a4.w = 1;
    return result;
}

mat4 mat_scale(float x, float y, float z)
{
    mat4 result = {};
    result.a1.x = x;
    result.a2.y = y;
    result.a3.z = z;
    result.a4.w = 1;
    return result;
}

mat4 mat_scale(vec3 s)
{
    return mat_scale(s.x, s.y, s.z);
}

mat4 mat_rotate_x(float angle_in_radians)
{
    float c = cos(angle_in_radians);
    float s = sin(angle_in_radians);
    mat4 result = m_id4();
    result.a2.y = c;
    result.a2.z = s;
    result.a3.y = -s;
    result.a3.z = c;
    return result;
}

mat4 mat_rotate_y(float angle_in_radians)
{
    float c = cos(angle_in_radians);
    float s = sin(angle_in_radians);
    mat4 result = m_id4();
    result.a1.x = c;
    result.a1.z = -s;
    result.a3.x = s;
    result.a3.z = c;
    return result;
}

mat4 mat_rotate_z(float angle_in_radians)
{
    float c = cos(angle_in_radians);
    float s = sin(angle_in_radians);
    mat4 result = m_id4();
    result.a1.x = c;
    result.a1.y = s;
    result.a2.x = -s;
    result.a2.y = c;
    return result;
}

mat4 mat_translate(vec3 x)
{
    mat4 result = m_id4();
    result.a4.x = x.x;
    result.a4.y = x.y;
    result.a4.z = x.z;
    return result;
}

mat4 mat_translate(float x, float y, float z)
{
    mat4 result = m_id4();
    result.a4.x = x;
    result.a4.y = y;
    result.a4.z = z;
    return result;
}

mat4 mat_ortho(float left, float right, float bottom, float top)
{
    mat4 result = m_id4();
    result.a1.x = 2.0f / (right - left);
    result.a2.y = 2.0f / (top - bottom);
    result.a4.x = (left+right)/(left-right);
    result.a4.y = (bottom+top)/(bottom-top);
    return result;
}

mat4 mat_ortho_depth(float left, float right, float bottom, float top, float zn, float zf)
// Map x from [left, right] to [-1, +1]
//     y from [bottom, top] to [-1, +1]
//     z from [zn, zf]      to [-1, +1]
{
    mat4 result = {};
    result.a1.x = 2.0f / (right - left);
    result.a2.y = 2.0f / (top - bottom);
    result.a3.z = 2.0f / (zn - zf);
    result.a4.x = (right + left) / (left - right);
    result.a4.y = (top + bottom) / (bottom - top);
    result.a4.z = (zf + zn) / (zn - zf);
    result.a4.w = 1.0f;
    return result;
}

mat4 mat_perspective(float fov, float width, float height, float zn, float zf)
{
    mat4 result = {};
    float a = width / height;
    result.a1.x = 1.0f / (a * tan(fov / 2.0f));
    result.a2.y = 1.0f / tan(fov / 2.0f);
    result.a3.z = (zn + zf) / (zn - zf);
    result.a3.w = -1.0f;
    result.a4.z = 2.0f * zn * zf / (zn - zf);
    return result;
}

bool m_intersect_xy_plane(vec3 dir, float h, vec2 *xy)
// dir (input): Normalized direction vector
//   h (input): The origin frame's height above the plane
//      return: X and Y coordinate of intersection point
{
    if (dir.z > -0.2f) // Too steep
    {
        return false;
    }
    float t = -h / dir.z;
    xy->x = t*dir.x;
    xy->y = t*dir.y;
    return true;
}

// Solves for the closest intersection points between two rays.
bool m_intersect_ray_ray(vec3 rd_a, vec3 rd_b, vec3 ro_a, vec3 ro_b, float *t_a, float *t_b)
// rd_a,rd_b (in): Normalized direction vectors
// ro_a,ro_b (in): Origin of ray a and ray b
// t_a,t_b  (out): The solution to t_a rd_a + ro_a = t_b rd_b + ro_b
{
    float ab = -m_dot(rd_a, rd_b);
    float dt = 1.0f - ab*ab;
    if (dt == 0.0f)
        return false;
    vec3 x = ro_b-ro_a;
    float y1 = m_dot(rd_a, x);
    float y2 = m_dot(-rd_b, x);
    *t_a = (y1 - ab*y2)/dt;
    *t_b = (y2 - ab*y1)/dt;
    return true;
}

// Solves for the intersection between (ro + t*rd) and the positive half-space
// of the flat cone spanned by vectors a and b.
bool m_intersect_ray_flat_cone(vec3 ro, vec3 rd, vec3 a, vec3 b, float *out_t)
{
    vec3 n = m_cross(a, b);
    float d = m_dot(n, rd);
    if (d == 0.0f)
        return false;
    float t = -m_dot(n, ro)/d;
    vec3 p = ro + rd*t;
    bool in_right_region =
        m_dot(n, m_cross(a, p)) >= 0.0f &&
        m_dot(n, m_cross(b, p)) <= 0.0f;
    if (in_right_region && t >= 0.0f)
    {
        *out_t = t;
        return true;
    }
    return false;
}

/////////////// Intersection tests ///////////////

// CIRCLE CIRCLE INTERSECTION
// Both x1 and x2 are set even if there was only one intersection
// RETURN
//  FALSE No intersection or infinitely many intersections
//   TRUE One or two intersections
bool
m_is_circle_circle(vec2 a, vec2 b, float ra, float rb, vec2 *x1, vec2 *x2)
{
    vec2 d = b-a;
    vec2 p = m_vec2(-d.y, d.x);
    float ld = m_dot(d,d);
    if (ld < 0.001f)
    {
        return false;
    }
    else
    {
        float t = (ra*ra-rb*rb+ld)/(2.0f*ld);
        float D = ra*ra/ld - t*t;
        if (D < 0.0f)
        {
            return false;
        }
        else
        {
            float s = sqrt(D);
            *x1 = a + t*d + s*p;
            *x2 = a + t*d - s*p;
            return true;
        }
    }
}

/////////////// Camera projection models ///////////////

vec2 m_project_pinhole(float f, float u0, float v0, vec3 p)
//  f  (in): Focal length
// u0  (in): Center of pinhole projection in x measured from left of image
// v0  (in): Center of fisheye projection in y measured from top of image
// xyz (in): Camera-space coordinate (OpenGL convention)
// uv (out): Pixel coordinate measured from top-left of image (DirectX convention)
{
    return m_vec2(u0 - f*p.x/p.z, v0 + f*p.y/p.z);
}

// Inverse-projects a pixel (u,v) to unit direction vector (x,y,z)
vec3 m_ray_pinhole(float f, float u0, float v0, vec2 uv)
//  f   (input): Ideal pinhole fisheye camera model parameter (r = f x theta)
// u0   (input): Center of pinhole projection in x measured from left of image
// v0   (input): Center of pinhole projection in y measured from top of image
// uv   (input): Pixel coordinate measured from top-left of image (DirectX convention)
// xyz (output): Camera-space ray from camera origin through pixel (OpenGL convention)
{
    float ff = f*f;
    float du = (uv.x - u0);
    float dv = (v0 - uv.y);
    float rr = du*du + dv*dv;
    if (rr < 0.00001f)
    {
        return m_vec3(0.0f,0.0f,-1.0f);
    }
    else
    {
        float r = sqrtf(rr);
        float cosphi = du/r;
        float sinphi = dv/r;
        float costheta = 1.0f / sqrtf(1.0f + rr/ff);
        float sintheta = (r/f) / sqrtf(1.0f + rr/ff);
        float x = sintheta*cosphi;
        float y = sintheta*sinphi;
        float z = -costheta;
        return m_vec3(x, y, z);
    }
}

// Projects a point (x,y,z) to pixel (u,v) and returns associated Jacobian evaluated
vec2 m_project_equidistant(float f, float u0, float v0, vec3 p)
//  f  (input): Equidistant fisheye camera model parameter (r = f x theta)
// u0  (input): Center of fisheye projection in x measured from left of image
// v0  (input): Center of fisheye projection in y measured from top of image
// xyz (input): Camera-space coordinate (OpenGL convention)
// uv (output): Pixel coordinate measured from top-left of image (DirectX convention)
{
    float l = sqrtf(p.x*p.x+p.y*p.y);
    if (l < 0.001f)
    {
        return m_vec2(u0, v0);
    }
    else
    {
        float t = atanf(-l/p.z);
        float r = f*t;
        return m_vec2(u0 + r*p.x/l, v0 - r*p.y/l);
    }
}

// Inverse-projects a pixel (u,v) to unit direction vector (x,y,z)
vec3 m_ray_equidistant(float f, float u0, float v0, vec2 uv)
//  f   (input): Equidistant fisheye camera model parameter (r = f x theta)
// u0   (input): Center of fisheye projection in x measured from left of image
// v0   (input): Center of fisheye projection in y measured from top of image
// uv   (input): Pixel coordinate measured from top-left of image (DirectX convention)
// xyz (output): Camera-space ray from camera origin through pixel (OpenGL convention)
{
    float du = uv.x-u0;
    float dv = v0-uv.y;
    float r = sqrtf(du*du+dv*dv);
    if (r < 0.00001f)
    {
        return m_vec3(0.0f, 0.0f, -1.0f);
    }
    else
    {
        float t = r / f;
        float s = sinf(t);
        float c = cosf(t);
        float x = s*du/r;
        float y = s*dv/r;
        float z = -c;
        return m_vec3(x, y, z);
    }
}

#endif
