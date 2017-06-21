#include <math.h>

void bilinear(unsigned char *src, int w, int h, float x, float y, unsigned char *r, unsigned char *g, unsigned char *b)
{
    int x0 = (int)floorf(x);
    int x1 = (int)ceilf(x);
    int y0 = (int)floorf(y);
    int y1 = (int)ceilf(y);
    if (x0 < 0) x0 = 0;
    if (x0 > w-1) x0 = w-1;
    if (x1 < 0) x1 = 0;
    if (x1 > w-1) x1 = w-1;
    if (y0 < 0) y0 = 0;
    if (y0 > h-1) y0 = h-1;
    if (y1 < 0) y1 = 0;
    if (y1 > h-1) y1 = h-1;

    unsigned char r00 = src[(y0*w+x0)*3+0];
    unsigned char r10 = src[(y0*w+x1)*3+0];
    unsigned char r01 = src[(y1*w+x0)*3+0];
    unsigned char r11 = src[(y1*w+x1)*3+0];

    unsigned char g00 = src[(y0*w+x0)*3+1];
    unsigned char g10 = src[(y0*w+x1)*3+1];
    unsigned char g01 = src[(y1*w+x0)*3+1];
    unsigned char g11 = src[(y1*w+x1)*3+1];

    unsigned char b00 = src[(y0*w+x0)*3+2];
    unsigned char b10 = src[(y0*w+x1)*3+2];
    unsigned char b01 = src[(y1*w+x0)*3+2];
    unsigned char b11 = src[(y1*w+x1)*3+2];

    float dx = x-x0;
    float dy = y-y0;

    unsigned char r0 = r00 + (r10-r00)*dx;
    unsigned char r1 = r01 + (r11-r01)*dx;
    *r = r0 + (r1-r0)*dy;

    unsigned char g0 = g00 + (g10-g00)*dx;
    unsigned char g1 = g01 + (g11-g01)*dx;
    *g = g0 + (g1-g0)*dy;

    unsigned char b0 = b00 + (b10-b00)*dx;
    unsigned char b1 = b01 + (b11-b01)*dx;
    *b = b0 + (b1-b0)*dy;
}

void project_pinhole(float f, float u0, float v0, vec3 p)
{
    float u = u0 - f*p.x/p.z;
    float v = v0 + f*p.y/p.z;
    glVertex2f(u, v);
}

void view_rectify(unsigned char *I, int Ix, int Iy, float f_f, float u0_f, float v0_f, mat3 rot, vec3 pos)
// rot: R_cam^world
// pos: T_cam/world^world
{
    if (!I)
        return;

    #if 1
    const int Rx = 400;
    const int Ry = 300;
    static unsigned char R[Rx*Ry*3];

    static float yfov_p = 160.0f*3.14f/180.0f;
    float f_p = Ry / tanf(yfov_p/2.0f);
    float u0_p = Rx/2.0f;
    float v0_p = Ry/2.0f;

    // find center point in camera projected onto the ground
    vec3 center_ground;
    {
        vec3 dir = rot*m_vec3(0.0f, 0.0f, -1.0f);
        center_ground = pos + (-pos.z / dir.z)*dir;
    }
    vec3 pos_p = m_vec3(center_ground.x, center_ground.y, 1.0f);
    mat3 irot = m_transpose(rot);

    // render what we would have seen as if we had a pinhole camera
    // looking down at the ground center point from pos_p
    for (int v = 0; v < Ry; v++)
    for (int u = 0; u < Rx; u++)
    {
        // find the ray going through pixel (u,v) in the ideal pinhole
        vec3 dir;
        {
            float f = f_p;
            float u0 = u0_p;
            float v0 = v0_p;

            float ff = f*f;
            float du = (u - u0);
            float dv = (v0 - v);
            float rr = du*du + dv*dv;
            if (rr < 0.00001f)
            {
                dir.x = 0.0f;
                dir.y = 0.0f;
                dir.z = -1.0f;
            }
            else
            {
                float r = sqrtf(rr);
                float cosphi = du/r;
                float sinphi = dv/r;
                float costheta = 1.0f / sqrtf(1.0f + rr/ff);
                float sintheta = (r/f) / sqrtf(1.0f + rr/ff);
                dir.x = sintheta*cosphi;
                dir.y = sintheta*sinphi;
                dir.z = -costheta;
            }
        }

        // intersect that ray with the ground
        vec3 p = pos_p + (-pos_p.z / dir.z)*dir;

        // transform the intersection point into the space of the fisheye camera
        p = irot*(p - pos);

        // project that point onto the fisheye camera image
        float u_src, v_src;
        {
            float f = f_f;
            float u0 = u0_f;
            float v0 = v0_f;
            float x = p.x;
            float y = p.y;
            float z = p.z;
            float l = sqrtf(x*x+y*y);
            if (l < 0.001f)
            {
                u_src = u0_f;
                v_src = v0_f;
            }
            else
            {
                float t = atanf(-l/z);
                float r = f*t;
                u_src = u0_f + r*x/l;
                v_src = v0_f - r*y/l;
            }
        }

        // lookup the color in the image, if inside, set to black otherwise
        if (u_src >= 0.0f && u_src <= Ix && v_src >= 0.0f && v_src <= Iy)
        {
            unsigned char cr,cg,cb;
            bilinear(I, Ix, Iy, u_src, v_src, &cr, &cg, &cb);
            R[(u + v*Rx)*3 + 0] = cr;
            R[(u + v*Rx)*3 + 1] = cg;
            R[(u + v*Rx)*3 + 2] = cb;
        }
        else
        {
            R[(u + v*Rx)*3 + 0] = 0;
            R[(u + v*Rx)*3 + 1] = 0;
            R[(u + v*Rx)*3 + 2] = 0;
        }
    }

    vdbOrtho(-1.0f, +1.0f, +1.0f, -1.0f);
    vdbSetTexture2D(0, R, Rx, Ry, GL_RGB, GL_UNSIGNED_BYTE, GL_NEAREST, GL_NEAREST);
    vdbDrawTexture2D(0);
    #else

    const int Rx = 256;
    const int Ry = 256;
    static vec2 texels[Rx*Ry];

    static float yfov_p = 160.0f*3.14f/180.0f;
    float f_p = Ry / tanf(yfov_p/2.0f);
    float u0_p = Rx/2.0f;
    float v0_p = Ry/2.0f;

    // find center point in camera projected onto the ground
    vec3 center_ground;
    {
        vec3 dir = rot*m_vec3(0.0f, 0.0f, -1.0f);
        center_ground = pos + (-pos.z / dir.z)*dir;
    }
    vec3 pos_p = m_vec3(center_ground.x, center_ground.y, 1.0f);
    mat3 irot = m_transpose(rot);

    // render what we would have seen as if we had a pinhole camera
    // looking down at the ground center point from pos_p
    for (int v = 0; v < Ry; v++)
    for (int u = 0; u < Rx; u++)
    {
        // find the ray going through pixel (u,v) in the ideal pinhole
        vec3 dir;
        {
            float f = f_p;
            float u0 = u0_p;
            float v0 = v0_p;

            float ff = f*f;
            float du = (u - u0);
            float dv = (v0 - v);
            float rr = du*du + dv*dv;
            if (rr < 0.00001f)
            {
                dir.x = 0.0f;
                dir.y = 0.0f;
                dir.z = -1.0f;
            }
            else
            {
                float r = sqrtf(rr);
                float cosphi = du/r;
                float sinphi = dv/r;
                float costheta = 1.0f / sqrtf(1.0f + rr/ff);
                float sintheta = (r/f) / sqrtf(1.0f + rr/ff);
                dir.x = sintheta*cosphi;
                dir.y = sintheta*sinphi;
                dir.z = -costheta;
            }
        }

        // intersect that ray with the ground
        vec3 p = pos_p + (-pos_p.z / dir.z)*dir;

        // transform the intersection point into the space of the fisheye camera
        p = irot*(p - pos);

        // project that point onto the fisheye camera image
        float u_src, v_src;
        {
            float f = f_f;
            float u0 = u0_f;
            float v0 = v0_f;
            float x = p.x;
            float y = p.y;
            float z = p.z;
            float l = sqrtf(x*x+y*y);
            if (l < 0.001f)
            {
                u_src = u0_f;
                v_src = v0_f;
            }
            else
            {
                float t = atanf(-l/z);
                float r = f*t;
                u_src = u0_f + r*x/l;
                v_src = v0_f - r*y/l;
            }
        }

        texels[u + v*Rx].x = u_src/Ix;
        texels[u + v*Rx].y = 1.0f - v_src/Iy;
    }

    vdbSetTexture2D(0, I, Ix, Iy, GL_RGB, GL_UNSIGNED_BYTE, GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER);
    vdbOrtho(0, Rx, 0, Ry);
    glEnable(GL_TEXTURE_2D);
    vdbBindTexture2D(0);
    {
        float color[] = { 0.0f, 0.0f, 0.0f, 1.0f };
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, color);
    }
    glBegin(GL_TRIANGLES);
    for (int v = 0; v < Ry-1; v++)
    for (int u = 0; u < Rx-1; u++)
    {
        float s1 = texels[u + v*Rx].x;
        float t1 = texels[u + v*Rx].y;
        float s2 = texels[u+1 + v*Rx].x;
        float t2 = texels[u + (v+1)*Rx].y;
        glColor4f(1,1,1,1); glTexCoord2f(s1,t1); glVertex2f(u,v);
        glColor4f(1,1,1,1); glTexCoord2f(s2,t1); glVertex2f(u+1,v);
        glColor4f(1,1,1,1); glTexCoord2f(s2,t2); glVertex2f(u+1,v+1);
        glColor4f(1,1,1,1); glTexCoord2f(s2,t2); glVertex2f(u+1,v+1);
        glColor4f(1,1,1,1); glTexCoord2f(s1,t2); glVertex2f(u,v+1);
        glColor4f(1,1,1,1); glTexCoord2f(s1,t1); glVertex2f(u,v);
    }
    glEnd();
    glDisable(GL_TEXTURE_2D);

    #endif

    {
        vdbOrtho(0.0f, Rx, Ry, 0.0f);
        glLines(2.0f);
        glColor4f(1.0f, 0.2f, 0.1f, 1.0f);
        vec3 p11 = m_vec3(-0.5f, -0.5f, -pos_p.z);
        vec3 p21 = m_vec3(+0.5f, -0.5f, -pos_p.z);
        vec3 p22 = m_vec3(+0.5f, +0.5f, -pos_p.z);
        vec3 p12 = m_vec3(-0.5f, +0.5f, -pos_p.z);
        project_pinhole(f_p,u0_p,v0_p, p11); project_pinhole(f_p,u0_p,v0_p, p21);
        project_pinhole(f_p,u0_p,v0_p, p21); project_pinhole(f_p,u0_p,v0_p, p22);
        project_pinhole(f_p,u0_p,v0_p, p22); project_pinhole(f_p,u0_p,v0_p, p12);
        project_pinhole(f_p,u0_p,v0_p, p12); project_pinhole(f_p,u0_p,v0_p, p11);
        glEnd();
    }

    {
        vdbOrtho(0.0, 1.0f, 0.0f, 1.0f);
        glLines(2.0f);
        glColor4f(1.0f, 0.2f, 0.1f, 1.0f); glVertex2f(0.05f, 0.05f); glVertex2f(0.1f, 0.05f);
        glColor4f(0.1f, 1.0f, 0.2f, 1.0f); glVertex2f(0.05f, 0.05f); glVertex2f(0.05f, 0.1f);
        glEnd();
    }

    ImGui::Begin("README");
    ImGui::SliderFloat("Zoom in/out", &yfov_p, 0.0f, 3.0f);
    ImGui::End();
}
