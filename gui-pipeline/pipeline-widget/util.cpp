#include "util.h"

#include <QtOpenGL>

void glTranslatef(Point const& p, double const& z){
    glTranslatef(p.x, p.y, z);
}

void glVertex(Point const& p){
    glVertex2f(p.x, p.y);
}

void glBox(BBox const& b){
    glBegin(GL_QUADS);
    glVertex2f(b.min.x, b.max.y);
    glVertex2f(b.min.x, b.min.y);
    glVertex2f(b.max.x, b.min.y);
    glVertex2f(b.max.x, b.max.y);
    glEnd();
}

// TODO: fast sin/cos in degrees with lookup tables etc
static float rad(float const& f){
    return f * M_PI/180;
}

void glArc(double const& radius, double const& start, double const& end, unsigned segments){
    if(!segments || end <= start) return;
    
    glScaled(radius, radius, 1.0);
    glBegin(GL_LINE_STRIP);
    for(float theta = end; theta >= start; theta += (start-end)/segments){
        glVertex2f(std::sin(rad(theta)), std::cos(rad(theta)));
    }
    glEnd();
    glScaled(1.0/radius, 1.0/radius, 1.0);
}

void glSegment(double const& radius, double const& start, double const& end, unsigned segments){
    if(!segments || end <= start) return;
    
    glScaled(radius, radius, 1.0);
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(0, 0);
    for(float theta = end; theta >= start; theta += (start-end)/segments){
        glVertex2f(std::sin(rad(theta)), std::cos(rad(theta)));
    }
    glEnd();
    glScaled(1.0/radius, 1.0/radius, 1.0);
}

void glCircle(double const& radius, unsigned segments){
    glSegment(radius, 0, 360, segments);
}

void glCircleOutline(double const& radius, unsigned segments){
    glArc(radius, 0, 360, segments);
}

void glColor(Colour const& c){
    glColor4fv(c.rgba);
}

