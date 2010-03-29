#include "util.h"

#include <QtOpenGL>

void glTranslatef(Point const& p, double const& z){
    glTranslatef(p.x, p.y, z);
}

void glBox(BBox const& b){
    glBegin(GL_QUADS);
    glVertex2f(b.min.x, b.max.y);
    glVertex2f(b.min.x, b.min.y);
    glVertex2f(b.max.x, b.min.y);
    glVertex2f(b.max.x, b.max.y);
    glEnd();
}

void glColor(Colour const& c){
    glColor4fv(c.rgba);
}

