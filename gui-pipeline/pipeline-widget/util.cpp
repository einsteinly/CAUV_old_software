#include "util.h"

#include <QtOpenGL>

const static double Small_Value_D = 1e-6;

template<>
const float ColourValueTraits<float>::min = 0.0f;
template<>
const float ColourValueTraits<float>::max = 1.0f;

void glTranslatef(Point const& p, double const& z){
    glTranslatef(p.x, p.y, z);
}

void glVertex(Point const& p){
    glVertex2f(p.x, p.y);
}

static void _glBox(BBox const& b){
    glVertex2f(b.min.x, b.max.y);
    glVertex2f(b.min.x, b.min.y);
    glVertex2f(b.max.x, b.min.y);
    glVertex2f(b.max.x, b.max.y);
}

void glBox(BBox const& b, double const& cr){
    const int corner_segs = 8;
    if(cr < Small_Value_D){
        glBegin(GL_QUADS);
        _glBox(b);
        glEnd();
    }else{
        glBegin(GL_QUADS);
            _glBox(BBox(b.min + Point(cr, cr), b.max - Point(cr, cr)));

            _glBox(BBox(b.min.x     , b.min.y + cr, b.min.x + cr, b.max.y - cr));
            _glBox(BBox(b.max.x - cr, b.min.y + cr, b.max.x     , b.max.y - cr));
            _glBox(BBox(b.min.x + cr, b.min.y     , b.max.x - cr, b.min.y + cr));
            _glBox(BBox(b.min.x + cr, b.max.y - cr, b.max.x - cr, b.max.y     ));
        glEnd();
        
        glPushMatrix();
            glTranslatef(Point(b.min.x + cr, b.min.y + cr));
            glSegment(cr, -180, -90, corner_segs);

            glTranslatef(Point(b.w() - 2*cr, 0));
            glSegment(cr, 90, 180, corner_segs);
            
            glTranslatef(Point(0, b.h() - 2*cr));
            glSegment(cr, 0, 90, corner_segs);

            glTranslatef(Point(2*cr - b.w(), 0));
            glSegment(cr, -90, 0, corner_segs);
        glPopMatrix();
    }
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

void glVertices(std::vector<Point> const& points){
    std::vector<Point>::const_iterator i;
    for(i = points.begin(); i != points.end(); i++)
        glVertex(*i);
}

static std::vector<Point> linearInterp(Point const& a, Point const& b, int segments){
    float w = 1.0f;
    const float w_inc = 1.0f/segments;
    std::vector<Point> r;
    for(int i = 0; i <= segments; i++, w-=w_inc)
        r.push_back(a * w + b * (1.0f - w));
    return r;
}

static std::vector<Point> linearInterp(std::vector<Point> const& a, std::vector<Point> const& b){
    std::vector<Point> r;
    std::vector<Point>::const_iterator i, j;
    float w = 1.0f;
    const float w_inc = 1.0f/(a.size()-1);
    for(i=a.begin(), j=b.begin(); i!=a.end() && j!=b.end(); i++, j++, w-=w_inc)
        r.push_back(*i * w + *j * (1.0f-w));
    return r;
}

void glBezier(Point const& a, Point const& b, Point const& c, Point const& d, int segments){
    std::vector<Point> ab = linearInterp(a, b, segments);
    std::vector<Point> bc = linearInterp(b, c, segments);
    std::vector<Point> cd = linearInterp(c, d, segments);
    std::vector<Point> abbc = linearInterp(ab, bc);
    std::vector<Point> bccd = linearInterp(bc, cd);
    glVertices(linearInterp(abbc, bccd));
}

void glBezier(Point const& a, Point const& b, Point const& c, int segments){
    std::vector<Point> ab = linearInterp(a, b, segments);
    std::vector<Point> bc = linearInterp(b, c, segments);
    glVertices(linearInterp(ab, bc));
}

void glColor(Colour const& c){
    glColor4fv(c.rgba);
}

