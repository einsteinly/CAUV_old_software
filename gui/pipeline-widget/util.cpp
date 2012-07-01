/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include "util.h"

#include <QtOpenGL>

#include <debug/cauv_debug.h>

const static double Small_Value_D = 1e-6;

namespace cauv{
namespace gui{
template<>
const float ColourValueTraits<float>::min = 0.0f;
template<>
const float ColourValueTraits<float>::max = 1.0f;
} //namespace gui
} //namespace cauv

void glTranslatef(cauv::gui::Point const& p, double const& z){
    glTranslatef(p.x, p.y, z);
}

void glVertex(cauv::gui::Point const& p){
    glVertex2f(p.x, p.y);
}

static void _glBox(cauv::gui::BBox const& b){
    glVertex2f(b.min.x, b.max.y);
    glVertex2f(b.min.x, b.min.y);
    glVertex2f(b.max.x, b.min.y);
    glVertex2f(b.max.x, b.max.y);
}

void glBox(cauv::gui::BBox const& b, double const& cr, unsigned corner_segs){
    if(cr < Small_Value_D){
        glBegin(GL_QUADS);
        _glBox(b);
        glEnd();
    }else{
        glBegin(GL_QUADS);
            _glBox(cauv::gui::BBox(b.min + cauv::gui::Point(cr, cr), b.max - cauv::gui::Point(cr, cr)));

            _glBox(cauv::gui::BBox(b.min.x     , b.min.y + cr, b.min.x + cr, b.max.y - cr));
            _glBox(cauv::gui::BBox(b.max.x - cr, b.min.y + cr, b.max.x     , b.max.y - cr));
            _glBox(cauv::gui::BBox(b.min.x + cr, b.min.y     , b.max.x - cr, b.min.y + cr));
            _glBox(cauv::gui::BBox(b.min.x + cr, b.max.y - cr, b.max.x - cr, b.max.y     ));
        glEnd();
        
        glPushMatrix();
            glTranslatef(cauv::gui::Point(b.min.x + cr, b.min.y + cr));
            glSegment(cr, -180, -90, corner_segs);

            glTranslatef(cauv::gui::Point(b.w() - 2*cr, 0));
            glSegment(cr, 90, 180, corner_segs);
            
            glTranslatef(cauv::gui::Point(0, b.h() - 2*cr));
            glSegment(cr, 0, 90, corner_segs);

            glTranslatef(cauv::gui::Point(2*cr - b.w(), 0));
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

void glVertices(std::vector<cauv::gui::Point> const& points){
    std::vector<cauv::gui::Point>::const_iterator i;
    for(i = points.begin(); i != points.end(); i++)
        glVertex(*i);
}

static std::vector<cauv::gui::Point> linearInterp(cauv::gui::Point const& a, cauv::gui::Point const& b, int segments){
    float w = 1.0f;
    const float w_inc = 1.0f/segments;
    std::vector<cauv::gui::Point> r;
    for(int i = 0; i <= segments; i++, w-=w_inc)
        r.push_back(a * w + b * (1.0f - w));
    return r;
}

static std::vector<cauv::gui::Point> linearInterp(std::vector<cauv::gui::Point> const& a, std::vector<cauv::gui::Point> const& b){
    std::vector<cauv::gui::Point> r;
    std::vector<cauv::gui::Point>::const_iterator i, j;
    float w = 1.0f;
    const float w_inc = 1.0f/(a.size()-1);
    for(i=a.begin(), j=b.begin(); i!=a.end() && j!=b.end(); i++, j++, w-=w_inc)
        r.push_back(*i * w + *j * (1.0f-w));
    return r;
}

void glBezier(cauv::gui::Point const& a, cauv::gui::Point const& b, cauv::gui::Point const& c, cauv::gui::Point const& d, int segments){
    std::vector<cauv::gui::Point> ab = linearInterp(a, b, segments);
    std::vector<cauv::gui::Point> bc = linearInterp(b, c, segments);
    std::vector<cauv::gui::Point> cd = linearInterp(c, d, segments);
    std::vector<cauv::gui::Point> abbc = linearInterp(ab, bc);
    std::vector<cauv::gui::Point> bccd = linearInterp(bc, cd);
    glVertices(linearInterp(abbc, bccd));
}

void glBezier(cauv::gui::Point const& a, cauv::gui::Point const& b, cauv::gui::Point const& c, int segments){
    std::vector<cauv::gui::Point> ab = linearInterp(a, b, segments);
    std::vector<cauv::gui::Point> bc = linearInterp(b, c, segments);
    glVertices(linearInterp(ab, bc));
}

void glColor(cauv::gui::Colour const& c){
    glColor4fv(c.rgba);
}

void _printGlErr(int err, const char* file, int line) {
	switch(GLuint(err)) {
		case 0:
			break;
		case GL_INVALID_ENUM:
			error() << "GL_INVALID_ENUM: " << file << ":" << line;
			break;
		case GL_INVALID_VALUE:
			error() << "GL_INVALID_VALUE: " << file << ":" << line;
			break;
		case GL_INVALID_OPERATION:
			error() << "GL_INVALID_OPERATION: " << file << ":" << line;
			break;
		case GL_STACK_OVERFLOW:
			error() << "GL_STACK_OVERFLOW: " << file << ":" << line;
			break;
		case GL_STACK_UNDERFLOW:
			error() << "GL_STACK_UNDERFLOW: " << file << ":" << line;
			break;
		case GL_OUT_OF_MEMORY:
			error() << "GL_OUT_OF_MEMORY: " << file << ":" << line;
			break;
		default:
			error() << "unknown OpenGL error: " << file << ":" << line;
			break;
	}
}
