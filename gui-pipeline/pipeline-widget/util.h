#ifndef __UTIL_H__
#define __UTIL_H__

#include <cmath>

template<typename T>
class V2D{
    public:
        V2D() : x(0), y(0){ }
        V2D(T const& x, T const& y) : x(x), y(y){ }

        V2D<T>& operator+=(V2D<T> const& r){ x += r.x; y += r.y; return *this; }
        V2D<T>& operator-=(V2D<T> const& r){ x -= r.x; y -= r.y; return *this; }
        V2D<T>& operator+=(T const& r){ x += r; y += r; return *this; }
        V2D<T>& operator-=(T const& r){ x -= r; y -= r; return *this; }
        V2D<T>& operator*=(T const& r){ x *= r; y *= r; return *this; }
        V2D<T>& operator/=(T const& r){ x /= r; y /= r; return *this; }

        friend V2D<T> operator+(V2D<T> const& l, V2D<T> const& r){ return V2D<T>(l) += r; }
        friend V2D<T> operator-(V2D<T> const& l, V2D<T> const& r){ return V2D<T>(l) -= r; }
        friend V2D<T> operator+(V2D<T> const& l, T const& r){ return V2D<T>(l) += r; }
        friend V2D<T> operator-(V2D<T> const& l, T const& r){ return V2D<T>(l) -= r; }
        friend V2D<T> operator*(V2D<T> const& l, T const& r){ return V2D<T>(l) *= r; }
        friend V2D<T> operator/(V2D<T> const& l, T const& r){ return V2D<T>(l) /= r; }
        
        T sxx() const{ return x*x + y*y; }

        T x, y;
};
typedef V2D<double> Point;

template<typename T>
class _BB{
    public:
        _BB() : min(), max(){ }
        _BB(V2D<T> const& min, V2D<T> const& max) : min(min), max(max){ }
        _BB(T const& xmin, T const& ymin, T const& xmax, T const& ymax)
            : min(xmin, ymin), max(xmax, ymax){ }
        
        _BB<T>& operator|=(_BB<T> const& r){
            if(r.max.x > max.x) max.x = r.max.x;
            if(r.max.y > max.y) max.y = r.max.y;
            if(r.min.y < min.y) min.y = r.min.y;
            if(r.min.x < min.x) min.x = r.min.x;
            return *this;
        }
        
        _BB<T>& operator&=(_BB<T> const& r){
            if(r.max.x < max.x) max.x = r.max.x;
            if(r.max.y < max.y) max.y = r.max.y;
            if(r.min.y > min.y) min.y = r.min.y;
            if(r.min.x > min.x) min.x = r.min.x;
            return *this;
        }

        _BB<T>& operator+=(V2D<T> const& p){ min += p; max += p; return *this; }
        _BB<T>& operator-=(V2D<T> const& p){ min -= p; max -= p; return *this; }

        friend _BB<T> operator|(_BB<T> const& l, _BB<T> const& r){ return _BB<T>(l) |= r; }
        friend _BB<T> operator&(_BB<T> const& l, _BB<T> const& r){ return _BB<T>(l) &= r; }
        friend _BB<T> operator+(_BB<T> const& l, V2D<T> const& r){ return _BB<T>(l) += r; }
        friend _BB<T> operator-(_BB<T> const& l, V2D<T> const& r){ return _BB<T>(l) -= r; }

        bool contains(T const& x, T const& y) const{
            return (x >= min.x) && (x <= max.x) && (y >= min.y) && (y <= max.y);
        }
        bool contains(V2D<T> const& p) const{
            return contains(p.x, p.y);
        }
        T area() const{ return (max - min).x * (max - min).y; }
        T w() const{ return max.x - min.x; }
        T h() const{ return max.y - min.y; }
    
        V2D<T> min, max;
};
typedef _BB<double> BBox;

class Colour{
    public:
        Colour(float const& v, float const& a = 1.0f){
            rgba[0] = rgba[1] = rgba[2] = v; rgba[3] = a;
        }
        Colour(float const& r, float const& g, float const& b, float const& a = 1.0f){
            rgba[0] = r; rgba[1] = g; rgba[2] = b; rgba[3] = a;
        }

        float rgba[4];
};


// Usful overloaded & utility functions:
void glTranslatef(Point const& p, double const& z = 0.0);
void glVertex(Point const& p);
void glBox(BBox const& b);
void glArc(double const& radius, double const& start, double const& end, unsigned segments);
void glSegment(double const& radius, double const& start, double const& end, unsigned segments);
void glCircle(double const& radius, unsigned segments = 24);
void glCircleOutline(double const& radius, unsigned segments = 24);
void glColor(Colour const& c);

#define glCheckError() glPrintErr(glGetError())

#define glPrintErr(err) \
	switch(GLuint(err)){ \
		case 0: \
			break; \
		case GL_INVALID_ENUM: \
			error() << "GL_INVALID_ENUM: "__FILE__":"<<__LINE__; \
			break; \
		case GL_INVALID_VALUE: \
			error() << "GL_INVALID_VALUE: "__FILE__":"<<__LINE__; \
			break; \
		case GL_INVALID_OPERATION: \
			error() << "GL_INVALID_OPERATION: "__FILE__":"<<__LINE__; \
			break; \
		case GL_STACK_OVERFLOW: \
			error() << "GL_STACK_OVERFLOW: "__FILE__":"<<__LINE__; \
			break; \
		case GL_STACK_UNDERFLOW: \
			error() << "GL_STACK_UNDERFLOW: "__FILE__":"<<__LINE__; \
			break; \
		case GL_OUT_OF_MEMORY: \
			error() << "GL_OUT_OF_MEMORY: "__FILE__":"<<__LINE__; \
			break; \
		default: \
			error() << "unknown OpenGL error: "__FILE__":"<<__LINE__; \
			break; \
	}


template<typename T>
inline static T roundZ(T const& v){
    if(v >= 0) return std::floor(v);
    else return std::ceil(v);
}

template<typename T>
inline static T roundA(T const& v){
    if(v >= 0) return std::ceil(v);
    else return std::floor(v);
}


#endif // ndef __UTIL_H__

#ifdef __CAUV_DEBUG_H__
#ifndef __UTIL_H__STREAMOPS__
#define __UTIL_H__STREAMOPS__

template<typename charT, typename traits, typename T>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, V2D<T> const& p){
    return os << "(" << p.x << "," << p.y << ")";
}

template<typename charT, typename traits, typename T>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, _BB<T> const& b){
    return os << "BBox {" << b.min << "," << b.max << "}";
}

#endif // ndef __UTIL_H_STREAMOPS__
#endif // def __CAUV_DEBUG_H__




