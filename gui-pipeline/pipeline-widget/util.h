#ifndef __UTIL_H__
#define __UTIL_H__

#include <cmath>


// useful utility functions:
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

template<typename T>
inline static T round(T const& v){
    return std::floor(v+0.5);
}

template<typename T>
inline static T min(T const& a, T const& b){
    return a < b? a : b;
}

template<typename T>
inline static T max(T const& a, T const& b){
    return a > b? a : b;
}

template<typename T1, typename T2, typename T3>
inline static T2 _clamp(T1 const& low, T2 const& a, T3 const& high){
    return (a < low)? low : ((a < high)? a : high);
}

template<typename T1, typename T2>
bool lessDereferenced(T1 const& l, T2 const& r){
    return *l < *r;
}

// useful structures:
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
        _BB(T const& w, T const& h) : min(0, -h), max(w, 0){ }
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

template<typename T>
struct ColourValueTraits{static const T min; static const T max; };

template<typename T, typename tT=ColourValueTraits<T> >
class _Colour{
    public:
        typedef _Colour<T, tT> this_t;
        _Colour(T const& v, T const& a = tT::max){
            rgba[0] = rgba[1] = rgba[2] = v; rgba[3] = a;
        }
        _Colour(T const& r, T const& g, T const& b, T const& a = tT::max){
            rgba[0] = r; rgba[1] = g; rgba[2] = b; rgba[3] = a;
        }
        // TODO: don't treat the alpha channel like a colour channel...
        // scalar multiply (_clamped)
        this_t& operator*=(T const& s){
            for(int i=0; i<4; i++)
                rgba[i] = _clamp(tT::min, rgba[i] * s, tT::max);
            return *this;
        }
        // multiply
        this_t& operator*=(this_t const& r){
            for(int i=0; i<4; i++)
                rgba[i] *= r.rgba[i] / tT::max;
            return *this;
        }
        // screen
        this_t& operator/=(this_t const& r){
            for(int i=0; i<4; i++)
                rgba[i] = tT::max - (tT::max-rgba[i]) * (tT::max-r.rgba[i]) / tT::max;
            return *this;
        }
        // add (_clamped)
        this_t& operator+=(this_t const& r){
            for(int i=0; i<4; i++)
                rgba[i] = _clamp(tT::min, rgba[i] + r.rgba[i], tT::max);
            return *this;
        }
        // subtract (_clamped)
        this_t& operator-=(this_t const& r){
            for(int i=0; i<4; i++)
                rgba[i] = _clamp(tT::min, rgba[i] - r.rgba[i], tT::max);
            return *this;
        }
        // soft light... probably
        this_t& operator%=(this_t const& r){
            for(int i=0; i<4; i++)
                rgba[i] = std::pow(rgba[i], std::pow(2, 2*((tT::max+tT::min)/2 - r.rgba[i])));
            return *this;
        }
        // hard light
        this_t& operator&=(this_t const& r){
            for(int i=0; i<3; i++)
                if(r.rgba[i] < (tT::max+tT::min)/2)
                    rgba[i] = ((1-r.a())*rgba[i]) +
                              r.a()*(rgba[i] * 2 * r.rgba[i]);
                else
                    rgba[i] = ((1-r.a())*rgba[i]) +
                              r.a()*(tT::max - 2 * (tT::max - rgba[i]) * (tT::max - r.rgba[i]));
            rgba[3] = _clamp(tT::min, rgba[3] + r.rgba[3], tT::max);
            return *this;
        }
        // overlay
        this_t operator|=(this_t const& r){
            for(int i=0; i<4; i++)
                if(rgba[i] < (tT::max+tT::min)/2)
                    rgba[i] *= 2 * r.rgba[i];
                else
                    rgba[i] = tT::max - 2 * (tT::max - rgba[i]) * (tT::max - r.rgba[i]);
            return *this;
        }

        friend this_t operator*(this_t const& l, T const& r){ return this_t(l) *= r; }
        friend this_t operator*(this_t const& l, this_t const& r){ return this_t(l) *= r; }
        friend this_t operator/(this_t const& l, this_t const& r){ return this_t(l) /= r; }
        friend this_t operator+(this_t const& l, this_t const& r){ return this_t(l) += r; }
        friend this_t operator-(this_t const& l, this_t const& r){ return this_t(l) -= r; }
        friend this_t operator%(this_t const& l, this_t const& r){ return this_t(l) %= r; }
        friend this_t operator&(this_t const& l, this_t const& r){ return this_t(l) &= r; }
        friend this_t operator|(this_t const& l, this_t const& r){ return this_t(l) |= r; }

        T r() const{ return rgba[0]; }
        T g() const{ return rgba[1]; }
        T b() const{ return rgba[2]; }
        T a() const{ return rgba[3]; }

        T rgba[4];
};
typedef _Colour<float> Colour;


// useful overloads etc
void glTranslatef(Point const& p, double const& z = 0.0);
void glVertex(Point const& p);
void glBox(BBox const& b, double const& corner_radius = 0.0f);
void glArc(double const& radius, double const& start, double const& end, unsigned segments);
void glSegment(double const& radius, double const& start, double const& end, unsigned segments);
void glCircle(double const& radius, unsigned segments = 24);
void glCircleOutline(double const& radius, unsigned segments = 24);
void glBezier(Point const& a, Point const& b, Point const& c, Point const& d, int segments=24);
void glBezier(Point const& a, Point const& b, Point const& c, int segments=24);
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

template<typename charT, typename traits, typename T>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, _Colour<T> const& c){
    return os << "(" << c.r() <<","<< c.g() <<","<< c.b() <<","<< c.a() << ")";
}

#endif // ndef __UTIL_H_STREAMOPS__
#endif // def __CAUV_DEBUG_H__




