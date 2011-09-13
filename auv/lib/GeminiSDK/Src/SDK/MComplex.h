#ifndef __MCOMPLEX_H__
#define __MCOMPLEX_H__

#ifndef _WIN32
	#include <iostream>
#endif
#include <math.h>

// class needs forward declaration as we are using it as argument.
template <class T> class Complex;

// function prototypes need declared before they can be used in the class.
template <class T>
Complex<T> operator + (const Complex<T> &, const Complex<T> &);

template <class T>
Complex<T> operator - (const Complex<T> &, const Complex<T> &);

template <class T>
Complex<T> operator * (const Complex<T> &, const Complex<T> &);

template <class T>
Complex<T> operator / (const Complex<T> &, const Complex<T> &);

template <class T>
int operator < (const Complex<T> &, const Complex<T> &);

template <class T>
int operator <= (const Complex<T> &, const Complex<T> &);

template <class T>
int operator > (const Complex<T> &, const Complex<T> &);

template <class T>
int operator >= (const Complex<T> &, const Complex<T> &);

template <class T>
int operator == (const Complex<T> &, const Complex<T> &);

template <class T>
int operator != (const Complex<T> &, const Complex<T> &);

template <class T>
class Complex
{
   public:
      // Data Fields : Real part and Imaginary part
      T real;
      T imag;

      // Constructors and Destructor
      Complex();
      Complex(T);
      Complex(T, T);
      Complex(const Complex<T> &);
      ~Complex();

      // Member Functions
      T realPart() const;
      T imagPart() const;
      double magnitude() const;
      double argument() const;
      Complex<T> conjugate();
      Complex<T> negate();

      // Arithmetic operators and relational operators
      const Complex<T> & operator = (const Complex<T> &);
      Complex<T> operator - () const;
      Complex<T> operator += (const Complex<T> &);
      Complex<T> operator -= (const Complex<T> &);
      Complex<T> operator *= (const Complex<T> &);
      Complex<T> operator /= (const Complex<T> &);

		// We need the :: to make sure these are visible to the global scope otherwise
		// similiar operators in this class hide them. The brackets are needed to 
		// make sure that the :: doesn't get associated with the class.
		friend Complex<T> (::operator + <>)(const Complex<T> &, const Complex<T> &);
      friend Complex<T> (::operator - <>)(const Complex<T> &, const Complex<T> &);
      friend Complex<T> (::operator * <>)(const Complex<T> &, const Complex<T> &);
      friend Complex<T> (::operator / <>)(const Complex<T> &, const Complex<T> &);

      friend int operator < <>(const Complex<T> &, const Complex<T> &);
      friend int operator <= <>(const Complex<T> &, const Complex<T> &);
      friend int operator > <>(const Complex<T> &, const Complex<T> &);
      friend int operator >= <>(const Complex<T> &, const Complex<T> &);
      friend int operator == <>(const Complex<T> &, const Complex<T> &);
      friend int operator != <>(const Complex<T> &, const Complex<T> &);

      // I/O stream functions
//      friend istream & operator >> (istream&, Complex<T> &);
//      friend ostream & operator << (ostream&, const Complex<T> &);
};

//
// Constructors, Destructor and Copy constructor
//

template <class T> Complex<T>::Complex() : real(T(0)), imag(T(0)) {}

template <class T> Complex<T>::Complex(T r) : real(r), imag(T(0)) {}

template <class T> Complex<T>::Complex(T r, T i) 
   : real(r), imag(i) {}

template <class T> Complex<T>::Complex(const Complex<T> &c)
   : real(c.real), imag(c.imag) {}

template <class T> Complex<T>::~Complex() {}

//
// Member Functions
//
template <class T> T Complex<T>::realPart() const
{ return real; }

template <class T> T Complex<T>::imagPart() const
{ return imag; }

template <class T> double Complex<T>::magnitude() const
{ return sqrt(double(real*real+imag*imag)); }

template <class T> double Complex<T>::argument() const
{ return atan2(double(imag),double(real)); }

template <class T> Complex<T> Complex<T>::conjugate()
{ return Complex<T>(real, imag = -imag); }

template <class T> Complex<T> Complex<T>::negate()
{ return Complex<T>(real = -real, imag); }

//
// Arithmetic Operators and Relational Operators
//
template <class T> 
const Complex<T> & Complex<T>::operator = (const Complex<T> &c)
{
   real = c.real; imag = c.imag;
   return *this;
}

template <class T> Complex<T> Complex<T>::operator - () const
{ return Complex<T>(-real, -imag); }

template <class T>
Complex<T> Complex<T>::operator += (const Complex<T> &c)
{ return *this = *this + c; }

template <class T>
Complex<T> Complex<T>::operator -= (const Complex<T> &c)
{ return *this = *this - c; }

template <class T>
Complex<T> Complex<T>::operator *= (const Complex<T> &c)
{ return *this = *this * c; }

template <class T>
Complex<T> Complex<T>::operator /= (const Complex<T> &c)
{ return *this = *this / c; }

template <class T>
Complex<T> operator + (const Complex<T> &c1, const Complex<T> &c2)
{ return Complex<T>(c1.real+c2.real, c1.imag+c2.imag); }

template <class T>
Complex<T> operator - (const Complex<T> &c1, const Complex<T> &c2)
{ return Complex<T>(c1.real-c2.real, c1.imag-c2.imag); }

template <class T>
Complex<T> operator * (const Complex<T> &c1, const Complex<T> &c2)
{
  return Complex<T>(c1.real*c2.real - c1.imag*c2.imag,
                    c1.real*c2.imag + c1.imag*c2.real);
}

template <class T>
Complex<T> operator / (const Complex<T> &c1, const Complex<T> &c2)
{
   T modulus = c2.real * c2.real + c2.imag * c2.imag;
   static T zero(0);

   if (modulus == zero)
   {
        std::cerr << "Error : Zero divisor" << std::endl;
        return Complex<T>(zero,zero);
   }

   return Complex<T>((c1.imag * c2.imag + c1.real * c2.real) / modulus,
                     (c1.imag * c2.real - c1.real * c2.imag) / modulus);
}

template <class T>
int operator <  (const Complex<T> &c1, const Complex<T> &c2)
{ return (c1.real*c1.real+c1.imag*c1.imag < c2.real*c2.real+c2.imag*c2.imag); }

template <class T>
int operator <= (const Complex<T> &c1, const Complex<T> &c2)
{ return (c1<c2) || (c1==c2); }

template <class T>
int operator >  (const Complex<T> &c1, const Complex<T> &c2)
{ return (c1.real*c1.real+c1.imag*c1.imag > c2.real*c2.real+c2.imag*c2.imag); }

template <class T>
int operator >= (const Complex<T> &c1, const Complex<T> &c2)
{ return (c1>c2) || (c1==c2); }

template <class T>
int operator == (const Complex<T> &c1, const Complex<T> &c2)
{ return (c1.real==c2.real) && (c1.imag==c2.imag); }

template <class T>
int operator != (const Complex<T> &c1, const Complex<T> &c2)
{ return !(c1==c2); }

/*
//
// I/O Stream Functions
//
template <class T>
istream & operator >> (istream &s, Complex<T> &z)
{
   static T zero(0);
   T r,i;
   char c;

   s >> ws;             // remove leading whitespace
   c = s.peek();        // peek next character

   if (c=='(')
   {
        c = s.get();    // Clear '('
        s >> r;         // read real part
        s >> ws;        // remove extra whitespace
        c = s.peek();   // peek next character
        if (c == ',')
        {
           c = s.get();
           s >> i;      // read imaginary part
           s >> ws;
           c = s.peek();
        }
        else i = zero;  // no imaginary part
        if (c != ')') s.clear(s.rdstate() | ios::badbit);
   }
   else                 // it is a real number
   {
        s >> r;         // read real part
        i = zero;       // initialize real part
   }
   z = Complex<T>(r,i);
   return s;
}

template <class T>
ostream & operator << (ostream &s, const Complex<T> &c)
{
   static T zero(0);

   if (c.imag==zero) return s << c.real;
   if (c.real==zero) return s << c.imag << "i";
   return s << "(" << c.real << "," << c.imag << "i)";
}
*/

#endif //__MCOMPLEX_H__
