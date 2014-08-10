#include "Vec3.hpp"

#include <math.h> // sin()/cos()

// Inline assembly square root
double inline __declspec (naked) __fastcall sqrt14(double n)
{
 _asm fld qword ptr [esp+4]
 _asm fsqrt
 _asm ret 8
}

namespace lt
{

Vec3::Vec3()
: x(0), y(0), z(0), w(1)
{}

Vec3::Vec3(const Scalar& x, const Scalar& y, const Scalar& z)
: x(x), y(y), z(z), w(1)
{}

Vec3::Vec3(const Scalar& x, const Scalar& y, const Scalar& z, const Scalar& w)
: x(x), y(y), z(z), w(w)
{}

const Vec3 Vec3::operator+(const Vec3& rhs) const
{
	return Vec3(x+rhs.x, y+rhs.y, z+rhs.z);
}

const Vec3 Vec3::operator- (const Vec3& rhs) const
{
	return Vec3(x-rhs.x, y-rhs.y, z-rhs.z);
}

const Vec3 Vec3::operator+ (const Scalar& scalar) const
{
	return Vec3(x+scalar, y+scalar, z+scalar);
}

const Vec3 Vec3::operator- (const Scalar& scalar) const
{
	return Vec3(x-scalar, y-scalar, z-scalar);
}

const Vec3 Vec3::operator* (const Scalar& scalar) const
{
	return Vec3(x*scalar, y*scalar, z*scalar);
}

const Vec3 Vec3::operator/ (const Scalar& scalar) const
{
	return Vec3(x/scalar, y/scalar, z/scalar);
}

const Vec3 Vec3::operator- () const
{
	return Vec3(-x, -y, -z);
}

Vec3& Vec3::operator+= (const Vec3& rhs)
{
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;

	return *this;
}

Vec3& Vec3::operator-= (const Vec3& rhs)
{
	x -= rhs.x;
	y -= rhs.y;
	z -= rhs.z;
	
	return *this;
}

Vec3& Vec3::operator*= (const Scalar& rhs)
{
	x *= rhs;
	y *= rhs;
	z *= rhs;
	
	return *this;
}

Vec3& Vec3::operator/= (const Scalar& rhs)
{
	x /= rhs;
	y /= rhs;
	z /= rhs;

	return *this;
}

const Vec3 Vec3::cross(const Vec3& rhs) const
{
	Vec3 temp;

	temp.x = (this->y * rhs.z) - (this->z * rhs.y);
	temp.y = (this->z * rhs.x) - (this->x * rhs.z);
	temp.z = (this->x * rhs.y) - (this->y * rhs.x);
	
	return temp;
}

const Scalar Vec3::dot(const Vec3& rhs) const
{
	return (this->x*rhs.x) + (this->y*rhs.y) + (this->z*rhs.z);
}

const Scalar Vec3::length() const
{
	return (Scalar)sqrt14(this->dot(*this));
}

Vec3& Vec3::normalize()
{
	if(x != 0 || y != 0 || z != 0)
	{
		float mul = length();
		x /= mul;
		y /= mul;
		z /= mul;
	}

	return *this;
}

const Vec3 Vec3::normalized() const
{
	if(x != 0 || y != 0 || z != 0)
	{
		return (*this) / length();
	}
	else
	{
		return *this;
	}
}

const bool Vec3::nearEqual( const Vec3 &rhs, const Scalar &degOfError ) const
{
	return fabs(x-rhs.x) <= degOfError && 
		   fabs(y-rhs.y) <= degOfError && 
		   fabs(z-rhs.z) <= degOfError;
}

Scalar& Vec3::operator[] (unsigned int index)
{
	switch (index)
	{
	case 0:  return x; break;
	case 1:  return y; break;
	case 2:  return z; break;
	default: return w;
	}	
}

const Scalar& Vec3::get(unsigned int index) const
{
	switch (index)
	{
	case 0:  return x; break;
	case 1:  return y; break;
	case 2:  return z; break;
	default: return w;
	}	
}

const Vec3 Vec3::rotate(const Scalar &angle, const Vec3 &axis) const
{
	static const Scalar DEG_TO_RAD = 0.0174532925f;
	Vec3 rotatedVector;

	Scalar ux = axis.x*x;
	Scalar uy = axis.x*y;
	Scalar uz = axis.x*z;
	Scalar vx = axis.y*x;
	Scalar vy = axis.y*y;
	Scalar vz = axis.y*z;
	Scalar wx = axis.z*x;
	Scalar wy = axis.z*y;
	Scalar wz = axis.z*z;
	Scalar sa = sin(angle * DEG_TO_RAD);
	Scalar ca = cos(angle * DEG_TO_RAD);

	rotatedVector.x = axis.x*(ux+vy+wz)+(x*(axis.y*axis.y+axis.z*axis.z)-axis.x*(vy+wz))*ca+(-wy+vz)*sa;
	rotatedVector.y = axis.y*(ux+vy+wz)+(y*(axis.x*axis.x+axis.z*axis.z)-axis.y*(ux+wz))*ca+(wx-uz)*sa;
	rotatedVector.z = axis.z*(ux+vy+wz)+(z*(axis.x*axis.x+axis.y*axis.y)-axis.z*(ux+vy))*ca+(-vx+uy)*sa;

	return rotatedVector;
}

} // namespace lt