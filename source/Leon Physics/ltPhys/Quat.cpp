#include "Quat.hpp"

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

Quat::Quat()
: x(0), y(0), z(0), w(0)
{}

Quat::Quat(const Scalar x, const Scalar y, const Scalar z, const Scalar w)
: x(x), y(y), z(z), w(w)
{}

Quat::Quat(const Vec3 &axis, const Scalar angle)
{
	const static Scalar DEG_TO_RAD = 0.0174532925f;
	// Convert angle to Radians
	Scalar radAngle = angle * DEG_TO_RAD;
	Scalar sinHalfAng = sin(radAngle/2);

	x = axis.x * sinHalfAng;
	y = axis.y * sinHalfAng;
	z = axis.z * sinHalfAng;

	w = cos(radAngle/2);
}

//Quat::Quat(const Scalar yaw, const Scalar pitch, const Scalar roll)
//{
//
//}
//
//const Quat Quat::operator + (const Quat &rhs) const
//{
//
//}
//
//const Quat Quat::operator - (const Quat &rhs) const
//{
//
//}

const Quat Quat::operator * (const Quat &rhs) const
{
	Vec3 xyz(x, y, z);
	Vec3 xyzRight(rhs.x, rhs.y, rhs.z);

	Scalar tempW = w*rhs.w - xyz.dot(xyzRight);
	Vec3 temp = xyz.cross(xyzRight) + xyzRight*w + xyz*rhs.w;

	return Quat(temp.x, temp.y, temp.z, tempW);
}

//const Quat& Quat::operator += (const Quat &rhs)
//{
//
//}
//
//const Quat& Quat::operator -= (const Quat &rhs)
//{
//
//}
//
Quat& Quat::operator *= (const Quat &rhs)
{
	Vec3 xyz(x, y, z);
	Vec3 xyzRight(rhs.x, rhs.y, rhs.z);

	Scalar tempW = w*rhs.w - xyz.dot(xyzRight);
	Vec3 temp = xyz.cross(xyzRight) + xyzRight*w + xyz*rhs.w;

	x = temp.x;
	y = temp.y;
	z = temp.z;
	w = tempW;

	return *this;
}

//const Quat& Quat::operator *= (const Scalar rhs)
//{
//
//}

//const Scalar Quat::dot(const Quat &rhs) const
//{
//
//}
//
//const Scalar Quat::length() const
//{
//
//}
//
//const Scalar Quat::length2() const
//{
//
//}
//
//const Scalar Quat::angle(const Quat &rhs) const
//{
//
//}



const Scalar Quat::getAngle() const
{
	const static Scalar RAD_TO_DEG = 57.2957795f;
	return 2 * acos(w) * RAD_TO_DEG;
}

const Vec3 Quat::getAxis() const
{
	Scalar commonTerm = (Scalar)sqrt14(1 - w*w);
	
	return Vec3(x / commonTerm, y / commonTerm, z / commonTerm);
}

//const Quat& Quat::inverse() const
//{
//
//}
//
//void normalize()
//{
//
//}

} // namespace lt