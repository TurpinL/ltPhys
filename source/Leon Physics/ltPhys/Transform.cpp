#include "Transform.hpp"

namespace lt
{

Transform::Transform()
{
	loadIdentity();
}

Transform::Transform(const Scalar &a, const Scalar &b, const Scalar &c, const Scalar &d,
		   const Scalar &e, const Scalar &f, const Scalar &g, const Scalar &h, 
		   const Scalar &i, const Scalar &j, const Scalar &k, const Scalar &l,
		   const Scalar &m, const Scalar &n, const Scalar &o, const Scalar &p )
{
	m_data[0]  = a; m_data[1]  = b; m_data[2]  = c; m_data[3]  = d;
	m_data[4]  = e; m_data[5]  = f; m_data[6]  = g; m_data[7]  = h;
	m_data[8]  = i; m_data[9]  = j; m_data[10] = k; m_data[11] = l;
	m_data[12] = m; m_data[13] = h; m_data[14] = o; m_data[15] = p;
}

Transform::Transform(const Vec3 &position, const Quat &orientation)
{
	loadIdentity();
	translate(position);
	rotate(orientation);
}

Transform& Transform::loadIdentity()
{
	m_data[0]  = 1; m_data[1]  = 0; m_data[2]  = 0; m_data[3]  = 0;
	m_data[4]  = 0; m_data[5]  = 1; m_data[6]  = 0; m_data[7]  = 0;
	m_data[8]  = 0; m_data[9]  = 0; m_data[10] = 1; m_data[11] = 0;
	m_data[12] = 0; m_data[13] = 0; m_data[14] = 0; m_data[15] = 1;

	return *this;
}

Transform& Transform::scale(const Vec3 &xyz)
{
	m_data[0] *= xyz.x;
	m_data[1] *= xyz.y;
	m_data[2] *= xyz.z;

	m_data[4] *= xyz.x;
	m_data[5] *= xyz.y;
	m_data[6] *= xyz.z;
 	
	m_data[ 8] *= xyz.x;
	m_data[ 9] *= xyz.y;
	m_data[10] *= xyz.z;

	return *this;
}

Transform& Transform::translate(const Vec3 &xyz)
{
	m_data[3]  += m_data[0]*xyz.x + m_data[1]*xyz.x + m_data[ 2]*xyz.x;
	m_data[7]  += m_data[4]*xyz.y + m_data[5]*xyz.y + m_data[ 6]*xyz.y;
	m_data[11] += m_data[8]*xyz.z + m_data[9]*xyz.z + m_data[10]*xyz.z;

	return *this;
}

Transform& Transform::rotate(const Quat &angle)
{
	// Equation reference, "http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm"
	Transform temp;
	// Assumes temp loads as an identity matrix

	// Aliases for readability
	const Scalar& x = angle.x;
	const Scalar& y = angle.y;
	const Scalar& z = angle.z;
	const Scalar& w = angle.w;
	// Variables to reduce calculations
	Scalar sqX = angle.x*angle.x;
	Scalar sqY = angle.y*angle.y;
	Scalar sqZ = angle.z*angle.z;

	temp.m_data[0]  = 1 - 2*sqY - 2*sqZ;
	temp.m_data[4]  =     2*x*y + 2*z*w;
	temp.m_data[8]  =     2*x*z - 2*y*w;

	temp.m_data[1]  =     2*x*y - 2*z*w;
	temp.m_data[5]  = 1 - 2*sqX - 2*sqZ;
	temp.m_data[9]  =     2*y*z + 2*x*w;
	
	temp.m_data[2]  =     2*x*z + 2*y*w;
	temp.m_data[6]  =     2*y*z - 2*x*w;
	temp.m_data[10] = 1 - 2*sqX - 2*sqY;

	*this = *this * temp;

	return *this;
}

void Transform::getOpenGLMatrix(Scalar *destination) const
{
	destination[ 0] = m_data[ 0]; destination[ 4] = m_data[ 1]; destination[ 8] = m_data[ 2]; destination[12] = m_data[ 3]; 
	destination[ 1] = m_data[ 4]; destination[ 5] = m_data[ 5]; destination[ 9] = m_data[ 6]; destination[13] = m_data[ 7]; 
	destination[ 2] = m_data[ 8]; destination[ 6] = m_data[ 9]; destination[10] = m_data[10]; destination[14] = m_data[11]; 
	destination[ 3] = m_data[12]; destination[ 7] = m_data[13]; destination[11] = m_data[14]; destination[15] = m_data[15]; 
}

const Transform Transform::operator* (const Transform& rhs) const
{
	Transform product;

	// Define some variables to make the equation more readable
	Scalar* x = product.m_data;
	const Scalar* a = m_data;
	const Scalar* b = rhs.m_data;

	// Column 1
	x[ 0] = a[ 0]*b[ 0] + a[ 1]*b[ 4] + a[ 2]*b[ 8] + a[ 3]*b[12];
	x[ 4] = a[ 4]*b[ 0] + a[ 5]*b[ 4] + a[ 6]*b[ 8] + a[ 7]*b[12];
	x[ 8] = a[ 8]*b[ 0] + a[ 9]*b[ 4] + a[10]*b[ 8] + a[11]*b[12];
	x[12] = a[12]*b[ 0] + a[13]*b[ 4] + a[14]*b[ 8] + a[15]*b[12];
	// Column 2
	x[ 1] = a[ 0]*b[ 1] + a[ 1]*b[ 5] + a[ 2]*b[ 9] + a[ 3]*b[13];
	x[ 5] = a[ 4]*b[ 1] + a[ 5]*b[ 5] + a[ 6]*b[ 9] + a[ 7]*b[13];
	x[ 9] = a[ 8]*b[ 1] + a[ 9]*b[ 5] + a[10]*b[ 9] + a[11]*b[13];
	x[13] = a[12]*b[ 1] + a[13]*b[ 5] + a[14]*b[ 9] + a[15]*b[13];
	// Column 3
	x[ 2] = a[ 0]*b[ 2] + a[ 1]*b[ 6] + a[ 2]*b[10] + a[ 3]*b[14];
	x[ 6] = a[ 4]*b[ 2] + a[ 5]*b[ 6] + a[ 6]*b[10] + a[ 7]*b[14];
	x[10] = a[ 8]*b[ 2] + a[ 9]*b[ 6] + a[10]*b[10] + a[11]*b[14];
	x[14] = a[12]*b[ 2] + a[13]*b[ 6] + a[14]*b[10] + a[15]*b[14];
	// Column 4
	x[ 3] = a[ 0]*b[ 3] + a[ 1]*b[ 7] + a[ 2]*b[11] + a[ 3]*b[15];
	x[ 7] = a[ 4]*b[ 3] + a[ 5]*b[ 7] + a[ 6]*b[11] + a[ 7]*b[15];
	x[11] = a[ 8]*b[ 3] + a[ 9]*b[ 7] + a[10]*b[11] + a[11]*b[15];
	x[15] = a[12]*b[ 3] + a[13]*b[ 7] + a[14]*b[11] + a[15]*b[15];

	return product;
}

const Vec3 Transform::operator* (const Vec3& rhs) const
{
	return Vec3( rhs.x*m_data[0] + rhs.y*m_data[1] + rhs.z*m_data[2]  + rhs.w*m_data[3],
				 rhs.x*m_data[4] + rhs.y*m_data[5] + rhs.z*m_data[6]  + rhs.w*m_data[7],
				 rhs.x*m_data[8] + rhs.y*m_data[9] + rhs.z*m_data[10] + rhs.w*m_data[11],
				 rhs.w);
}

const Vec3 Transform::getPosition() const
{
	return Vec3(m_data[3], m_data[7], m_data[11]);
}

const Scalar Transform::get(const int index) const
{
	return m_data[index];
}

Scalar& Transform::operator[] (int index)
{
	return m_data[index];
}

const Vec3 Transform::getAxisVector(const int index) const
{
	return Vec3(m_data[index], m_data[index+4], m_data[index+8]);
}

Transform Transform::Identity()
{
	return Transform().loadIdentity();
}

const Vec3 Transform::transformInvP(const Vec3& point) const
{
	Vec3 temp = point;
	temp.x -= m_data[3];
	temp.x -= m_data[7];
	temp.x -= m_data[11];
		
	return Vec3(
		temp.x*m_data[0] + temp.y*m_data[4] + temp.z*m_data[8],
		temp.x*m_data[1] + temp.y*m_data[5] + temp.z*m_data[9],
		temp.x*m_data[2] + temp.y*m_data[6] + temp.z*m_data[10]
	);
}

} // namespace lt