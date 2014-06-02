#include "Mat3.hpp"

#include <cmath>

namespace lt
{

Mat3::Mat3()
{

}

Mat3::Mat3(const Scalar &a, const Scalar &b, const Scalar &c, 
		   const Scalar &d, const Scalar &e, const Scalar &f, 
	       const Scalar &g, const Scalar &h, const Scalar &i )
{
	m_data[0] = a; m_data[1] = b; m_data[2] = c;
	m_data[3] = d; m_data[4] = e; m_data[5] = f;
	m_data[6] = g; m_data[7] = h; m_data[8] = i;
}

Mat3::Mat3(const Scalar data[9])
{
	m_data[0] = data[0]; m_data[1] = data[1]; m_data[2] = data[2]; 
	m_data[3] = data[3]; m_data[4] = data[4]; m_data[5] = data[5]; 
	m_data[6] = data[6]; m_data[7] = data[7]; m_data[8] = data[8]; 
}

Mat3::Mat3(const Vec3& diagonals)
{
	m_data[1] = 0; m_data[2] = 0;
	m_data[3] = 0; m_data[5] = 0;
	m_data[6] = 0; m_data[7] = 0; 

	m_data[0] = diagonals.x;
	m_data[4] = diagonals.y;
	m_data[8] = diagonals.z;
}

const Vec3 Mat3::operator* (const Vec3 &rhs) const
{
	return Vec3(
		rhs.x * m_data[0] + // X
		rhs.y * m_data[1] +
		rhs.z * m_data[2], 
			
		rhs.x * m_data[3] + // Y
		rhs.y * m_data[4] +
		rhs.z * m_data[5], 
			
		rhs.x * m_data[6] + // Z
		rhs.y * m_data[7] +
		rhs.z * m_data[8] );
}

const Mat3 Mat3::operator* (const Mat3 &rhs) const
{
	Mat3 result;
	result[0] = m_data[0]*rhs.m_data[0] + m_data[1]*rhs.m_data[3] + m_data[2]*rhs.m_data[6];
	result[1] = m_data[0]*rhs.m_data[1] + m_data[1]*rhs.m_data[4] + m_data[2]*rhs.m_data[7];
	result[2] = m_data[0]*rhs.m_data[2] + m_data[1]*rhs.m_data[5] + m_data[2]*rhs.m_data[8];

	result[3] = m_data[3]*rhs.m_data[0] + m_data[4]*rhs.m_data[3] + m_data[5]*rhs.m_data[6];
	result[4] = m_data[3]*rhs.m_data[1] + m_data[4]*rhs.m_data[4] + m_data[5]*rhs.m_data[7];
	result[5] = m_data[3]*rhs.m_data[2] + m_data[4]*rhs.m_data[5] + m_data[5]*rhs.m_data[8];

	result[6] = m_data[6]*rhs.m_data[0] + m_data[7]*rhs.m_data[3] + m_data[8]*rhs.m_data[6];
	result[7] = m_data[6]*rhs.m_data[1] + m_data[7]*rhs.m_data[4] + m_data[8]*rhs.m_data[7];
	result[8] = m_data[6]*rhs.m_data[2] + m_data[7]*rhs.m_data[5] + m_data[8]*rhs.m_data[8];

	return result;
}

Mat3& Mat3::setIdentity()
{
	m_data[0] = 1; m_data[1] = 0; m_data[2] = 0;
	m_data[3] = 0; m_data[4] = 1; m_data[5] = 0;
	m_data[6] = 0; m_data[7] = 0; m_data[8] = 1;

	return *this;
}

Mat3& Mat3::invert()
{
	// Precalculate some reused multiplications
	Scalar ae = m_data[0]*m_data[4];
	Scalar af = m_data[0]*m_data[5];
	Scalar bd = m_data[1]*m_data[3];
	Scalar cd = m_data[2]*m_data[3];
	Scalar bg = m_data[1]*m_data[6];
	Scalar cg = m_data[2]*m_data[6];

	// Calculate the determinant
	Scalar det = (ae*m_data[8]) - (af*m_data[7]) +
			        (bd*m_data[8]) + (cd*m_data[7]) +
			        (bg*m_data[5]) - (cg*m_data[4]);
		
	// Make sure the determinant is non-zero
	if (det != 0)
	{
		Scalar temp[9]; // Temporary array so we aren't modifying the same matrix that we're reading from.
		Scalar invDet = 1 / det;

		temp[0] = invDet *  (m_data[4]*m_data[8] - m_data[5]*m_data[7]);
		temp[1] = invDet * -(m_data[1]*m_data[8] - m_data[2]*m_data[7]);
		temp[2] = invDet *  (m_data[1]*m_data[5] - m_data[2]*m_data[4]);
		temp[3] = invDet * -(m_data[3]*m_data[8] - m_data[5]*m_data[6]);
		temp[4] = invDet *  (m_data[0]*m_data[8] - cg);
		temp[5] = invDet * -(af - cd);
		temp[6] = invDet *  (m_data[3]*m_data[7] - m_data[4]*m_data[6]);
		temp[7] = invDet * -(m_data[0]*m_data[7] - bg);
		temp[8] = invDet *  (ae - bd);

		// Now copy the temp data into this mat3.
		*this = Mat3(temp);
	}

	return *this;
}

const Mat3 Mat3::inverse() const
{
	return Mat3(*this).invert();
}

Mat3& Mat3::transpose()
{
	Scalar temp[9]; // Temporary array so we aren't modifying the same matrix that we're reading from.

    temp[0] = m_data[0];
	temp[1] = m_data[3];
	temp[2] = m_data[6];

	temp[3] = m_data[1];
    temp[4] = m_data[4];
	temp[5] = m_data[7];

	temp[6] = m_data[2];
	temp[7] = m_data[5];
    temp[8] = m_data[8];

	*this = Mat3(temp);

	return *this;
}

const Mat3& Mat3::transposed() const
{
	return Mat3(*this).transpose();
}

const Scalar& Mat3::get(const int index) const
{
	return m_data[index];
}

Scalar& Mat3::operator[] (const int index)
{
	return m_data[index];
}

void constructOrthonormalBasis(Vec3 &x, const Vec3 &y, Vec3 &z)
{
	// Set the y axis to a vector not in the direction of x
	if(abs(x.y) > abs(x.x))
	{
		x = lt::Vec3(0.f, 1.f, 0.f);
	}
	else
	{
		x = lt::Vec3(1.f, 0.f, 0.f);
	}

	// Calculate Z from the vector product of x and y
	z = y.cross(x);

	// Check that y and x aren't parallel
	if(z.dot(z) != (Scalar)0.0)
	{
		// Calculate y from the product of z and x
		x = z.cross(y);

		// Normalize the output vectors
		x.normalize();
		z.normalize();
	}
}

const Mat3 constructOrthonormalBasis(const Vec3 &y)
{
	Vec3 x, z;

	constructOrthonormalBasis(x, y, z);

	return Mat3(x.x, y.x, z.x,
				x.y, y.y, z.y,
				x.z, y.z, z.z);
}

} // namespace lt