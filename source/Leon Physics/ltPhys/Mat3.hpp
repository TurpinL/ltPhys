#ifndef LTPHYS_MAT3_H
#define LTPHYS_MAT3_H

#include "Scalar.hpp"
#include "Vec3.hpp"

namespace lt
{

/**  Mat3.h
 *	@brief 
 *
 *  @author Leon Turpin
 *  @date August 2013
 */
class Mat3
{
public:
	Mat3();
	Mat3(const Scalar& a, const Scalar& b, const Scalar& c, 
		 const Scalar& d, const Scalar& e, const Scalar& f, 
		 const Scalar& g, const Scalar& h, const Scalar& i );
	Mat3(const Scalar data[9]); 
	Mat3(const Vec3& diagonals);

	const Vec3 operator* (const Vec3 &rhs) const;
	const Mat3 operator* (const Mat3 &rhs) const;
	Mat3& invert();
	const Mat3 inverse() const;
	Mat3& transpose();
	const Mat3& transposed() const;
	Mat3& setIdentity(); 
	const Scalar Mat3::get(const int index) const;
	Scalar& operator[] (const int index);
 
private:
	Scalar m_data[9];
};

} // namespace lt

#endif // LTPHYS_MAT3_H