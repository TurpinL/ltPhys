#ifndef LTPHYS_TRANSFORM_H
#define LTPHYS_TRANSFORM_H

#include "Scalar.hpp"
#include "Vec3.hpp"
#include "Quat.hpp"

namespace lt
{

/** 
 *	@brief A 3d transformation represented with a 4x4 matrix.
 *
 *  @author Leon Turpin
 *  @date August 2013
 */
class Transform
{
public:
	Transform();
	Transform(const Scalar &a, const Scalar &b, const Scalar &c, const Scalar &d,
		 const Scalar &e, const Scalar &f, const Scalar &g, const Scalar &h, 
		 const Scalar &i, const Scalar &j, const Scalar &k, const Scalar &l,
		 const Scalar &m, const Scalar &n, const Scalar &o, const Scalar &p );
	Transform(const Vec3 &position, const Quat &orientation);
	Transform(const Scalar data[16]);

	Transform& loadIdentity();
	Transform& scale(const Vec3 &xyz);
	Transform& translate(const Vec3 &xyz);
	Transform& rotate(const Quat &angle);

	void getOpenGLMatrix(Scalar *destination) const;

	const Vec3 operator* (const Vec3& rhs) const; 
	const Transform operator* (const Transform& rhs) const; 

	const Vec3 getPosition() const;
	
	const Scalar get(const int index) const;
	Scalar& operator[] (const int index);

	static Transform Identity();

private:
	Scalar m_data[16];
};

} // namespace lt

#endif // LTPHYS_TRANSFORM_H