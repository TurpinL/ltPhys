#ifndef LTPHYS_TRANSFORM_H
#define LTPHYS_TRANSFORM_H

#include "Scalar.hpp"
#include "Vec3.hpp"
#include "Quat.hpp"

namespace lt
{

////////////////////////////////////////////////////////////
///	@brief Defines a 4x4 transformation matrix, row major.
///
/// @author Leon Turpin
/// @date August 2013
////////////////////////////////////////////////////////////
class Transform
{
public:
	////////////////////////////////////////////////////////////
    /// @brief Default constructor
    ///
    /// Fill matrix with junk data.
    ///
    ////////////////////////////////////////////////////////////
	Transform();

    ////////////////////////////////////////////////////////////
	/// @brief Construct a transform from a 4x4 matrix
	///
	/// @param a row 1 column 1
	/// @param b row 1 column 2
	/// @param c row 1 column 3
	/// @param d row 1 column 4
	/// @param e row 2 column 1
	/// @param f etc
	///
    ////////////////////////////////////////////////////////////
	Transform(const Scalar &a, const Scalar &b, const Scalar &c, const Scalar &d,
		 const Scalar &e, const Scalar &f, const Scalar &g, const Scalar &h, 
		 const Scalar &i, const Scalar &j, const Scalar &k, const Scalar &l,
		 const Scalar &m, const Scalar &n, const Scalar &o, const Scalar &p );
	
	////////////////////////////////////////////////////////////
	/// @brief Construct a transform from a position and orientation
	///
	/// @param position Position for this transform to represent
	/// @param orientation Orientation for this transform to represent
	///
	////////////////////////////////////////////////////////////
	Transform(const Vec3 &position, const Quat &orientation);

	////////////////////////////////////////////////////////////
	/// @brief Construct a transform from a 4x4 matrix.
	///
	/// @param data 4x4 matrix data
	///
	////////////////////////////////////////////////////////////
	Transform(const Scalar data[16]);

	////////////////////////////////////////////////////////////
	/// @brief Set this transform to an identity matrix
	/// 
	/// @return Reference to this matrix
	///
	////////////////////////////////////////////////////////////
	Transform& loadIdentity();

	////////////////////////////////////////////////////////////
	/// @brief Apply a scale transform to this transform.
	///
	/// @param xyz Componenets to scale this vector by
	///
	/// @return Reference to this matrix
	///
	////////////////////////////////////////////////////////////
	Transform& scale(const Vec3 &xyz);

	////////////////////////////////////////////////////////////
	/// @brief Apply a translation to this transform
	///
	///
	/// @param xyz Componenets to translate this vector by
	///
	/// @return Reference to this matrix
	///
	////////////////////////////////////////////////////////////
	Transform& translate(const Vec3 &xyz);

	////////////////////////////////////////////////////////////
	/// @brief apply a rotation to this transform
	///
	/// @param angle Rotation to rotate this vector by.
	///
	/// @return Reference to this matrix
	///
	////////////////////////////////////////////////////////////
	Transform& rotate(const Quat &angle);

	////////////////////////////////////////////////////////////
	/// @brief Fills destination array with column major version
	/// of this matrix. OpenGL needs it that way.
	///
	/// @param destination 
	///
	////////////////////////////////////////////////////////////
	void getOpenGLMatrix(Scalar destination[16]) const;

	////////////////////////////////////////////////////////////	
	/// @brief Transforms a position or vector with this transform
	///
	/// @param rhs right operand
	///
	/// @return The resulting vector
	///
	////////////////////////////////////////////////////////////
	const Vec3 operator* (const Vec3& rhs) const; 

	////////////////////////////////////////////////////////////
	/// @brief Combine two transformations. Non-commutative.
	/// 
	/// @param rhs right operand
	///
	/// @return Resultinf transform
	///
	////////////////////////////////////////////////////////////	
	const Transform operator* (const Transform& rhs) const; 

	////////////////////////////////////////////////////////////	
	/// @brief Get the translation that this transform represents
	///
	/// @return Translation that this transform represents.
	///
	////////////////////////////////////////////////////////////	
	const Vec3 getPosition() const;
	
	////////////////////////////////////////////////////////////
	/// @brief Gets the element of this matrix at the given index
	/// 
	/// @param index Index of the element to get.
	///
	/// @return The element at the index
	///
	////////////////////////////////////////////////////////////
	const Scalar get(const int index) const;

	////////////////////////////////////////////////////////////
	/// @brief Access the matrix data with array subscription operator
	///
	/// @param Index of element to access
	///
	/// @return Reference to element at given index.
	///
	////////////////////////////////////////////////////////////
	Scalar& operator[] (const int index);

	////////////////////////////////////////////////////////////
	/// @brief Gets a vector representing one axis (one column)
	/// in the matrix
	/// 
	/// @param i the column to return
	///
	/// @return The vector.
	///
	////////////////////////////////////////////////////////////
	const Vec3 getAxisVector(const int index) const;

	////////////////////////////////////////////////////////////
	/// @brief returns and identity matrix
	///
	/// @return Identity matrix.
	///
	////////////////////////////////////////////////////////////
	static Transform Identity();

	////////////////////////////////////////////////////////////
	/// @brief Transform the given point by the transformational
	/// inverse of this matrix.
	///
	/// @return Transformed point.
	///
	////////////////////////////////////////////////////////////
	//const Vec3 transformP(const Vec3& point) const;

	////////////////////////////////////////////////////////////
	/// @brief Transform the given point by the transformational
	/// inverse of this matrix.
	///
	/// @return Transformed point.
	///
	////////////////////////////////////////////////////////////
	const Vec3 transformInvP(const Vec3& point) const;

	////////////////////////////////////////////////////////////
	/// @brief Transform the given vector by the transformational
	/// inverse of this matrix. Different to transformP(...) as it 
	/// ignores rotation
	///
	/// @return Transformed point.
	///
	////////////////////////////////////////////////////////////
	//const Vec3 transformV(const Vec3& vector) const;

	////////////////////////////////////////////////////////////
	/// @brief Transform the given vector by the transformational
	/// inverse of this matrix. Different to transformInvP(...) as it 
	/// ignores rotation
	///
	/// @return Transformed point.
	///
	////////////////////////////////////////////////////////////
	//const Vec3 transformInvV(const Vec3& vector) const;

private:
	Scalar m_data[16];
};

} // namespace lt

#endif // LTPHYS_TRANSFORM_H