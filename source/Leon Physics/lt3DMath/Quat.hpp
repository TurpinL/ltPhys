#ifndef LTPHYS_QUAT_H
#define LTPHYS_QUAT_H

#include "Scalar.hpp"
#include "Vec3.hpp"

namespace lt
{

////////////////////////////////////////////////////////////
/// @brief Utility class for manipulating Quaternions
///
/// @author Leon Turpin
/// @date December 2013
////////////////////////////////////////////////////////////
class Quat
{
public:
	////////////////////////////////////////////////////////////
    /// @brief Default constructor
    ///
    /// Creates a Quat(0, 0, 0, 1).
    ///
    ////////////////////////////////////////////////////////////
	Quat();

	////////////////////////////////////////////////////////////
    /// @brief Construct a quaternion from its components
    ///
    /// @param X X component
    /// @param Y Y component
    /// @param Z Z component
    /// @param W W component
    ///
    ////////////////////////////////////////////////////////////
	Quat(const Scalar x, const Scalar y, const Scalar z, const Scalar w);
	
	////////////////////////////////////////////////////////////
    /// @brief Construct a quaternion from an axis angle
    ///
    /// @param axis Axis to rotate around
	/// @param angle Angle of rotation in degrees
    ///
    ////////////////////////////////////////////////////////////
	Quat(const Vec3 &axis, const Scalar angle);

	//Quat(const Scalar yaw, const Scalar pitch, const Scalar roll);
	//const Quat operator + (const Quat &rhs) const;
	//const Quat operator - (const Quat &rhs) const;

	////////////////////////////////////////////////////////////
    /// @brief Performs quaternion multiplication. Used to 
	/// combine quaternions.
    ///
	/// @param rhs right operand (a quaternion)
    ///
	/// @return Quaternion product.
	///
    ////////////////////////////////////////////////////////////
	const Quat operator * (const Quat &rhs) const;

	//const Quat& operator += (const Quat &rhs);
	//const Quat& operator -= (const Quat &rhs);

	////////////////////////////////////////////////////////////
    /// @brief Performs quaternion multiplication. Used to 
	/// combine quaternions and assigns the result to this quat.
    ///
	/// @param rhs right operand (a quaternion)
	///
	/// @return Reference to this quaternion
    ///
    ////////////////////////////////////////////////////////////
	Quat& operator *= (const Quat &rhs);

	//const Quat& operator *= (const Scalar rhs);
	//const Scalar dot(const Quat &rhs) const;

	////////////////////////////////////////////////////////////
	/// @brief Get the length of the quaternion
	///
	/// @return Length of the quaternion
	/// 
    ////////////////////////////////////////////////////////////
	const Scalar length() const;

	////////////////////////////////////////////////////////////
	/// @brief Get the squared length of the quaternion
	///
	/// @return Squared length of the quaternion
	/// 
    ////////////////////////////////////////////////////////////
	const Scalar length2() const;

	//const Scalar angle(const Quat &rhs) const;

	////////////////////////////////////////////////////////////
	/// @brief Get the angle of the quaternion. To be paired with
	/// the getAxis() function for a full representation of this
	/// quaternion's orientation.
	///
	/// @return Angle of the quaternion
	/// 
    ////////////////////////////////////////////////////////////
	const Scalar getAngle() const;

	////////////////////////////////////////////////////////////
	/// @brief Get the axis of the quaternion. To be paired with
	/// the getAxis() function for a full representation of this
	/// quaternion's orientation.
	///
	/// @return axis of the quaternion
	/// 
    ////////////////////////////////////////////////////////////
	const Vec3 getAxis() const;

	//const Quat& inverse() const;

	////////////////////////////////////////////////////////////
	/// @brief Normalize the quaternion. Though a quaternion 
	/// should never not be normalized, they tend to become 
	/// unormalized due to floating point inaccuracies.
    ////////////////////////////////////////////////////////////
	Quat& normalize();

	Scalar x; ///< X Component
	Scalar y; ///< Y Component
	Scalar z; ///< Z Component
	Scalar w; ///< W Component
};

} // namespace lt

#endif // LTPHYS_QUAT_H
