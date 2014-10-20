#ifndef LTPHYS_VEC3_H
#define LTPHYS_VEC3_H

#include "Scalar.hpp"

namespace lt
{

/**  Vec3.h
 *  Header file for the Vec3 class
 *	@brief A C++ implementation of an XYZ vector.
 *
 *  \author Leon Turpin
 *  \date August 2013
 */
class Vec3
{
public:
	////////////////////////////////////////////////////////////
    /// \brief Default constructor
    ///
    /// Creates a Vector3(0, 0, 0, 1).
    ///
    ////////////////////////////////////////////////////////////
	Vec3();
	
	/**
	 * @brief Initializes a vector with a set of values
	 *
	 * @param x The x component
	 * @param y The y component
	 * @param z The z component
	 */
	Vec3(const Scalar& x, const Scalar& y, const Scalar& z);

	/**
	 * @brief Initializes a vector with a set of values
	 *
	 * @param x The x component
	 * @param y The y component
	 * @param z The z component
	 * @param w The w component
	 */
	Vec3(const Scalar& x, const Scalar& y, const Scalar& z, const Scalar& w);

	/**
	 * @brief Adds two vectors together.
	 *
	 * @param rhs The right hand side of the equation
	 */
    const Vec3 operator+ (const Vec3& rhs) const;

   /**
	 * @brief Subtracts the left vector from the right vector.
	 *
	 * @param rhs The right hand side of the equation
	 */
	const Vec3 operator- (const Vec3& rhs) const;
   
	/**
	 * @brief Adds a scalar to a vector.
	 *
	 * @param rhs The right hand side of the equation
	 */
	const Vec3 operator+ (const Scalar& scalar) const;
    
	/**
	 * @brief Subtracts a scalar from a vector.
	 *
	 * @param rhs The right hand side of the equation
	 */
	const Vec3 operator- (const Scalar& scalar) const;
   
	/**
	 * @brief Multiplies a vector by a scalar.
	 *
	 * @param rhs The right hand side of the equation
	 */
	const Vec3 operator* (const Scalar& scalar) const;
	
	/**
	 * @brief Divides a vector by a scalar.
	 *
	 * @param rhs The right hand side of the equation
	 */
	const Vec3 operator/ (const Scalar& scalar) const;

	/**
	 * @brief returns the vector with each component negated
	 */
	const Vec3 operator- () const;

	/** 
	 * @brief Overload of binary operator +=
	 *
	 * Performs a memberwise addition of both vectors,
	 * then assignesthe result to the left operand.
	 *
	 * @param rhs Right operand.
	 */
	Vec3& operator+= (const Vec3& rhs);

	/** 
	 * @brief Overload of binary operator -=
	 *
	 * Performs a memberwise subtraction of both vectors,
	 * then assignesthe result to the left operand.
	 *
	 * @param rhs Right operand.
	 */
	Vec3& operator-= (const Vec3& rhs);

	/** 
	 * @brief Overload of binary operator *=
	 *
	 * Performs a memberwise multiplication by the right operand,
	 * then assignesthe result to the left operand.
	 *
	 * @param rhs Right operand.
	 */
	Vec3& operator*= (const Scalar& rhs);

	/** 
	 * @brief Overload of binary operator /=
	 *
	 * Performs a memberwise division by the right operand,
	 * then assignesthe result to the left operand.
	 *
	 * @param rhs Right operand.
	 */
	 Vec3& operator/= (const Scalar& rhs);

	/**
	 * @brief Calculates the dot product of the two vectors
	 *
	 * @param rhs The right hand side of the equation
	 */
	const Vec3 cross(const Vec3 &rhs) const;
   
	/**
	 * @brief Calculates the dot product of the two vectors
	 *
	 * @param rhs The right hand side of the equation
	 */
	const Scalar dot(const Vec3 &rhs) const;

    /**
	 * @brief Calculates the length of a vector.
	 *
	 * @param rhs The right hand side of the equation
	 */
	const Scalar length() const;

	/**
	 * @brief Normalizes the vector making it a unit vector.
	 */
	Vec3& normalize();

	/**
	 * @brief Returns the vector, normalized.
	 */
	const Vec3 normalized() const;

	/**
	 * @brief Checks equality with a degree of error
	 *
	 * @param rhs The vector to compare with
	 * @param degOfError The degree of error
	 */
	const bool nearEqual( const Vec3 &rhs, const Scalar &degOfError ) const;

	/**
	 * @brief Rotates the vector around a specified axis.
	 *
	 * @param angle The angle of rotation.
	 * @param axis The axis to rotate about.
	 */
	const Vec3 rotate(const Scalar &angle, const Vec3 &axis) const;

	/**
	 * @brief accesses the vector components.
	 *
	 * @param index x = 0, y = 1, z = 2, w = 3.
	 */
	Scalar& operator[] (unsigned int index); 

	/**
	 * @brief accesses the vector components.
	 *
	 * @param index x = 0, y = 1, z = 2, w = 3.
	 */
	const Scalar& get(unsigned int index) const;

	Scalar x, y, z, w; 
};

} // namespace lt

#endif // LTPHYS_VEC3_H