#ifndef LTPHYS_FGENSRPING_H
#define LTPHYS_FGENSRPING_H

#include "../lt3DMath/lt3DMath.hpp"

#include "ForceGenerator.hpp"
#include "RigidBody.hpp"

namespace lt
{

/** FGenSpring.hpp
 *	@brief Applies a spring force on one rigid body, using another rigid
 *  body as an anchor for the end of the spring.
 *
 *  @author Leon Turpin
 *  @date January 2014
 */
class FGenSpring : public ForceGenerator
{
public:
	////////////////////////////////////////////////////////////
	/// @brief Default Constructor
	////////////////////////////////////////////////////////////
	FGenSpring();

	FGenSpring(const Vec3& pivotInParent, RigidBody *other, const Vec3& pivotInOther, const Scalar& springConstant, const Scalar &restLength = 0.0f, bool isStretchOnly = false);

	////////////////////////////////////////////////////////////
	/// @brief Called by the World class to apply force to the 
	/// associated rigid body
	////////////////////////////////////////////////////////////
	void updateForce(RigidBody& parent, const Scalar& timeStep);

	void setPivotInParent(const Vec3& pivot);
	void setOther(RigidBody *other);
	void setPivotInOther(const Vec3& pivot);
	void setSpringConstant(lt::Scalar constant);
	void setRestLength(lt::Scalar length);
	void setIsStretchOnly(bool isStretchOnly);

private:
	RigidBody *m_other; // The body at the other end of the spring

	bool m_isStretchOnly; // If true the spring doesn't "push" only pulls

	Vec3 m_pivotInParent;
	Vec3 m_pivotInOther;

	Scalar m_springConstant;

	Scalar m_restLength;
};

} // namespace lt

#endif // LTPHYS_FGENSRPING_H