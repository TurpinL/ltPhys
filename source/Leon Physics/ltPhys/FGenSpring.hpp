#ifndef LTPHYS_FGENSRPING2_H
#define LTPHYS_FGENSRPING2_H

#include "../lt3DMath/lt3DMath.hpp"

#include "ForceGenerator.hpp"
#include "RigidBody.hpp"

namespace lt
{

/** FGenSpring2.hpp
 *	@brief 
 *
 *  @author Leon Turpin
 *  @date January 2014
 */
class FGenSpring2 : public ForceGenerator
{
public:
	FGenSpring2();

	FGenSpring2(const Vec3& pivotInParent, RigidBody *other, const Vec3& pivotInOther, const Scalar& springConstant, const Scalar &restLength = 0.0f, bool isStretchOnly = false);

	void updateForce(RigidBody& parent, const Scalar& timeStep);

private:
	RigidBody *m_other; // The body at the other end of the spring

	bool m_isStretchOnly; // If true the spring doesn't "push" only pulls

	Vec3 m_pivotInParent;
	Vec3 m_pivotInOther;

	Scalar m_springConstant;

	Scalar m_restLength;
};

} // namespace lt

#endif // LTPHYS_FGENSRPING2_H