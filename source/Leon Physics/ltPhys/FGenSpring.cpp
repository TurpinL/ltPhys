#include "FGenSpring.hpp"

#include <cmath>

namespace lt
{
FGenSpring::FGenSpring() {}

FGenSpring::FGenSpring(const Vec3& pivotInParent, RigidBody* other, const Vec3& pivotInOther, const Scalar& springConstant, const Scalar &restLength, bool isStretchOnly)
: m_pivotInParent(pivotInParent), m_pivotInOther(pivotInOther), m_other(other), m_springConstant(springConstant), m_restLength(restLength), m_isStretchOnly(isStretchOnly)
{}

void FGenSpring::updateForce(RigidBody &parent, const Scalar &timeStep)
{
	Vec3 relPivotInParent = parent.getPointInWorldSpace(m_pivotInParent);
	Vec3 relPivotInOther = m_other->getPointInWorldSpace(m_pivotInOther);

	Vec3 force = parent.getPosition() + relPivotInParent;
	force -= m_other->getPosition() + relPivotInOther;

	Scalar magnitude = force.length();
	magnitude -= m_restLength;

	if(!m_isStretchOnly || (m_isStretchOnly && magnitude > 0))
	{
		magnitude *= m_springConstant;

		force.normalize();
		force *= -magnitude;
		parent.applyForce(force, relPivotInParent);
	}
}

void FGenSpring::setPivotInParent(const Vec3& pivot) {m_pivotInParent = pivot;}
void FGenSpring::setOther(RigidBody *other) {m_other = other;}
void FGenSpring::setPivotInOther(const Vec3& pivot) {m_pivotInOther = pivot;}
void FGenSpring::setSpringConstant(lt::Scalar constant) {m_springConstant = constant;}
void FGenSpring::setRestLength(lt::Scalar length) {m_restLength = length;}
void FGenSpring::setIsStretchOnly(bool isStretchOnly) {m_isStretchOnly = isStretchOnly;}

} // namespace lt