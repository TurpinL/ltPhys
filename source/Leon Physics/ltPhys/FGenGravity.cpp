#include "FGenGravity.hpp"

namespace lt
{
FGenGravity::FGenGravity() {}

FGenGravity::FGenGravity(const Scalar &gravity)
: m_gravity(gravity)
{}

void FGenGravity::updateForce(RigidBody &rigidBody, const Scalar &timeStep)
{
	const Scalar& invMass = rigidBody.getInvMass();

	if(invMass != 0)
	{
		rigidBody.applyCentralForce( Vec3(0, m_gravity / rigidBody.getInvMass(), 0) );
	}
}

void FGenGravity::setGravity(const Scalar &gravity)
{
	m_gravity = gravity;
}

const Scalar FGenGravity::getGravity() const
{
	return m_gravity;
}

} // namespace lt