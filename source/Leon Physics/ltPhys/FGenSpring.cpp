#include "FGenSpring.hpp"

#include <cmath>

namespace lt
{
	FGenSpring::FGenSpring() {}

	FGenSpring::FGenSpring(RigidBody *other, const Scalar &springConstant, const Scalar &restLength)
	: m_other(other), m_springConstant(springConstant), m_restLength(restLength)
	{}

	void FGenSpring::updateForce(RigidBody &rigidBody, const Scalar &timeStep)
	{
		Vec3 force = rigidBody.getPosition();
		force -= m_other->getPosition();

		Scalar magnitude = force.length();
		magnitude -= m_restLength;
		magnitude *= m_springConstant;

		force.normalize();
		force *= -magnitude;
		rigidBody.applyCentralForce(force);
	}

} // namespace lt