#include "ContactManifold.hpp"

namespace lt
{

ContactManifold::ContactManifold(RigidBody &body0, RigidBody &body1)
: m_body0(body0), m_body1(body1)
{}

RigidBody& ContactManifold::getBody0()
{
	return m_body0;
}

RigidBody& ContactManifold::getBody1()
{
	return m_body1;
}

void ContactManifold::addContactPoint(const ContactPoint& contactPt)
{
	m_contactPoints.push_back(contactPt);
}

int ContactManifold::getNumContacts() const
{
	return m_contactPoints.size();
}

ContactPoint ContactManifold::getContactPoint(int index)
{
	return m_contactPoints[index];
}

const ContactPoint ContactManifold::getContactPoint(int index) const
{
	return m_contactPoints[index];
}


} // namespace lt

