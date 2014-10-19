#ifndef LTPHYS_CONTACTMANIFOLD_HPP
#define LTPHYS_CONTACTMANIFOLD_HPP

#include <vector>

#include "RigidBody.hpp"
#include "ContactPoint.hpp"

namespace lt
{

class ContactManifold
{
public:
	ContactManifold(RigidBody &body0, RigidBody &body1);

	RigidBody& getBody0();
	RigidBody& getBody1();

	void addContactPoint(const ContactPoint& contactPt);

	int getNumContacts() const;

	ContactPoint getContactPoint(int index);
	const ContactPoint getContactPoint(int index) const;

private:
	RigidBody &m_body0;
	RigidBody &m_body1;
	std::vector<ContactPoint> m_contactPoints;
};

} // namespace lt

#endif // LTPHYS_CONTACTMANIFOLD_HPP