#ifndef LTPHYS_CONTACTMANIFOLD_HPP
#define LTPHYS_CONTACTMANIFOLD_HPP

#include <vector>

#include "RigidBody.hpp"
#include "ContactPoint.hpp"

namespace lt
{

////////////////////////////////////////////////////////////
/// @brief Stores a set of contacts between two rigid bodies.
///
/// @author Leon Turpin
/// @date October 2014
////////////////////////////////////////////////////////////
class ContactManifold
{
public:
	////////////////////////////////////////////////////////////
	/// @brief Construct a new contact manifold between the two bodies
	///
	/// @param body0 A body of the manifold
	/// @param body1 Another body of the manifold
    ////////////////////////////////////////////////////////////
	ContactManifold(RigidBody &body0, RigidBody &body1);

	////////////////////////////////////////////////////////////
	/// @brief Get the first body of the manifold
	/// 
	/// @return The first body of the manifold
    ////////////////////////////////////////////////////////////
	RigidBody& getBody0();

	////////////////////////////////////////////////////////////
	/// @brief Get the second body of the manifold
	/// 
	/// @return The second body of the manifold
    ////////////////////////////////////////////////////////////
	RigidBody& getBody1();

	////////////////////////////////////////////////////////////
	/// @brief Add a contact point to the manifold.
	/// 
	/// @param contactPt The new contact to add to the manifold.
    ////////////////////////////////////////////////////////////
	void addContactPoint(const ContactPoint& contactPt);

	////////////////////////////////////////////////////////////
	/// @brief Get the number of contact points stored in the manifold.
	/// 
	/// @return Number of contact points stored in the manifold.
    ////////////////////////////////////////////////////////////
	int getNumContacts() const;

	////////////////////////////////////////////////////////////
	/// @brief Get the contact point at the specified index.
	/// 
	/// @param index The index of the contact point to get.
	///
	/// @return The contact point at the specified index.
    ////////////////////////////////////////////////////////////
	ContactPoint getContactPoint(int index);

	////////////////////////////////////////////////////////////
	/// @brief Get the contact point at the specified index.
	/// 
	/// @param index The index of the contact point to get.
	///
	/// @return The contact point at the specified index.
    ////////////////////////////////////////////////////////////
	const ContactPoint getContactPoint(int index) const;

private:
	RigidBody &m_body0;
	RigidBody &m_body1;
	std::vector<ContactPoint> m_contactPoints;
};

} // namespace lt

#endif // LTPHYS_CONTACTMANIFOLD_HPP