#ifndef LTPHYS_COLLISIONREGISTRY_H
#define LTPHYS_COLLISIONREGISTRY_H

#include <vector>
#include "Contact.hpp"
#include "CollisionShape.hpp"

namespace lt
{

////////////////////////////////////////////////////////////
///	@brief Associates collision shapes with rigid bodies
/// and stores find contacts.
///
/// @author Leon Turpin
/// @date February 2014
////////////////////////////////////////////////////////////
class CollisionRegistry
{
public:	
	////////////////////////////////////////////////////////////
	/// @brief Construct a collision registry with a default 
	/// max of 100 contacts
	////////////////////////////////////////////////////////////
	CollisionRegistry();

	////////////////////////////////////////////////////////////
	/// @brief Construct a collision registry with a given number
	/// of max contact
	///
	/// @param maxContacts Number of max contacts
	///
	////////////////////////////////////////////////////////////
	CollisionRegistry(unsigned int maxContacts);

	////////////////////////////////////////////////////////////
	/// @brief Test for collisions between shapes in the regitry.
	/// 
	/// Modifies the collisionData, use getCollisionData to review
	/// collisions
	////////////////////////////////////////////////////////////
	void findContacts();

	////////////////////////////////////////////////////////////
	/// @brief Adds a collision shape at an offset to a rigid body
	/// to the system.
	///
	/// @param body Body to add to the system
	/// @param shape Collision shape to associate with the rigid body
	/// @param offset Offset offset of collision shape from rigid body
	///
	////////////////////////////////////////////////////////////
	void add(RigidBody* body, CollisionShape* shape, const Transform& offset = Transform::Identity());

	////////////////////////////////////////////////////////////
	/// @brief Removes a rigid body and shape pair from the system
	/// 
	/// @param body Rigid body to remove.
	/// @param shape Collision shape to remove.
	/// 
	////////////////////////////////////////////////////////////
	void remove(RigidBody* body, CollisionShape* shape);

	////////////////////////////////////////////////////////////
	/// @brief Remove a rigid body from the system
	///
	/// @param body Rigid body to remove.
	///
	////////////////////////////////////////////////////////////
	void remove(RigidBody* body);

	////////////////////////////////////////////////////////////
	/// @brief Remove a collision shape from the system
	///
	/// @param shape Collision shape to remove.
	///
	////////////////////////////////////////////////////////////
	void remove(CollisionShape* shape);

	////////////////////////////////////////////////////////////
	/// @brief Remove all shapes and bodies from the system
	////////////////////////////////////////////////////////////
	void clear();

	////////////////////////////////////////////////////////////
	/// @brief Set the max number of contact the system will
	/// try to find before it stops.
	///
	/// @param newMaxContacts Number of contacts to try to find
	///
	////////////////////////////////////////////////////////////
	void setMaxContacts(unsigned int newMaxContacts);

	////////////////////////////////////////////////////////////
	/// @brief Get the max number of contact the system will
	/// try to find before it stops.
	///
	/// @return newMaxContacts Number of contacts to try to find
	///
	////////////////////////////////////////////////////////////
	unsigned int getMaxContacts() const;

	////////////////////////////////////////////////////////////
	/// @brief Get the data created from last findContacts() call 
	///
	/// @return Collision data
	///
	////////////////////////////////////////////////////////////
	const CollisionData& getCollisionData() const;


private:
	CollisionData m_collisionData;
	std::vector<CollisionRegistration> m_registry;

	void removeElement(int index);

	/** Takes two CollisionRegistrations and finds the best collision function for the pair. */
	void checkCollision(const CollisionRegistration& shapeA, const CollisionRegistration& shapeB);
};

} // namespace lt

#endif // LTPHYS_COLLISIONREGISTRY_H
