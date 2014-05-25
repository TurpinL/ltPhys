#ifndef LTPHYS_COLLISIONREGISTRY_H
#define LTPHYS_COLLISIONREGISTRY_H

#include <vector>
#include "Contact.hpp"
#include "CollisionShape.hpp"

namespace lt
{

/** CollisionRegistry.hpp
 *	@brief 
 *
 *  @author Leon Turpin
 *  @date February 2014
 */
class CollisionRegistry
{
public:	
	CollisionRegistry();
	CollisionRegistry(unsigned int maxContacts);

	void findContacts();

	void add(RigidBody* body, CollisionShape* shape, const Transform& offset = Transform::Identity());
	void remove(RigidBody* body, CollisionShape* shape);
	void remove(RigidBody* body);
	void remove(CollisionShape* shape);
	void clear();

	void setMaxContacts(unsigned int newMaxContacts);
	unsigned int getMaxContacts() const;
	const CollisionData& getCollisionData() const;


private:
	CollisionData m_collisionData;
	std::vector<CollisionRegistration> m_registry;

	void _removeElement(int index);

	/** Takes two CollisionRegistrations and finds the best collision function for the pair. */
	void _checkCollision(const CollisionRegistration& shapeA, const CollisionRegistration& shapeB);
};

} // namespace lt

#endif // LTPHYS_COLLISIONREGISTRY_H
