#ifndef LTPHYS_CONTACTGENERATOR_H
#define LTPHYS_CONTACTGENERATOR_H

#include <vector>

#include "../lt3DMath/lt3DMath.hpp"

#include "CollisionShape.hpp"
#include "ShapeSphere.hpp"
#include "ShapeHalfspace.hpp"
#include "ShapeBox.hpp"
#include "ContactManifold.hpp"
#include "ContactPoint.hpp"

namespace lt
{

////////////////////////////////////////////////////////////
/// @brief Provides a set of static functions to check contact with.
///
/// @author Leon Turpin
/// @date February 2014
////////////////////////////////////////////////////////////
class ContactGenerator
{
public:	
	static void generateContacts(std::vector<RigidBody*>& rigidBodies, std::vector<ContactManifold>& contactManifolds);
	static void checkCollision(RigidBody &rbA, RigidBody &rbB, std::vector<ContactManifold>& contactManifolds);

	////////////////////////////////////////////////////////////
	/// @brief Check for contact between a sphere and a sphere
	////////////////////////////////////////////////////////////
	static void sphere_sphere(const CollisionShape &a, const RigidBody &rbA, const CollisionShape &b, const RigidBody &rbB, ContactManifold &contactManifold);

	//static unsigned int sphere_box(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData);

	////////////////////////////////////////////////////////////
	/// @brief Check for contact between a sphere and a halfspace
	////////////////////////////////////////////////////////////
	static void sphere_halfspace(const CollisionShape &a, const RigidBody &rbA, const CollisionShape &b, const RigidBody &rbB, ContactManifold &contactManifold);

	////////////////////////////////////////////////////////////
	/// @brief Check for contact between a box and box
	////////////////////////////////////////////////////////////
	static void box_box(const CollisionShape &a, const RigidBody &rbA, const CollisionShape &b, const RigidBody &rbB, ContactManifold &contactManifold);

	////////////////////////////////////////////////////////////
	/// @brief Check for contact between a box and a halfspace
	////////////////////////////////////////////////////////////
	static void box_halfspace(const CollisionShape &a, const RigidBody &rbA, const CollisionShape &b, const RigidBody &rbB, ContactManifold &contactManifold);
};

} // namespace lt

#endif // LTPHYS_CONTACTGENERATOR_H
