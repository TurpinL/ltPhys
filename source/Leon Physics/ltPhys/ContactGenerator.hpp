#ifndef LTPHYS_CONTACTGENERATOR_H
#define LTPHYS_CONTACTGENERATOR_H

#include "Contact.hpp"
#include "CollisionShape.hpp"
#include "ShapeSphere.hpp"
#include "ShapeHalfspace.hpp"
#include "ShapeBox.hpp"
#include "ShapeTerrain.hpp"
#include "Scalar.hpp"
#include "Transform.hpp"
#include "Vec3.hpp"
#include "Quat.hpp"

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
	////////////////////////////////////////////////////////////
	/// @brief Check for contact between a sphere and a sphere
	////////////////////////////////////////////////////////////
	static unsigned int sphere_sphere(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData);

	//static unsigned int sphere_box(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData);

	////////////////////////////////////////////////////////////
	/// @brief Check for contact between a sphere and a halfspace
	////////////////////////////////////////////////////////////
	static unsigned int sphere_halfspace(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData);

	////////////////////////////////////////////////////////////
	/// @brief Check for contact between a sphere and terrain
	////////////////////////////////////////////////////////////
	static unsigned int sphere_terrain(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData);

	////////////////////////////////////////////////////////////
	/// @brief Check for contact between a box and box
	////////////////////////////////////////////////////////////
	static unsigned int box_box(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData);

	////////////////////////////////////////////////////////////
	/// @brief Check for contact between a box and a halfspace
	////////////////////////////////////////////////////////////
	static unsigned int box_halfspace(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData);

	////////////////////////////////////////////////////////////
	/// @brief Check for contact between a box and a terrain
	////////////////////////////////////////////////////////////
	static unsigned int box_terrain(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData);
};

} // namespace lt

#endif // LTPHYS_CONTACTGENERATOR_H
