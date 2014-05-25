#ifndef LTPHYS_CONTACTRESOLVER_H
#define LTPHYS_CONTACTRESOLVER_H

#include "Contact.hpp"
#include "CollisionShape.hpp"
#include "ShapeSphere.hpp"
#include "ShapeHalfspace.hpp"
#include "ShapeBox.hpp"
#include "ShapeTerrain.hpp"
#include "Transform.hpp"

namespace lt
{

/** ContactResolver.hpp
 *	@brief
 *
 *  @author Leon Turpin
 *  @date May 2014
 */
class ContactResolver
{
public:
	static unsigned int sphere_sphere(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData);
	static unsigned int sphere_box(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData);
	static unsigned int sphere_halfspace(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData);
	static unsigned int sphere_terrain(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData);
	//static unsigned int box_box(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData);
	static unsigned int box_halfspace(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData);
	static unsigned int box_terrain(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData);
};

} // namespace lt

#endif // LTPHYS_CONTACTGENERATOR_H
