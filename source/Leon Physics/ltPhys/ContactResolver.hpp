#ifndef LTPHYS_CONTACTRESOLVER_H
#define LTPHYS_CONTACTRESOLVER_H

#include <list>
#include <vector>

#include "ContactManifold.hpp"

namespace lt
{

////////////////////////////////////////////////////////////
/// @brief Stores the velocity response of one collision 
////////////////////////////////////////////////////////////
struct CollisionResponse
{
	RigidBody *body;
	Vec3 changeInVelocity;
	Vec3 changeInAngularVelocity;
};

/** ContactResolver.hpp
 *	@brief Resolves the contacts found by the contact generator.
 *
 *  @author Leon Turpin
 *  @date May 2014
 */
class ContactResolver
{
public:
	////////////////////////////////////////////////////////////		
	/// @brief Applies velocities and moves objects to resolve 
	/// interpenetration and collision forces.
	///
	/// @param colData List of collisions to resolve.
	///
	////////////////////////////////////////////////////////////			
	void resolveContacts(std::vector<ContactManifold> &contactManifolds);
private:
	void resolveMotion(ContactManifold& manifold, std::list<CollisionResponse> &collisionResponseRegistry);
	void calcImpulse(ContactManifold& contact, std::list<CollisionResponse> &collisionResponseRegistry);
	void resolveAllInterpenetrations(std::vector<ContactManifold> &contactManifolds);
	void resolveInterpenetration(ContactManifold& manifold);
};

} // namespace lt

#endif // LTPHYS_CONTACTGENERATOR_H
