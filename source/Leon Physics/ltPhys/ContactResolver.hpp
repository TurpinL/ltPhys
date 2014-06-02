#ifndef LTPHYS_CONTACTRESOLVER_H
#define LTPHYS_CONTACTRESOLVER_H

#include "Contact.hpp"
#include <list>

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
	void resolveContacts(const lt::CollisionData &colData);
private:
	void resolveMotion(const Contact& contact, std::list<CollisionResponse> &collisionResponseRegistry);
	void calcImpulse(const Contact& contact, std::list<CollisionResponse> &collisionResponseRegistry);
	void resolveAllInterpenetrations(const lt::CollisionData &colData);
	void resolveInterpenetration(Contact& contact, Vec3 angleChange[2], Vec3 positionChange[2]);
	void recalcPenetrations(const lt::CollisionData &colData, Vec3 angleChange[2], Vec3 positionChange[2], const Contact& deepestPenetrator);

};

} // namespace lt

#endif // LTPHYS_CONTACTGENERATOR_H
