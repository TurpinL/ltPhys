#ifndef LTPHYS_CONTACT_H
#define LTPHYS_CONTACT_H

#include "Scalar.hpp"
#include "Vec3.hpp"
#include "Transform.hpp"
#include "RigidBody.hpp"

namespace lt
{

/**
 * "A contact represents two bodies in contact. Resolving a
 * contact removes their interpenetration, and applies sufficient
 * impulse to keep them apart. Colliding bodies may also rebound.
 * Contacts can be used to represent positional joints, by making
 * the contact constraint keep the bodies in their correct
 * orientation." - Ian Millington
 *
 *  @author Leon Turpin
 *  @date February 2014
 */
struct Contact
{
	/** The position of the contact in world co-ordinates */
	Vec3 position; 

	/** Direction of the contact in world co-ordinates points from body 2 to body 1 */
	Vec3 normal; 

	/** 
	 * The depth of penetration.
	 * If both bodies are specified then 
	 * the contact point should be midway between
	 * the inter-penetrating points. 
	 */
	Scalar penetration; 

	/** Set it to 1, unless there's multiple points of contact in the same collision. Then it should be set to 1/numContacts */
	Scalar impulseModifier;

	/** Stores the two bodies that are in contact */
	RigidBody* body[2];
};

/**
 * "A helper structure that contains information for the detector to use
 * in building its contact data." - Ian Millington
 *
 *  @author Leon Turpin
 *  @date February 2014
 */
struct CollisionData
{
	/** An array of contacts to write to. */
	Contact *contacts;

	/** The maximum number of contacts "Contact *contacts" can store. */
	unsigned int size;

	/** The number of elements left in "Contact *contacts" */
	unsigned int contactsLeft;
};

class CollisionShape;
/**
 *  @author Leon Turpin
 *  @date February 2014
 */
struct CollisionRegistration 
{
	CollisionShape *shape; // Required
	RigidBody *body; // Can be null
	Transform offset;
};

}

#endif // LTPHYS_CONTACT_H