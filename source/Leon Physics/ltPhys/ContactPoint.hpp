#ifndef LTPHYS_CONTACTPOINT_HPP
#define LTPHYS_CONTACTPOINT_HPP

#include "../lt3DMath/lt3DMath.hpp"

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
 *  @date October 2014
 */
struct ContactPoint
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
};

} // namespace lt

#endif // LTPHYS_CONTACTPOINT_HPP