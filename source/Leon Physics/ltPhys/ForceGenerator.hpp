#ifndef LTPHYS_FORCEGENERATOR_H
#define LTPHYS_FORCEGENERATOR_H

#include "Scalar.hpp"
#include "RigidBody.hpp"

namespace lt
{

/** ForceGenerator.hpp
 *	@brief 
 *
 *  Based heavily on Ian Millington's book "Game Physics Engine Development"
 *
 *  @author Leon Turpin
 *  @date December 2013
 */
class ForceGenerator
{
public:
	virtual void updateForce(RigidBody &rigidBody, const Scalar &timeStep) = 0;
};

} // namespace lt

#endif // LTPHYS_FORCEGENERATOR_H