#ifndef LTPHYS_FORCEGENERATOR_H
#define LTPHYS_FORCEGENERATOR_H

#include "../lt3DMath/lt3DMath.hpp"

#include "RigidBody.hpp"

namespace lt
{

////////////////////////////////////////////////////////////
///	@brief Abstract class for force generator. Force generators
/// are registered to a rigid body through the World class.
/// They are called by the World and apply forces to their 
/// associated rigid bodies
///
/// Based heavily on Ian Millington's book "Game Physics Engine Development"
///
/// @author Leon Turpin
/// @date December 2013
////////////////////////////////////////////////////////////
class ForceGenerator
{
public:
	////////////////////////////////////////////////////////////
	/// @brief Called by the World class to apply force to the 
	/// associated rigid body
	////////////////////////////////////////////////////////////
	virtual void updateForce(RigidBody &rigidBody, const Scalar &timeStep) = 0;
};

} // namespace lt

#endif // LTPHYS_FORCEGENERATOR_H