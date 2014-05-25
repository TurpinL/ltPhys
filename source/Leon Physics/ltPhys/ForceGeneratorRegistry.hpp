#ifndef LTPHYS_FORCEGENERATORREGISTRY_H
#define LTPHYS_FORCEGENERATORREGISTRY_H

#include "RigidBody.hpp"
#include "Scalar.hpp"
#include "ForceGenerator.hpp"
#include <vector>

namespace lt
{

////////////////////////////////////////////////////////////
/// @brief Associates a force generator with a rigid body
/// to be passed into the ForceGeneratorRegistry
/// 
/// @author Leon Turpin
/// @date December 2013
////////////////////////////////////////////////////////////
struct ForceGenRegistration
{
	RigidBody *body;
	ForceGenerator *forceGen;
};

////////////////////////////////////////////////////////////
///	@brief Registers force generators to rigid bodies and 
/// supplies a set of functions to manage the relations
///
/// Based heavily on Ian Millington's book "Game Physics Engine Development"
///
/// @author Leon Turpin
/// @date December 2013
////////////////////////////////////////////////////////////
class ForceGeneratorRegistry
{
public:
	////////////////////////////////////////////////////////////
	/// @brief Defautl Constructor
	////////////////////////////////////////////////////////////
	ForceGeneratorRegistry();

	////////////////////////////////////////////////////////////
	/// @brief Calls the all the force generator's updaet functions
	/// 
	/// @param timeStep passed to the force generators as the
	/// time in seconds that has elapsed since the last update.
	/// 
	////////////////////////////////////////////////////////////
	void updateForces(Scalar timeStep);

	////////////////////////////////////////////////////////////
	/// @brief Register a force generator with a rigid body to the system
	////////////////////////////////////////////////////////////
	void add(RigidBody *body, ForceGenerator *forceGenerator);

	////////////////////////////////////////////////////////////
	/// @brief Remove all instances of the given body with the given 
	/// force generator from the system.
	////////////////////////////////////////////////////////////	
	void remove(RigidBody *body, ForceGenerator *forceGenerator);

	////////////////////////////////////////////////////////////	
	/// @brief Remove all registrations with the given rigid 
	/// body from the system
	////////////////////////////////////////////////////////////	
	void remove(RigidBody *body);

	////////////////////////////////////////////////////////////	
	/// @brief Remove all registrations with the given force generator
	/// from the system
	////////////////////////////////////////////////////////////	
	void remove(ForceGenerator *forceGenerator);

	////////////////////////////////////////////////////////////	
	/// @brief Clear all registrations from the system
	////////////////////////////////////////////////////////////	
	void clear();

private:
	std::vector<ForceGenRegistration> m_registry;

	void _removeElement(int index);
};

} // namespace lt

#endif // LTPHYS_FORCEGENERATORREGISTRY_H