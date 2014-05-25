#ifndef LTPHYS_FORCEGENERATORREGISTRY_H
#define LTPHYS_FORCEGENERATORREGISTRY_H

#include "RigidBody.hpp"
#include "ForceGenerator.hpp"
#include <vector>

namespace lt
{

/** 
 *	@brief 
 *  @author Leon Turpin
 *  @date December 2013
 */
struct ForceGenRegistration
{
	RigidBody *body;
	ForceGenerator *forceGen;
};

/** ForceGeneratoRegistry.hpp
 *	@brief 
 *
 *  Based heavily on Ian Millington's book "Game Physics Engine Development"
 *
 *  @author Leon Turpin
 *  @date December 2013
 */
class ForceGeneratorRegistry
{
public:
	ForceGeneratorRegistry();

	void updateForces(Scalar timeStep);

	void add(RigidBody *body, ForceGenerator *forceGenerator);
	void remove(RigidBody *body, ForceGenerator *forceGenerator);
	void remove(RigidBody *body);
	void remove(ForceGenerator *forceGenerator);
	void clear();

private:
	std::vector<ForceGenRegistration> m_registry;

	void _removeElement(int index);
};

} // namespace lt

#endif // LTPHYS_FORCEGENERATORREGISTRY_H