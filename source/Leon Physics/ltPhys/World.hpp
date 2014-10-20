#ifndef LTPHYS_WORLD_H
#define LTPHYS_WORLD_H

#include <vector>
#include <list>

#include "../lt3DMath/lt3DMath.hpp"

#include "RigidBody.hpp"
#include "ContactGenerator.hpp"
#include "CollisionShape.hpp"
#include "ForceGeneratorRegistry.hpp"
#include "ContactResolver.hpp"
#include "ContactManifold.hpp"

namespace lt
{

////////////////////////////////////////////////////////////
/// @brief Manages the whole simulation world. 
///
/// Only stores references to the rigid bodies, collision
/// shapes, constrains and such. So changes to those 
/// objects outside the class will be affect the world.
/// Make sure you remove the objects from the system
/// before you delete them.
///
/// @author Leon Turpin
/// @date February 2014
////////////////////////////////////////////////////////////
class World
{
public:
	////////////////////////////////////////////////////////////
	/// @brief Default Constructor
	///
	/// The constructor initializes the physics world
	///
	////////////////////////////////////////////////////////////
	World();

	////////////////////////////////////////////////////////////
	/// @brief Simulates the world for the given timeStep. 
	///
	/// @param timeStep Time to simulate the world for
	///
	////////////////////////////////////////////////////////////
	void stepSimulation(const Scalar& timeStep);
	
	////////////////////////////////////////////////////////////
	/// @brief Register a rigid body to this world. 
	/// 
	/// @param body Rigid Body to add to the world
	///
	////////////////////////////////////////////////////////////
	void addRigidBody(RigidBody* body);
	
	////////////////////////////////////////////////////////////
	/// @brief Removes a rigid body from the world.
	///
	/// @param body Rigid body to remove.
	///
	////////////////////////////////////////////////////////////	
	void removeRigidBody(RigidBody* body);

	////////////////////////////////////////////////////////////		
	/// @brief Add a force generator to the world.
	///
	/// @param body Rigid body for the force generator to 
	/// interact with.
	/// @param forceGenerator Force generator to add to the world.
	///
	////////////////////////////////////////////////////////////		
	void addForceGenerator(RigidBody *body, ForceGenerator *forceGenerator);

	////////////////////////////////////////////////////////////		
	/// @brief Remove force generator from given rigid body
	///
	/// @param body Rigid body to remove force generator from
	/// @param forceGenerator Force generator to remove from 
	/// the body
	///
	////////////////////////////////////////////////////////////		
	void removeForceGenerator(RigidBody *body, ForceGenerator *forceGenerator);

	const std::vector<RigidBody*>& getRigidBodyList();
	const std::vector<ContactManifold>& World::getContactManifolds();	

private:
	std::vector<RigidBody*> m_rigidBodies;
	ForceGeneratorRegistry m_forceGenRegistry;
	ContactResolver contactResolver;
	std::vector<ContactManifold> m_contactManifolds;

	void integrateBodies(const Scalar& timeStep);
};

} // namespace lt

#endif // LTPHYS_WORLD_H