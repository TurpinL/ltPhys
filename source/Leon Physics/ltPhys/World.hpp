#ifndef LTPHYS_WORLD_H
#define LTPHYS_WORLD_H

#include <vector>
#include <list>
#include "Vec3.hpp"
#include "Scalar.hpp"
#include "RigidBody.hpp"
#include "CollisionRegistry.hpp"
#include "CollisionShape.hpp"
#include "ForceGeneratorRegistry.hpp"
#include "ContactResolver.hpp"

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
	bool isOldInterpenetration;

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
	/// @brief Registers a rigid body with a collision shape
	///
	/// @param body Rigid body to add to the world
	/// @param shape Collision shape to associate with the
	/// rigid body
	/// @param offset The offset of the collision shape
	/// from the rigid body.
	///
	////////////////////////////////////////////////////////////
	void addRigidBody(RigidBody* body, CollisionShape* shape, const Transform& offset = Transform::Identity());
	
	////////////////////////////////////////////////////////////
	/// @brief Removes a rigid body from the world.
	///
	/// @param body Rigid body to remove.
	///
	////////////////////////////////////////////////////////////	
	void removeRigidBody(RigidBody* body);


	////////////////////////////////////////////////////////////	
	/// @brief Registers a collision shape to a rigid body
	///
	/// @param body Rigid body to add to the world
	/// @param shape Collision shape to associate with the
	/// rigid body
	/// @param offset The offset of the collision shape
	/// from the rigid body.
	///
	////////////////////////////////////////////////////////////	
	void addCollisionShape(RigidBody* body, CollisionShape* shape, const Transform& offset = Transform::Identity());
	
	////////////////////////////////////////////////////////////	
	/// @brief Removes a collision shape from a rigid body.
	///
	/// @param body Rigid body that contains this collision shape
	/// @param shape Collision shape to remove from the body
	///
	////////////////////////////////////////////////////////////		
	void removeCollisionShape(RigidBody* body, CollisionShape* shape);

	////////////////////////////////////////////////////////////	
	/// @brief Removes all collision shapes from a given 
	/// rigid body.
	///
	/// @param body Rigid body to remove collision shapes from.
	///
	////////////////////////////////////////////////////////////		
	void removeCollisionShape(RigidBody* body);

	////////////////////////////////////////////////////////////		
	/// @brief Removes all instances of givin collision shape
	/// from the world.
	///
	/// @param shape Collision shape to remove
	///
	////////////////////////////////////////////////////////////		
	void removeCollisionShape(CollisionShape* shape);

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

	////////////////////////////////////////////////////////////
	/// @brief This is only public for debugging porpoises.
	/// Should definitely be changed to private sometime.
	////////////////////////////////////////////////////////////		
	CollisionRegistry m_collisionRegistry;

	////////////////////////////////////////////////////////////			
	/// @brief Constructs orthonormal basis vectors based on a given X vector.
	///
	/// @param x A normalized vector that you want to base the orthonormal basis around.
	/// @param y calculated basis vector. Not read from in this function.
	/// @param z calculated basis vector. Not read from in this function.
	///
	////////////////////////////////////////////////////////////			 
	static void constructOrthonormalBasis(const Vec3 &x, Vec3* y, Vec3* z);

	////////////////////////////////////////////////////////////			
	/// @brief Constructs orthonormal basis vectors based on a given X vector.
	///
	/// @param x A normalized vector that you want to base the orthonormal basis around.
	///
	////////////////////////////////////////////////////////////	
	static const Mat3 constructOrthonormalBasis(const Vec3 &x);
private:
	std::vector<RigidBody*> m_rigidBodies;
	ForceGeneratorRegistry m_forceGenRegistry;
	ContactResolver contactResolver;

	void integrateBodies(const Scalar& timeStep);
};

} // namespace lt

#endif // LTPHYS_WORLD_H