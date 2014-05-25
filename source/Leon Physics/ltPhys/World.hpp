#ifndef LTPHYS_WORLD_H
#define LTPHYS_WORLD_H

#include <vector>
#include <list>
#include "Scalar.hpp"
#include "RigidBody.hpp"
#include "CollisionRegistry.hpp"
#include "CollisionShape.hpp"
#include "ForceGeneratorRegistry.hpp"

namespace lt
{

struct CollisionResponse
{
	RigidBody *body;
	Vec3 changeInVelocity;
	Vec3 changeInAngularVelocity;
};

/**  World.h
 *  Header file for the World class
 *	@brief 
 *
 *  @author Leon Turpin
 *  @date February 2014
 */
class World
{
public:
	bool isOldInterpenetration;

	World();

	void stepSimulation(const Scalar& timeStep);
	
	void addRigidBody(RigidBody* body);
	void addRigidBody(RigidBody* body, CollisionShape* shape, const Transform& offset = Transform::Identity());
	void removeRigidBody(RigidBody* body);

	void addCollisionShape(RigidBody* body, CollisionShape* shape, const Transform& offset = Transform::Identity());
	void removeCollisionShape(RigidBody* body, CollisionShape* shape);
	void removeCollisionShape(RigidBody* body);
	void removeCollisionShape(CollisionShape* shape);

	void addForceGenerator(RigidBody *body, ForceGenerator *forceGenerator);
	void removeForceGenerator(RigidBody *body, ForceGenerator *forceGenerator);

	CollisionRegistry m_collisionRegistry;

	/**
	 * @brief Constructs orthonormal basis vectors based on a given X vector.
	 *
	 * @param x A normalized vector that you want to base the orthonormal basis around.
	 * @param y calculated basis vector. Not read from in this function.
	 * @param z calculated basis vector. Not read from in this function.
	 */
	static void constructOrthonormalBasis(const Vec3 &x, Vec3* y, Vec3* z);
	static const Mat3 constructOrthonormalBasis(const Vec3 &x);
private:
	std::vector<RigidBody*> m_rigidBodies;
	ForceGeneratorRegistry m_forceGenRegistry;

	void _resolveCollisions(const Scalar& timeStep);
	void _resolveCollisionMotion(const Contact& contact, std::list<CollisionResponse> &collisionResponseRegistry);
	void _calcCollisionImpulse(const Contact& contact, std::list<CollisionResponse> &collisionResponseRegistry);
	void _resolveCollisionInterpenetration(const Contact& contact);
	void _integrateBodies(const Scalar& timeStep);
};

} // namespace lt

#endif // LTPHYS_WORLD_H