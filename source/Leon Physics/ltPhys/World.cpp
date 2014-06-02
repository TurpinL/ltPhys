#include "World.hpp"

namespace lt
{

//--------------------------
//	PUBLICS			
//--------------------------

World::World()
{
	isOldInterpenetration = false;
	m_collisionRegistry.setMaxContacts(100);
}

void World::stepSimulation(const Scalar& timeStep)
{
	// Update forces
	m_forceGenRegistry.updateForces(timeStep);

	// Move bodies
	integrateBodies(timeStep);

	// Detect Contacts
	m_collisionRegistry.findContacts();
	// Then resolve them
	contactResolver.resolveContacts(m_collisionRegistry.getCollisionData());
}

void World::addRigidBody(RigidBody* body)
{
	// Add the body
	m_rigidBodies.push_back(body);
}

void World::addRigidBody(RigidBody* body, CollisionShape* shape, const Transform& offset)
{
	addRigidBody(body);

	m_collisionRegistry.add(body, shape, offset);
}

void World::removeRigidBody(RigidBody* body)
{
	// Find the given body
	for (unsigned int i = 0; i < m_rigidBodies.size(); i++)
	{
		// Check for a match
		if (m_rigidBodies[i] == body)
		{
			// Swap this element and the end so as not to leave holes.
			m_rigidBodies[i] = m_rigidBodies[m_rigidBodies.size() - 1]; 
			// Delete the duplicated element.
			m_rigidBodies.pop_back();

			break; // Our job's done.
		}
	}
}

void World::addCollisionShape(RigidBody* body, CollisionShape* shape, const Transform& offset)
{
	m_collisionRegistry.add(body, shape, offset);
}

void World::removeCollisionShape(RigidBody* body, CollisionShape* shape)
{
	m_collisionRegistry.remove(body, shape);
}

void World::removeCollisionShape(RigidBody* body)
{
	m_collisionRegistry.remove(body);
}

void World::removeCollisionShape(CollisionShape* shape)
{
	m_collisionRegistry.remove(shape);
}


void World::addForceGenerator(RigidBody *body, ForceGenerator *forceGenerator)
{
	m_forceGenRegistry.add(body, forceGenerator);
}

void World::removeForceGenerator(RigidBody *body, ForceGenerator *forceGenerator)
{
	m_forceGenRegistry.remove(body, forceGenerator);
}

//--------------------------
//	PRIVATES			
//--------------------------

void World::integrateBodies(const Scalar& timeStep)
{
	// Integrate all the rigid bodies
	for (unsigned int i = 0; i < m_rigidBodies.size(); i++)
	{
		m_rigidBodies[i]->integrate(timeStep);
	}
}

} // namespace lt