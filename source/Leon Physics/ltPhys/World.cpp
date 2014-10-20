#include "World.hpp"

namespace lt
{

//--------------------------
//	PUBLICS			
//--------------------------

World::World()
{}

void World::stepSimulation(const Scalar& timeStep)
{
	// Update forces
	m_forceGenRegistry.updateForces(timeStep);

	// Move bodies
	integrateBodies(timeStep);

	// Clear Contacts, generate new ones, then resolve them
	m_contactManifolds.clear();
	ContactGenerator::generateContacts(m_rigidBodies, m_contactManifolds);
	contactResolver.resolveContacts(m_contactManifolds);
}

void World::addRigidBody(RigidBody* body)
{
	// Add the body
	m_rigidBodies.push_back(body);
}

void World::removeRigidBody(RigidBody* body)
{
	// Find the given body
	for (unsigned int i = 0; i < m_rigidBodies.size(); i++)
	{
		// Check for a match
		if (m_rigidBodies[i] == body)
		{
			m_forceGenRegistry.remove(body);
			// Swap this element and the end so as not to leave holes.
			m_rigidBodies[i] = m_rigidBodies[m_rigidBodies.size() - 1]; 
			// Delete the duplicated element.
			m_rigidBodies.pop_back();

			break; // Our job's done.
		}
	}
}

void World::addForceGenerator(RigidBody *body, ForceGenerator *forceGenerator)
{
	m_forceGenRegistry.add(body, forceGenerator);
}

void World::removeForceGenerator(RigidBody *body, ForceGenerator *forceGenerator)
{
	m_forceGenRegistry.remove(body, forceGenerator);
}

const std::vector<RigidBody*>& World::getRigidBodyList()
{
	return m_rigidBodies;
}

const std::vector<ContactManifold>& World::getContactManifolds()
{
	return m_contactManifolds;
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