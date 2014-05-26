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
			m_rigidBodies[i] = m_rigidBodies[m_rigidBodies.size()]; 
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

void World::constructOrthonormalBasis(const Vec3 &x, Vec3* y, Vec3* z)
{
	// Set the y axis to a vector not in the direction of x
	if(abs(x.x) > abs(x.y))
	{
		*y = lt::Vec3(0.f, 1.f, 0.f);
	}
	else
	{
		*y = lt::Vec3(1.f, 0.f, 0.f);
	}

	// Calculate Z from the vector product of x and y
	*z = x.cross(*y);

	// Check that y and x aren't parallel
	if(z->dot(*z) != (Scalar)0.0)
	{
		// Calculate y from the product of z and x
		*y = z->cross(x);

		// Normalize the output vectors
		y->normalize();
		z->normalize();
	}
}

const Mat3 World::constructOrthonormalBasis(const Vec3 &x)
{
	Vec3 y, z;

	constructOrthonormalBasis(x, &y, &z);

	return Mat3(x.x, y.x, z.x,
				x.y, y.y, z.y,
				x.z, y.z, z.z);
}

} // namespace lt