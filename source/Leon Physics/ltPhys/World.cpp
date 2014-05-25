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
	_integrateBodies(timeStep);

	// Detect Contacts
	m_collisionRegistry.findContacts();
	// Then resolve them
	_resolveCollisions(timeStep);
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

void World::_integrateBodies(const Scalar& timeStep)
{
	// Integrate all the rigid bodies
	for (unsigned int i = 0; i < m_rigidBodies.size(); i++)
	{
		m_rigidBodies[i]->integrate(timeStep);
	}
}

void World::_resolveCollisions(const Scalar& timeStep)
{
	const lt::CollisionData &colData = m_collisionRegistry.getCollisionData();
	std::list<CollisionResponse> collisionResponseRegistry;

	// Constructs a list of impulses to apply to the rigid bodiess
	for (unsigned int i = 0; i < colData.size - colData.contactsLeft; i++)
	{
		if(colData.contacts[i].body[0] != nullptr && colData.contacts[i].body[1] != nullptr)
		{
			_resolveCollisionMotion(colData.contacts[i], collisionResponseRegistry);
		}
	}

	// Store the collision responses. Velocities aren't modified immediately
	// because a change in velocity may affect the closing velocity of other collisions.
	while (!collisionResponseRegistry.empty())
	{
		CollisionResponse &curResponse = collisionResponseRegistry.front();
		
		curResponse.body->setVelocity(curResponse.body->getVelocity() + curResponse.changeInVelocity);
		curResponse.body->setAngularVelocity(curResponse.body->getAngularVelocity() + curResponse.changeInAngularVelocity);

		collisionResponseRegistry.pop_front();
	}

	for (unsigned int i = 0; i < colData.size - colData.contactsLeft; i++)
	{
		if(colData.contacts[i].body[0] != nullptr && colData.contacts[i].body[1] != nullptr)
		{
			_resolveCollisionInterpenetration(colData.contacts[i]);
		}
	}
}

// Assumes both bodies are not nullptr
void World::_resolveCollisionMotion(const Contact& contact, std::list<CollisionResponse> &collisionResponseRegistry)
{
	// Calculate sum of the two inverse masses
	Scalar totalInvMass = contact.body[0]->getInvMass() + contact.body[1]->getInvMass();

	if(totalInvMass != 0)
	{
		// Apply the impulse.
		_calcCollisionImpulse(contact, collisionResponseRegistry);
	}
}

// Assumes both bodies are not nullptr
// I don't understand all of this, the code was written from psuedo code in "Game Physics, Second Edition" by David H Eberly, page 481.
void World::_calcCollisionImpulse(const Contact& contact, std::list<CollisionResponse> &collisionResponseRegistry)
{
	RigidBody &A = *contact.body[0];
	RigidBody &B = *contact.body[1];
	
	// Calculate restitution of collision
	Scalar restitution = A.getRestitution() * B.getRestitution();

	// Find the offset of the collision from the centre of mass of each body.
	Vec3 contactPosA = contact.position - A.getPosition();
	Vec3 contactPosB = contact.position - B.getPosition();

	Vec3 kA = contactPosA.cross(contact.normal);
	Vec3 kB = contactPosB.cross(contact.normal);

	Vec3 uA = A.getInvInertiaTensorWorld() * kA;
	Vec3 uB = B.getInvInertiaTensorWorld() * kB;

	Scalar numer = -(1 + restitution)
			* ( (contact.normal.dot(A.getVelocity() - B.getVelocity())) // Closing velocity due to linear velocity of centre of Mass
			+ A.getAngularVelocity().dot(kA) // Closing velocity of point due to rotation of body A?
			- B.getAngularVelocity().dot(kB) ); // Closing velocity of point due to rotation of body B?
	
	Scalar denom = A.getInvMass() + B.getInvMass() + kA.dot(uA) + kB.dot(uB);
	Scalar f = numer / denom;
	Vec3 impulse = contact.normal * f;

	// Create collision responses
	CollisionResponse responseOfBodyA;
	CollisionResponse responseOfBodyB;

	responseOfBodyA.body = &A;
	responseOfBodyB.body = &B;

	responseOfBodyA.changeInVelocity = (impulse * A.getInvMass());
	responseOfBodyB.changeInVelocity = (-impulse * B.getInvMass());

	responseOfBodyA.changeInAngularVelocity = uA * f;
	responseOfBodyB.changeInAngularVelocity = -uB * f;

	collisionResponseRegistry.push_back(responseOfBodyA);
	collisionResponseRegistry.push_back(responseOfBodyB);
}

void World::_resolveCollisionInterpenetration(const Contact& contact)
{
	if(!isOldInterpenetration)
	{
		Scalar totalInertia = 0;
		Scalar angularInertia[2];
		Scalar linearInertia[2];
		Scalar angularMove[2];
		Scalar linearMove[2];
		Vec3 relativeContactPosition[2];

		for(unsigned int i = 0; i < 2; i++) 
		{
			if(contact.body[i])
			{
				Mat3 inverseInertiaTensor = contact.body[i]->getInvInertiaTensorWorld();
			
				relativeContactPosition[i] = contact.position - contact.body[i]->getPosition();

				Vec3 angularInertiaWorld = relativeContactPosition[i].cross(contact.normal);
				angularInertiaWorld = inverseInertiaTensor * angularInertiaWorld;
				angularInertiaWorld = angularInertiaWorld.cross(relativeContactPosition[i]);
				angularInertia[i] = angularInertiaWorld.dot(contact.normal);

				linearInertia[i] = contact.body[i]->getInvMass();

				totalInertia += linearInertia[i] + angularInertia[i];
			}
		}

		if(totalInertia > 0) 
		{
			// Calculate linear and angular movement
			Scalar inverseInertia = 1 / totalInertia;
			linearMove[0] =  contact.penetration * linearInertia[0] * inverseInertia;
			linearMove[1] = -contact.penetration * linearInertia[1] * inverseInertia;
			angularMove[0] =  contact.penetration * angularInertia[0] * inverseInertia;
			angularMove[1] = -contact.penetration * angularInertia[1] * inverseInertia;

			for(unsigned int i = 0; i < 2; i++)
			{
				if(contact.body[i])
				{
					// Apply linear movement
					contact.body[i]->setPosition(contact.normal * linearMove[i] + contact.body[i]->getPosition());

					if(angularMove[i] != 0)
					{
						// Calculate the torque required to move the relative contact position 1 unit.
						const Mat3 &inverseInertiaTensor = contact.body[i]->getInvInertiaTensorWorld();
						Vec3 impulsiveTorque = relativeContactPosition[i].cross(contact.normal);
						Vec3 impulsePerMove = inverseInertiaTensor * impulsiveTorque;

						Vec3 rotationPerMove = impulsePerMove * 1/angularInertia[i];
						Vec3 rotation = rotationPerMove * angularMove[i];
				
						// Apply angular movement
						Quat newAngle = contact.body[i]->getAngle();
						newAngle = Quat(Vec3(1.0f, 0.0f, 0.0f), rotation.x) * newAngle;
						newAngle = Quat(Vec3(0.0f, 1.0f, 0.0f), rotation.y) * newAngle;
						newAngle = Quat(Vec3(0.0f, 0.0f, 1.0f), rotation.z) * newAngle;

						contact.body[i]->setAngle(newAngle);
					}
				}
			}
		}
	}
	else
	{
		Scalar totalInvMass = 0;

		if (contact.body[0] != nullptr)
		{
			totalInvMass += contact.body[0]->getInvMass();
		}

		if (contact.body[1] != nullptr)
		{
			totalInvMass += contact.body[1]->getInvMass();
		}

		if(totalInvMass != 0)
		{
			Vec3 movement = contact.normal * contact.penetration * contact.impulseModifier;

			if (contact.body[0] != nullptr)
			{
				contact.body[0]->setPosition( contact.body[0]->getPosition() + (movement * (contact.body[0]->getInvMass() / totalInvMass)) );
			}

			if (contact.body[1] != nullptr)
			{
				contact.body[1]->setPosition( contact.body[1]->getPosition() - (movement * (contact.body[1]->getInvMass() / totalInvMass)) );
			}
		}
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