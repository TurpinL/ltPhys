#include "ContactResolver.hpp"

namespace lt
{

void ContactResolver::resolveContacts(const lt::CollisionData &colData)
{
	std::list<CollisionResponse> collisionResponseRegistry;

	// Constructs a list of impulses to apply to the rigid bodiess
	for (unsigned int i = 0; i < colData.size - colData.contactsLeft; i++)
	{
		if(colData.contacts[i].body[0] != nullptr && colData.contacts[i].body[1] != nullptr)
		{
			resolveMotion(colData.contacts[i], collisionResponseRegistry);
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

	resolveAllInterpenetrations(colData);
}

void ContactResolver::resolveMotion(const Contact& contact, std::list<CollisionResponse> &collisionResponseRegistry)
{
	// Calculate sum of the two inverse masses
	Scalar totalInvMass = contact.body[0]->getInvMass() + contact.body[1]->getInvMass();

	if(totalInvMass != 0)
	{
		// Apply the impulse.
		calcImpulse(contact, collisionResponseRegistry);
	}
}

void ContactResolver::calcImpulse(const Contact& contact, std::list<CollisionResponse> &collisionResponseRegistry)
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

void ContactResolver::resolveAllInterpenetrations(const lt::CollisionData &colData)
{
	for (unsigned int i = 0; i < colData.size - colData.contactsLeft; i++)
	{
		if(colData.contacts[i].body[0] != nullptr && colData.contacts[i].body[1] != nullptr)
		{
			resolveInterpenetration(colData.contacts[i]);
		}
	}
}

void ContactResolver::resolveInterpenetration(const Contact& contact)
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

	
} // namespace lt