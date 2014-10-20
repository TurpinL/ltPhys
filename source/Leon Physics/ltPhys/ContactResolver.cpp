#include "ContactResolver.hpp"

#include <iostream>

namespace lt
{

void ContactResolver::resolveContacts(std::vector<ContactManifold> &contactManifolds)
{
	std::list<CollisionResponse> collisionResponseRegistry;

	// Constructs a list of impulses to apply to the rigid bodies
	for (unsigned int i = 0; i < contactManifolds.size(); i++)
	{
		resolveMotion(contactManifolds[i], collisionResponseRegistry);
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

	resolveAllInterpenetrations(contactManifolds);
}

void ContactResolver::resolveMotion(ContactManifold& manifold, std::list<CollisionResponse> &collisionResponseRegistry)
{
	// Calculate sum of the two inverse masses
	Scalar totalInvMass = manifold.getBody0().getInvMass() + manifold.getBody1().getInvMass();

	if(totalInvMass != 0)
	{
		// Apply the impulse.
		calcImpulse(manifold, collisionResponseRegistry);
	}
}

void ContactResolver::calcImpulse(ContactManifold& manifold, std::list<CollisionResponse> &collisionResponseRegistry)
{
	// Get the two bodies
	RigidBody& A = manifold.getBody0(); 
	RigidBody& B = manifold.getBody1(); 

	// Calculate restitution of collision;
	Scalar restitution = A.getRestitution() * B.getRestitution();

	// Collision responses
	CollisionResponse responseOfBodyA;
	CollisionResponse responseOfBodyB;

	responseOfBodyA.body = &A;
	responseOfBodyB.body = &B;


	int numContacts = manifold.getNumContacts();

	for(int i = 0 ; i < numContacts; i++)
	{
		ContactPoint &pt = manifold.getContactPoint(i);

		Vec3 contactPos = pt.position;
		Vec3 contactPosA = contactPos - A.getPosition();
		Vec3 contactPosB = contactPos - B.getPosition();

		Vec3 normal = pt.normal;

		// Maybe ptA/ptB should be used instead of contactPosA/contactPosB
		Vec3 kA = contactPosA.cross(normal); // Temp variable to store reused equation
		Vec3 kB = contactPosB.cross(normal); // Temp variable to store reused equation
		Vec3 uA = A.getInvInertiaTensorWorld() * kA; // Temp variable to store reused equation
		Vec3 uB = B.getInvInertiaTensorWorld() * kB; // Temp variable to store reused equation

		// Calculate the numerator and denominator for the impulse equation. 
		Scalar numer = -(1 + restitution) * (
				normal.dot(A.getVelocity() - B.getVelocity()) + // Linear closing velocity
				A.getAngularVelocity().dot(kA) - // Rotational closing velocity of body A.
				B.getAngularVelocity().dot(kB)   // Rotational closing velocity of body B.
			);
		
		Scalar denom = A.getInvMass() + B.getInvMass() + 
				kA.dot(uA) + kB.dot(uB);

		Scalar f = numer/denom;
		Vec3 impulse = normal * f / (Scalar)numContacts;

		responseOfBodyA.changeInVelocity += impulse * A.getInvMass();
		responseOfBodyB.changeInVelocity += -impulse * B.getInvMass();
	
		responseOfBodyA.changeInAngularVelocity += uA * f;
		responseOfBodyB.changeInAngularVelocity += -uB * f;
	
		
	}
	
	collisionResponseRegistry.push_back(responseOfBodyA);
	collisionResponseRegistry.push_back(responseOfBodyB);
}

void ContactResolver::resolveAllInterpenetrations(std::vector<ContactManifold> &contactManifolds)
{
	for (unsigned int i = 0; i < contactManifolds.size(); i++)
	{
		resolveInterpenetration(contactManifolds[i]);
	}
}

void ContactResolver::resolveInterpenetration(ContactManifold& manifold)
{
	// Get the two bodies
	// TODO: Figure out a proper way to do this without breaking constness
	RigidBody* bodies[2];
	bodies[0] = &manifold.getBody0(); 
	bodies[1] = &manifold.getBody1(); 

	int numContacts = manifold.getNumContacts();

	for(int j = 0 ; j < numContacts; j++)
	{
		Scalar totalInertia = 0;
		Scalar angularInertia[2];
		Scalar linearInertia[2];
		Scalar angularMove[2];
		Vec3 angleChange[2];
		Scalar linearMove[2];
		Vec3 positionChange[2];
		Vec3 relativeContactPosition[2];

		ContactPoint &pt = manifold.getContactPoint(j);
		Vec3 contactPos = pt.position;
		Vec3 normal = pt.normal;

		for(unsigned int i = 0; i < 2; i++) 
		{
			if(bodies[i])
			{
				Mat3 inverseInertiaTensor = bodies[i]->getInvInertiaTensorWorld();
			
				relativeContactPosition[i] = contactPos - bodies[i]->getPosition();

				Vec3 angularInertiaWorld = relativeContactPosition[i].cross(normal);
				angularInertiaWorld = inverseInertiaTensor * angularInertiaWorld;
				angularInertiaWorld = angularInertiaWorld.cross(relativeContactPosition[i]);
				angularInertia[i] = angularInertiaWorld.dot(normal);

				linearInertia[i] = bodies[i]->getInvMass();

				totalInertia += linearInertia[i] + angularInertia[i];
			}
		}

		if(totalInertia > 0) 
		{
			// Calculate linear and angular movement.
			Scalar inverseInertia = 1 / totalInertia;
			linearMove[0] = pt.penetration * linearInertia[0] * inverseInertia / numContacts;
			linearMove[1] = -pt.penetration * linearInertia[1] * inverseInertia / numContacts;
			angularMove[0] =  pt.penetration * angularInertia[0] * inverseInertia / numContacts;
			angularMove[1] = -pt.penetration * angularInertia[1] * inverseInertia / numContacts;

			for(unsigned int i = 0; i < 2; i++)
			{
				if(bodies[i])
				{
					// Apply linear movement
					positionChange[i] = normal * linearMove[i];
					bodies[i]->setPosition(positionChange[i] + bodies[i]->getPosition());

					if(angularMove[i] != 0)
					{
						// Calculate the torque required to move the relative contact position 1 unit.
						const Mat3 &inverseInertiaTensor = bodies[i]->getInvInertiaTensorWorld();
						Vec3 impulsiveTorque = relativeContactPosition[i].cross(normal);
						Vec3 impulsePerMove = inverseInertiaTensor * impulsiveTorque;

						Vec3 rotationPerMove = impulsePerMove * 1/angularInertia[i];
						angleChange[i] = Vec3(0, 0, 0);//rotationPerMove * angularMove[i];
				
						// Apply angular movement
						Quat newAngle = bodies[i]->getAngle();
						newAngle = Quat(Vec3(1.0f, 0.0f, 0.0f), angleChange[i].x) * newAngle;
						newAngle = Quat(Vec3(0.0f, 1.0f, 0.0f), angleChange[i].y) * newAngle;
						newAngle = Quat(Vec3(0.0f, 0.0f, 1.0f), angleChange[i].z) * newAngle;

						bodies[i]->setAngle(newAngle);
					}
				}
			}
		}
	}
}

} // namespace lt