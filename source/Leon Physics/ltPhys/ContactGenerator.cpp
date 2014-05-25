#include "ContactGenerator.hpp"

namespace lt
{

unsigned int ContactGenerator::sphere_sphere(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData)
{
	// Make sure we have contacts left and Check if they're spheres.
	if (collisionData->contactsLeft <= 0 ||
	   a.shape->getShapeType() != LT_SHAPE_SPHERE || 
	   b.shape->getShapeType() != LT_SHAPE_SPHERE ) 
	{
		return 0;		
	}

	//  then typecast their shape to a sphere
	const ShapeSphere& shapeA = *(ShapeSphere*)a.shape;
	const ShapeSphere& shapeB = *(ShapeSphere*)b.shape;

	// Get the sphere positions
	Vec3 posA = a.offset.getPosition();
	Vec3 posB = b.offset.getPosition();

	if (a.body != nullptr)
	{
		posA += a.body->getTransform().getPosition();
	}

	if (b.body != nullptr)
	{
		posB += b.body->getTransform().getPosition();
	}
	
	// Find the vector between the two object
	Vec3 midLine = posA - posB;
	Scalar distance = midLine.length();

	// Check for collision
	if (distance <= 0.0f || distance >= shapeA.getRadius() + shapeB.getRadius())
	{
		return 0;
	}

	Contact& contact = collisionData->contacts[collisionData->size - collisionData->contactsLeft];
	collisionData->contactsLeft--;

	// Manually normalize the midLine, so the Vec3 class doesn't have to recalculate the length.
	contact.normal = midLine * ( ((Scalar)1.0) / distance );
	contact.position = posA + (midLine * (Scalar)-0.5);
	contact.penetration = (shapeA.getRadius() + shapeA.getRadius() - distance);
	contact.impulseModifier = 1;
	contact.body[0] = a.body;
	contact.body[1] = b.body;
	
	return 1;
}

//unsigned int ContactGenerator::sphere_box(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData)
//{
//	// Make sure we have contacts left and Check if they're the right shapes.
//	if (collisionData->contactsLeft <= 0 ||
//	   a.shape->getShapeType() != LT_SHAPE_SPHERE || 
//	   b.shape->getShapeType() != LT_SHAPE_BOX ) 
//	{
//		return 0;		
//	}
//
//}
//
unsigned int ContactGenerator::sphere_halfspace(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData)
{
	// Make sure we have contacts left and Check if they're the right shapes.
	if (collisionData->contactsLeft <= 0 ||
	   a.shape->getShapeType() != LT_SHAPE_SPHERE || 
	   b.shape->getShapeType() != LT_SHAPE_HALFSPACE ) 
	{
		return 0;		
	}

	// then typecast their appropriate shapes
	const ShapeSphere& sphere = *(ShapeSphere*)a.shape;
	const ShapeHalfspace& halfspace = *(ShapeHalfspace*)b.shape;

	// Get the positions and halfspace normal
	Vec3 posSphere = a.offset.getPosition();
	Vec3 posHalfspace = b.offset.getPosition();
	Vec3 normHalfspace = b.offset * Vec3(0.f, 1.f, 0.f, 0.f);

	if (a.body != nullptr)
	{
		posSphere += a.body->getTransform().getPosition();
	}

	if (b.body != nullptr)
	{
		posHalfspace += b.body->getTransform().getPosition();
		normHalfspace = b.body->getTransform() * normHalfspace;
	}

	// Find the distance from the plane to the sphere
	Scalar distance = normHalfspace.dot(posSphere) - sphere.getRadius() - (posHalfspace).y;

	// Check collision
	if(distance < 0)
	{
		// Create the contact
		Contact& contact = collisionData->contacts[collisionData->size - collisionData->contactsLeft];
		collisionData->contactsLeft--;

		contact.body[0] = a.body;
		contact.body[1] = b.body;
		contact.normal = normHalfspace;
		contact.penetration = -distance;
		contact.impulseModifier = 1;
		contact.position = posSphere + -contact.normal*(sphere.getRadius() - contact.penetration/2);

		return 1;
	}

	return 0;
}

unsigned int ContactGenerator::sphere_terrain(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData)
{
	// Make sure we have contacts left and Check if they're the right shapes.
	if (collisionData->contactsLeft <= 0 ||
	   a.shape->getShapeType() != LT_SHAPE_SPHERE || 
	   b.shape->getShapeType() != LT_SHAPE_TERRAIN ) 
	{
		return 0;		
	}

	// then typecast their appropriate shapes
	const ShapeSphere& sphere = *(ShapeSphere*)a.shape;
	const ShapeTerrain& terrain = *(ShapeTerrain*)b.shape;

	// Check that the terrain has a heightmap
	if (terrain.getTerrainData() == nullptr) 
	{
		return 0;		
	}

	// Get the positions
	Vec3 posSphere = a.offset.getPosition();
	Vec3 posTerrain = b.offset.getPosition();

	if (a.body != nullptr)
	{
		posSphere += a.body->getTransform().getPosition();
	}

	if (b.body != nullptr)
	{
		posTerrain += b.body->getTransform().getPosition();
	}

	// Find the height of the terrain at the x/z of the sphere.
	const TerrainData *terrainData = terrain.getTerrainData();

	Vec3 contactPosition = posSphere;

	// Calculate the size of each cell in the heightmap
	Vec3 sphereHeightMapPos(posSphere - posTerrain); // Position of the sphere relative to the heightmap.

	lt::Vec3 contactNormal;
	contactPosition.y = terrainData->getHeight(contactNormal, sphereHeightMapPos.x, sphereHeightMapPos.z);
	contactPosition.y += posTerrain.y;
	contactPosition.y += ((posSphere.y - sphere.getRadius()) - contactPosition.y) * 0.5f;

	if(posSphere.y - sphere.getRadius() < contactPosition.y)
	{
		// Create the contact
		Contact& contact = collisionData->contacts[collisionData->size - collisionData->contactsLeft];
		collisionData->contactsLeft--;

		contact.body[0] = a.body;
		contact.body[1] = b.body;
		contact.normal = Vec3(0, -1, 0);
		contact.penetration = 2 * (posSphere.y - sphere.getRadius() -  contactPosition.y);
		contact.impulseModifier = 1;
		contact.position = contactPosition;

		return 1;
	}

	return 0;
}

//
//unsigned int ContactGenerator::box_box(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData)
//{
//	// Make sure we have contacts left and Check if they're the right shapes.
//	if (collisionData->contactsLeft <= 0 ||
//	   a.shape->getShapeType() != LT_SHAPE_BOX || 
//	   b.shape->getShapeType() != LT_SHAPE_BOX ) 
//	{
//		return 0;		
//	}
//
//}

unsigned int ContactGenerator::box_halfspace(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData)
{
	// Make sure we have contacts left and Check if they're the right shapes.
	if (collisionData->contactsLeft <= 0 ||
	   a.shape->getShapeType() != LT_SHAPE_BOX || 
	   b.shape->getShapeType() != LT_SHAPE_HALFSPACE ) 
	{
		return 0;		
	}

	// then typecast their appropriate shapes
	const ShapeBox& box = *(ShapeBox*)a.shape;
	const ShapeHalfspace& halfspace = *(ShapeHalfspace*)b.shape;

	const Vec3& boxHalfExtents = box.getHalfExtents();

	// generate a array of all of the box's vertices
	Vec3 boxVertex[8] = 
	{
		Vec3(-boxHalfExtents.x, -boxHalfExtents.y, -boxHalfExtents.z),
		Vec3(-boxHalfExtents.x, -boxHalfExtents.y, +boxHalfExtents.z),
		Vec3(-boxHalfExtents.x, +boxHalfExtents.y, -boxHalfExtents.z),
		Vec3(-boxHalfExtents.x, +boxHalfExtents.y, +boxHalfExtents.z),
		Vec3(+boxHalfExtents.x, -boxHalfExtents.y, -boxHalfExtents.z),
		Vec3(+boxHalfExtents.x, -boxHalfExtents.y, +boxHalfExtents.z),
		Vec3(+boxHalfExtents.x, +boxHalfExtents.y, -boxHalfExtents.z),
		Vec3(+boxHalfExtents.x, +boxHalfExtents.y, +boxHalfExtents.z),
	};

	// Apply collision object offset
	for (int i = 0; i < 8; i++)
	{
		boxVertex[i] = a.offset * boxVertex[i];
	}

	// Apply attatched rigid body's offset.
	if (a.body != nullptr)
	{
		const Transform& boxTransform = a.body->getTransform();

		for (int i = 0; i < 8; i++)
		{
			boxVertex[i] = boxTransform * boxVertex[i];
		}
	}

	// Calculate halfspace's position and normal
	Vec3 posHalfspace = b.offset.getPosition();
	Vec3 normHalfspace = b.offset * Vec3(0.f, 1.f, 0.f, 0.f);

	if (b.body != nullptr)
	{
		posHalfspace += b.body->getTransform().getPosition();
		normHalfspace = b.body->getTransform() * normHalfspace;
	}

	// Check each vertice for intersection with the halfspace
	Scalar vertexDistance;

	unsigned int numContacts = 0;

	for (int i = 0; i < 8; i++)
	{
		vertexDistance = boxVertex[i].dot(normHalfspace) - posHalfspace.y;

		if(vertexDistance <= 0)
		{
			// Create contact data
			Contact& contact = collisionData->contacts[collisionData->size - collisionData->contactsLeft];
			collisionData->contactsLeft--;
			numContacts++;

			contact.body[0] = a.body;
			contact.body[1] = b.body;
			contact.normal = normHalfspace;
			contact.penetration = -vertexDistance;
			contact.position = boxVertex[i] + contact.normal*(contact.penetration*0.5f);

			// As we're potentially making multiple contacts, check if we have contacts left.
			if(collisionData->contactsLeft <= 0)
			{
				break;
			}
		}
	}

	// Set the contact's impulseModifier now that we know how many contacts exist
	Scalar impulseModifier = (Scalar)1 / numContacts;

	for(unsigned int i = 0; i < numContacts; i++)
	{
		collisionData->contacts[collisionData->size - collisionData->contactsLeft - i - 1].impulseModifier = impulseModifier;
	}

	return numContacts;
}

unsigned int ContactGenerator::box_terrain(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData)
{
	// Make sure we have contacts left and Check if they're the right shapes.
	if (collisionData->contactsLeft <= 0 ||
	   a.shape->getShapeType() != LT_SHAPE_BOX || 
	   b.shape->getShapeType() != LT_SHAPE_TERRAIN ) 
	{
		return 0;		
	}

	// then typecast their appropriate shapes
	const ShapeBox& box = *(ShapeBox*)a.shape;
	const ShapeTerrain& terrain = *(ShapeTerrain*)b.shape;
	
	const Vec3& boxHalfExtents = box.getHalfExtents();

	// generate a array of all of the box's vertices
	Vec3 boxVertex[8] = 
	{
		Vec3(-boxHalfExtents.x, -boxHalfExtents.y, -boxHalfExtents.z),
		Vec3(-boxHalfExtents.x, -boxHalfExtents.y, +boxHalfExtents.z),
		Vec3(-boxHalfExtents.x, +boxHalfExtents.y, -boxHalfExtents.z),
		Vec3(-boxHalfExtents.x, +boxHalfExtents.y, +boxHalfExtents.z),
		Vec3(+boxHalfExtents.x, -boxHalfExtents.y, -boxHalfExtents.z),
		Vec3(+boxHalfExtents.x, -boxHalfExtents.y, +boxHalfExtents.z),
		Vec3(+boxHalfExtents.x, +boxHalfExtents.y, -boxHalfExtents.z),
		Vec3(+boxHalfExtents.x, +boxHalfExtents.y, +boxHalfExtents.z),
	};

	// Apply collision object offset
	for (int i = 0; i < 8; i++)
	{
		boxVertex[i] = a.offset * boxVertex[i];
	}

	// Apply attatched rigid body's offset.
	if (a.body != nullptr)
	{
		const Transform& boxTransform = a.body->getTransform();

		for (int i = 0; i < 8; i++)
		{
			boxVertex[i] = boxTransform * boxVertex[i];
		}
	}

	Vec3 posTerrain = b.offset.getPosition();

	if (b.body != nullptr)
	{
		posTerrain += b.body->getTransform().getPosition();
	}

	// Find the height of the terrain at the x/z of the sphere.
	const TerrainData *terrainData = terrain.getTerrainData();

	// Check each vertice for intersection with the halfspace
	Scalar vertexDistance;
	Vec3 vertRelPosition;

	unsigned int numContacts = 0;
	Vec3 contactNormal;

	for (int i = 0; i < 8; i++)
	{
		vertRelPosition = boxVertex[i] - posTerrain;

		vertexDistance = boxVertex[i].y - (posTerrain.y + terrainData->getHeight(contactNormal, vertRelPosition.x, vertRelPosition.z));

		if(vertexDistance <= 0)
		{
			// Create contact data
			Contact& contact = collisionData->contacts[collisionData->size - collisionData->contactsLeft];
			collisionData->contactsLeft--;
			numContacts++;

			contact.body[0] = a.body;
			contact.body[1] = b.body;
			contact.normal = contactNormal.normalized();
			contact.penetration = -vertexDistance;
			contact.position = boxVertex[i] + contact.normal*(contact.penetration*0.5f);

			// As we're potentially making multiple contacts, check if we have contacts left.
			if(collisionData->contactsLeft <= 0)
			{
				break;
			}
		}
	}

	// Set the contact's impulseModifier now that we know how many contacts exist
	Scalar impulseModifier = (Scalar)1 / numContacts;

	for(unsigned int i = 0; i < numContacts; i++)
	{
		collisionData->contacts[collisionData->size - collisionData->contactsLeft - i - 1].impulseModifier = impulseModifier;
	}

	return numContacts;
}

} // namespace lt