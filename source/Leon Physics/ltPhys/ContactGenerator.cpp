#include "ContactGenerator.hpp"

#include <cmath>
#include <iostream>

namespace lt
{

static inline Scalar transformToAxis(const ShapeBox &box, const Transform &boxTransform, const Vec3 &axis); 
static inline Scalar penetrationOnAxis(const ShapeBox &boxA, const Transform &boxATransform, const ShapeBox &boxB, const Transform &boxBTransform, const Vec3 &axis, const Vec3 &separation);
static inline bool tryAxis(const ShapeBox &boxA, const Transform &boxATransform, const ShapeBox &boxB, const Transform &boxBTransform, Vec3 axis, const Vec3 &separation, unsigned int index, Scalar &smallestPenetration, unsigned int &smallestCase);
static inline Vec3 contactPoint(const Vec3 &pOne, const Vec3 &dOne, Scalar sizeOne, const Vec3 &pTwo, const Vec3 &dTwo, Scalar sizeTwo, bool useOne);

void fillPointFaceBoxBox(const ShapeBox &boxA, const Transform &boxATransform, RigidBody *boxABody, const ShapeBox &boxB, const Transform &boxBTransform, RigidBody *boxBBody, const Vec3 &separation, CollisionData *collisionData, unsigned bestPen, Scalar penetration);

unsigned int ContactGenerator::sphere_sphere(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData)
{
	// Make sure we have contacts left and Check if they're spheres.
	if (collisionData->contactsLeft <= 0 ||
	   a.shape->getShapeType() != SHAPE_SPHERE || 
	   b.shape->getShapeType() != SHAPE_SPHERE ) 
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
//	   a.shape->getShapeType() != SHAPE_SPHERE || 
//	   b.shape->getShapeType() != SHAPE_BOX ) 
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
	   a.shape->getShapeType() != SHAPE_SPHERE || 
	   b.shape->getShapeType() != SHAPE_HALFSPACE ) 
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

unsigned int ContactGenerator::box_box(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData)
{
	// Make sure we have contacts left and Check if they're the right shapes.
	if (collisionData->contactsLeft <= 0 ||
	   a.shape->getShapeType() != SHAPE_BOX || 
	   b.shape->getShapeType() != SHAPE_BOX ) 
	{
		return 0;		
	}

	// then typecast their appropriate shapes
	const ShapeBox& boxA = *(ShapeBox*)a.shape;
	const ShapeBox& boxB = *(ShapeBox*)b.shape;

	const Vec3& boxAHalfExtents = boxA.getHalfExtents();
	const Vec3& boxBHalfExtents = boxB.getHalfExtents();

	Transform& boxATransform = Transform::Identity();
	Transform& boxBTransform = Transform::Identity();

	// Vector between box centres.
	Vec3 separation = Vec3();

	if(a.body != nullptr)
	{
		separation -= a.body->getPosition();
		boxATransform =  a.body->getTransform();
	} 

	if(b.body != nullptr)
	{
		separation += b.body->getPosition();
		boxBTransform =  b.body->getTransform();
	}

	Scalar smallestPen = SCALAR_MAX;
	unsigned int bestPen = 0xffffffff;

	// Check each axis, keeping track of the axis with the smallest penetration.
	// Stops when if finds an axis without penetration.
	if (!tryAxis(boxA, boxATransform, boxB, boxBTransform, boxATransform.getAxisVector(0), separation, 0, smallestPen, bestPen) ||
        !tryAxis(boxA, boxATransform, boxB, boxBTransform, boxATransform.getAxisVector(1), separation, 1, smallestPen, bestPen) ||
        !tryAxis(boxA, boxATransform, boxB, boxBTransform, boxATransform.getAxisVector(2), separation, 2, smallestPen, bestPen) ||

        !tryAxis(boxA, boxATransform, boxB, boxBTransform, boxBTransform.getAxisVector(0), separation, 3, smallestPen, bestPen) ||
        !tryAxis(boxA, boxATransform, boxB, boxBTransform, boxBTransform.getAxisVector(1), separation, 4, smallestPen, bestPen) ||
        !tryAxis(boxA, boxATransform, boxB, boxBTransform, boxBTransform.getAxisVector(2), separation, 5, smallestPen, bestPen) )
	{
		return 0;
	}

	unsigned bestSingleAxis = bestPen;

	if (!tryAxis(boxA, boxATransform, boxB, boxBTransform, boxATransform.getAxisVector(0).cross(boxBTransform.getAxisVector(0)), separation,  6, smallestPen, bestPen) ||
        !tryAxis(boxA, boxATransform, boxB, boxBTransform, boxATransform.getAxisVector(0).cross(boxBTransform.getAxisVector(1)), separation,  7, smallestPen, bestPen) ||
        !tryAxis(boxA, boxATransform, boxB, boxBTransform, boxATransform.getAxisVector(0).cross(boxBTransform.getAxisVector(2)), separation,  8, smallestPen, bestPen) ||

        !tryAxis(boxA, boxATransform, boxB, boxBTransform, boxATransform.getAxisVector(1).cross(boxBTransform.getAxisVector(0)), separation,  9, smallestPen, bestPen) ||
        !tryAxis(boxA, boxATransform, boxB, boxBTransform, boxATransform.getAxisVector(1).cross(boxBTransform.getAxisVector(1)), separation, 10, smallestPen, bestPen) ||
        !tryAxis(boxA, boxATransform, boxB, boxBTransform, boxATransform.getAxisVector(1).cross(boxBTransform.getAxisVector(2)), separation, 11, smallestPen, bestPen) ||

        !tryAxis(boxA, boxATransform, boxB, boxBTransform, boxATransform.getAxisVector(2).cross(boxBTransform.getAxisVector(0)), separation, 12, smallestPen, bestPen) ||
        !tryAxis(boxA, boxATransform, boxB, boxBTransform, boxATransform.getAxisVector(2).cross(boxBTransform.getAxisVector(1)), separation, 13, smallestPen, bestPen) ||
        !tryAxis(boxA, boxATransform, boxB, boxBTransform, boxATransform.getAxisVector(2).cross(boxBTransform.getAxisVector(2)), separation, 14, smallestPen, bestPen) )
	{
		return 0;
	}

	// We've found a collision, and we know which of the axes gave the smallest penetration.
	if (bestPen < 3)
	{
		// Vertex of boxB in face of boxA
		// TODO: Do stuff
		//std::cout << bestPen << " Vertex of boxB in face of boxA\n";
		fillPointFaceBoxBox(boxA, boxATransform, a.body, boxB, boxBTransform, b.body, separation, collisionData, bestPen, smallestPen);
		return 1;
	}
	else if (bestPen < 6)
	{
		// Vertex of boxA in face of boxB
		// TODO: Do stuff
		//std::cout << bestPen << " Vertex of boxA in face of boxB\n";
		fillPointFaceBoxBox(boxB, boxBTransform, b.body, boxA, boxATransform, a.body, -separation, collisionData, bestPen-3, smallestPen);
		return 1;
	}
	else
	{
		// Edge Edge contact.
		//std::cout << "Colliding >= 6";
		//std::cout << bestPen << " Edge-Edge\n";

		// Find which axis.
		bestPen -= 6;
		unsigned int axisIndexA = bestPen / 3;
		unsigned int axisIndexB = bestPen % 3;
		Vec3 axisA = boxATransform.getAxisVector(axisIndexA);
		Vec3 axisB = boxBTransform.getAxisVector(axisIndexB);
		Vec3 axis = axisA.cross(axisB);
		axis.normalize();

		// Axis should point from box one to box two.
		if ( axis.dot(separation) > 0 )
		{
			axis = -axis;
		}

		Vec3 ptOnEdgeA = boxAHalfExtents;
		Vec3 ptOnEdgeB = -boxBHalfExtents;

		for (unsigned int i = 0; i < 3; i++)
		{
			if (i == axisIndexA) 
			{ 
				ptOnEdgeA[i] = 0; 
			}
			else if (boxATransform.getAxisVector(i).dot(axis) > 0) 
			{ 
				ptOnEdgeA[i] = -ptOnEdgeA[i]; 
			}

			if (i == axisIndexB) 
			{ 
				ptOnEdgeB[i] = 0;
			}
			else if (boxBTransform.getAxisVector(i).dot(axis) > 0) 
			{ 
				ptOnEdgeB[i] = -ptOnEdgeB[i]; 
			}
		}

		// Transform the points into world coordinates.
		ptOnEdgeA = boxATransform * ptOnEdgeA;
		ptOnEdgeB = boxBTransform * ptOnEdgeB;

		// Find the point of closest approach of the two
		// line-segments.
		Vec3 vertex = contactPoint(ptOnEdgeA, axisA, boxAHalfExtents.get(axisIndexA), ptOnEdgeB, axisB, boxBHalfExtents.get(axisIndexB), bestSingleAxis > 2);

		// Create contact data
		Contact& contact = collisionData->contacts[collisionData->size - collisionData->contactsLeft];
		collisionData->contactsLeft--;

		contact.body[0] = a.body;
		contact.body[1] = b.body;
		contact.normal = axis;
		contact.penetration = smallestPen;
		contact.position = vertex;
		contact.impulseModifier = 1;

		return 1;
	}

	return 0;
}

unsigned int ContactGenerator::box_halfspace(const CollisionRegistration& a, const CollisionRegistration& b, CollisionData *collisionData)
{
	// Make sure we have contacts left and Check if they're the right shapes.
	if (collisionData->contactsLeft <= 0 ||
	   a.shape->getShapeType() != SHAPE_BOX || 
	   b.shape->getShapeType() != SHAPE_HALFSPACE ) 
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

static inline Vec3 getShapeAxis(const ShapeBox &box, const Transform &boxTransform, unsigned int index)
{
	return Vec3(boxTransform.get(index), boxTransform.get(index+4), boxTransform.get(index+8));
}

static inline Scalar transformToAxis(const ShapeBox &box, const Transform &boxTransform, const Vec3 &axis)
{
	return 
		box.getHalfExtents().x * abs(axis.dot( getShapeAxis(box, boxTransform, 0) )) + 
		box.getHalfExtents().y * abs(axis.dot( getShapeAxis(box, boxTransform, 1) )) + 
		box.getHalfExtents().z * abs(axis.dot( getShapeAxis(box, boxTransform, 2) )); 
}

static inline Scalar penetrationOnAxis(const ShapeBox &boxA, const Transform &boxATransform, 
	const ShapeBox &boxB, const Transform &boxBTransform, const Vec3 &axis, const Vec3 &separation)
{
	Scalar projectionA = transformToAxis(boxA, boxATransform, axis); 
	Scalar projectionB = transformToAxis(boxB, boxBTransform, axis);

	Scalar distance = abs( separation.dot(axis) );

	return projectionA + projectionB - distance;
}

static inline bool tryAxis(const ShapeBox &boxA, const Transform &boxATransform, 
	const ShapeBox &boxB, const Transform &boxBTransform, Vec3 axis, 
	const Vec3 &separation, unsigned int index, Scalar &smallestPenetration, unsigned int &smallestCase)
{
	// Omit almost parallel axes and normalize
	if (axis.dot(axis) < 0.0001) return true;
	axis.normalize();

	Scalar penetration = penetrationOnAxis(boxA, boxATransform, boxB, boxBTransform, axis, separation);

	if (penetration < 0) return false;

	if(penetration < smallestPenetration)
	{
		smallestPenetration = penetration;
		smallestCase = index;
	}

	return true;
}

// Taken from Ian Millington's book "Game Physics Engine Development"
static inline Vec3 contactPoint(const Vec3 &pOne, const Vec3 &dOne, Scalar sizeOne, const Vec3 &pTwo, const Vec3 &dTwo, Scalar sizeTwo, bool useOne)
{
	Vec3 toSt, cOne, cTwo;
	Scalar dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
	Scalar denom, mua, mub;

	smOne = dOne.dot(dOne);
	smTwo = dTwo.dot(dTwo);
	dpOneTwo = dTwo.dot(dOne);

	toSt = pOne - pTwo;
	dpStaOne = dOne.dot(toSt);
	dpStaTwo = dTwo.dot(toSt);

	denom = smOne * smTwo - dpOneTwo * dpOneTwo;

	// Zero denominator indicates parallel lines
	if (abs(denom) < 0.0001f)
	{
		return useOne ? pOne : pTwo;
	}

	mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
	mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

	// If either of the edges has the nearest point out
    // of bounds, then the edges aren't crossed, we have
    // an edge-face contact. Our point is on the edge, which
    // we know from the useOne parameter.
	if (mua > sizeOne ||
		mua < -sizeOne || 
		mub > sizeTwo ||
		mub < -sizeTwo)
	{
		return useOne ? pOne : pTwo;
	}
	else
	{
		cOne = pOne + dOne * mua;
		cTwo = pTwo + dTwo * mub;

		return cOne * 0.5f + cTwo * 0.5f; 
	}
}


void fillPointFaceBoxBox(const ShapeBox &boxA, const Transform &boxATransform, RigidBody *boxABody,
	const ShapeBox &boxB, const Transform &boxBTransform, RigidBody *boxBBody,
	const Vec3 &separation, CollisionData *collisionData, unsigned bestPen, Scalar penetration)
{
	Vec3 normal = getShapeAxis(boxA, boxATransform, bestPen);
	if (getShapeAxis(boxA, boxATransform, bestPen).dot(separation) > 0)
	{
		normal = -normal;
	}

	Vec3 vertex = boxB.getHalfExtents();
	if (getShapeAxis(boxB, boxBTransform, 0).dot(normal) < 0) { vertex.x = -vertex.x; }
	if (getShapeAxis(boxB, boxBTransform, 1).dot(normal) < 0) { vertex.y = -vertex.y; }
	if (getShapeAxis(boxB, boxBTransform, 2).dot(normal) < 0) { vertex.z = -vertex.z; }

	// Create contact data
	Contact& contact = collisionData->contacts[collisionData->size - collisionData->contactsLeft];
	collisionData->contactsLeft--;

	contact.body[0] = boxABody;
	contact.body[1] = boxBBody;
	contact.normal = normal;
	contact.penetration = penetration;
	contact.position = boxBTransform * vertex - (normal * penetration * 0.5);
}

} // namespace lt