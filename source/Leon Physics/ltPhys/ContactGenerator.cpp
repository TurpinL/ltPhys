#include "ContactGenerator.hpp"

#include <cmath>
#include <iostream>

namespace lt
{

static inline Scalar transformToAxis(const ShapeBox &box, const Transform &boxTransform, const Vec3 &axis); 
static inline Scalar penetrationOnAxis(const ShapeBox &boxA, const Transform &boxATransform, const ShapeBox &boxB, const Transform &boxBTransform, const Vec3 &axis, const Vec3 &separation);
static inline bool tryAxis(const ShapeBox &boxA, const Transform &boxATransform, const ShapeBox &boxB, const Transform &boxBTransform, Vec3 axis, const Vec3 &separation, unsigned int index, Scalar &smallestPenetration, unsigned int &smallestCase);
static inline Vec3 contactPoint(const Vec3 &pOne, const Vec3 &dOne, Scalar sizeOne, const Vec3 &pTwo, const Vec3 &dTwo, Scalar sizeTwo, bool useOne);

void fillPointFaceBoxBox(const ShapeBox &boxA, const Transform &boxATransform, const ShapeBox &boxB, const Transform &boxBTransform, const Vec3 &separation, ContactManifold &contactManifold, unsigned bestPen, Scalar penetration, bool doSwapBodies);
static inline Vec3 getShapeAxis(const ShapeBox &box, const Transform &boxTransform, unsigned int index);
static inline Scalar transformToAxis(const ShapeBox &box, const Transform &boxTransform, const Vec3 &axis);

void ContactGenerator::generateContacts(std::vector<RigidBody*>& rigidBodies, std::vector<ContactManifold>& contactManifolds)
{
	// For each rigid body with each other rigid body
	for(unsigned int i = 0; i < rigidBodies.size(); i++)
	{
		if(rigidBodies[i]->numCollisionShapes() != 0)
		{
			for (unsigned int j = i+1; j < rigidBodies.size(); j++)
			{
				if(rigidBodies[j]->numCollisionShapes() != 0)
				{
					checkCollision(*rigidBodies[i], *rigidBodies[j], contactManifolds);
				}
			}
		}
	}
}

void ContactGenerator::checkCollision(RigidBody &rbA, RigidBody &rbB, std::vector<ContactManifold>& contactManifolds)
{
	const std::set<const CollisionShape*>& colShapesA = rbA.getCollisionShapes();
	const std::set<const CollisionShape*>& colShapesB = rbB.getCollisionShapes();

	ContactManifold normManifold(rbA, rbB);
	ContactManifold swappedManifold(rbB, rbA);

	std::set<const CollisionShape*>::iterator i;
	std::set<const CollisionShape*>::iterator j;
	for (i = colShapesA.begin(); i != colShapesA.end(); ++i)
	{
		for (j = colShapesB.begin(); j != colShapesB.end(); ++j)
		{
			// Get shape types
			ShapeType shapeAType = (*i)->getShapeType();
			ShapeType shapeBType = (*j)->getShapeType();
			const CollisionShape *shape1 = *i;
			const CollisionShape *shape2 = *j;
			RigidBody *body1 = &rbA;
			RigidBody *body2 = &rbB;
			ContactManifold *curManifold = &normManifold;

			if (shapeAType > shapeBType)
			{
				const CollisionShape *tempReg = shape1;
				shape1 = shape2;
				shape2 = tempReg;
				ShapeType tempShape = shapeAType;
				shapeAType = shapeBType;
				shapeBType = tempShape;
				RigidBody *tempBody = body1; 
				body1 = body2;
				body2 = tempBody;
				curManifold = &swappedManifold;
			}

			//HACK: check Collisions else if thing. Make this a better thing
			if(shapeAType == SHAPE_SPHERE && shapeBType == SHAPE_SPHERE)
			{
				ContactGenerator::sphere_sphere(*shape1, *body1, *shape2, *body2, *curManifold);
			}
			else if(shapeAType == SHAPE_SPHERE && shapeBType == SHAPE_HALFSPACE)
			{
				ContactGenerator::sphere_halfspace(*shape1, *body1, *shape2, *body2, *curManifold);
			}
			else if(shapeAType == SHAPE_BOX && shapeBType == SHAPE_BOX)
			{
				ContactGenerator::box_box(*shape1, *body1, *shape2, *body2, *curManifold);
			}
			else if(shapeAType == SHAPE_BOX && shapeBType == SHAPE_HALFSPACE)
			{
				ContactGenerator::box_halfspace(*shape1, *body1, *shape2, *body2, *curManifold);
			}
			else
			{
				//std::cout << "CollisionRegistry::Unhandled collision type (" << shapeAType << ", " << shapeBType << ")\n";
			}
		}
	}

	// Convert all swapped manifolds to normal manifolds.
	for(int i = 0; i < swappedManifold.getNumContacts(); ++i)
	{
		ContactPoint newContact;

		newContact = swappedManifold.getContactPoint(i);

		newContact.normal = -newContact.normal;

		normManifold.addContactPoint(newContact);
	}

	if(normManifold.getNumContacts() > 0)
	{
		contactManifolds.push_back(normManifold);
	}
}

void ContactGenerator::sphere_sphere(const CollisionShape &a, const RigidBody &rbA, const CollisionShape &b, const RigidBody &rbB, ContactManifold &contactManifold)
{
	// Make sure we have contacts left and Check if they're spheres.
	if ( a.getShapeType() != SHAPE_SPHERE || b.getShapeType() != SHAPE_SPHERE ) { return; }
	
	//  then typecast their shape to a sphere
	const ShapeSphere& shapeA = (const ShapeSphere&)a;
	const ShapeSphere& shapeB = (const ShapeSphere&)b;

	// Get the sphere positions
	Vec3 posA = a.getOffset().getPosition() + rbA.getPosition();
	Vec3 posB = b.getOffset().getPosition() + rbB.getPosition();

	// Find the vector between the two object
	Vec3 midLine = posA - posB;
	Scalar distance = midLine.length();

	// Check for collision
	if (distance <= 0.0f || distance >= shapeA.getRadius() + shapeB.getRadius())
	{
		return;
	}

	// Create contact data
	ContactPoint newContact;

	newContact.normal = midLine * ( ((Scalar)1.0) / distance );
	newContact.position = posA + (midLine * (Scalar)-0.5);
	newContact.penetration = (shapeA.getRadius() + shapeA.getRadius() - distance);

	contactManifold.addContactPoint(newContact);
}

void ContactGenerator::sphere_halfspace(const CollisionShape &a, const RigidBody &rbA, const CollisionShape &b, const RigidBody &rbB, ContactManifold &contactManifold)
{
	// Make sure we have contacts left and Check if they're the right shapes.
	if ( a.getShapeType() != SHAPE_SPHERE || b.getShapeType() != SHAPE_HALFSPACE ) { return; }

	// then typecast their appropriate shapes
	const ShapeSphere& sphere = (const ShapeSphere&)a;
	const ShapeHalfspace& halfspace = (const ShapeHalfspace&)b;

	// Get the positions and halfspace normal
	Vec3 posSphere = a.getOffset().getPosition() + rbA.getTransform().getPosition();
	Vec3 posHalfspace = a.getOffset().getPosition() + rbB.getTransform().getPosition();;
	Vec3 normHalfspace = rbB.getTransform() * b.getOffset() * Vec3(0.f, 1.f, 0.f, 0.f);

	// Find the distance from the plane to the sphere
	Scalar distance = normHalfspace.dot(posSphere) - sphere.getRadius() - (posHalfspace).y;

	// Check collision
	if(distance < 0)
	{
		// Create contact data
		ContactPoint newContact;

		newContact.normal = normHalfspace;
		newContact.position = posSphere + -newContact.normal*(sphere.getRadius() - newContact.penetration/2);
		newContact.penetration = -distance;

		contactManifold.addContactPoint(newContact);
	}

	return;
}

void ContactGenerator::box_box(const CollisionShape &a, const RigidBody &rbA, const CollisionShape &b, const RigidBody &rbB, ContactManifold &contactManifold)
{
	// Make sure we have contacts left and Check if they're boxes.
	if ( a.getShapeType() != SHAPE_BOX || b.getShapeType() != SHAPE_BOX ) { return; }

	// then typecast their appropriate shapes
	const ShapeBox& boxA = (const ShapeBox&)a;
	const ShapeBox& boxB = (const ShapeBox&)b;

	const Vec3& boxAHalfExtents = boxA.getHalfExtents();
	const Vec3& boxBHalfExtents = boxB.getHalfExtents();

	Transform boxATransform = rbA.getTransform() * a.getOffset();
	Transform boxBTransform = rbB.getTransform() * b.getOffset();

	// Vector between box centres.
	Vec3 separation = boxBTransform.getPosition() - boxATransform.getPosition();

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
		return;
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
		return;
	}

	// We've found a collision, and we know which of the axes gave the smallest penetration.
	if (bestPen < 3)
	{
		// Vertex of boxB in face of boxA
		// TODO: Do stuff
		//std::cout << bestPen << " Vertex of boxB in face of boxA\n";
		fillPointFaceBoxBox(boxA, boxATransform, boxB, boxBTransform, separation, contactManifold, bestPen, smallestPen, false);
		return;
	}
	else if (bestPen < 6)
	{
		// Vertex of boxA in face of boxB
		// TODO: Do stuff
		//std::cout << bestPen << " Vertex of boxA in face of boxB\n";
		fillPointFaceBoxBox(boxA, boxATransform, boxB, boxBTransform, -separation, contactManifold, bestPen-3, smallestPen, true);
		return;
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
		ContactPoint newContact;
		
		newContact.normal = axis;
		newContact.penetration = smallestPen;
		newContact.position = vertex;
		
		contactManifold.addContactPoint(newContact);
	}

	return;
}

void ContactGenerator::box_halfspace(const CollisionShape &a, const RigidBody &rbA, const CollisionShape &b, const RigidBody &rbB, ContactManifold &contactManifold)
{
	// Make sure we have contacts left and Check if they're the right shapes.
	if (a.getShapeType() != SHAPE_BOX || b.getShapeType() != SHAPE_HALFSPACE ) { return; }

	// then typecast their appropriate shapes
	const ShapeBox& box = (const ShapeBox&)a;
	const ShapeHalfspace& halfspace = (const ShapeHalfspace&)b;

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

	const Transform& boxTransform = rbA.getTransform();

	// Apply collision object offset
	for (int i = 0; i < 8; i++)
	{
		boxVertex[i] = boxTransform * a.getOffset() * boxVertex[i];
	}

	// Calculate halfspace's position and normal
	Vec3 posHalfspace = b.getOffset().getPosition() + rbB.getPosition();
	Vec3 normHalfspace = rbB.getTransform() * b.getOffset() * Vec3(0.f, 1.f, 0.f, 0.f);

	// Check each vertice for intersection with the halfspace
	Scalar vertexDistance;

	unsigned int numContacts = 0;

	for (int i = 0; i < 8; i++)
	{
		vertexDistance = boxVertex[i].dot(normHalfspace) - posHalfspace.y;

		if(vertexDistance <= 0)
		{
			// Create contact data
			ContactPoint newContact;

			newContact.normal = normHalfspace;
			newContact.penetration = -vertexDistance;
			newContact.position = boxVertex[i] + newContact.normal*(newContact.penetration*0.5f);

			contactManifold.addContactPoint(newContact);
		}
	}

	return;
}

void fillPointFaceBoxBox(const ShapeBox &boxA, const Transform &boxATransform,
						 const ShapeBox &boxB, const Transform &boxBTransform,
						 const Vec3 &separation, ContactManifold &contactManifold, unsigned bestPen, Scalar penetration, bool doSwapBodies)
{
	const ShapeBox *box0 = &boxA;
	const Transform *box0Transform = &boxATransform;

	const ShapeBox *box1 = &boxB;
	const Transform *box1Transform = &boxBTransform;

	if (doSwapBodies)
	{
		box0 = &boxB;
		box0Transform = &boxBTransform;
		box1 = &boxA;
		box1Transform = &boxATransform;
	}

	Vec3 normal = getShapeAxis(*box0, *box0Transform, bestPen);
	if (getShapeAxis(*box0, *box0Transform, bestPen).dot(separation) > 0)
	{
		normal = -normal;
	}

	Vec3 vertex = box1->getHalfExtents();
	if (getShapeAxis(*box1, *box1Transform, 0).dot(normal) < 0) { vertex.x = -vertex.x; }
	if (getShapeAxis(*box1, *box1Transform, 1).dot(normal) < 0) { vertex.y = -vertex.y; }
	if (getShapeAxis(*box1, *box1Transform, 2).dot(normal) < 0) { vertex.z = -vertex.z; }

	// Create contact data
	ContactPoint newContact;

	newContact.normal = (doSwapBodies) ? -normal : normal;
	newContact.penetration = penetration;
	newContact.position = *box1Transform * vertex - (normal * penetration * 0.5);

	contactManifold.addContactPoint(newContact);
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

} // namespace lt