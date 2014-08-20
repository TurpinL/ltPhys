#include "CollisionRegistry.hpp"
#include "ContactGenerator.hpp"

#include "iostream"

namespace lt
{

//--------------------------
//	PUBLICS			
//--------------------------

CollisionRegistry::CollisionRegistry()
{
	m_collisionData.contacts = nullptr;
	m_collisionData.contactsLeft = 0;
	m_collisionData.size = 0;
}

CollisionRegistry::CollisionRegistry(unsigned int maxContacts)
{
	m_collisionData.contacts = new Contact[maxContacts];
	m_collisionData.contactsLeft = maxContacts;
	m_collisionData.size = maxContacts;
}

void CollisionRegistry::findContacts()
{
	// Reset the contact list
	m_collisionData.contactsLeft = m_collisionData.size;

	for (unsigned int i = 0; i < m_registry.size(); i++)
	{
		// Quit if we can't have more collisions
		if(m_collisionData.contactsLeft == 0)
		{
			break;
		}

		for (unsigned int j = i+1; j < m_registry.size(); j++)
		{
			// Quit if we can't have more collisions
			if(m_collisionData.contactsLeft == 0)
			{
				break;
			}

			// Don't collide if they are the same rigid body, unless they're both null.
			if(m_registry[i].body != m_registry[j].body ||
				(m_registry[i].body == nullptr && m_registry[i].body == nullptr))
			{
				checkCollision(m_registry[i], m_registry[j]);
			}
		}
	}
}

void CollisionRegistry::add(RigidBody* body, CollisionShape* shape, const Transform& offset)
{
	// Construct the registration
	CollisionRegistration newRegistration = {shape, body, offset};
	// Add the pair to the registry
	m_registry.push_back(newRegistration);
}

void CollisionRegistry::remove(RigidBody* body, CollisionShape* shape)
{
	// Find all registrations that contain both the given body and shape.
	for (unsigned int i = 0; i < m_registry.size(); i++)
	{
		// Check for a match
		if (body == m_registry[i].body && 
			shape == m_registry[i].shape)
		{
			removeElement(i);
			// Decrease the iterator to check the, now different, current element again. 
			i--;
		}
	}
}

void CollisionRegistry::remove(RigidBody* body)
{
	// Find all registrations that contain both the given body.
	for (unsigned int i = 0; i < m_registry.size(); i++)
	{
		// Check for a match
		if (body == m_registry[i].body)
		{
			removeElement(i);
			// Decrease the iterator to check the, now different, current element again. 
			i--;
		}
	}
}

void CollisionRegistry::remove(CollisionShape *shape)
{
	// Find all registrations that contain both the given shape.
	for (unsigned int i = 0; i < m_registry.size(); i++)
	{
		// Check for a match
		if (shape == m_registry[i].shape)
		{
			removeElement(i);
			// Decrease the iterator to check the, now different, current element again. 
			i--;
		}
	}
}

void CollisionRegistry::clear()
{
	m_registry.clear();
}

void CollisionRegistry::setMaxContacts(unsigned int newMaxContacts)
{
	delete[] m_collisionData.contacts;

	m_collisionData.contacts = new Contact[newMaxContacts];
	m_collisionData.contactsLeft = newMaxContacts;
	m_collisionData.size = newMaxContacts;
}

unsigned int CollisionRegistry::getMaxContacts() const
{
	return m_collisionData.size;
}

const CollisionData& CollisionRegistry::getCollisionData() const
{
	return m_collisionData;
}

const std::vector<CollisionRegistration>& CollisionRegistry::getCollisionRegistry() const
{
	return m_registry;
}

//--------------------------
//	PRIVATES			
//--------------------------

void CollisionRegistry::removeElement(int index)
{
	// Swap this element and the end so as not to leave holes.
	m_registry[index] = m_registry[m_registry.size()]; 
	// Delete the duplicated element.
	m_registry.pop_back();
}

void CollisionRegistry::checkCollision(const CollisionRegistration& shapeA, const CollisionRegistration& shapeB)
{
	// Get the shape types
	const ShapeType& shapeAType = shapeA.shape->getShapeType();
	const ShapeType& shapeBType = shapeB.shape->getShapeType();

	//HACK: check Collisions else if thing. Make this a better thing
	if(shapeAType == SHAPE_SPHERE && shapeBType == SHAPE_SPHERE)
	{
		ContactGenerator::sphere_sphere(shapeA, shapeB, &m_collisionData);
	}
	else if(shapeAType == SHAPE_SPHERE && shapeBType == SHAPE_HALFSPACE)
	{
		ContactGenerator::sphere_halfspace(shapeA, shapeB, &m_collisionData);
	}
	else if(shapeAType == SHAPE_BOX && shapeBType == SHAPE_HALFSPACE)
	{
		ContactGenerator::box_halfspace(shapeA, shapeB, &m_collisionData);
	}
	else if(shapeAType == SHAPE_SPHERE && shapeBType == SHAPE_TERRAIN)
	{
		ContactGenerator::sphere_terrain(shapeA, shapeB, &m_collisionData);
	}
	else if(shapeAType == SHAPE_BOX && shapeBType == SHAPE_TERRAIN)
	{
		ContactGenerator::box_terrain(shapeA, shapeB, &m_collisionData);
	}
	else if(shapeAType == SHAPE_BOX && shapeBType == SHAPE_BOX)
	{
		ContactGenerator::box_box(shapeA, shapeB, &m_collisionData);
	}
	else
	{
		//std::cout << "CollisionRegistry::Unhandled collision type (" << shapeAType << ", " << shapeBType << ")\n";
	}
}


} // namespace lt