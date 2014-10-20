#include "ForceGeneratorRegistry.hpp"

namespace lt
{

ForceGeneratorRegistry::ForceGeneratorRegistry()
{

}

void ForceGeneratorRegistry::updateForces(Scalar timeStep)
{
	// Call the force generators on their paired rigid body
	for (unsigned int i = 0; i < m_registry.size(); i++)
	{
		m_registry[i].forceGen->updateForce(*m_registry[i].body, timeStep);
	}
}

void ForceGeneratorRegistry::add(RigidBody *body, ForceGenerator *forceGenerator)
{
	// Construct the registration
	ForceGenRegistration newRegistration = {body, forceGenerator};
	// Add the pair to the registry
	m_registry.push_back(newRegistration);
}

void ForceGeneratorRegistry::remove(RigidBody *body, ForceGenerator *forceGenerator)
{
	// Find all registrations that contain both the given body and forceGenerator.
	for (unsigned int i = 0; i < m_registry.size(); i++)
	{
		// Check for a match
		if (body == m_registry[i].body && 
			forceGenerator == m_registry[i].forceGen)
		{
			_removeElement(i);
			// Decrease the iterator to check the, now different, current element again. 
			i--;
		}
	}
}

void ForceGeneratorRegistry::remove(RigidBody *body)
{
	// Find all registrations that contain both the given body.
	for (unsigned int i = 0; i < m_registry.size(); i++)
	{
		// Check for a match
		if (body == m_registry[i].body)
		{
			_removeElement(i);
			// Decrease the iterator to check the, now different, current element again. 
			i--;
		}
	}
}

void ForceGeneratorRegistry::remove(ForceGenerator *forceGenerator)
{
	// Find all registrations that contain both the given forceGenerator.
	for (unsigned int i = 0; i < m_registry.size(); i++)
	{
		// Check for a match
		if (forceGenerator == m_registry[i].forceGen)
		{
			_removeElement(i);
			// Decrease the iterator to check the, now different, current element again. 
			i--;
		}
	}
}

void ForceGeneratorRegistry::clear()
{
	m_registry.clear();
}

//--------------------------
//	PRIVATES			
//--------------------------

void ForceGeneratorRegistry::_removeElement(int index)
{
	// Swap this element and the end so as not to leave holes.
	m_registry[index] = m_registry[m_registry.size() - 1]; 
	// Delete the duplicated element.
	m_registry.pop_back();
}


} // namespace lt