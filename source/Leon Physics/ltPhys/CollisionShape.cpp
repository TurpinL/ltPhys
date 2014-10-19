#include "CollisionShape.hpp"

namespace lt
{

	void CollisionShape::setOffset(const Transform& offset)
	{
		m_offset = offset;
	}

	const Transform& CollisionShape::getOffset() const
	{
		return m_offset;
	}
	
} // namespace lt