#include "ShapeBox.hpp"

namespace lt
{

ShapeBox::ShapeBox() 
{

}

void ShapeBox::setHalfExtents(const Vec3& halfExtents)
{
	m_halfExtents = halfExtents;
}

const Vec3& ShapeBox::getHalfExtents() const
{
	return m_halfExtents;
}

} // namespace lt