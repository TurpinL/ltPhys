#include "ShapeBox.hpp"

namespace lt
{

ShapeBox::ShapeBox() 
{

}

ShapeBox::ShapeBox(const Vec3& halfExtents)
{
	m_halfExtents = halfExtents;
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