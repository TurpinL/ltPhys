#include "ShapeSphere.hpp"

namespace lt
{

ShapeSphere::ShapeSphere() 
{
	m_radius = 0;
}

ShapeSphere::ShapeSphere(const Scalar& radius)
: m_radius(radius)
{}

void ShapeSphere::setRadius(const Scalar& radius)
{
	m_radius = radius;
}

const Scalar& ShapeSphere::getRadius() const
{
	return m_radius;
}

} // namespace lt