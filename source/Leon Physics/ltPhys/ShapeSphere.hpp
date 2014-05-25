#ifndef LTPHYS_SHAPESPHERE_H
#define LTPHYS_SHAPESPHERE_H

#include "CollisionShape.hpp"
#include "Scalar.hpp"

namespace lt
{

/**  ShapeSphere.hpp
 *	@brief 
 *
 *  @author Leon Turpin
 *  @date February 2014
 */
class ShapeSphere : public CollisionShape
{
public:
	ShapeSphere();
	ShapeSphere(const Scalar& radius);

	void setRadius(const Scalar& radius);

	const Scalar& getRadius() const;
	virtual ShapeType getShapeType() const { return LT_SHAPE_SPHERE; }

private:
	Scalar m_radius;
};

} // namespace lt

#endif // LTPHYS_SHAPESPHERE_H
