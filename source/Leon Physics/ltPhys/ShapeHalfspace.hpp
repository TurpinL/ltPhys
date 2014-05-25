#ifndef LTPHYS_SHAPEHALFSPACE_H
#define LTPHYS_SHAPEHALFSPACE_H

#include "CollisionShape.hpp"

namespace lt
{

/**  ShapeHalfspace.hpp
 *	@brief 
 *
 *  @author Leon Turpin
 *  @date February 2014
 */
class ShapeHalfspace : public CollisionShape
{
public:
	ShapeHalfspace();

	virtual ShapeType getShapeType() const { return LT_SHAPE_HALFSPACE; }
};

} // namespace lt

#endif // LTPHYS_SHAPEHALFSPACE_H
