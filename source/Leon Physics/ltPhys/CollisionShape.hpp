#ifndef LTPHYS_COLLISIONSHAPE_H
#define LTPHYS_COLLISIONSHAPE_H

namespace lt
{

enum ShapeType
{
	LT_SHAPE_NULL = 0,
	LT_SHAPE_SPHERE = 1,
	LT_SHAPE_BOX = 2,
	LT_SHAPE_HALFSPACE = 3,
	LT_SHAPE_TERRAIN = 4
};

/**  CollisionShape.hpp
 *	\brief 
 *
 *  \author Leon Turpin
 *  \date February 2014
 */
class CollisionShape
{
public:
	virtual ~CollisionShape() {}

	virtual ShapeType getShapeType() const = 0;
};

} // namespace lt

#endif // LTPHYS_COLLISIONSHAPE_H
