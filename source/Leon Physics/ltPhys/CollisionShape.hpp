#ifndef LTPHYS_COLLISIONSHAPE_H
#define LTPHYS_COLLISIONSHAPE_H

#include "../lt3DMath/lt3DMath.hpp"

namespace lt
{

////////////////////////////////////////////////////////////
/// @brief An enum of collision shape types
///
/// @author Leon Turpin
/// @date February 2014
////////////////////////////////////////////////////////////
enum ShapeType
{
	SHAPE_NULL = 0,
	SHAPE_SPHERE = 1,
	SHAPE_BOX = 2,
	SHAPE_HALFSPACE = 3,
};

////////////////////////////////////////////////////////////
/// @brief Abstract collision shape class
///
/// @author Leon Turpin
/// @date February 2014
////////////////////////////////////////////////////////////
class CollisionShape
{
public:
	////////////////////////////////////////////////////////////
	/// @brief Default destructor
	////////////////////////////////////////////////////////////
	virtual ~CollisionShape() {}

	////////////////////////////////////////////////////////////
	/// @brief Get the collision shape's type 
	///
	/// @return Collision shape type.
	///
	////////////////////////////////////////////////////////////
	virtual ShapeType getShapeType() const = 0;

	void setOffset(const Transform& offset);
	const Transform& getOffset() const;
private:
	Transform m_offset;
};

} // namespace lt

#endif // LTPHYS_COLLISIONSHAPE_H
