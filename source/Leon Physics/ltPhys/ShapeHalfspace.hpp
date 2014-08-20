#ifndef LTPHYS_SHAPEHALFSPACE_H
#define LTPHYS_SHAPEHALFSPACE_H

#include "CollisionShape.hpp"

namespace lt
{

////////////////////////////////////////////////////////////
/// @brief Hold shape information for a halfspace.
///
/// Halfspaces don't really need any information. 
/// They are entierly defined by their offset to a rigidbody or world.
///
/// @author Leon Turpin
/// @date February 2014
////////////////////////////////////////////////////////////
class ShapeHalfspace : public CollisionShape
{
public:
	////////////////////////////////////////////////////////////
	/// @brief Default constructor
	////////////////////////////////////////////////////////////
	ShapeHalfspace();

	virtual ShapeType getShapeType() const { return SHAPE_HALFSPACE; }
};

} // namespace lt

#endif // LTPHYS_SHAPEHALFSPACE_H
