#ifndef LTPHYS_SHAPEBOX_H
#define LTPHYS_SHAPEBOX_H

#include "CollisionShape.hpp"
#include "Vec3.hpp"

namespace lt
{

/**  ShapeBox.hpp
 *	@brief 
 *
 *  @author Leon Turpin
 *  @date February 2014
 */
class ShapeBox : public CollisionShape
{
public:
	ShapeBox();
	ShapeBox(const Vec3& halfExtents);

	virtual ShapeType getShapeType() const { return LT_SHAPE_BOX; }

	void setHalfExtents(const Vec3& halfExtents);
	const Vec3& getHalfExtents() const;

private:
	Vec3 m_halfExtents;
};

} // namespace lt

#endif // LTPHYS_SHAPEHALFSPACE_H
