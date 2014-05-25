#ifndef LTPHYS_SHAPEBOX_H
#define LTPHYS_SHAPEBOX_H

#include "CollisionShape.hpp"
#include "Vec3.hpp"

namespace lt
{

////////////////////////////////////////////////////////////
///	@brief Stores information for a box collision shape.
///
/// @author Leon Turpin
/// @date February 2014
////////////////////////////////////////////////////////////
class ShapeBox : public CollisionShape
{
public:
	////////////////////////////////////////////////////////////
	/// @brief default constructor
	///
	/// Creates a box with junk data as it's half extents.
	///
	////////////////////////////////////////////////////////////
	ShapeBox();

	////////////////////////////////////////////////////////////
	/// @brief Construct a box from it's half extents.
	///
	/// @param halfExtents The half sizes of each of the boxes sides
	///
	////////////////////////////////////////////////////////////
	ShapeBox(const Vec3& halfExtents);

	virtual ShapeType getShapeType() const { return LT_SHAPE_BOX; }

	////////////////////////////////////////////////////////////
	/// @brief Set the box's half extents
	///
	/// @param halfExtents The half sizes of each of the boxes sides
	///
	////////////////////////////////////////////////////////////
	void setHalfExtents(const Vec3& halfExtents);

	////////////////////////////////////////////////////////////
	/// @brief Get the box's half extents
	///
	/// @return halfExtents The half sizes of each of the boxes sides
	///
	////////////////////////////////////////////////////////////
	const Vec3& getHalfExtents() const;

private:
	Vec3 m_halfExtents;
};

} // namespace lt

#endif // LTPHYS_SHAPEHALFSPACE_H
