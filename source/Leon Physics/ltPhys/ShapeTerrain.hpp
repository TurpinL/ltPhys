#ifndef LTPHYS_SHAPETERRAIN_H
#define LTPHYS_SHAPETERRAIN_H

#include "CollisionShape.hpp"
#include "TerrainData.hpp"

namespace lt
{

/**  ShapeTerrain.hpp
 *	\brief 
 *
 *  \author Leon Turpin
 *  \date March 2014
 */
class ShapeTerrain : public CollisionShape
{
public:
	ShapeTerrain();

	virtual ShapeType getShapeType() const { return LT_SHAPE_TERRAIN; }

	void setTerrainData(TerrainData *terrainData);
	const TerrainData* getTerrainData() const;
private:
	TerrainData *m_terrainData;
};

} // namespace lt

#endif // LTPHYS_SHAPETERRAIN_H
