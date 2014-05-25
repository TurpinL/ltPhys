#ifndef LTPHYS_SHAPETERRAIN_H
#define LTPHYS_SHAPETERRAIN_H

#include "CollisionShape.hpp"
#include "TerrainData.hpp"

namespace lt
{

////////////////////////////////////////////////////////////
///	@brief Data for a terrain collision 
///
/// @author Leon Turpin
/// @date March 2014
//////////////////////////////////////////////////////////// 
class ShapeTerrain : public CollisionShape
{
public:
	//////////////////////////////////////////////////////////// 
	/// @brief Default Constructor
	///
	/// Creates a terrain with null terrain data
	/// 
	//////////////////////////////////////////////////////////// 
	ShapeTerrain();

	virtual ShapeType getShapeType() const { return LT_SHAPE_TERRAIN; }

	//////////////////////////////////////////////////////////// 
	/// @brief Sets this terrain's terrain data to terrainData
	///
	/// @param terrainData data to pass to this terrain shape
	///
	//////////////////////////////////////////////////////////// 
	void setTerrainData(TerrainData *terrainData);

	//////////////////////////////////////////////////////////// 
	/// @brief Gets this terrain's terrain data.
	///
	/// @return Terrain data
	///
	//////////////////////////////////////////////////////////// 
	const TerrainData* getTerrainData() const;
private:
	TerrainData *m_terrainData;
};

} // namespace lt

#endif // LTPHYS_SHAPETERRAIN_H
