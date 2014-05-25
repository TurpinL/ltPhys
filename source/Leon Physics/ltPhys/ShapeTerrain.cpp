#include "ShapeTerrain.hpp"

namespace lt
{

ShapeTerrain::ShapeTerrain() 
{
	m_terrainData = nullptr;
}

void ShapeTerrain::setTerrainData(TerrainData *terrainData)
{
	m_terrainData = terrainData;
}

const TerrainData* ShapeTerrain::getTerrainData() const
{
	return m_terrainData;
}

} // namespace lt