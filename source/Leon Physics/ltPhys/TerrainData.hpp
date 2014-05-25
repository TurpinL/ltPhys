#ifndef TERRAINDATA_HPP
#define TERRAINDATA_HPP

#include "Vec3.hpp"
#include "Mesh.hpp"

/**  TerrainData.hpp
 *	@brief 
 *
 *  Heightmap stored as heights from 0 to 1.
 *
 *  @author Leon Turpin
 *  @date March 2014
 */
class TerrainData
{
public:
	TerrainData();
	TerrainData(float *heightMap, unsigned int hmLength, unsigned int hmWidth, const lt::Vec3 &terrainSize);

	float getHeight(unsigned int x, unsigned int z) const;
	float getHeight(lt::Vec3& normal, float x, float z) const;

	void setHeightMap(float *heightMap, unsigned int hmLength, unsigned int hmWidth);
	void setTerrainSize(const lt::Vec3 &terrainSize);
	
	unsigned int getHeigthMapWidth() const;
	unsigned int getHeigthMapLength() const;
	const float* getHeightMap() const;
	const lt::Vec3& getTerrainSize() const; 
	const Mesh& getMesh() const;

private:
	void clearMesh();
	void calcMeshData();
	//float TerrainData::getOffset(const unsigned int &x, const unsigned int &z) const;

	Mesh mesh;

	float *m_heightMapData;
	unsigned int m_hmLength; // X dimension
	unsigned int m_hmWidth; // Z dimension
	lt::Vec3 m_terrainSize;
};

#endif // TERRAINDATA_HPP
