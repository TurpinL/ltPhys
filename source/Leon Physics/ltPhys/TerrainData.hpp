#ifndef TERRAINDATA_HPP
#define TERRAINDATA_HPP

#include "Vec3.hpp"
#include "Mesh.hpp"

////////////////////////////////////////////////////////////
/// @brief Stores heightmap info and size for a terrain.
///
/// Note: Heightmap stored as heights from 0 to 1.
///
/// @author Leon Turpin
/// @date March 2014
////////////////////////////////////////////////////////////
class TerrainData
{
public:
	////////////////////////////////////////////////////////////
    /// @brief Default constructor
    ////////////////////////////////////////////////////////////
	TerrainData();

	////////////////////////////////////////////////////////////
    /// @brief Construct the terrain data from a heightmap, 
	///	heightmap dimensions and terrain dimension 
    ///
    /// @param heightMap the 2d array of heigts, each element 
	/// should be between 0 and 1.
    /// @param hmLength Size of the heightMap in the x direction
    /// @param hmWidth Size of the heightMap in the z direction
    /// @param terrainSize Size of the terrain to render/collide
    ///
    ////////////////////////////////////////////////////////////
	TerrainData(float *heightMap, unsigned int hmLength, unsigned int hmWidth, const lt::Vec3 &terrainSize);

    ////////////////////////////////////////////////////////////
    /// @brief Gets the height at the given integral
	/// heightmap co-ordinate
	///
	/// Note: Does NOT check if co-ordinates are in bounds
	///
	/// @param x X co-ordinate
	/// @param z Z co-ordinate
	///
	/// @return The height at the co-ordinates
	///
    ////////////////////////////////////////////////////////////
	float getHeight(unsigned int x, unsigned int z) const;
	
	////////////////////////////////////////////////////////////
    /// @brief Calculate the height at the given floating point
	/// heightmap co-ordinate
    ///
	/// Note: Checks if co-ordinates are in bounds
	///
	/// @param normal The normal of the terrain at the point.
	/// @param x X co-ordinate
	/// @param z Z co-ordinate
	///
	/// @return The height at the co-ordinates
	///
    ////////////////////////////////////////////////////////////
	float getHeight(lt::Vec3& normal, float x, float z) const;

	////////////////////////////////////////////////////////////
	///	@brief Set the heightmap data.
	///
	/// @param heightMap the 2d array of heigts, each element 
	/// should be between 0 and 1.
    /// @param hmLength Size of the heightMap in the x direction
    /// @param hmWidth Size of the heightMap in the z direction
	///
	////////////////////////////////////////////////////////////
	void setHeightMap(float *heightMap, unsigned int hmLength, unsigned int hmWidth);
	
	////////////////////////////////////////////////////////////
	///	@brief Set the dimesions of the terrain for rendering/collisions
	///
	/// @param terrainSize Size of the terrain to render/collide
	///
	////////////////////////////////////////////////////////////
	void setTerrainSize(const lt::Vec3 &terrainSize);
	
	////////////////////////////////////////////////////////////
	/// @brief get the width of the height map data.
	///
	/// @return Height map width.
	///
	////////////////////////////////////////////////////////////
	unsigned int getHeightMapWidth() const;

	////////////////////////////////////////////////////////////
	/// @brief get the width of the height map data.
	///
	/// @return Height map width.
	///
	////////////////////////////////////////////////////////////
	unsigned int getHeightMapLength() const;

	////////////////////////////////////////////////////////////
	/// @brief Get the heightmap data.
	/// 
	/// This 1d vector represents a 2d heightmap. Dimensions
	/// can be gotten with 
	/// unsigned int getHeightMapWidth() const;
	/// and unsigned int getHeightMapLength() const;
	///
	/// @return Heightmap data.
	///
	////////////////////////////////////////////////////////////
	const float* getHeightMap() const;

	////////////////////////////////////////////////////////////
	///	@brief Get the dimesions of the terrain for rendering/collisions
	///
	/// @return terrainSize Size of the terrain to render/collide
	///
	////////////////////////////////////////////////////////////
	const lt::Vec3& getTerrainSize() const; 

	////////////////////////////////////////////////////////////
	/// @brief Get the mesh data of this heightmap for rendering
	/// Contains vectors, normals and textureco-ordinates
	///
	/// @return A mesh of this heightmap.
	///
	////////////////////////////////////////////////////////////
	const Mesh& getMesh() const;

private:
	void clearMesh();
	void calcMeshData();
	//float TerrainData::getOffset(const unsigned int &x, const unsigned int &z) const;

	Mesh m_mesh;

	float *m_heightMapData;
	unsigned int m_hmLength; // X dimension
	unsigned int m_hmWidth; // Z dimension
	lt::Vec3 m_terrainSize;
};

#endif // TERRAINDATA_HPP
