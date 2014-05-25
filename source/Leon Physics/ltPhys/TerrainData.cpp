#include "TerrainData.hpp"
#include <cmath>

const lt::Vec3 calcTriangleNorm(const lt::Vec3 &a, const lt::Vec3 &b, const lt::Vec3 &c)
{
	return ((b - a).cross(c - a)).normalized();
}

////////////////////////////////////////
//			PUBLICS
////////////////////////////////////////

TerrainData::TerrainData() 
{
	m_heightMapData = nullptr;
	m_hmLength = 0;
	m_hmWidth = 0;
	m_terrainSize = lt::Vec3(0, 0, 0);
}

TerrainData::TerrainData(float *heightMap, unsigned int hmLength, unsigned int hmWidth, const lt::Vec3 &terrainSize)
{
	m_heightMapData = nullptr;
	m_terrainSize = terrainSize;
	setHeightMap(heightMap, hmLength, hmWidth);
}

//float TerrainData::getOffset(const unsigned int &x, const unsigned int &z) const
//{
//	// Check out of bounds
//	if ( (x > 0) || (z > 0) || (x < m_hmLength - 1) || (z < m_hmWidth - 1) )
//	{
//		return -1;
//	}
//
//	return m_heightMapData[x + (z * m_hmLength)];
//}

float TerrainData::getHeight(lt::Vec3& normal, float x, float z) const
{
	// Convert the co-ordinates from world space into grid space
	float gx = ((float)x / m_terrainSize.x) * ((float)m_hmLength - 1);
	float gz = ((float)z / m_terrainSize.z) * ((float)m_hmWidth - 1);

	// Check out of bounds
	if ( (gx < 0) || (gz < 0) || (gx > m_hmLength - 1) || (gz > m_hmWidth - 1) )
	{
		// The heights should only be positive values, so we return -1 to indicate an invalid variable.
		return -1; 
	}

	float h0 = getHeight((unsigned int)gx  , (unsigned int)gz+1); // Bottom Left
	float h1 = getHeight((unsigned int)gx  , (unsigned int)gz  ); // Top Left
	float h2 = getHeight((unsigned int)gx+1, (unsigned int)gz+1); // Bottom Right
	float h3 = getHeight((unsigned int)gx+1, (unsigned int)gz  ); // Top Right

	// Calculate the co-ordinate position within the cell.
	float cx = fmod(gx, 1);
	float cz = fmod(gz, 1);

	if(cz > cx)
	{
		normal = calcTriangleNorm(lt::Vec3(0, h0, 1), lt::Vec3(1, h2, 1), lt::Vec3(0, h1, 0));
		return m_terrainSize.y * (h0 + cx * (h2 - h0) + (1.0f - cz) * (h1 - h0));
	}
	else
	{
		normal = calcTriangleNorm(lt::Vec3(0, h1, 0), lt::Vec3(1, h2, 1), lt::Vec3(1, h3, 0));
		return m_terrainSize.y * (h3 + (1.0f - cx) * (h1 - h3) + cz * (h2 - h3));
	}
}

void TerrainData::setHeightMap(float *heightMap, unsigned int hmLength, unsigned int hmWidth)
{
	m_hmLength = hmLength;
	m_hmWidth = hmWidth;

	// Calculate the total number of elements in the heightmap
	unsigned int mapSize = m_hmLength * m_hmWidth; 

	// Delete the old heightmap data if any exists.
	if(m_heightMapData != nullptr)
	{
		delete[] m_heightMapData;
	}

	// Allocate memory to store the new data.
	m_heightMapData = new float[mapSize];

	for (unsigned int i = 0; i < mapSize; i++)
	{
		m_heightMapData[i] = heightMap[i];
	}

	calcMeshData();
}

void TerrainData::setTerrainSize(const lt::Vec3 &terrainSize)
{
	m_terrainSize = terrainSize;
}

unsigned int TerrainData::getHeightMapLength() const
{
	return m_hmLength;
}

unsigned int TerrainData::getHeightMapWidth() const
{
	return m_hmWidth;
}

const float* TerrainData::getHeightMap() const
{
	return m_heightMapData;
}

float TerrainData::getHeight(unsigned int x, unsigned int z) const
{
	return m_heightMapData[x + (z * m_hmLength)];
}

const lt::Vec3& TerrainData::getTerrainSize() const
{
	return m_terrainSize;
}

const Mesh& TerrainData::getMesh() const
{
	return m_mesh;
}

////////////////////////////////////////
//			PRIVATES
////////////////////////////////////////

void TerrainData::clearMesh()
{
	// Delete the previous m_mesh data, if it had any.
	if(m_mesh.verts != nullptr) { delete[] m_mesh.verts; }
	if(m_mesh.norms != nullptr) { delete[] m_mesh.norms; }
	if(m_mesh.texCoords != nullptr) { delete[] m_mesh.texCoords; }
	if(m_mesh.indices != nullptr) { delete[] m_mesh.indices; }

	m_mesh.numVerts = 0;
	m_mesh.numNorms = 0;
	m_mesh.numTexCoords = 0;
	m_mesh.numIndices = 0;
}

void TerrainData::calcMeshData()
{
	clearMesh();

	// Verts normas and texture co-ordinates should be the same.
	// One for each heightmap element.
	unsigned int numCells = m_hmLength * m_hmWidth;

	m_mesh.numVerts = numCells;
	m_mesh.verts = new lt::Vec3[m_mesh.numVerts];

	m_mesh.numNorms = numCells;
	m_mesh.norms = new lt::Vec3[m_mesh.numNorms];

	m_mesh.numTexCoords = numCells;
	m_mesh.texCoords = new lt::Vec3[m_mesh.numTexCoords];

	// Each set of neighbouring 4 elements of the heightmap will share 2 triangles.
	// 3 vertices per triangle. 2 * 3 = 6. Hence the 6 in the equation.
	m_mesh.numIndices = (m_hmLength - 1) * (m_hmWidth - 1) * 6;
	m_mesh.indices = new GLuint[m_mesh.numIndices];

	// Calculate the distance between each point.
	float xStride = m_terrainSize.x / (m_hmLength - 1);
	float zStride = m_terrainSize.z / (m_hmWidth - 1);
	
	float heightMult = m_terrainSize.y;
	// Calculate Vertex Positions and Texture Coordinates.
	for (unsigned int x = 0; x < m_hmLength; x++)
	{
		for (unsigned int z = 0; z < m_hmWidth; z++)
		{
			unsigned int index = x + (z * m_hmLength);

			m_mesh.verts[index] = lt::Vec3(x * xStride, m_heightMapData[index] * heightMult, z * zStride);
			m_mesh.texCoords[index] = lt::Vec3(x * xStride, z * zStride, 0);
		}
	}

	// Calculate Normals
	const int Z = m_hmLength;
	const int X = 1;

	for (unsigned int x = 0; x < m_hmLength; x++)
	{
		for (unsigned int z = 0; z < m_hmWidth; z++)
		{
			int i = x + (z * Z);

			// Ignore edge cases for now.
			if(!(x == 0 || x == m_hmLength - 1 || z == 0 || z == m_hmWidth - 1))
			{
				// Sample surrounding nodes.
				m_mesh.norms[i] =  calcTriangleNorm(m_mesh.verts[i], m_mesh.verts[i-X-Z], m_mesh.verts[i-X]);
				m_mesh.norms[i] += calcTriangleNorm(m_mesh.verts[i], m_mesh.verts[i-Z  ], m_mesh.verts[i-X-Z]);
				m_mesh.norms[i] += calcTriangleNorm(m_mesh.verts[i], m_mesh.verts[i+X  ], m_mesh.verts[i-Z]);
				m_mesh.norms[i] += calcTriangleNorm(m_mesh.verts[i], m_mesh.verts[i+X+Z], m_mesh.verts[i+X]);
				m_mesh.norms[i] += calcTriangleNorm(m_mesh.verts[i], m_mesh.verts[i+Z  ], m_mesh.verts[i+X+Z]);
				m_mesh.norms[i] += calcTriangleNorm(m_mesh.verts[i], m_mesh.verts[i-X  ], m_mesh.verts[i+Z]);
				m_mesh.norms[i].normalize();
			}
			else
			{
				// Give the edges a verticle normal.
				m_mesh.norms[i] = lt::Vec3(0.0f, 1.0f, 0.0f);
			}
		}
	}

	// Calculate Indices
	int curIndex = 0;
	for (unsigned int x = 0; x < m_hmLength - 1; x++)
	{
		for (unsigned int z = 0; z < m_hmWidth - 1; z++)
		{
			m_mesh.indices[curIndex] = (x  ) + ((z+1) * m_hmLength); curIndex++;
			m_mesh.indices[curIndex] = (x  ) + ((z  ) * m_hmLength); curIndex++;
			m_mesh.indices[curIndex] = (x+1) + ((z+1) * m_hmLength); curIndex++;

			m_mesh.indices[curIndex] = (x+1) + ((z+1) * m_hmLength); curIndex++;
			m_mesh.indices[curIndex] = (x  ) + ((z  ) * m_hmLength); curIndex++;
			m_mesh.indices[curIndex] = (x+1) + ((z  ) * m_hmLength); curIndex++;
		}
	}
}