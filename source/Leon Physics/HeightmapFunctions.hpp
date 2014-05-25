#ifndef HEIGHTMAPFUNCTIONS_HPP
#define HEIGHTMAPFUNCTIONS_HPP

namespace heightMap
{
	
void genFromRAW(float heightMap[], unsigned int hmLength, unsigned int hmWidth, const char *fileName); 
void genHeightMapRandom(float heightMap[], unsigned int hmLength, unsigned int hmWidth);
void genHeightMapFaultFormation(float heightMap[], unsigned int hmLength, unsigned int hmWidth, unsigned int numIterations, float smoothWeight);
void firFilter(float heightMap[], unsigned int hmLength, unsigned int hmWidth, float weight);

} // namespace heightMap

#endif // HEIGHTMAPFUNCTIONS_HPP