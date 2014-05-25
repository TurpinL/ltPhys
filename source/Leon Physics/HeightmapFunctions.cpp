#include "HeightmapFunctions.hpp"

#include <random>
#include <iostream>
#include <fstream>
#include <cstdlib>

namespace heightMap
{

void genFromRAW(float heightMap[], unsigned int hmLength, unsigned int hmWidth, const char *fileName)
{
	// Open raw file
	std::ifstream rawImgFile;
	rawImgFile.open(fileName, std::ios::in|std::ios::binary|std::ios::ate);

	std::streampos size = 0;
	char *rawImg = nullptr;

	if(rawImgFile.is_open())
	{
		// Read data from the file.
		size = rawImgFile.tellg();
		rawImg = new char[(unsigned int)size];
		rawImgFile.seekg(0, std::ios::beg);
		rawImgFile.read(rawImg, size);
		rawImgFile.close();

		// Load data into heightmap
		for(unsigned int x = 0; x < hmLength; x++)
		{
			for(unsigned int z = 0; z < hmWidth; z++)
			{
				heightMap[x + z*hmLength] = (unsigned char)rawImg[(x + z*hmLength) % size] / 255.0f;
			}
		}
	}
	else
	{
		std::cout << "Couldn't open file \"" << fileName << "\"\n"; 
	}
}
	
void genHeightMapRandom(float heightMap[], unsigned int hmLength, unsigned int hmWidth)
{
	int numCells = hmLength * hmWidth;

	for (int i = 0; i < numCells; i++)
	{
		heightMap[i] = rand() / (float)RAND_MAX; 
	}
}

void genHeightMapFaultFormation(float heightMap[], unsigned int hmLength, unsigned int hmWidth, unsigned int numIterations, float smoothWeight)
{
	unsigned int numCells = hmLength * hmWidth;

	// Zero the heightmap
	for(unsigned int i = 0; i < numCells; i++)
	{
		heightMap[i] = 0; 
	}

	// Max and min heights of elements in the heightmap.
	float maxHeight = 1;
	float minHeight = 0;

	for (unsigned int i = 0; i < numIterations; i++)
	{
		float displacement = maxHeight-((maxHeight-minHeight) * i) / numIterations;

		// Pick random point
		unsigned int xPoint = rand() % hmLength;
		unsigned int zPoint = rand() % hmWidth;

		// Generate random Gradient 
		int rise = (rand() % 10000) - 5000;
		int run = (rand() % 10000) - 5000;
		float gradient;
		if(run == 0)
			gradient = 0;
		else
			gradient = (float)rise / (float)run;

		// Find the Z intercept
		int zIntercept = xPoint - (int)(gradient * zPoint);

		// Randomly selects wether cells above or below the equation will be raised.
		bool isRaised = (rand() % 2) == 0;

		// For each point on the heightmap, calculate if it's below the linear equation 
		// x = gradient * z + zIntercept
		for (unsigned int z = 0; z < hmWidth; z++)
		{
			// Calculate the value of the linear equation at this x.
			unsigned int xValue = (int)(gradient * z) + zIntercept;

			for (unsigned int x = 0; x < hmLength; x++)
			{
				if(x > xValue == isRaised)
				{
					heightMap[x + (z * hmLength)] += displacement;
				}
			}
		}

		firFilter(heightMap, hmLength, hmWidth, smoothWeight);
	}

	// Find max displacement
	float max = 0;
	float min = heightMap[0];
	for (unsigned int i = 0; i < numCells; i++)
	{
		if (heightMap[i] > max)
		{
			max = heightMap[i];
		}

		if (heightMap[i] < min)
		{
			min = heightMap[i];
		}
	}

	float normalizer = 1 / ((max - min) / (float)maxHeight);

	// Normalize the heights and assign them to the heightmap
	for(unsigned int i = 0; i < numCells; i++)
	{
		heightMap[i] = (heightMap[i] - min) * normalizer; 
	}
}

void filterPass(float heightMap[], int size, int increment, float weight)
{
	float yPrev = heightMap[0];
	int j = increment;
	float k = weight;

	for( int i = 1; i < size; i++)
	{
		heightMap[j] = k * yPrev + (1 - k) * heightMap[j];
		yPrev = heightMap[j];
		j += increment;
	}
}

void firFilter(float heightMap[], unsigned int hmLength, unsigned int hmWidth, float weight)
{
	//erode left to right, starting at the beginning of each row 
	for (unsigned int i = 0; i < hmLength; i++) 
		filterPass(&heightMap[hmLength*i], hmLength, 1, weight); 
 
	//erode right to left, starting at the end of each row 
	for (unsigned int i = 0; i < hmLength; i++) 
		filterPass(&heightMap[hmLength*i+hmLength-1], hmLength, -1, weight); 
 
	//erode top to bottom, starting at the beginning of each column 
	for (unsigned int i = 0; i < hmWidth; i++) 
		filterPass(&heightMap[i], hmWidth, hmLength, weight); 
 
	//erode from bottom to top, starting from the end of each column 
	for (unsigned int i = 0; i < hmWidth; i++) 
		filterPass(&heightMap[hmLength*(hmLength-1)+i], hmWidth, -(int)hmLength, weight); 
}

} // namespace heightMap