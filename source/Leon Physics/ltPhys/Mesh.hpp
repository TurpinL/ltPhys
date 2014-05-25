#ifndef MESH_HPP
#define MESH_HPP

#include <GL\glew.h>
#include "Vec3.hpp"

/**
 * @brief Stores data for a mesh to be rendered
 */
struct Mesh
{
	lt::Vec3 *verts;
	GLuint  numVerts;

	lt::Vec3 *norms;
	GLuint  numNorms;

	lt::Vec3 *texCoords;
	GLuint  numTexCoords;

	// Stored as  in sets of three indices.
	// [vert, norm, tex, v, n, t, v, n...]
	GLuint *indices;
	GLuint  numIndices;

	Mesh()
	: verts(nullptr), norms(nullptr), texCoords(nullptr), indices(nullptr),
		numVerts(0), numNorms(0), numTexCoords(0), numIndices(0)
	{}
};

#endif // MESH_HPP