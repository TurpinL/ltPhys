#include "PhysicsDemo.hpp"
#include <iostream>
#include <cmath>

void drawMesh(const Mesh &mesh);
void drawBox(lt::Vec3 halfExtents);
void drawSphere(float radius, unsigned int segments);

void PhysicsDemo::display()
{
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
		glLoadIdentity();

		float transform[16];

		glTranslatef(0.0f, 0.0f, m_camZoom);
		glRotatef(m_camAng.x, 1.0f, 0.0f, 0.0f);
		glRotatef(m_camAng.y, 0.0f, 1.0f, 0.0f);
		glRotatef(m_camAng.z, 0.0f, 0.0f, 1.0f);

		// Reference Axes
		glDisable(GL_LIGHTING);
			glBegin(GL_LINES);
				glColor3f(1.0f, 0.0f, 0.0f);
				glVertex3f( 1.0f,  0.0f,  0.0f);	
				glVertex3f( 0.0f,  0.0f,  0.0f);	

				glColor3f(0.0f, 1.0f, 0.0f);
				glVertex3f( 0.0f,  1.0f,  0.0f);	
				glVertex3f( 0.0f,  0.0f,  0.0f);	

				glColor3f(0.0f, 0.0f, 1.0f);
				glVertex3f( 0.0f,  0.0f,  1.0f);	
				glVertex3f( 0.0f,  0.0f,  0.0f);	
			glEnd();

			// Draw Collision information.
			const lt::CollisionData& colData = world.m_collisionRegistry.getCollisionData();
			for(unsigned int i = 0; i < colData.size - colData.contactsLeft; i++)
			{
				glPointSize(8);
				glLineWidth(4);
				lt::Vec3& pos = colData.contacts[i].position;
				lt::Vec3& norm = colData.contacts[i].normal;
				lt::Scalar& pen = colData.contacts[i].penetration;

				lt::Vec3 start = pos + (norm * (pen*0.5f));
				lt::Vec3 end   = pos + (norm * (pen*-0.5f));

				glBegin(GL_POINTS);
					glColor3f(1.f, 0.5f, 0.f);
					glVertex3f(pos.x, pos.y, pos.z);
				glEnd();

				glBegin(GL_LINES);
					glColor3f(1.f, 1.f, 1.f);
					glVertex3f(start.x, start.y, start.z);
					glVertex3f(end.x, end.y, end.z);
				glEnd();

				glPointSize(1);
				glLineWidth(1);

				glPushMatrix();
					glTranslatef(pos.x, pos.y, pos.z);

					// Draw axes that are oriented by the collision normal
					lt::Mat3 basis = lt::World::constructOrthonormalBasis(norm);

					lt::Vec3 x = basis * lt::Vec3(1.f, 0.f, 0.f);
					lt::Vec3 y = basis * lt::Vec3(0.f, 1.f, 0.f);
					lt::Vec3 z = basis * lt::Vec3(0.f, 0.f, 1.f);

					lt::Scalar scale = 0.1f;

					glBegin(GL_LINES);
						glColor3f(1.0f, 0.0f, 0.0f);
						glVertex3f( norm.x * scale,  norm.y * scale,  norm.z * scale);	
						glVertex3f( 0.0f,  0.0f,  0.0f);	

						glColor3f(0.0f, 1.0f, 0.0f);
						glVertex3f( y.x * scale,  y.y * scale,  y.z * scale);	
						glVertex3f( 0.0f,  0.0f,  0.0f);	

						glColor3f(0.0f, 0.0f, 1.0f);
						glVertex3f( z.x * scale,  z.y * scale,  z.z * scale);	
						glVertex3f( 0.0f,  0.0f,  0.0f);	
					glEnd();

				glPopMatrix();
			}

		glEnable(GL_LIGHTING);
		// Sphere 1
		glPolygonMode(GL_FRONT, GL_LINE);
		glPushMatrix();
			m_controlledBody.getTransform().getOpenGLMatrix(transform);	
			glMultMatrixf(transform);

			glColor3f(1.0f, 0.0f, 0.0f);
			drawSphere(m_staticSphereShape.getRadius(), 20);
		glPopMatrix();

		// Sphere 2
		glPolygonMode(GL_FRONT, GL_LINE);
		glPushMatrix();
			lt::Transform::Identity().getOpenGLMatrix(transform);	
			glMultMatrixf(transform);

			glColor3f(1.0f, 1.0f, 0.0f);
			drawSphere(m_staticSphereShape.getRadius(), 20);
		glPopMatrix();

		// Phys Sphere
		glPushMatrix();
			m_myRigidBody.getTransform().getOpenGLMatrix(transform);	
			glMultMatrixf(transform);

			glColor3f(1.0f, 0.0f, 1.0f);
			drawSphere(m_staticSphereShape.getRadius(), 20);
		glPopMatrix();

		//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

		// Halfspace
		glPushMatrix();
			m_groundBody.getTransform().getOpenGLMatrix(transform);	
			glMultMatrixf(transform);

			glBegin(GL_TRIANGLE_STRIP);
				glNormal3f(0.f, 1.f, 0.f);
				glColor3f(0.2f, 1.f, 0.2f);
				glVertex3f(-10.f, 0,  10.f);
				glVertex3f(-10.f, 0, -10.f);
				glVertex3f( 10.f, 0,  10.f);
				glVertex3f( 10.f, 0, -10.f);
			glEnd();
		glPopMatrix();

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		// Phys Box
		glPushMatrix();
			m_box.getTransform().getOpenGLMatrix(transform);	
			glMultMatrixf(transform);

			glColor3f(0.0f, 0.5f, 1.0f);
			drawBox(m_boxShape.getHalfExtents());
		glPopMatrix();

		// Render Terrain
		glPushMatrix();
			m_terrainBody.getTransform().getOpenGLMatrix(transform);
			glMultMatrixf(transform);
			glColor3f(0.5f, 1.0f, 0.0f);
			drawMesh(m_terrainShape.getTerrainData()->getMesh());
		glPopMatrix();

		// Test Spring
		glDisable(GL_LIGHTING);
		glPushMatrix();
			lt::Vec3 p1;
			lt::Vec3 p2 = m_controlledBody.getPosition();

			glColor3f(1.0f, 0.5f, 0.0f);
			glBegin(GL_LINES);
				p1 = m_box.getPosition() + m_box.getPointInWorldSpace(m_boxSpringOffset);
				glVertex3f(p1.x, p1.y, p1.z);
				glVertex3f(p2.x, p2.y, p2.z);

				p1 = m_myRigidBody.getPosition() + m_myRigidBody.getPointInWorldSpace(lt::Vec3(-0.5f, 0.f, 0.f));
				glVertex3f(p1.x, p1.y, p1.z);
				glVertex3f(p2.x, p2.y, p2.z);
			glEnd();

		glPopMatrix();


		glEnable(GL_LIGHTING);

		glFlush();
	SDL_GL_SwapBuffers();
}

void drawMesh(const Mesh &mesh)
{
	//lt::Vec3 colour[3] = 
	//{
	//	lt::Vec3(1.0f, 0.0f, 0.0f),
	//	lt::Vec3(0.0f, 1.0f, 0.0f),
	//	lt::Vec3(0.0f, 0.0f, 1.0f),
	//};
		
	glBegin(GL_TRIANGLES);
		for (GLuint i = 0; i < mesh.numIndices; i++)
		{
			GLuint index = mesh.indices[i];
			//glColor3f(colour[i%3].x, colour[i%3].y, colour[i%3].z);
			glNormal3f(mesh.norms[index].x, mesh.norms[index].y, mesh.norms[index].z);
			glTexCoord2f(mesh.texCoords[index].x, mesh.texCoords[index].y);
			glVertex3f(mesh.verts[index].x, mesh.verts[index].y, mesh.verts[index].z);
		}
	glEnd();
}

const float CUBE_VERTS[][3] = {
	{-1.0f,  1.0f, -1.0f}, 
	{-1.0f, -1.0f, -1.0f},
	{ 1.0f,  1.0f, -1.0f},
	{ 1.0f, -1.0f, -1.0f},
	{ 1.0f,  1.0f,  1.0f},
	{ 1.0f, -1.0f,  1.0f},
	{-1.0f,  1.0f,  1.0f},
	{-1.0f, -1.0f,  1.0f},
};
const int CUBE_VERT_INDEX[] = 
{ 
	1, 0, 2, 1, 2, 3,	// +Z
	3, 2, 4, 3, 4, 5,	// +X
	5, 4, 6, 5, 6, 7,	// -Z
	7, 6, 0, 7, 0, 1,	// -X
	0, 6, 4, 0, 4, 2,	// +Y
	1, 7, 5, 1, 5, 3	// -Y 
};
const float CUBE_NORMS[][3] = {
	{ 0.0f,  0.0f, -1.0f}, 
	{ 1.0f,  0.0f,  0.0f}, 
	{ 0.0f,  0.0f,  1.0f}, 
	{-1.0f,  0.0f,  0.0f}, 
	{ 0.0f,  1.0f,  0.0f}, 
	{ 0.0f, -1.0f,  0.0f}, 
};
const int CUBE_NORM_INDEX[] = 
{ 
	0, 0, 0, 0, 0, 0,	// +Z
	1, 1, 1, 1, 1, 1,	// +X
	2, 2, 2, 2, 2, 2,	// -Z
	3, 3, 3, 3, 3, 3,	// -X
	4, 4, 4, 4, 4, 4,	// +Y
	5, 5, 5, 5, 5, 5	// -Y 
};

void drawBox(lt::Vec3 halfExtents)
{
	glBegin(GL_TRIANGLES);
		for(int i = 0; i < 36; i++)
		{
			glNormal3f(CUBE_NORMS[CUBE_NORM_INDEX[i]][0], CUBE_NORMS[CUBE_NORM_INDEX[i]][1], CUBE_NORMS[CUBE_NORM_INDEX[i]][2]);
			glVertex3f(CUBE_VERTS[CUBE_VERT_INDEX[i]][0] * halfExtents.x, CUBE_VERTS[CUBE_VERT_INDEX[i]][1] * halfExtents.y, CUBE_VERTS[CUBE_VERT_INDEX[i]][2] * halfExtents.z);
		}
	glEnd();
}

void drawSphere(float radius, unsigned int segments)
{
	lt::Vec3 *sphereNormals;

	static const float TAU = 6.283185307f;
	static const float PI  = 3.141592654f;

	const unsigned int numSlices = segments;
	const unsigned int numStacks = segments / 2 + 1;
	const float sliceAngleStep = TAU / (float)numSlices;
	const float stackAngleStep = PI / (float)(numStacks - 1);
	
	sphereNormals = new lt::Vec3[numSlices * numStacks];

	float curSliceAngle = 0;
	float curStackAngle = 0;

	int index = 0;
	
	for (unsigned int curStack = 0; curStack < numStacks; curStack++)
	{
		for (unsigned int curSlice = 0; curSlice < numSlices; curSlice++)
		{
			float stackRadius = sin(curStackAngle);

			sphereNormals[index] = lt::Vec3(stackRadius*sin(curSliceAngle), cos(curStackAngle), stackRadius*cos(curSliceAngle));

			curSliceAngle += sliceAngleStep;
			index++;
		}
		curSliceAngle = 0;
		curStackAngle += stackAngleStep;
	}
	

	glBegin(GL_TRIANGLES);
		for(unsigned int i = 0; i < numSlices * numStacks - segments - 1; i++)
		{
			//glColor3f(i/(float)(numSlices * numStacks - segments), 1 - i/(float)(numSlices * numStacks - segments), 0.0f);
			glNormal3f(sphereNormals[i].x, sphereNormals[i].y, sphereNormals[i].z);
			glVertex3f(sphereNormals[i].x * radius, sphereNormals[i].y * radius, sphereNormals[i].z * radius);
			glNormal3f(sphereNormals[i+segments].x, sphereNormals[i+segments].y, sphereNormals[i+segments].z);
			glVertex3f(sphereNormals[i+segments].x * radius, sphereNormals[i+segments].y * radius, sphereNormals[i+segments].z * radius);
			glNormal3f(sphereNormals[i+1].x, sphereNormals[i+1].y, sphereNormals[i+1].z);
			glVertex3f(sphereNormals[i+1].x * radius, sphereNormals[i+1].y * radius, sphereNormals[i+1].z * radius);

			//glColor3f(1 - i/(float)(numSlices * numStacks - segments), i/(float)(numSlices * numStacks - segments), 0.0f);
			glNormal3f(sphereNormals[i+segments+1].x, sphereNormals[i+segments+1].y, sphereNormals[i+segments+1].z);
			glVertex3f(sphereNormals[i+segments+1].x * radius, sphereNormals[i+segments+1].y * radius, sphereNormals[i+segments+1].z * radius);
			glNormal3f(sphereNormals[i+1].x, sphereNormals[i+1].y, sphereNormals[i+1].z);
			glVertex3f(sphereNormals[i+1].x * radius, sphereNormals[i+1].y * radius, sphereNormals[i+1].z * radius);
			glNormal3f(sphereNormals[i+segments].x, sphereNormals[i+segments].y, sphereNormals[i+segments].z);
			glVertex3f(sphereNormals[i+segments].x * radius, sphereNormals[i+segments].y * radius, sphereNormals[i+segments].z * radius);
		}
	glEnd();

	delete[] sphereNormals;
}

