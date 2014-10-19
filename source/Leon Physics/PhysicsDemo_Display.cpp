#include "PhysicsDemo.hpp"
#include <iostream>
#include <cmath>

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

		//lt::Vec3 camPos = -m_box.getTransform().getPosition();
		//glTranslatef(camPos.x, camPos.y, camPos.z);

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
			const std::vector<lt::ContactManifold> &contactManifolds = world.getContactManifolds();

			
			for(unsigned int i = 0; i < contactManifolds.size(); i++)
			{
				int numContacts = contactManifolds[i].getNumContacts();
				for(int j = 0; j < numContacts; j++)
				{
					glPointSize(8);
					glLineWidth(4);

					lt::ContactPoint curContact = contactManifolds[i].getContactPoint(j);

					lt::Vec3& pos = curContact.position;
					lt::Vec3& norm = curContact.normal;
					lt::Scalar& pen = curContact.penetration;

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
						lt::Mat3 basis = lt::constructOrthonormalBasis(norm);

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
			}

		glEnable(GL_LIGHTING);
		//glPolygonMode(GL_FRONT, GL_LINE);		
		
		const std::vector<lt::RigidBody*> &rigidBodies = world.getRigidBodyList();

		for (unsigned int i = 0; i < rigidBodies.size(); i++)
		{
			const lt::RigidBody &curBody = *rigidBodies[i];

			const std::set<const lt::CollisionShape*>& colShapesA = curBody.getCollisionShapes();
			std::set<const lt::CollisionShape*>::iterator shapeIter;
			for (shapeIter = colShapesA.begin(); shapeIter != colShapesA.end(); ++shapeIter)
			{
				const lt::CollisionShape &curShape = **shapeIter;

				glPushMatrix();
					curBody.getTransform().getOpenGLMatrix(transform);
					glMultMatrixf(transform);
					curShape.getOffset().getOpenGLMatrix(transform);
					glMultMatrixf(transform);

					glColor3f( ((i+1)*1234567)%32 / 32.0f, 
							   ((i+2)*1234567)%32 / 32.0f, 
							   ((i+3)*1234567)%32 / 32.0f);

					switch (curShape.getShapeType())
					{
					case lt::SHAPE_BOX:
						drawBox( ((lt::ShapeBox&)curShape).getHalfExtents() );
						break;
					case lt::SHAPE_SPHERE:
						drawSphere( ((lt::ShapeSphere&)curShape).getRadius(), 10 );
						break;
					case lt::SHAPE_HALFSPACE:
						glBegin(GL_TRIANGLE_STRIP);
							glNormal3f(0.f, 1.f, 0.f);
							glVertex3f(-10000.f, 0,  10000.f);
							glVertex3f(-10000.f, 0, -10000.f);
							glVertex3f( 10000.f, 0,  10000.f);
							glVertex3f( 10000.f, 0, -10000.f);
						glEnd();
						break;
					}
				glPopMatrix();
			}
		}

		// Test Spring
		glDisable(GL_LIGHTING);
		glPushMatrix();
			lt::Vec3 p1;
			lt::Vec3 p2 = m_controlledBody.getPosition();

			glColor3f(1.0f, 0.5f, 0.0f);
			glBegin(GL_LINES);
				p1 = m_bodySmall.getPosition() + m_bodySmall.getPointInWorldSpace(m_boxSpringOffset);
				glVertex3f(p1.x, p1.y, p1.z);
				glVertex3f(p2.x, p2.y, p2.z);
			glEnd();

		glPopMatrix();

		glEnable(GL_LIGHTING);

		glFlush();
	SDL_GL_SwapBuffers();
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
	1, 5, 7, 1, 3, 5	// -Y 
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

