#include "PhysicsDemo.hpp"
#include "iostream"

void PhysicsDemo::handleEvent(SDL_Event *E)
{
	switch (E->type)
	{
	case SDL_VIDEORESIZE:
		reshape(E->resize.w, E->resize.h, 90);
		break;

	case SDL_QUIT:
		exit(0);
		break;

	case SDL_KEYDOWN:
		{
			/*float hmLength = 200;
			float hmWidth = 200;
			float *heightMap = new float[(int)(hmLength * hmWidth)];
			genPerlinLand(hmWidth, hmLength, heightMap);
			testTerrain->setHeightMap(heightMap, hmLength, hmWidth);
			delete heightMap;*/
		}
		break;

	case SDL_MOUSEBUTTONDOWN:
		if(E->button.button == SDL_BUTTON_WHEELUP)
			m_camZoom += 0.4f;
		else if(E->button.button == SDL_BUTTON_WHEELDOWN)
			m_camZoom -= 0.4f;
		else if(E->button.button == SDL_BUTTON_RIGHT)
			m_controlledBody.applyCentralForce(lt::Vec3(0.0f, 20000.0f, 0.0f));
	}
}

void PhysicsDemo::reshape(int w, int h, double fov)
{
	const double DEG_TO_RAD = 0.0174532925;
	double ratio; // The ratio between width and height

	// Render to the whole window
	glViewport(0, 0, w, h);

	// Swap to the projection matrix
	glMatrixMode(GL_PROJECTION);
		glLoadIdentity();

		double nearClipping = 0.1;
		double extent = nearClipping * tan(fov*DEG_TO_RAD*0.5);

		if(w <= h)
		{
			ratio = (double)h / (double)w;
			glFrustum(-extent, extent, -extent*ratio, extent*ratio, nearClipping, 1000.0);
		}
		else
		{
			ratio = (double)w / (double)h;
			glFrustum(-extent*ratio, extent*ratio, -extent, extent, nearClipping, 1000.0);
		}
	glMatrixMode(GL_MODELVIEW);
}