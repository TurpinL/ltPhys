#ifndef PHYSICSDEMO_HPP
#define PHYSICSDEMO_HPP

#define BOX_COUNT 30

#include <SDL.h> // SDL v1.2.15
#include <GL/glew.h> // GLEW v1.9.0
#include "ltPhys.hpp"

class PhysicsDemo
{
public:
	PhysicsDemo();
	void gameLogicLoop();
private:
	lt::World world;

	lt::FGenGravity m_gravity;

	lt::ShapeBox m_boxShapeLarge;
	lt::ShapeBox m_boxShapeSmall;
	lt::ShapeBox m_boxShapePlayer;
	lt::ShapeHalfspace m_shapeGround;

	lt::FGenSpring2 m_spring;

	lt::Vec3 m_boxSpringOffset;

	lt::RigidBody m_controlledBody;
	lt::RigidBody m_bodyGround;
	lt::RigidBody m_bodyLarge;
	lt::RigidBody m_bodySmall;

	void init();
		void initSDL();
		void initGL();
		void initPhysics();
		bool m_isInitialized;

	void handleEvent(SDL_Event *E);
		void reshape(int w, int h, double fov);

	void idle();
		void calcFrameTime();	
			float m_frameTime; 
			unsigned int m_lastClock;
			int m_frameCount;
		int m_mouseX;
		int m_lastMouseX;
		int m_mouseY;
		int m_lastMouseY;
	void display();
		void testRender(const TerrainData &terrain);
		lt::Vec3 m_camAng;
		float m_camZoom;
};

#endif // PHYSICSDEMO_HPP