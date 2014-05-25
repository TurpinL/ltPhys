#ifndef PHYSICSDEMO_HPP
#define PHYSICSDEMO_HPP

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

	// Phys Sphere
	lt::FGenSpring2 m_spring;
	lt::RigidBody m_myRigidBody;
	lt::RigidBody m_controlledBody;

	// Physics Cuboid
	lt::RigidBody m_box;
	lt::FGenSpring2 m_boxSpring;
	lt::Vec3 m_boxSpringOffset;

	// Shapes
	lt::ShapeSphere m_staticSphereShape;
	lt::ShapeBox m_boxShape;
	lt::ShapeHalfspace m_groundShape;
	lt::ShapeTerrain m_terrainShape;
	TerrainData *m_terrainData;

	// Static Bodies
	lt::RigidBody m_groundBody;
	lt::RigidBody m_staticBallBody;
	lt::RigidBody m_terrainBody;

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