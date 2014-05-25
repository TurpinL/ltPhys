 #include "PhysicsDemo.hpp"
#include "HeightmapFunctions.hpp"

#include <random>

const lt::Vec3 inertiaTensorCuboid(const lt::Vec3& dimensions, const lt::Scalar& mass);

void PhysicsDemo::init()
{
	initSDL();
	initGL();
	initPhysics();

	m_camAng = lt::Vec3(20.0f, 0.0f, 0.0f);
	m_camZoom = -5.0;

	m_staticSphereShape.setRadius(0.5);

	m_controlledBody.setPosition(lt::Vec3(0.0f, 2.0f, 0.f));
	m_controlledBody.setAngle(lt::Quat());
	m_controlledBody.setInvMass(0);
	m_controlledBody.setRestitution(0.4f);

	m_frameCount = 0;
	m_lastClock = SDL_GetTicks();
}

void PhysicsDemo::initSDL()
{
	SDL_Init( SDL_INIT_VIDEO ); // Init SDL
	SDL_SetVideoMode( 800, 500, 32, SDL_RESIZABLE | SDL_OPENGL ); // Create context window
	SDL_WM_SetCaption( "Leon PhysicDemo V1", nullptr ); // Set Window Title
}

void PhysicsDemo::initGL()
{
	glewInit();

	glClearColor(1.0f, 0.9f, 0.8f, 1.0f);

	// GL Enables
	glEnable(GL_DEPTH_TEST);

	glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		glEnable(GL_COLOR_MATERIAL);
}

void PhysicsDemo::initPhysics()
{
	float density;

	m_gravity.setGravity(-9.8f);

	// Test Sphere
	m_myRigidBody.setPosition(lt::Vec3(0.0f, 3.0f, 0.0f));
	m_myRigidBody.setDamping(0.7f);
	m_myRigidBody.setAngularDamping(0.7f);
	m_myRigidBody.setMass(1.0f);
	m_myRigidBody.setRestitution(0.4f);
	//m_myRigidBody.setInertiaTensor( lt::Vec3(0.36f, 0.36f, 0.36f) );
	m_myRigidBody.setInvInertiaTensor( lt::Vec3(0.1f, 0.1f, 0.1f) );
	m_spring = lt::FGenSpring2(lt::Vec3(-0.5f, 0.f, 0.f), &m_controlledBody, lt::Vec3(0.f, 0.f, 0.f), 4.8f, 4.5f, true);

	/*m_spring = lt::FGenSpring(&m_controlledBody, 10.0f, 1.2f);
	m_fGenRegistry.add(&m_myRigidBody, &m_spring);*/

	// Phys Box
	density = 100.f;

	m_boxShape.setHalfExtents(lt::Vec3(0.5f, 0.5f, 0.5f));
	m_box.setPosition(lt::Vec3(2.0f, 2.0f, 0.0f));
	//m_box.setAngle(lt::Quat(lt::Vec3(0, 0, 1), 50.f));
	m_box.setDamping(0.7f);
	m_box.setAngularDamping(0.7f);
	m_box.setRestitution(0.7f);
	const lt::Vec3& boxDimensions = m_boxShape.getHalfExtents();
	m_box.setMass(0.7f);

	//m_box.setInertiaTensor( inertiaTensorCuboid(m_boxShape.getHalfExtents(), m_box.getMass()) );
	m_box.setInertiaTensor( lt::Vec3(0.1f, 0.1f, 0.1f) );

	m_boxSpringOffset = lt::Vec3(0.5f, 0.f, 0.f);
	m_boxSpring = lt::FGenSpring2(m_boxSpringOffset, &m_controlledBody, lt::Vec3(0.f, 0.f, 0.f), 4.8f, 4.5f, true);

	world.addRigidBody(&m_myRigidBody, &m_staticSphereShape);
	world.addRigidBody(&m_box, &m_boxShape);
	world.addRigidBody(&m_controlledBody, &m_staticSphereShape);
	world.addForceGenerator(&m_myRigidBody, &m_gravity);
	world.addForceGenerator(&m_myRigidBody, &m_spring);
	world.addForceGenerator(&m_box, &m_boxSpring);
	world.addForceGenerator(&m_box, &m_gravity);

	// Initialize static bodies
	m_groundBody.setPosition(lt::Vec3(0, -10, 0));
	m_groundBody.setAngle(lt::Quat(lt::Vec3(1.0f, 0.0f, 0.0f), 0.0f));
	m_groundBody.setRestitution(0.0f);
	m_groundBody.setInvMass(0.0f);
	m_groundBody.setInvInertiaTensor(lt::Vec3(0, 0, 0));

	m_staticBallBody.setPosition(lt::Vec3(0, 0, 0));
	m_staticBallBody.setRestitution(1.0f);
	m_staticBallBody.setInvMass(0.0f);
	m_staticBallBody.setInvInertiaTensor(lt::Vec3(0, 0, 0));

	world.addRigidBody(&m_groundBody, &m_groundShape);
	world.addRigidBody(&m_staticBallBody, &m_staticSphereShape);

	// Terrain
	// Create heightmap
	int hmLength = 128;
	int hmWidth = 128;
	float *heightMap = new float[hmWidth * hmLength];

	/*for(int i = 0; i < hmWidth * hmLength; i++)
	{
		if((i+3) % 4 <= 1)
			heightMap[i] = 0;
		else
			heightMap[i] = 1;
	}*/
	heightMap::genHeightMapFaultFormation(heightMap, hmLength, hmWidth, 32, 0.4f);
	//heightMap::genFromRAW(heightMap, hmLength, hmWidth, "hm64.raw");
	m_terrainData = new TerrainData(heightMap, hmLength, hmWidth, lt::Vec3(100.0f, 20.0f, 100.0f));
	delete[] heightMap;

	m_terrainShape.setTerrainData(m_terrainData);

	m_terrainBody.setPosition(m_terrainData->getTerrainSize() * -0.5 - lt::Vec3(0, 6, 0));
	m_terrainBody.setRestitution(0.0f);
	m_terrainBody.setInvMass(0.0f);
	m_terrainBody.setInvInertiaTensor(lt::Vec3(0, 0, 0));

	world.addRigidBody(&m_terrainBody, &m_terrainShape);

	/*world.addCollisionShape(nullptr, &m_staticSphereShape, lt::Transform(lt::Vec3(0.f, 0.f, 0.f), lt::Quat()));
	world.addCollisionShape(nullptr, &m_groundShape, lt::Transform(lt::Vec3(0.f, 0.f, 0.f), lt::Quat()));*/
}

const lt::Vec3 inertiaTensorCuboid(const lt::Vec3& dimensions, const lt::Scalar& mass)
{
	const lt::Scalar mult = mass / 12.f;
	const lt::Scalar sqx = dimensions.x*dimensions.x;
	const lt::Scalar sqy = dimensions.y*dimensions.y;
	const lt::Scalar sqz = dimensions.z*dimensions.z;

	return lt::Vec3( mult * (sqy + sqz),
					 mult * (sqx + sqz),
					 mult * (sqx + sqy) );
}
