 #include "PhysicsDemo.hpp"

#include <random>

const lt::Vec3 inertiaTensorCuboid(const lt::Vec3& dimensions, const lt::Scalar& mass);

void PhysicsDemo::init()
{
	initSDL();
	initGL();
	initPhysics();

	m_camAng = lt::Vec3(20.0f, 0.0f, 0.0f);
	m_camZoom = -5.0;

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
	m_gravity.setGravity(-9.8f);

	// Controlled Point
	m_boxShapePlayer.setHalfExtents(lt::Vec3(0.35f, 1.1f, 0.35f));
	m_controlledBody.setPosition(lt::Vec3(1.0f, 3.0f, 2.1f));
	m_controlledBody.setAngle(lt::Quat());
	m_controlledBody.setDamping(0.7f);
	m_controlledBody.setAngularDamping(0.7f);
	m_controlledBody.setMass(80.0f);
	m_controlledBody.setInvInertiaTensor(lt::Vec3());
	m_controlledBody.setRestitution(-0.4f);
	m_controlledBody.addCollisionShape(&m_boxShapePlayer);

	// Table
	m_tableTop = new lt::ShapeBox(lt::Vec3(0.7f, 0.1f, 0.85f));
	m_tableTop->setOffset(lt::Transform(lt::Vec3(0.0f, 0.75f, 0.0f), lt::Quat()));
	
	m_tableLeg1 = new lt::ShapeBox(lt::Vec3(0.7f, 0.75f, 0.1f));
	m_tableLeg1->setOffset(lt::Transform(lt::Vec3(0.0f, 1.5f, -0.75f), lt::Quat()));
	
	m_tableLeg2 = new lt::ShapeBox(lt::Vec3(0.1f, 0.75f, 0.1f));
	m_tableLeg2->setOffset(lt::Transform(lt::Vec3(0.6f, 0.0f, -0.75f), lt::Quat()));
		
	m_tableLeg3 = new lt::ShapeBox(lt::Vec3(0.1f, 0.75f, 0.1f));
	m_tableLeg3->setOffset(lt::Transform(lt::Vec3(-0.6f, 0.0f, 0.75f), lt::Quat()));

	m_tableLeg4 = new lt::ShapeBox(lt::Vec3(0.1f, 0.75f, 0.1f));
	m_tableLeg4->setOffset(lt::Transform(lt::Vec3(-0.6f, 0.0f, -0.75f), lt::Quat()));

	m_tableLeg5 = new lt::ShapeBox(lt::Vec3(0.1f, 0.75f, 0.1f));
	m_tableLeg5->setOffset(lt::Transform(lt::Vec3(0.6f, 0.0f, 0.75f), lt::Quat()));

	m_table.setPosition(lt::Vec3(0.0f, 4.0f, 0.0f));
	m_table.setDamping(0.7f);
	m_table.setAngularDamping(0.7f);
	m_table.setRestitution(0.7f);
	m_table.setMass(30.0f);
	m_table.setInertiaTensor( lt::Vec3(50.0f, 56.0f, 50.0f) );
	m_table.addCollisionShape(m_tableTop);
	m_table.addCollisionShape(m_tableLeg1);
	m_table.addCollisionShape(m_tableLeg2);
	m_table.addCollisionShape(m_tableLeg3);
	m_table.addCollisionShape(m_tableLeg4);
	m_table.addCollisionShape(m_tableLeg5);

	// Big Box
	m_boxShapeLarge.setHalfExtents(lt::Vec3(2, 2, 2));
	m_bodyLarge.setPosition(lt::Vec3(1.0f, 3.0f, 0.0f));
	m_bodyLarge.setDamping(0.7f);
	m_bodyLarge.setAngularDamping(0.7f);
	m_bodyLarge.setRestitution(0.7f);
	m_bodyLarge.setMass(30.0f);
	m_bodyLarge.setInertiaTensor( lt::Vec3(50.0f, 56.0f, 50.0f) );
	m_bodyLarge.addCollisionShape(&m_boxShapeLarge);

	// Small Box
	m_boxShapeSmall.setHalfExtents(lt::Vec3(0.35f, 0.12f, 0.35f));
	m_bodySmall.setPosition(lt::Vec3(0.0f, 2.0f, 2.0f));
	m_bodySmall.setDamping(0.7f);
	m_bodySmall.setAngularDamping(0.7f);
	m_bodySmall.setRestitution(0.1f);
	m_bodySmall.setMass(0.5f);
	m_bodySmall.setInertiaTensor( lt::Vec3(0.06f, 0.1f, 0.06f) );
	m_bodySmall.addCollisionShape(&m_boxShapeSmall);

	// Ground Body
	m_bodyGround.setPosition(lt::Vec3());
	m_bodyGround.setRestitution(0.7f);
	m_bodyGround.setInvMass(0);
	m_bodyGround.setInvInertiaTensor( lt::Vec3() );
	m_bodyGround.addCollisionShape(&m_shapeGround);

	m_boxSpringOffset = lt::Vec3(0.0f, 0.12f, 0.0f);
	m_spring = lt::FGenSpring2(m_boxSpringOffset, &m_controlledBody, lt::Vec3(0.f, 0.f, 0.f), 6.0f, 1.0f, true);

	//// Controlled Body
	world.addRigidBody(&m_controlledBody);
	world.addForceGenerator(&m_controlledBody, &m_gravity);

	//// Table Body
	world.addRigidBody(&m_table);
	world.addForceGenerator(&m_table, &m_gravity);

	// Small Box
	world.addRigidBody(&m_bodySmall);
	world.addForceGenerator(&m_bodySmall, &m_gravity);
	world.addForceGenerator(&m_bodySmall, &m_spring);

	// Ground
	world.addRigidBody(&m_bodyGround);

	// Large Box
	world.addRigidBody(&m_bodyLarge);
	world.addForceGenerator(&m_bodyLarge, &m_gravity);
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
