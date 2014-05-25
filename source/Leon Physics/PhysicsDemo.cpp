#include "PhysicsDemo.hpp"

PhysicsDemo::PhysicsDemo()
{
	m_isInitialized = false;
}

void PhysicsDemo::gameLogicLoop()
{
	if(!m_isInitialized)
	{
		init();
	}

	while (true)
	{
		SDL_Event E;

		while (SDL_PollEvent(&E))
		{
			handleEvent(&E);
		}

		idle();
	}
}
