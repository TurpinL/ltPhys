#include "PhysicsDemo.hpp"

#include <iostream>

void PhysicsDemo::idle()
{
	calcFrameTime();
	// Cap the update rate to 30FPS.
	m_frameTime = (m_frameTime > 0.032) ? 0.032f : m_frameTime;

	Uint8 *keyState = SDL_GetKeyState(NULL);

	SDL_GetMouseState(&m_mouseX, &m_mouseY);

	// Camera rotation
	if(SDL_GetMouseState(nullptr, nullptr) & SDL_BUTTON(1))
	{
		m_camAng.y += (m_mouseX - m_lastMouseX) * 0.2f;
		m_camAng.x += (m_mouseY - m_lastMouseY) * 0.2f;
	}

	// Cap pitch between 90 and -90
	m_camAng.x = (m_camAng.x > 90) ? 90 : m_camAng.x;
	m_camAng.x = (m_camAng.x < -90) ? -90 : m_camAng.x;

	m_controlledBody.setAngle(lt::Quat(lt::Vec3(1.0f, 0.0f, 0.0f), 90 * m_frameTime * (keyState[SDLK_k] - keyState[SDLK_i])) * m_controlledBody.getAngle());
	m_controlledBody.setAngle(lt::Quat(lt::Vec3(0.0f, 1.0f, 0.0f), 90 * m_frameTime * (keyState[SDLK_l] - keyState[SDLK_j])) * m_controlledBody.getAngle());
	m_controlledBody.setAngle(lt::Quat(lt::Vec3(0.0f, 0.0f, 1.0f), 90 * m_frameTime * (keyState[SDLK_u] - keyState[SDLK_o])) * m_controlledBody.getAngle());

	float xDelta = (keyState[SDLK_d] - keyState[SDLK_a]) * 6.0f;
	float yDelta = (keyState[SDLK_q] - keyState[SDLK_e]) * 6.0f;
	float zDelta = (keyState[SDLK_s] - keyState[SDLK_w]) * 6.0f;
	//m_controlledBody.setPosition(m_controlledBody.getPosition() + lt::Vec3(xDelta, yDelta, zDelta));
	m_controlledBody.setVelocity(lt::Vec3(xDelta, m_controlledBody.getVelocity().y, zDelta));
	//m_controlledBody.setVelocity(lt::Vec3(xDelta, yDelta, zDelta));

	// Move stuff
	lt::Scalar deltaZ = (keyState[SDLK_t] - keyState[SDLK_y]) * 100 * m_frameTime;

	// Physics
	if(!keyState[SDLK_SPACE])
	{
		world.stepSimulation(m_frameTime);
	}

	m_lastMouseX = m_mouseX;
	m_lastMouseY = m_mouseY;

	display(); 
}

void PhysicsDemo::calcFrameTime()
{
	unsigned int curTime = SDL_GetTicks();
	m_frameTime = ((float)(curTime-m_lastClock))/(float)1000.0f;

	m_frameCount++;

	// FPS Printer
	if(m_frameCount % 20 == 0)
		std::cout << (1.0f / m_frameTime) << std::endl;

	m_lastClock = curTime;
}