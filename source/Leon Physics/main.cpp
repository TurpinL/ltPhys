#include <iostream>
#include "ltPhys.hpp"
#include "PhysicsDemo.hpp"

int main(int argc, char **argv)
{
	PhysicsDemo physDemo;

	physDemo.gameLogicLoop();
	
	return 0;
}
