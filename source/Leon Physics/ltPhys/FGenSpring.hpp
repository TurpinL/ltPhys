#ifndef LTPHYS_FGENSRPING_H
#define LTPHYS_FGENSRPING_H

#include "ForceGenerator.hpp"
#include "Scalar.hpp"
#include "RigidBody.hpp"

namespace lt
{

/** FGenSpring.hpp
 *	@brief 
 *
 *  @author Leon Turpin
 *  @date December 2013
 */
class FGenSpring : public ForceGenerator
{
public:
	FGenSpring();

	FGenSpring(RigidBody *other, const Scalar &springConstant, const Scalar &restLength);

	void updateForce(RigidBody &rigidBody, const Scalar &timeStep);

private:
	RigidBody *m_other; // The body at the other end of the spring

	Scalar m_springConstant;

	Scalar m_restLength;
};

} // namespace lt

#endif // LTPHYS_FGENSRPING_H