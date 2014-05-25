#ifndef LTPHYS_FGENGRAVITY_H
#define LTPHYS_FGENGRAVITY_H

#include "ForceGenerator.hpp"
#include "Scalar.hpp"
#include "RigidBody.hpp"

namespace lt
{

/** FGenGravity.hpp
 *	@brief 
 *
 *  @author Leon Turpin
 *  @date December 2013
 */
class FGenGravity : public ForceGenerator
{
public:
	FGenGravity();

	FGenGravity(const Scalar &gravity);

	void updateForce(RigidBody &rigidBody, const Scalar &timeStep);

	void setGravity(const Scalar &gravity);

	const Scalar getGravity() const;

private:
	Scalar m_gravity;
};

} // namespace lt

#endif // LTPHYS_FGENGRAVITY_H