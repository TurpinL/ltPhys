#ifndef LTPHYS_FGENGRAVITY_H
#define LTPHYS_FGENGRAVITY_H

#include "ForceGenerator.hpp"
#include "RigidBody.hpp"
#include "Scalar.hpp"
#include "Vec3.hpp"

namespace lt
{

////////////////////////////////////////////////////////////
///	@brief Applies a gravitational force to a rigid body.
///
/// @author Leon Turpin
/// @date December 2013
////////////////////////////////////////////////////////////
class FGenGravity : public ForceGenerator
{
public:
	////////////////////////////////////////////////////////////
	/// @brief Creates a gravity force generator with junk data 
	/// as a gravity force.
	////////////////////////////////////////////////////////////
	FGenGravity();
	
	////////////////////////////////////////////////////////////
	/// @brief Creates a force generator from a gravity force
	///
	/// @param gravity Gravitational force applied by the generator.
	///
	////////////////////////////////////////////////////////////
	FGenGravity(const Scalar &gravity);

	void updateForce(RigidBody &rigidBody, const Scalar &timeStep);

	////////////////////////////////////////////////////////////
	/// @brief Set the gravitational force of the generator.
	///
	/// @param gravity Gravitational force applied by the generator.
	///
	////////////////////////////////////////////////////////////
	void setGravity(const Scalar &gravity);

	////////////////////////////////////////////////////////////
	/// @brief Get the gravitational force of the generator.
	/// 
	/// @return The gravitational force of this generator.
	///
	////////////////////////////////////////////////////////////	
	const Scalar getGravity() const;

private:
	Scalar m_gravity;
};

} // namespace lt

#endif // LTPHYS_FGENGRAVITY_H