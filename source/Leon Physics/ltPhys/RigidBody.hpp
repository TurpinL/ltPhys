#ifndef LTPHYS_RIGIDBODY_H
#define LTPHYS_RIGIDBODY_H

#include <set>

#include "Scalar.hpp"
#include "Vec3.hpp"
#include "Quat.hpp"
#include "Mat3.hpp"
#include "Transform.hpp"
#include "CollisionShape.hpp"

namespace lt
{

////////////////////////////////////////////////////////////
/// @brief A basic non-deformable physics object.
///
/// @author Leon Turpin
/// @date December 2013
////////////////////////////////////////////////////////////
class RigidBody
{
public:
	////////////////////////////////////////////////////////////
	/// @brief Default constructor
    ////////////////////////////////////////////////////////////
	RigidBody(); 

    ////////////////////////////////////////////////////////////
	/// @brief Simulates this physics objects for "timeStep" seconds
	/// 
	/// @param timeStep Time in seconds to simulate.
	/// 
    ////////////////////////////////////////////////////////////
	void integrate(const Scalar& timeStep);

    ////////////////////////////////////////////////////////////
	/// @brief Apply a force to the centre of mass of the body
	/// 
	/// @param force Force to apply.
	///
    ////////////////////////////////////////////////////////////
	void applyCentralForce(const Vec3& force);

    ////////////////////////////////////////////////////////////
	/// @brief Apply a world aligned offset force to the body
	///
	/// @param force Force to apply
	/// @param offset World alligned offset of force
	/// 
    ////////////////////////////////////////////////////////////
	void applyForce(const Vec3& force, const Vec3& offset);

	////////////////////////////////////////////////////////////
	/// @brief Apply a body alligned offset force to the body
	///
	/// @param force Force to apply
	/// @param offset Body alligned offset of force
	/// 
    ////////////////////////////////////////////////////////////
	void applyRelForce(const Vec3& force, const Vec3& offset);

    ////////////////////////////////////////////////////////////
	/// @brief Applies torque to the body
	///
	/// @param torque Torque to apply
	///
    ////////////////////////////////////////////////////////////
	void applyTorque(const Vec3& torque);

    ////////////////////////////////////////////////////////////
	/// @brief Clear all forces accumulated by the body.
    ////////////////////////////////////////////////////////////
	void clearForces();

    ////////////////////////////////////////////////////////////
	/// @brief Converts body space position into world space.
	///
	/// @param point Body space point to convert into worldspace
	///
	/// @return Body space position in world space.
	///
    ////////////////////////////////////////////////////////////	
	const Vec3 getPointInWorldSpace(const Vec3& point) const;

    ////////////////////////////////////////////////////////////
	/// @brief Sets the position of the body
    ////////////////////////////////////////////////////////////	
	void setPosition(const Vec3& position);

    ////////////////////////////////////////////////////////////
	/// @brief Sets the velocity of the body
    ////////////////////////////////////////////////////////////
	void setVelocity(const Vec3& velocity);

    ////////////////////////////////////////////////////////////
	/// @brief Sets the acceleration of the body
    ////////////////////////////////////////////////////////////
	void setAcceleration(const Vec3& acceleration);

    ////////////////////////////////////////////////////////////
	/// @brief Sets the angle of the body
    ////////////////////////////////////////////////////////////
	void setAngle(const Quat& angle);

    ////////////////////////////////////////////////////////////
	/// @brief Sets the angular velocity of the body
    ////////////////////////////////////////////////////////////
	void setAngularVelocity(const Vec3& angVel);

    ////////////////////////////////////////////////////////////
	/// @brief Sets the mass of the body from an inverse mass
    ////////////////////////////////////////////////////////////
	void setInvMass(const Scalar& invMass);

    ////////////////////////////////////////////////////////////
	/// @brief Sets the mass of the body
    ////////////////////////////////////////////////////////////
	void setMass(const Scalar& mass);

    ////////////////////////////////////////////////////////////
	/// @brief Sets the damping of the body
    ////////////////////////////////////////////////////////////
	void setDamping(const Scalar& damping);

    ////////////////////////////////////////////////////////////
	/// @brief Sets the angular damping of the body
    ////////////////////////////////////////////////////////////
	void setAngularDamping(const Scalar& angularDamping);

    ////////////////////////////////////////////////////////////
	/// @brief Sets the coefficient of restitution of the body
    ////////////////////////////////////////////////////////////
	void setRestitution(const Scalar& restitution);

    ////////////////////////////////////////////////////////////
	/// @brief Sets the inertia tensor of the body from inertia products 
    ////////////////////////////////////////////////////////////
	void setInertiaTensor(const Vec3& inertiaProducts);

	////////////////////////////////////////////////////////////
	/// @brief Sets the inertia tensor of the body
    ////////////////////////////////////////////////////////////
	void setInertiaTensor(const Mat3& inertiaTensor);

	////////////////////////////////////////////////////////////
	/// @brief Sets the inertia tensor of the body from an inverse inertia products
    ////////////////////////////////////////////////////////////
	void setInvInertiaTensor(const Vec3& inertiaProducts);

	////////////////////////////////////////////////////////////
	/// @brief Sets the inertia tensor of the body from an inverse inertia tensor
    ////////////////////////////////////////////////////////////
	void setInvInertiaTensor(const Mat3& inverseInertiaTensor);

	void addCollisionShape(const CollisionShape* colShape);
	void removeCollisionShape(const CollisionShape* colShape);
	int numCollisionShapes() const;

	const Vec3& getPosition() const;
	const Vec3& getVelocity() const;
	const Vec3& getAcceleration() const;
	const Quat& getAngle() const;
	const Vec3& getAngularVelocity() const;
	const Scalar& getInvMass() const;
	const Scalar getMass() const;
	const Scalar& getDamping() const;
	const Scalar& getAngularDamping() const;
	const Scalar& getRestitution() const;
	const Mat3 getInertiaTensor() const;
	const Mat3& getInvInertiaTensor() const;
	const Mat3& getInvInertiaTensorWorld() const;
	const Transform& getTransform() const;
	const std::set<const CollisionShape*>& getCollisionShapes() const;

private:
	Vec3 m_pos; // Position
	Vec3 m_vel; // Velocity
	Vec3 m_accel; // Acceleration

	Quat m_ang; // angular orientation
	Vec3 m_angVel; // angular velocity
	Mat3 m_invInteriaTensor; // inverse interia tensor (Body aligned)
	
	// Derived Data
	Transform m_transform; // This rigid body's transformation matrix.
	Mat3 m_invInertiaTensorWorld; // inverse interia tensor (World aligned)

	Vec3 m_forceAccum; // Force accumulator
	Vec3 m_torqueAccum; // Torque accumulator

	Scalar m_invMass; // Inverse Mass
	Scalar m_damping; // Damping Coefficient.
	Scalar m_angDamping; // Angular Damping Coefficient
	Scalar m_restitution; // Coefficient of restitution

	std::set<const CollisionShape*> m_collisionShapes;

	void _clearAccums();
	void _calcDerivedData();
};

/**
 * @brief Do an inertia tensor rotation by a transformation matrix.
 *
 * Taken from "Game Physics Engine Development" by Ian Millington.
 * 
 * @param iitWorld The resulting world aligned inverse inertia tensor.
 * @param iitBody The original rigidbody aligned inverse inertia tensor.
 * @param rotMat The transformation to rotate the inertia tensor by.
 */
static inline void _transformInertiaTensor(Mat3 &iitWorld, const Mat3 &iitBody, const Transform &rotMat);

} // namespace lt

#endif // LTPHYS_RIGIDBODY_H