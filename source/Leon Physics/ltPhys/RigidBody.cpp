#include "RigidBody.hpp"

#include <cmath>
#include <iostream>

namespace lt
{

RigidBody::RigidBody()
{
	m_pos = Vec3(0.0f, 0.0f, 0.0f);
	m_vel = Vec3(0.0f, 0.0f, 0.0f);
	m_accel = Vec3(0.0f, 0.0f, 0.0f);

	m_forceAccum = Vec3(0.0f, 0.0f, 0.0f);
	m_torqueAccum = Vec3(0.0f, 0.0f, 0.0f);

	m_ang = Quat(0.0f, 0.0f, 0.0f, 1.0f);
	m_angVel = Vec3(0.0f, 0.0f, 0.0f);

	m_invMass = 1;
	m_damping = 1;
	m_angDamping = 1;
	m_restitution = 1;

	m_invInteriaTensor.setIdentity();
}

void RigidBody::integrate(const Scalar& timeStep)
{
	const Scalar DEG_TO_RAD = 57.2957795f;

	// Acceleration due to force
	m_accel = m_forceAccum * m_invMass;

	// Angular acceleration due to torque
	Vec3 angAccel = m_invInteriaTensor * m_torqueAccum;
	
	// Update Velocities
	m_vel += m_accel * timeStep;
	m_angVel += angAccel * timeStep;

	// Apply damping
	m_vel *= scalar_pow(m_damping, timeStep);
	m_angVel *= scalar_pow(m_angDamping, timeStep);

	// Update Position
	m_pos += m_vel * timeStep;

	// Shitty angle update
	m_ang = Quat(Vec3(1.0f, 0.0f, 0.0f), m_angVel.x * timeStep * DEG_TO_RAD) * m_ang;
	m_ang = Quat(Vec3(0.0f, 1.0f, 0.0f), m_angVel.y * timeStep * DEG_TO_RAD) * m_ang;
	m_ang = Quat(Vec3(0.0f, 0.0f, 1.0f), m_angVel.z * timeStep * DEG_TO_RAD) * m_ang;

	// Clear Accumulators
	_clearAccums();
	_calcDerivedData();
}

void RigidBody::applyCentralForce(const Vec3& force)
{
	m_forceAccum += force;
}

void RigidBody::applyForce(const Vec3& force, const Vec3& offset)
{
	m_torqueAccum += offset.cross(force);
	applyCentralForce(force);
}

void RigidBody::applyRelForce(const Vec3& force, const Vec3& offset)
{
	applyForce(force, getPointInWorldSpace(offset));
}

void RigidBody::applyTorque(const Vec3& torque)
{
	m_torqueAccum += torque;
}

void RigidBody::clearForces()
{
	m_torqueAccum = Vec3(0, 0, 0);
	m_forceAccum = Vec3(0, 0, 0);
}

const Vec3 RigidBody::getPointInWorldSpace(const Vec3& point) const
{
	Vec3 direction = point;
	direction.w = 0;
	return m_transform * direction;
}

//--------------------------
//	SETS			
//--------------------------
void RigidBody::setPosition(const Vec3& position) 
{ 
	m_pos = position;
	m_transform = Transform(m_pos, m_ang); // Update Transform
}

void RigidBody::setAngle(const Quat& angle) 
{ 
	m_ang = angle; 
	m_transform = Transform(m_pos, m_ang); // Update Transform
	_transformInertiaTensor(m_invInertiaTensorWorld, m_invInteriaTensor, m_transform); // Update World Inertia Tensor 
}

void RigidBody::setVelocity(const Vec3& velocity) { m_vel = velocity; }
void RigidBody::setAcceleration(const Vec3& acceleration) { m_accel = acceleration; }
void RigidBody::setAngularVelocity(const Vec3& angVel) { m_angVel = angVel; }
void RigidBody::setInvMass(const Scalar& invMass) { m_invMass = invMass; }
void RigidBody::setMass(const Scalar& mass) { m_invMass = 1 / mass; }
void RigidBody::setDamping(const Scalar& damping) { m_damping = damping; }
void RigidBody::setAngularDamping(const Scalar& angularDamping) { m_angDamping = angularDamping; }
void RigidBody::setRestitution(const Scalar& restitution) { m_restitution = restitution; }

void RigidBody::setInertiaTensor(const Mat3& inertiaTensor) 
{ 
	m_invInteriaTensor = inertiaTensor; 
	m_invInteriaTensor.invert();
	_transformInertiaTensor(m_invInertiaTensorWorld, m_invInteriaTensor, m_transform); // Update World Inertia Tensor 
}

void RigidBody::setInertiaTensor(const Vec3& inertiaProducts)
{
	m_invInteriaTensor = Mat3(inertiaProducts);
	m_invInteriaTensor.invert();
	_transformInertiaTensor(m_invInertiaTensorWorld, m_invInteriaTensor, m_transform); // Update World Inertia Tensor 
}

void RigidBody::setInvInertiaTensor(const Vec3& inertiaProducts)
{
	m_invInteriaTensor = Mat3(inertiaProducts);
	_transformInertiaTensor(m_invInertiaTensorWorld, m_invInteriaTensor, m_transform); // Update World Inertia Tensor 
}

void RigidBody::setInvInertiaTensor(const Mat3& inverseInertiaTensor)
{
	m_invInteriaTensor = inverseInertiaTensor;
	_transformInertiaTensor(m_invInertiaTensorWorld, m_invInteriaTensor, m_transform); // Update World Inertia Tensor 
}

//--------------------------
//	GETS			
//--------------------------
const Vec3& RigidBody::getPosition() const { return m_pos; }
const Vec3& RigidBody::getVelocity() const { return m_vel; }
const Vec3& RigidBody::getAcceleration() const { return m_accel; }
const Quat& RigidBody::getAngle() const { return m_ang; }
const Vec3& RigidBody::getAngularVelocity() const { return m_angVel; }
const Scalar RigidBody::getMass() const { return 1.0f / m_invMass; }
const Scalar& RigidBody::getInvMass() const { return m_invMass; }
const Scalar& RigidBody::getDamping() const { return m_damping; }
const Scalar& RigidBody::getAngularDamping() const { return m_angDamping; }
const Scalar& RigidBody::getRestitution() const { return m_restitution; }
const Mat3 RigidBody::getInertiaTensor() const { return m_invInteriaTensor.inverse(); }
const Mat3& RigidBody::getInvInertiaTensor() const { return m_invInteriaTensor; }
const Mat3& RigidBody::getInvInertiaTensorWorld() const { return m_invInertiaTensorWorld; }
const Transform& RigidBody::getTransform() const { return m_transform; }


//--------------------------
//	PRIVATES			
//--------------------------

void RigidBody::_clearAccums()
{
	// Clear force accumulator
	m_forceAccum = Vec3(0.0f, 0.0f, 0.0f);
	m_torqueAccum = Vec3(0.0f, 0.0f, 0.0f);
}

void RigidBody::_calcDerivedData()
{
	// Calculate the transformation matrix
	m_transform = Transform(m_pos, m_ang);
	
	// Calculate the inverse inertia tensor in world space.
	_transformInertiaTensor(m_invInertiaTensorWorld, m_invInteriaTensor, m_transform);
}

//--------------------------
//	HELPERS		
//--------------------------

static inline void _transformInertiaTensor(Mat3 &iitWorld, const Mat3 &iitBody, const Transform &rotMat)
{
	// Note that the implementation of this function was created by an automated code generator and optimizer.
	Scalar t4  = rotMat.get(0) * iitBody.get(0) + rotMat.get(1) * iitBody.get(3) + rotMat.get(2) * iitBody.get(6);
	Scalar t9  = rotMat.get(0) * iitBody.get(1) + rotMat.get(1) * iitBody.get(4) + rotMat.get(2) * iitBody.get(7);
	Scalar t14 = rotMat.get(0) * iitBody.get(2) + rotMat.get(1) * iitBody.get(5) + rotMat.get(2) * iitBody.get(8);

	Scalar t28 = rotMat.get(4) * iitBody.get(0) + rotMat.get(5) * iitBody.get(3) + rotMat.get(6) * iitBody.get(6);
	Scalar t33 = rotMat.get(4) * iitBody.get(1) + rotMat.get(5) * iitBody.get(4) + rotMat.get(6) * iitBody.get(7);
	Scalar t38 = rotMat.get(4) * iitBody.get(2) + rotMat.get(5) * iitBody.get(5) + rotMat.get(6) * iitBody.get(8);

	Scalar t52 = rotMat.get(8) * iitBody.get(0) + rotMat.get(9) * iitBody.get(3) + rotMat.get(10) * iitBody.get(6);
	Scalar t57 = rotMat.get(8) * iitBody.get(1) + rotMat.get(9) * iitBody.get(4) + rotMat.get(10) * iitBody.get(7);
	Scalar t62 = rotMat.get(8) * iitBody.get(2) + rotMat.get(9) * iitBody.get(5) + rotMat.get(10) * iitBody.get(8);

	iitWorld[0] = t4 *rotMat.get(0) + t9 *rotMat.get(1) + t14*rotMat.get(2);
	iitWorld[1] = t4 *rotMat.get(4) + t9 *rotMat.get(5) + t14*rotMat.get(6);
	iitWorld[2] = t4 *rotMat.get(8) + t9 *rotMat.get(9) + t14*rotMat.get(10);

	iitWorld[3] = t28*rotMat.get(0) + t33*rotMat.get(1) + t38*rotMat.get(2);
	iitWorld[4] = t28*rotMat.get(4) + t33*rotMat.get(5) + t38*rotMat.get(6);
	iitWorld[5] = t28*rotMat.get(8) + t33*rotMat.get(9) + t38*rotMat.get(10);

	iitWorld[6] = t52*rotMat.get(0) + t57*rotMat.get(1) + t62*rotMat.get(2);
	iitWorld[7] = t52*rotMat.get(4) + t57*rotMat.get(5) + t62*rotMat.get(6);
	iitWorld[8] = t52*rotMat.get(8) + t57*rotMat.get(9) + t62*rotMat.get(10);
}

} // namespace lt