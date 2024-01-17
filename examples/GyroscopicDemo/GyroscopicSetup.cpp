#include "GyroscopicSetup.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4 0.785398163397448309616
#endif

struct GyroscopicSetup : public CommonRigidBodyBase
{
	GyroscopicSetup(struct GUIHelperInterface* helper);

	virtual ~GyroscopicSetup()
	{
	}
	virtual void initPhysics();

	virtual void physicsDebugDraw(int debugFlags);

	void resetCamera()
	{
		float dist = 20;
		float pitch = -16;
		float yaw = 180;
		float targetPos[3] = {-2.4, 0.4, -0.24};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

static int gyroflags[4] = {
	0,  //none, no gyroscopic term
	BT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT,
	BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD,
	BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY};

static const char* gyroNames[4] = {
	"No Gyroscopic",
	"Explicit",
	"Implicit (World)",
	"Implicit (Body)"};

GyroscopicSetup::GyroscopicSetup(struct GUIHelperInterface* helper)
	: CommonRigidBodyBase(helper)
{
}

void GyroscopicSetup::initPhysics()
{
	m_guiHelper->setUpAxis(1);
	createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0, 0, 0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	btVector3 positions[4] = {
		btVector3(-10, 8, 4),
		btVector3(-5, 8, 4),
		btVector3(0, 8, 4),
		btVector3(5, 8, 4),
	};

	// GYRATION
	m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
	for (int i = 0; i < 4; i++)
	{
		btScalar CUBE_HALF_EXTENTS = 1.0f;
		btBoxShape* box = new btBoxShape(btVector3(CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS));
		m_collisionShapes.push_back(box);
		float mass = 1.f;

		btScalar angle = atan2(1.0f, sqrt(2.0f));
		btVector3 axis = btVector3(-1.0f, 0.0f, 0.0f);
		btQuaternion rot1 = btQuaternion(0.0f, 0.0f, M_PI_4);
		btQuaternion rot2 = btQuaternion(axis, angle);
		float degrees = 30.0f;
		angle = degrees / 180.0f * M_PI;
		axis = btVector3(0.0f, 0.0f, 1.0f);
		btQuaternion rot3 = btQuaternion(axis, angle);
		

		btVector3 localInertia{1,1,1};
		btRigidBody* body = new btRigidBody(1, 0, box, localInertia);
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(positions[i]);
		tr.setRotation(rot3*rot2*rot1);
		body->setCenterOfMassTransform(tr);

		tr.setIdentity();
		tr.setRotation(rot3);
		btVector3 angular_velocity = tr * btVector3(0, 20, 0);
		body->setAngularVelocity(angular_velocity);

		//body->setFriction(btSqrt(1));
		m_dynamicsWorld->addRigidBody(body);
		body->setFlags(gyroflags[i]);
		m_dynamicsWorld->getSolverInfo().m_maxGyroscopicForce = 10.f;
		body->setDamping(0.0000f, 0.000f); 

		btVector3 pivot(-CUBE_HALF_EXTENTS, -CUBE_HALF_EXTENTS, -CUBE_HALF_EXTENTS);
		btTypedConstraint* p2p = new btPoint2PointConstraint(*body, pivot);

		m_dynamicsWorld->addConstraint(p2p);
		p2p->setDbgDrawSize(btScalar(5.f));
	}

	{
		//btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(0.5)));
		btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);

		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, 0, 0));
		btRigidBody* groundBody;
		groundBody = createRigidBody(0, groundTransform, groundShape);
		groundBody->setFriction(btSqrt(2));
	}
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void GyroscopicSetup::physicsDebugDraw(int debugFlags)
{
	CommonRigidBodyBase::physicsDebugDraw(debugFlags);

	//render method names above objects
	for (int i = 0; i < m_dynamicsWorld->getNumCollisionObjects(); i++)
	{
		btRigidBody* body = btRigidBody::upcast(m_dynamicsWorld->getCollisionObjectArray()[i]);
		if (body && body->getInvMass() > 0)
		{
			btTransform tr = body->getWorldTransform();
			btVector3 pos = tr.getOrigin() + btVector3(0, 0, 2);
			btScalar size = 1;
			m_guiHelper->drawText3D(gyroNames[i], pos.x(), pos.y(), pos.z(), size);
		}
	}
}

class CommonExampleInterface* GyroscopicCreateFunc(CommonExampleOptions& options)
{
	return new GyroscopicSetup(options.m_guiHelper);
}
