/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "Bridge.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
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

struct Dof6Spring2SetupInternalData
{
	btRigidBody* m_TranslateSpringBody;
	btRigidBody* m_TranslateSpringBody2;
	btRigidBody* m_RotateSpringBody;
	btRigidBody* m_RotateSpringBody2;
	btRigidBody* m_BouncingTranslateBody;
	btRigidBody* m_MotorBody;
	btRigidBody* m_ServoMotorBody;
	btRigidBody* m_ChainLeftBody;
	btRigidBody* m_ChainRightBody;
	btGeneric6DofSpring2Constraint* m_ServoMotorConstraint;
	btGeneric6DofSpring2Constraint* m_ChainLeftConstraint;
	btGeneric6DofSpring2Constraint* m_ChainRightConstraint;

	float mDt;

	unsigned int frameID;
	Dof6Spring2SetupInternalData()
		: mDt(1. / 60.), frameID(0)
	{
	}
};


const int TOTAL_PLANKS = 10;
struct BridgeExample : public CommonRigidBodyBase
{
	BridgeExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~BridgeExample() {}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 20;
		float pitch = -16;
		float yaw = 180;
		float targetPos[3] = {-2.4, 0.4, -0.24};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

void BridgeExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -50, 0));
	{
		btScalar mass(0.);
		createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	}

	//create two fixed boxes to hold the planks

	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
		btScalar plankWidth = 0.4;
		btScalar plankHeight = 0.2;
		btScalar plankBreadth = 1;
		btScalar plankOffset = plankWidth;  //distance between two planks
		btScalar bridgeWidth = plankWidth * TOTAL_PLANKS + plankOffset * (TOTAL_PLANKS - 1);
		btScalar bridgeHeight = 5;
		btScalar halfBridgeWidth = bridgeWidth * 0.5f;

		btBoxShape* colShape = createBoxShape(btVector3(plankWidth, plankHeight, plankBreadth));

		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		//create a set of boxes to represent bridge
		btAlignedObjectArray<btRigidBody*> boxes;
		int lastBoxIndex = TOTAL_PLANKS - 1;
		for (int i = 0; i < TOTAL_PLANKS; ++i)
		{
			float t = float(i) / lastBoxIndex;
			t = -(t * 2 - 1.0f) * halfBridgeWidth;
			startTransform.setOrigin(btVector3(
				btScalar(t),
				bridgeHeight,
				btScalar(0)));
			boxes.push_back(createRigidBody((i == 0 || i == lastBoxIndex) ? 0 : mass, startTransform, colShape));
		}

		//add N-1 spring constraints
		for (int i = 0; i < TOTAL_PLANKS - 1; ++i)
		{
			btRigidBody* b1 = boxes[i];
			btRigidBody* b2 = boxes[i + 1];

			btPoint2PointConstraint* leftSpring = new btPoint2PointConstraint(*b1, *b2, btVector3(-0.5, 0, -0.5), btVector3(0.5, 0, -0.5));
			m_dynamicsWorld->addConstraint(leftSpring);

			btPoint2PointConstraint* rightSpring = new btPoint2PointConstraint(*b1, *b2, btVector3(-0.5, 0, 0.5), btVector3(0.5, 0, 0.5));
			m_dynamicsWorld->addConstraint(rightSpring);
		}
	}

	// HUMAN
	{
		float z{5};
		btScalar margin = 0.2f;

		// body
		// it's half values
		btScalar height{2};
		btScalar leg_height = 1.0f;
		btScalar arm_height = 0.7f;
		btScalar width{1};
		btScalar breadth{0.5};
		btScalar mass(10.f);
		btBoxShape* colShape = createBoxShape(btVector3(breadth, height, width));
		m_collisionShapes.push_back(colShape);
		btVector3 localInertia{};
		colShape->calculateLocalInertia(mass, localInertia);

		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(0, 2 * leg_height + height, z));
		auto body = createRigidBody(mass, tr, colShape);


		// leg 1
		mass = 5.0f;		
		colShape = createBoxShape(btVector3(width / 3, leg_height, width / 3));
		m_collisionShapes.push_back(colShape);
		colShape->calculateLocalInertia(mass, localInertia);

		tr.setIdentity();
		tr.setOrigin(btVector3(0, leg_height, z + width / 2));
		auto leg1 = createRigidBody(mass, tr, colShape);

		// leg 2
		// the same colision shape -> better performance
		tr.setIdentity();
		tr.setOrigin(btVector3(0, leg_height, z - width / 2));
		auto leg2 = createRigidBody(mass, tr, colShape);

		// arm 1
		mass = 0.5f;
		btScalar arm_width = width / 3;
		colShape = createBoxShape(btVector3(arm_width, arm_height, arm_width));
		m_collisionShapes.push_back(colShape);
		colShape->calculateLocalInertia(mass, localInertia);

		tr.setIdentity();
		tr.setOrigin(btVector3(0, 2 * leg_height + 2 * height - arm_width, z + width + arm_height + margin));
		tr.getBasis().setEulerZYX(-M_PI_2, 0, 0);
		auto arm1 = createRigidBody(mass, tr, colShape);

		// arm 2
		tr.setIdentity();
		tr.setOrigin(btVector3(0, 2 * leg_height + 2 * height - arm_width, z - width - arm_height - margin));
		tr.getBasis().setEulerZYX(M_PI_2, 0, 0);
		auto arm2 = createRigidBody(mass, tr, colShape);

		// head
		mass = 0.1;
		btScalar head_height = 0.6;
		colShape = createBoxShape(btVector3(head_height, head_height, head_height));
		m_collisionShapes.push_back(colShape);
		colShape->calculateLocalInertia(mass, localInertia);

		tr.setIdentity();
		tr.setOrigin(btVector3(0, 2*leg_height + 2*height, z));
		auto motionState = new btDefaultMotionState(tr);
		auto m_data = new Dof6Spring2SetupInternalData();
		m_data->m_RotateSpringBody = new btRigidBody(mass, motionState, colShape, localInertia);
		m_data->m_RotateSpringBody->setActivationState(DISABLE_DEACTIVATION);
		m_dynamicsWorld->addRigidBody(m_data->m_RotateSpringBody);

		// contraints
		// relate of center of mass !!!
		btTransform localA{}, localB{};
		localA.setIdentity();
		localA.getOrigin() = btVector3(0, height + margin, 0);
		localB.setIdentity();
		localB.setOrigin(btVector3(0, -head_height, 0));
		auto head_constraint = new btGeneric6DofSpring2Constraint(*body, *m_data->m_RotateSpringBody, localA, localB);
		head_constraint->setLimit(0, 0, 0);
		head_constraint->setLimit(1, 0, 0);
		head_constraint->setLimit(2, 0, 0);
		head_constraint->setLimit(3, 0, 0);
		head_constraint->setLimit(4, 0, 0);
		head_constraint->setLimit(5, 1, -1);
		head_constraint->enableSpring(5, true);
		head_constraint->setStiffness(5, 100);
		head_constraint->setDamping(5, 0);
		head_constraint->setEquilibriumPoint(0, 0);
		head_constraint->setDbgDrawSize(btScalar(2.f));
		m_dynamicsWorld->addConstraint(head_constraint, true);		
		
		btPoint2PointConstraint* body_leg1_spring = new btPoint2PointConstraint(*body, *leg1, btVector3(0, -height, width / 2), btVector3(0, leg_height, 0));
		body_leg1_spring->m_setting.m_damping = 0.0625;
		body_leg1_spring->m_setting.m_impulseClamp = 0.95;
		m_dynamicsWorld->addConstraint(body_leg1_spring);

		btPoint2PointConstraint* body_leg2_spring = new btPoint2PointConstraint(*body, *leg2, btVector3(0, -height, -width / 2), btVector3(0, leg_height, 0));
		body_leg2_spring->m_setting.m_damping = 0.0625;
		body_leg2_spring->m_setting.m_impulseClamp = 0.95;
		m_dynamicsWorld->addConstraint(body_leg2_spring);

		btPoint2PointConstraint* body_arm1_spring = new btPoint2PointConstraint(*body, *arm1, btVector3(0, height - arm_width, width), btVector3(0, arm_height + margin, 0));
		body_arm1_spring->m_setting.m_damping = 0.0625;
		body_arm1_spring->m_setting.m_impulseClamp = 0.95;
		m_dynamicsWorld->addConstraint(body_arm1_spring);

		btPoint2PointConstraint* body_arm2_spring = new btPoint2PointConstraint(*body, *arm2, btVector3(0, height - arm_width, -width), btVector3(0, arm_height + margin, 0));
		body_arm2_spring->m_setting.m_damping = 0.0625;
		body_arm2_spring->m_setting.m_impulseClamp = 0.95;
		m_dynamicsWorld->addConstraint(body_arm2_spring);
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void BridgeExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

CommonExampleInterface* ET_BridgeCreateFunc(CommonExampleOptions& options)
{
	return new BridgeExample(options.m_guiHelper);
}
