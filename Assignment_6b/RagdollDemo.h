/*
Bullet Continuous Collision Detection and Physics Library
RagdollDemo
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#ifndef RAGDOLLDEMO_H
#define RAGDOLLDEMO_H

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btDefaultMotionState.h"
#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

class RagdollDemo : public GlutDemoApplication
{

	btAlignedObjectArray<class RagDoll*> m_ragdolls;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

    btRigidBody*      body[9]; // one main body, 4x2 leg segments
    btCollisionShape* geom[9];
    bool pause;
    btHingeConstraint* joints[8];
    bool oneStep;

public:
	void initPhysics();

	void exitPhysics();

	virtual ~RagdollDemo()
	{
      int i;

		exitPhysics();
	}

	void spawnRagdoll(const btVector3& startOffset);

    btVector3 flipZY(btVector3 input) {
      btScalar temp;
      temp = input[1];
      input[1] = input[2];
      input[2] = temp;
      return input;
    }

    void CreateBox(int index, double x, double y, double z, 
                double length, double width, double height) {
      double mass = 1.0;
      btTransform transform;
      transform.setIdentity(); 
      transform.setOrigin(flipZY(btVector3(x, y, z)));
      btDefaultMotionState* motionState = new btDefaultMotionState(transform);

      btBoxShape *shape = new btBoxShape(flipZY(btVector3(length/2., width/2.,height/2.)));
      btVector3 localInertia(0.,0.,0.);
      shape->calculateLocalInertia(mass, localInertia);
      //shape->initializePolyhedralFeatures();
      btRigidBody::btRigidBodyConstructionInfo cInfo(mass,motionState,shape,localInertia);

      geom[index] = shape;
      //body[index] = new btRigidBody(mass, motionState, geom[index]);
      body[index] = new btRigidBody(cInfo);
      m_dynamicsWorld->addRigidBody(body[index]);
    }

    void DeleteObject(int i) {
      if (body[i]) {
        m_dynamicsWorld->removeRigidBody(body[i]);
        delete body[i];
        body[i] = NULL;
      }
      if (geom[i]) {
        delete geom[i];
        geom[i] = NULL;
      }
    }
    void DestroyHinge(int i) {
      if (joints[i]) {
        m_dynamicsWorld->removeConstraint(joints[i]);
        delete joints[i];
        joints[i] = NULL;
      }
    }
    void CreateCylinder(int index, 
                        double x, double y, double z, 
                        double r, double l, int alignment) {
      double mass = 1.0;
      btTransform transform;
      transform.setIdentity(); 
      transform.setOrigin(btVector3(x, z, y));
      btCylinderShape* shape;
      btVector3 halfExtents(r, r, r);
      switch (alignment) {
      case 1:
        halfExtents[0] = l/2.;
        //transform.getBasis().setEulerZYX(0., 0., M_PI_2);
        shape = new btCylinderShapeX(halfExtents);
        break;
      case 3: //2
        //transform.getBasis().setEulerZYX(0., 0., 0.);
        halfExtents[1] = l/2.;
        shape = new btCylinderShape(halfExtents);
        break;
      case 2: //3
        halfExtents[2] = l/2.;
        shape = new btCylinderShapeZ(halfExtents);
        //transform.getBasis().setEulerZYX(M_PI_2, 0., 0.);
        break;
      }
      transform.setIdentity(); 
      transform.setOrigin(btVector3(x, z, y));

      btTransform none;
      none.setIdentity();
      btDefaultMotionState* motionState = new btDefaultMotionState(transform);

      /* alignment = {1, 2, 3} ~ {x,y,z}*/
      btVector3 localInertia(0.,0.,0.);
      shape->calculateLocalInertia(mass, localInertia);
      btRigidBody::btRigidBodyConstructionInfo cInfo(mass,motionState,shape,localInertia);
      //cInfo.m_startWorldTransform = transform;
      geom[index] = shape;
      //body[index] = new btRigidBody(mass, motionState, geom[index]);
      body[index] = new btRigidBody(cInfo);
      m_dynamicsWorld->addRigidBody(body[index]);
    }

    btVector3 PointWorldToLocal(int index, btVector3 p) {
      btTransform local1 = body[index]->getCenterOfMassTransform().inverse();
      return local1 * p;
    }

    btVector3 AxisWorldToLocal(int index, btVector3 a) {
      btTransform local1 = body[index]->getCenterOfMassTransform().inverse();
      btVector3 zero(0,0,0);
      local1.setOrigin(zero);
      return local1 * a;
    }


    void CreateHinge(int index, int body1, int body2, 
                     double x, double y, double z,
                     double ax, double ay, double az) {
      btTransform local, local1, local2;
      local1 = body[body1]->getCenterOfMassTransform().inverse();
      local2 = body[body2]->getCenterOfMassTransform().inverse();
      local.setIdentity();
      btVector3 zero(0,0,0);
      btVector3 p(x, y, z);
      btVector3 a(ax, ay, az);

      btVector3 v = PointWorldToLocal(body1, btVector3(0,1,0)); 
      /* std::cerr << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl; */
      //printf("(%f, %f, %f)\n", v[0], v[1], v[2]);
      //local2 = local1;
      /* btVector3 p1 = local1 * flipZY(p); */
      /* btVector3 p2 = local2 * flipZY(p); */

      btVector3 p1 = PointWorldToLocal(body1, flipZY(p));
      btVector3 p2 = PointWorldToLocal(body2, flipZY(p));

      /* local1.setOrigin(zero); */
      /* local2.setOrigin(zero); */
      /* btVector3 a1 = local1 * flipZY(a); */
      /* btVector3 a2 = local2 * flipZY(a); */


      btVector3 a1 = AxisWorldToLocal(body1, flipZY(a));
      btVector3 a2 = AxisWorldToLocal(body2, flipZY(a));
      joints[index] = new btHingeConstraint(*body[body1], *body[body2],
                                                       p1, p2,
                                                       a1, a2, false);
      joints[index]->setLimit(-45.*3.14159/180., 45.*3.14159/180.);
      //joints[index] = joint;
      
      m_dynamicsWorld->addConstraint(joints[index], true);

    }

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	static DemoApplication* Create()
	{
		RagdollDemo* demo = new RagdollDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
	
};


#endif
