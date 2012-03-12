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
#include "GLDebugDrawer.h"
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

#define MY_PATH "/Users/shane/School/uvm/CSYS-395-evolutionary-robotics/csys-395-assignments/Assignment_10b/"

#define BATCH_MODE  1
#define READ_FILE_MODE   2
#define NORMAL_MODE 3

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
    btHingeConstraint* joints[8];
    bool pause;
    bool oneStep;
    int  IDs[10];
public:
    int  touches[10];
    btVector3 touchPoints[10];
    double weights[4][8];
    int mode;

    RagdollDemo() {
      pause = false;
      oneStep = false;
    }

	void initPhysics();

	void exitPhysics();

	virtual ~RagdollDemo()
	{
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
      //shape->setUserPointer(&IDs[index]);
      //shape->initializePolyhedralFeatures();
      btRigidBody::btRigidBodyConstructionInfo cInfo(mass,motionState,shape,localInertia);

      geom[index] = shape;
      //body[index] = new btRigidBody(mass, motionState, geom[index]);
      body[index] = new btRigidBody(cInfo);
      body[index]->setUserPointer(&IDs[index]);
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
    void DeleteJoint(int i) {
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
      btCylinderShape* shape;
      btVector3 halfExtents(r, r, r);
      switch (alignment) {
      case 1:
        halfExtents[0] = l/2.;
        transform.getBasis().setEulerZYX(0., 0., M_PI_2);
        shape = new btCylinderShapeX(halfExtents);
        break;
      case 3: //2
        transform.getBasis().setEulerZYX(0., 0., 0.);
        halfExtents[1] = l/2.;
        shape = new btCylinderShape(halfExtents);
        break;
      case 2: //3
        halfExtents[2] = l/2.;
        shape = new btCylinderShapeZ(halfExtents);
        transform.getBasis().setEulerZYX(M_PI_2, 0., 0.);
        break;
      }
      //shape = new btCylinderShape(btVector3(r, l/2., r)); 
      transform.setIdentity(); 
      transform.setOrigin(btVector3(x, z, y));

      btDefaultMotionState* motionState = new btDefaultMotionState(transform); 

      /* alignment = {1, 2, 3} ~ {x,y,z}*/
      btVector3 localInertia(0.,0.,0.);
      shape->calculateLocalInertia(mass, localInertia);

      btRigidBody::btRigidBodyConstructionInfo cInfo(mass,motionState,shape,localInertia);
      //cInfo.m_startWorldTransform = transform;
      geom[index] = shape;
      //body[index] = new btRigidBody(mass, motionState, geom[index]);
      body[index] = new btRigidBody(cInfo);
      body[index]->setUserPointer(&IDs[index]);
      m_dynamicsWorld->addRigidBody(body[index]);
    }

    btVector3 PointWorldToLocal(int index, btVector3 p) {
      btTransform local1 = body[index]->getCenterOfMassTransform().inverse();
      return local1 * p;
    }

    btVector3 AxisWorldToLocal(int index, btVector3 a) {
      btTransform local = body[index]->getCenterOfMassTransform();
      btMatrix3x3 rotation = local.getBasis();
      //printf("body %d world axis (%.1f, %.1f, %.1f)\n", index, a[0], a[1], a[2]);
      btVector3 b = rotation.transpose() * a;
      //printf("body %d local axis (%.1f, %.1f, %.1f)\n", index, b[0], b[1], b[2]);
      return b;
    }


    void CreateHinge(int index, int body1, int body2, 
                     double x, double y, double z,
                     double ax, double ay, double az) {
      btTransform local, local1, local2;
      btVector3 p(x, y, z);
      btVector3 a(ax, ay, az);

      btVector3 p1 = PointWorldToLocal(body1, flipZY(p));
      btVector3 p2 = PointWorldToLocal(body2, flipZY(p));
      btVector3 a1 = AxisWorldToLocal(body1, flipZY(a));
      btVector3 a2 = AxisWorldToLocal(body2, flipZY(a));
      joints[index] = new btHingeConstraint(*body[body1], *body[body2],
                                                       p1, p2,
                                                       a1, a2, false);
      joints[index]->setLimit(-45.*3.14159/180., 45.*3.14159/180.);
      m_dynamicsWorld->addConstraint(joints[index], true);

    }

    void ActuateJoint(int jointIndex, double desiredAngle, double dt) {
      double maxForce = 40.;
      /* ActuateJoint2(jointIndex, desiredAngle, dt); */
      /* return; */
      joints[jointIndex]->enableMotor(true);
      joints[jointIndex]->setMaxMotorImpulse(maxForce*dt);
      joints[jointIndex]->setMotorTarget(desiredAngle*M_PI/180., dt);
    }

    void ActuateJoint2(int jointIndex, double desiredAngle, double dt) {
      double currAngle = joints[jointIndex]->getHingeAngle();
      double diff = currAngle - desiredAngle*M_PI/180.;
      double k = 100.;
      double maxForce = 30.;
      joints[jointIndex]->enableAngularMotor(true, -k * diff, maxForce*dt);
    }

    virtual void renderme() {
      extern GLDebugDrawer	gDebugDrawer;
      GlutDemoApplication::renderme();
      //gDebugDrawer.drawSphere(btVector3(0.,0.,0.), 0.9, btVector3(1., 0., 0.));

      gDebugDrawer.drawSphere(btVector3(0.,0.,0.), 0.1, btVector3(0., 0., 0.));
      gDebugDrawer.drawSphere(btVector3(-1.,0.,0.), 0.1, btVector3(0., 0., 1.));
      for (int i = 0; i < 9; i++)
        if (touches[i])
          gDebugDrawer.drawSphere(touchPoints[i], 0.2, btVector3(1., 0., 0.));
    }

    void ReadWeights() {
      FILE *f;
      int i, j;
      do {
        f = fopen(MY_PATH "weights.dat", "r");
        usleep(200); // .2 seconds
      } while (f == NULL);

      for (j = 0; j < 8; j++) 
        for (i = 0; i < 4; i++) {
          fscanf(f, "%lf ", &(weights[i][j]));
          //printf("%lf ", weights[i][j]);
        }
      fclose(f);
  
      unlink(MY_PATH "weights.dat");
    }

    int ReadWeightsFromFile(char *filename) {
      FILE *f;
      int i, j;
      f = fopen(filename, "r");
      if (! f)
        return 1;               /* error! */

      for (j = 0; j < 8; j++) 
        for (i = 0; i < 4; i++) {
          fscanf(f, "%lf ", &(weights[i][j]));
          //printf("%lf ", weights[i][j]);
        }
      fclose(f);
      return 0;
    }


    void SaveFitness()
    {
      FILE *f;
      btVector3 pos = body[0]->getCenterOfMassPosition();
      f = fopen(MY_PATH "fit.dat", "w+");
      //fprintf(stdout, "fitness %lf\n", -pos[0]);
      /* printf("weights: "); */
      /* for (int j = 0; j < 8; j++) { */
      /*   for (int i = 0; i < 4; i++)  */
      /*     fprintf(stdout, "%lf ", (weights[i][j])); */
      /*   printf("\n"); */
      /* } */

      fprintf(f, "%lf", -pos[0]);
      fclose(f);
    }

    void CreateRobot()
    {
      CreateBox(0, 0., 0., 1., 1., 1., 0.2);
      // fore limbs
      CreateCylinder(1, 1., 0., 1., 0.1, 1., 1);
      CreateCylinder(2, -1., 0., 1., 0.1, 1., 1);
      CreateCylinder(3, 0., 1., 1., 0.1, 1., 2);
      CreateCylinder(4, 0., -1., 1., 0.1, 1., 2);

      CreateCylinder(5, 1.5,  0., .5, 0.1, 1., 3);
      CreateCylinder(6, -1.5, 0., .5, 0.1, 1., 3);
      CreateCylinder(7, 0.,  1.5, .5, 0.1, 1., 3);
      CreateCylinder(8, 0., -1.5, .5, 0.1, 1., 3);

      // btVector3 v = PointWorldToLocal(0, btVector3(0,1,0)); 
      // /* std::cerr << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl; */
      // printf("(%f, %f, %f)\n", v[0], v[1], v[2]);


      CreateHinge(0, 
                  0, 1, 
                  0.5, 0., 1., 
                  0.,  -1., 0.);

      CreateHinge(1, 0, 2, 
                  -.5, 0., 1., 
                  0., 1., 0.);

      CreateHinge(2, 0, 3, 
                  0., .5, 1., 
                  1., 0., 0.);

      CreateHinge(3, 0, 4, 
                  0., -.5, 1., 
                  -1., 0., 0.);

      CreateHinge(4, 1, 5, 
                  1.5, 0., 1., 
                  0., -1., 0.);

      CreateHinge(5, 2, 6, 
                  -1.5, 0., 1., 
                  0., 1., 0.);

      CreateHinge(6, 3, 7, 
                  0., 1.5, 1., 
                  1., 0., 0.);

      CreateHinge(7, 4, 8, 
                  0., -1.5, 1., 
                  -1., 0., 0.);

      for (int i = 0; i < 10; i++)
        touches[i] = 0;

    }

    void DeleteRobot()
    {
      int i;
      for (i = 0; i < 8; i++)
        DeleteJoint(i);

      for (i = 0; i < 9; i++)
        DeleteObject(i);
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
