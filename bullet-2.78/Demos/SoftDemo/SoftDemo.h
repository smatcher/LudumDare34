
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///btSoftBody implementation by Nathanael Presson

#ifndef SOFT_DEMO_H
#define SOFT_DEMO_H

#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics\Dynamics\btDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"

#include "TileSet.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

///collisions between two btSoftBody's
class btSoftSoftCollisionAlgorithm;

///collisions between a btSoftBody and a btRigidBody
class btSoftRididCollisionAlgorithm;
class btSoftRigidDynamicsWorld;


///CcdPhysicsDemo shows basic stacking using Bullet physics, and allows toggle of Ccd (using key '1')
class SoftDemo : public PlatformDemoApplication
{
public:

	btAlignedObjectArray<btSoftSoftCollisionAlgorithm*> m_SoftSoftCollisionAlgorithms;

	btAlignedObjectArray<btSoftRididCollisionAlgorithm*> m_SoftRigidCollisionAlgorithms;

	btSoftBodyWorldInfo	m_softBodyWorldInfo;

	

	bool								m_autocam;
	bool								m_cutting;
	bool								m_raycast;
	btScalar							m_animtime;
	btClock								m_clock;
	int									m_lastmousepos[2];
	btVector3							m_impact;
	btSoftBody::sRayCast				m_results;
	btSoftBody::Node*					m_node;
	btVector3							m_goal;
	bool								m_drag;


	btVector3							m_direction = btVector3(1,0,0);
	bool								m_left = false;
	bool								m_right = false;
	btSoftBody*							m_tartiflette;
	float								m_tartifletteVC;
	float								m_tartiflettePR;
	TileSet								m_city_tileset;
	std::vector<btRigidBody*>			m_cars;
	std::vector<btRigidBody*>			m_buildings;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>		m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;


	btConstraintSolver*	m_solver;

	btCollisionAlgorithmCreateFunc*	m_boxBoxCF;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	bool					m_bGraphicsInit = false;

public:

	void	initPhysics();

	void	exitPhysics();

	virtual void	myinit() override;

	SoftDemo() : m_drag(false)
	{
		setTexturing(true);
		setShadows(true);

		m_azi = 45.f;
		m_ele = 60.f;
	}
	virtual ~SoftDemo()
	{
		exitPhysics();
	}

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	void createStack( btCollisionShape* boxShape, float halfCubeSize, int size, float zPos );

	static DemoApplication* Create()
	{
		SoftDemo* demo = new SoftDemo;
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	virtual	void setDrawClusters(bool drawClusters);

	virtual const btSoftRigidDynamicsWorld*	getSoftDynamicsWorld() const
	{
		///just make it a btSoftRigidDynamicsWorld please
		///or we will add type checking
		return (btSoftRigidDynamicsWorld*) m_dynamicsWorld;
	}

	virtual btSoftRigidDynamicsWorld*	getSoftDynamicsWorld()
	{
		///just make it a btSoftRigidDynamicsWorld please
		///or we will add type checking
		return (btSoftRigidDynamicsWorld*) m_dynamicsWorld;
	}

	//
	void	clientResetScene();
	void	renderme();
	void	specialKeyboard(int key, int x, int y);
	void	specialKeyboardUp(int key, int x, int y);
	void	keyboardCallback(unsigned char key, int x, int y);
	void	mouseFunc(int button, int state, int x, int y);
	void	mouseMotionFunc(int x,int y);

	friend void	Init_Pressure(SoftDemo* pdemo);
};

#define MACRO_SOFT_DEMO(a) class SoftDemo##a : public SoftDemo\
{\
public:\
	static DemoApplication* Create()\
	{\
		SoftDemo* demo = new SoftDemo##a;\
		extern unsigned int current_demo;\
		current_demo=a;\
		demo->myinit();\
		demo->initPhysics();\
		return demo;\
	}\
};


MACRO_SOFT_DEMO(1) //Init_Pressure


#endif //CCD_PHYSICS_DEMO_H





