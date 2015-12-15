#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"

#include "../GimpactTestDemo/BunnyMesh.h"
#include "../GimpactTestDemo/TorusMesh.h"
#include <stdio.h> //printf debugging
#include "LinearMath/btConvexHull.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

#include "SoftDemo.h"
#include "GL_ShapeDrawer.h"
#include "GLDebugFont.h"
#include "GlutStuff.h"

#include "tgaloader.h"

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

extern float eye[3];
extern int glutScreenWidth;
extern int glutScreenHeight;

const int maxProxies = 32766;

static btVector3*	gGroundVertices=0;
static int*	gGroundIndices=0;
static float waveheight = 5.f;

const float TRIANGLE_SIZE=8.f;

#define CUBE_HALF_EXTENTS 1.5

GLuint	gTexIdTartif1	= 0;
GLuint	gTexIdTartif2	= 0;
btVector3	gTartifletteCenter	= btVector3(0.0f, 0.0f, 0.0f);

static const int cityXOffset = -7 * 15;
static const int cityYOffset = -7 * 15;

//
//void SoftDemo::createStack( btCollisionShape* boxShape, float halfCubeSize, int size, float zPos )
//{
//	btTransform trans;
//	trans.setIdentity();
//
//	for(int i=0; i<size; i++)
//	{
//		// This constructs a row, from left to right
//		int rowSize = size - i;
//		for(int j=0; j< rowSize; j++)
//		{
//			btVector3 pos;
//			pos.setValue(
//				-rowSize * halfCubeSize + halfCubeSize + j * 2.0f * halfCubeSize,
//				halfCubeSize + i * halfCubeSize * 2.0f,
//				zPos);
//
//			trans.setOrigin(pos);
//			btScalar mass = 1.f;
//
//			btRigidBody* body = 0;
//			body = localCreateRigidBody(mass,trans,boxShape);
//
//		}
//	}
//}


////////////////////////////////////

void SoftDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


	renderme();

	glFlush();
	swapBuffers();
}

//
// Big ball
//
//static void	Ctor_BigBall(SoftDemo* pdemo,btScalar mass=10)
//{
//	btTransform startTransform;
//	startTransform.setIdentity();
//	startTransform.setOrigin(btVector3(0,13,0));
//	pdemo->localCreateRigidBody(mass,startTransform,new btSphereShape(3));
//}
//
//
// Big plate
//
static btRigidBody*	Ctor_BigPlate(SoftDemo* pdemo,btScalar mass=15,btScalar height=4)
{
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,height,0.5));
	btRigidBody*		body=pdemo->localCreateRigidBody(mass,startTransform,new btBoxShape(btVector3(5,1,5)));
	body->setFriction(1);
	return(body);
}

//
// Linear stair
//
static void Ctor_LinearStair(SoftDemo* pdemo,const btVector3& org,const btVector3& sizes,btScalar angle,int count)
{
	btBoxShape*	shape=new btBoxShape(sizes);
	for(int i=0;i<count;++i)
	{
		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(org+btVector3(sizes.x()*i*2,sizes.y()*i*2,0));
		btRigidBody* body=pdemo->localCreateRigidBody(0,startTransform,shape);
		body->setFriction(1);
	}
}

//
// Softbox
//
//static btSoftBody* Ctor_SoftBox(SoftDemo* pdemo,const btVector3& p,const btVector3& s)
//{
//	const btVector3	h=s*0.5;
//	const btVector3	c[]={	p+h*btVector3(-1,-1,-1),
//		p+h*btVector3(+1,-1,-1),
//		p+h*btVector3(-1,+1,-1),
//		p+h*btVector3(+1,+1,-1),
//		p+h*btVector3(-1,-1,+1),
//		p+h*btVector3(+1,-1,+1),
//		p+h*btVector3(-1,+1,+1),
//		p+h*btVector3(+1,+1,+1)};
//	btSoftBody*		psb=btSoftBodyHelpers::CreateFromConvexHull(pdemo->m_softBodyWorldInfo,c,8);
//	psb->generateBendingConstraints(2);
//	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
//
//	return(psb);
//
//}


//
// Pressure
//
static void	Init_Pressure(SoftDemo* pdemo)
{
	btSoftBody*	psb=btSoftBodyHelpers::CreateEllipsoid(pdemo->m_softBodyWorldInfo,btVector3(0,10,10),
		btVector3(1,1,1)*6,
		512);
	psb->m_materials[0]->m_kLST	=	0.1;
	psb->m_cfg.kDF				=	1;
	psb->m_cfg.kDP				=	0.001; // fun factor...
	psb->m_cfg.kPR				=	pdemo->m_tartiflettePR	 = 100;
	psb->setTotalMass(30,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
	
	//Ctor_BigPlate(pdemo);
	//Ctor_LinearStair(pdemo,btVector3(0,0,0),btVector3(2,1,5),0,10);
	pdemo->m_autocam=true;

	pdemo->m_tartiflette = psb;
	pdemo->m_tartifletteVC = 0.08f;

	if(pdemo->m_bGraphicsInit)
		pdemo->m_city_tileset.loadFromFile("textures/city.txt");

	pdemo->m_cars.clear();
	srand(42);

	for (int i = 0; i < pdemo->m_city_tileset.width(); i++)
	{
		for (int j = 0; j < pdemo->m_city_tileset.height(); j++)
		{
			if (pdemo->m_city_tileset.isBuilding(i, j))
			{
				btScalar mass = 0;
				btScalar height = 10;
				btScalar width = 7.5;

				btTransform startTransform;
				startTransform.setIdentity();
				startTransform.setOrigin(btVector3(cityXOffset + 15*i + width, height / 2, cityYOffset + 15*j + width));
				btRigidBody*		body = pdemo->localCreateRigidBody(mass, startTransform, new btBoxShape(btVector3(width, height, width)));
				pdemo->m_buildings.push_back(body);
			}
			else
			{
				if(rand() < 0.3f*RAND_MAX)
				{
					btScalar mass = 10.0f;
					btScalar height = 0.75f;
					btScalar width = 7.5;

					btTransform startTransform;
					startTransform.setIdentity();
					startTransform.setOrigin(btVector3(cityXOffset + 15*i + width, height / 2, cityYOffset + 15*j + width));

					float sizeX = 3.0f;
					float sizeZ = 1.5f;

					btQuaternion rotation;
					rotation.setRotation(btVector3(0, 1, 0), 2 * M_PI * float(rand()) / RAND_MAX);
					startTransform.setRotation(rotation);

					btRigidBody*	pCar	= pdemo->localCreateRigidBody(mass, startTransform, new btBoxShape(btVector3(sizeX, height, sizeZ)));
					pdemo->m_cars.push_back(pCar);
				}
			}
		}
	}

	pdemo->m_nbEatenCars	= 0;
}


void SoftDemo::specialKeyboard(int key, int x, int y)
{
	switch (key)
	{
	case GLUT_KEY_LEFT: m_left = true; break;
	case GLUT_KEY_RIGHT: m_right = true; break;
	default:
		PlatformDemoApplication::specialKeyboard(key, x, y);
		break;
	}
}

void SoftDemo::specialKeyboardUp(int key, int x, int y)
{
	switch (key)
	{
	case GLUT_KEY_LEFT: m_left = false; break;
	case GLUT_KEY_RIGHT: m_right = false; break;
	default:
		PlatformDemoApplication::specialKeyboardUp(key, x, y);
		break;
	}
}

void	SoftDemo::clientResetScene()
{
	//m_azi = 0;


	m_cameraDistance = 30.f;
	m_cameraTargetPosition.setValue(0,0,0);

	DemoApplication::clientResetScene();
	/* Clean up	*/ 
	for(int i=m_dynamicsWorld->getNumCollisionObjects()-1;i>=0;i--)
	{
		btCollisionObject*	obj=m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody*		body=btRigidBody::upcast(obj);
		if(body&&body->getMotionState())
		{
			delete body->getMotionState();
		}
		while(m_dynamicsWorld->getNumConstraints())
		{
			btTypedConstraint*	pc=m_dynamicsWorld->getConstraint(0);
			m_dynamicsWorld->removeConstraint(pc);
			delete pc;
		}
		btSoftBody* softBody = btSoftBody::upcast(obj);
		if (softBody)
		{
			getSoftDynamicsWorld()->removeSoftBody(softBody);
		} else
		{
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body)
				m_dynamicsWorld->removeRigidBody(body);
			else
				m_dynamicsWorld->removeCollisionObject(obj);
		}
		delete obj;
	}


	//create ground object
	btTransform tr;
	tr.setIdentity();
	//tr.setOrigin(btVector3(0,-12,0));
	tr.setOrigin(btVector3(0,-CUBE_HALF_EXTENTS,0));

	btCollisionObject* newOb = new btCollisionObject();
	newOb->setWorldTransform(tr);
	newOb->setInterpolationWorldTransform( tr);
	
	//newOb->setCollisionShape(m_collisionShapes[0]);
	newOb->setCollisionShape(m_collisionShapes[1]);

	m_dynamicsWorld->addCollisionObject(newOb);

	m_softBodyWorldInfo.m_sparsesdf.Reset();

	m_softBodyWorldInfo.air_density		=	(btScalar)1.2;
	m_softBodyWorldInfo.water_density	=	0;
	m_softBodyWorldInfo.water_offset		=	0;
	m_softBodyWorldInfo.water_normal		=	btVector3(0,0,0);
	m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);


	m_autocam						=	false;
	m_raycast						=	false;
	m_cutting						=	false;
	m_results.fraction				=	1.f;
	Init_Pressure(this);
}

void SoftDemo::clientMoveAndDisplay()
{
	m_azi = 180.0f * atan2(m_direction.getX(), m_direction.getZ()) / M_PI;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT); 




	float ms = getDeltaTimeMicroseconds();
	float dt = ms / 1000000.f;//1.0/60.;	



	if (m_dynamicsWorld)
	{
		//during idle mode, just run 1 simulation step maximum, otherwise 4 at max
		int maxSimSubSteps = m_idle ? 1 : 4;
		//if (m_idle)
		//	dt = 1.0/420.f;

		static float gfDirectionSensitivityScale = 3.14f;

		if (m_left)
			m_direction = m_direction.rotate(btVector3(0, 1, 0), dt * gfDirectionSensitivityScale);
		else if(m_right)
			m_direction = m_direction.rotate(btVector3(0, 1, 0), -dt * gfDirectionSensitivityScale);

		static float gfForceScale = 1.0f;
		static float gfJumpScale = 0.3f;
		static float gfTimeIncrementer = 0.0f;
		static float gfTimeScale = 1.6f;
		gfTimeIncrementer += dt * gfTimeScale;

		float forceTimeFunc = 0.5f * (1.0f + sinf(gfTimeIncrementer));
		float jumpTimeFunc = 0.5f * (1.0f + sinf(gfTimeIncrementer - M_PI*0.25f));
		jumpTimeFunc = jumpTimeFunc * jumpTimeFunc;
		float forceScale = gfForceScale * forceTimeFunc;
		float jumpScale = gfJumpScale * jumpTimeFunc;

		m_tartiflette->addForce(forceScale * m_direction + jumpScale * btVector3(0,1,0));

		m_tartiflette->m_cfg.kVC = m_tartifletteVC;
		m_tartiflette->m_cfg.kPR = m_tartiflettePR;

		// Make cars go away from us
		for(int i=0 ; i < m_cars.size() ; i++)
		{
			const btVector3	vCarPos = m_cars[i]->getWorldTransform().getOrigin();
			btVector3	vTartifletteToCar	= vCarPos-gTartifletteCenter;
			vTartifletteToCar.safeNormalize();
		
			//btQuaternion	targetOrientation;
			//targetOrientation.setRotation(btVector3(0.0f, 1.0f, 0.0f), atan2(vTartifletteToCar.getY(), vTartifletteToCar.getX()));
			//
			//btTransform	transform	= m_cars[i]->getWorldTransform();
			//transform.setRotation(targetOrientation);
			//m_cars[i]->setWorldTransform(transform);
			static float gfForceScale= 100.0f;
			m_cars[i]->applyCentralForce(vTartifletteToCar*gfForceScale);
		}

		int numSimSteps;
		numSimSteps = m_dynamicsWorld->stepSimulation(dt);
		//numSimSteps = m_dynamicsWorld->stepSimulation(dt,10,1./240.f);
		if(m_drag)
		{
			m_node->m_v*=0;
		}

		m_softBodyWorldInfo.m_sparsesdf.GarbageCollect();

		//optional but useful: debug drawing


		// Eat cars!
		static float gfMinDistToEatCars = 4.0f;
		for(int i=m_cars.size()-1 ; i >= 0 ; i--)
		{
			const btVector3	vCarPos = m_cars[i]->getWorldTransform().getOrigin();
			float dist = (vCarPos-gTartifletteCenter).length();
			if(dist < gfMinDistToEatCars)
			{
				printf("MIAM!\n");
				gfMinDistToEatCars += 1.2f;
				m_dynamicsWorld->removeRigidBody(m_cars[i]);
				m_tartifletteVC *= 0.5f;
				m_tartiflettePR += 1200.0f;
				m_cars.erase(m_cars.begin() + i);
				m_nbEatenCars++;
			}
		}

		// Eat buildings!
		if(m_nbEatenCars > 10)
		{
			static float gfDistMinToEatBuildings = 15.0f;
			for(int i=m_buildings.size()-1 ; i >= 0 ; i--)
			{
				const btVector3	vBuildingPos = m_buildings[i]->getWorldTransform().getOrigin();
				float dist = (vBuildingPos-gTartifletteCenter).length();
				if(dist < gfDistMinToEatBuildings)
				{
					printf("MIAM BUILDING!\n");
					gfMinDistToEatCars += 1.2f;
					m_dynamicsWorld->removeRigidBody(m_buildings[i]);
					m_tartifletteVC *= 0.5f;
					m_tartiflettePR += 1200.0f;
					m_buildings.erase(m_buildings.begin() + i);
				}
			}
		}
	}


	renderme(); 

	//render the graphics objects, with center of mass shift

	updateCamera();



	glFlush();

	swapBuffers();

}



void	SoftDemo::renderme()
{
	btIDebugDraw*	idraw=m_dynamicsWorld->getDebugDrawer();

	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	m_dynamicsWorld->debugDrawWorld();

	btSoftRigidDynamicsWorld* softWorld = (btSoftRigidDynamicsWorld*)m_dynamicsWorld;
	for (  int i=0;i<softWorld->getSoftBodyArray().size();i++)
	{
		btSoftBody*	psb=(btSoftBody*)softWorld->getSoftBodyArray()[i];
		if (softWorld->getDebugDrawer() && !softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe))
		{
			btSoftBodyHelpers::DrawFrame(psb,softWorld->getDebugDrawer());
			btSoftBodyHelpers::Draw(psb,softWorld->getDebugDrawer(),softWorld->getDrawFlags());
		}
	}

	/* Bodies		*/ 
	btVector3	ps(0,0,0);
	int			nps=0;

	btSoftBodyArray&	sbs=getSoftDynamicsWorld()->getSoftBodyArray();
	for(int ib=0;ib<sbs.size();++ib)
	{
		btSoftBody*	psb=sbs[ib];
		nps+=psb->m_nodes.size();
		for(int i=0;i<psb->m_nodes.size();++i)
		{
			ps+=psb->m_nodes[i].m_x;
		}		
	}
	ps/=nps;
	if(m_autocam)
		m_cameraTargetPosition+=(ps-m_cameraTargetPosition)*0.05;
	
	int lineWidth=280;
	int xStart = m_glutScreenWidth - lineWidth;
	int yStart = 20;

	/*
	if((getDebugMode() & btIDebugDraw::DBG_NoHelpText)==0)
	{
		setOrthographicProjection();
		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);
		char buf[124] = "";
		
		glRasterPos3f(xStart, yStart, 0);
		GLDebugDrawString(xStart,20,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"] for next demo (%d)",current_demo);
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"c to visualize clusters");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"; to toggle camera mode");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
        sprintf(buf,"n,m,l,k for power and steering");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);


		resetPerspectiveProjection();
		glEnable(GL_LIGHTING);
	}
	*/
	//DemoApplication::renderme();

	
	myinit();

	updateCamera();

	m_city_tileset.draw(cityXOffset, cityYOffset);

	if (m_dynamicsWorld)
	{			
		if(m_enableshadows)
		{
			glClear(GL_STENCIL_BUFFER_BIT);
			glEnable(GL_CULL_FACE);
			renderscene(0);

			glDisable(GL_LIGHTING);
			glDepthMask(GL_FALSE);
			glDepthFunc(GL_LEQUAL);
			glEnable(GL_STENCIL_TEST);
			glColorMask(GL_FALSE,GL_FALSE,GL_FALSE,GL_FALSE);
			glStencilFunc(GL_ALWAYS,1,0xFFFFFFFFL);
			glFrontFace(GL_CCW);
			glStencilOp(GL_KEEP,GL_KEEP,GL_INCR);
			renderscene(1);
			glFrontFace(GL_CW);
			glStencilOp(GL_KEEP,GL_KEEP,GL_DECR);
			renderscene(1);
			glFrontFace(GL_CCW);

			glPolygonMode(GL_FRONT,GL_FILL);
			glPolygonMode(GL_BACK,GL_FILL);
			glShadeModel(GL_SMOOTH);
			glEnable(GL_DEPTH_TEST);
			glDepthFunc(GL_LESS);
			glEnable(GL_LIGHTING);
			glDepthMask(GL_TRUE);
			glCullFace(GL_BACK);
			glFrontFace(GL_CCW);
			glEnable(GL_CULL_FACE);
			glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);

			glDepthFunc(GL_LEQUAL);
			glStencilFunc( GL_NOTEQUAL, 0, 0xFFFFFFFFL );
			glStencilOp( GL_KEEP, GL_KEEP, GL_KEEP );
			glDisable(GL_LIGHTING);
			renderscene(2);
			glEnable(GL_LIGHTING);
			glDepthFunc(GL_LESS);
			glDisable(GL_STENCIL_TEST);
			glDisable(GL_CULL_FACE);
		}
		else
		{
			glDisable(GL_CULL_FACE);
			renderscene(0);
		}

		int	xOffset = 10;
		int yStart = 20;
		int yIncr = 20;


		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);

		//if ((m_debugMode & btIDebugDraw::DBG_NoHelpText)==0)
		//{
		//	setOrthographicProjection();
		//
		//	showProfileInfo(xOffset,yStart,yIncr);
		//	resetPerspectiveProjection();
		//}

		//glDisable(GL_LIGHTING);
	}

	updateCamera();


}

void	SoftDemo::setDrawClusters(bool drawClusters)
{
	if (drawClusters)
	{
		getSoftDynamicsWorld()->setDrawFlags(getSoftDynamicsWorld()->getDrawFlags()|fDrawFlags::Clusters);
	} else
	{
		getSoftDynamicsWorld()->setDrawFlags(getSoftDynamicsWorld()->getDrawFlags()& (~fDrawFlags::Clusters));
	}
}



void	SoftDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch(key)
	{
//	case	',':	m_raycast=!m_raycast;break;
//	case	';':	m_autocam=!m_autocam;break;
//	case	'c':	getSoftDynamicsWorld()->setDrawFlags(getSoftDynamicsWorld()->getDrawFlags()^fDrawFlags::Clusters);break;
//	case	'`':
//		{
//			btSoftBodyArray&	sbs=getSoftDynamicsWorld()->getSoftBodyArray();
//			for(int ib=0;ib<sbs.size();++ib)
//			{
//				btSoftBody*	psb=sbs[ib];
//				psb->staticSolve(128);
//			}
//		}
//		break;
	default:		DemoApplication::keyboardCallback(key,x,y);
	}
}

//
void	SoftDemo::mouseMotionFunc(int x,int y)
{
	if(m_node&&(m_results.fraction<1.f))
	{
		if(!m_drag)
		{
#define SQ(_x_) (_x_)*(_x_)
			if((SQ(x-m_lastmousepos[0])+SQ(y-m_lastmousepos[1]))>6)
			{
				m_drag=true;
			}
#undef SQ
		}
		if(m_drag)
		{
			m_lastmousepos[0]	=	x;
			m_lastmousepos[1]	=	y;		
		}
	}
	else
	{
		DemoApplication::mouseMotionFunc(x,y);
	}
}

//
void	SoftDemo::mouseFunc(int button, int state, int x, int y)
{
	DemoApplication::mouseFunc(button,state,x,y);
}


void	SoftDemo::initPhysics()
{
	///create concave ground mesh

	
	//m_azi = 0;

	btCollisionShape* groundShape = 0;
	{
		int i;
		int j;

		const int NUM_VERTS_X = 30;
		const int NUM_VERTS_Y = 30;
		const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
		const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);

		gGroundVertices = new btVector3[totalVerts];
		gGroundIndices = new int[totalTriangles*3];

		btScalar offset(-50);

		for ( i=0;i<NUM_VERTS_X;i++)
		{
			for (j=0;j<NUM_VERTS_Y;j++)
			{
				gGroundVertices[i+j*NUM_VERTS_X].setValue((i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
					//0.f,
					waveheight*sinf((float)i)*cosf((float)j+offset),
					(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
			}
		}

		int vertStride = sizeof(btVector3);
		int indexStride = 3*sizeof(int);

		int index=0;
		for ( i=0;i<NUM_VERTS_X-1;i++)
		{
			for (int j=0;j<NUM_VERTS_Y-1;j++)
			{
				gGroundIndices[index++] = j*NUM_VERTS_X+i;
				gGroundIndices[index++] = j*NUM_VERTS_X+i+1;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

				gGroundIndices[index++] = j*NUM_VERTS_X+i;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i;
			}
		}

		btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles,
			gGroundIndices,
			indexStride,
			totalVerts,(btScalar*) &gGroundVertices[0].x(),vertStride);

		bool useQuantizedAabbCompression = true;

		groundShape = new btBvhTriangleMeshShape(indexVertexArrays,useQuantizedAabbCompression);
		groundShape->setMargin(0.5);
	}

	m_collisionShapes.push_back(groundShape);

	btCollisionShape* groundBox = new btBoxShape (btVector3(260,CUBE_HALF_EXTENTS,260));
	m_collisionShapes.push_back(groundBox);

	btCompoundShape* cylinderCompound = new btCompoundShape;
	btCollisionShape* cylinderShape = new btCylinderShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS));
	btTransform localTransform;
	localTransform.setIdentity();
	cylinderCompound->addChildShape(localTransform,cylinderShape);
	btQuaternion orn(btVector3(0,1,0),SIMD_PI);
	localTransform.setRotation(orn);
	cylinderCompound->addChildShape(localTransform,cylinderShape);

	m_collisionShapes.push_back(cylinderCompound);


	m_dispatcher=0;

	///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();


	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;

	////////////////////////////
	///Register softbody versus softbody collision algorithm


	///Register softbody versus rigidbody collision algorithm


	////////////////////////////

	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);

	m_softBodyWorldInfo.m_broadphase = m_broadphase;

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	m_solver = solver;

	btDiscreteDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_dynamicsWorld = world;
	

	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));
	m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);


	//	clientResetScene();

	m_softBodyWorldInfo.m_sparsesdf.Initialize();
	clientResetScene();
}






void	SoftDemo::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		m_collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;



	delete m_collisionConfiguration;


}


void SoftDemo::myinit()
{
//	PlatformDemoApplication::myinit();

	GLfloat light_ambient[] = { btScalar(0.2), btScalar(0.2), btScalar(0.2), btScalar(1.0) };
	GLfloat light_diffuse[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0), btScalar(1.0) };
	GLfloat light_specular[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0), btScalar(1.0 )};
	/*	light_position is NOT default value	*/
	GLfloat light_position0[] = { btScalar(1.0), btScalar(10.0), btScalar(1.0), btScalar(0.0 )};
	GLfloat light_position1[] = { btScalar(-1.0), btScalar(-10.0), btScalar(-1.0), btScalar(0.0) };

	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);

	glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);


	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	//glClearColor(btScalar(0.7),btScalar(0.7),btScalar(0.7),btScalar(0));
	glClearColor(114.0f/255.0f, 200.0f/255.0f, 255.0f/255.0f, 0.0f);

	if(!m_bGraphicsInit)
	{
		m_bGraphicsInit	= true;
		// -------------
		// Load textures
		TGALoader	loader;
		loader.loadOpenGLTexture("textures/tartif1.tga", &gTexIdTartif1, TGA_BILINEAR);
		loader.loadOpenGLTexture("textures/tartif2.tga", &gTexIdTartif2, TGA_BILINEAR);

		// ------------
		// Load tileset(s)
		m_city_tileset.loadFromFile("textures/city.txt");

		clientResetScene(); // don't ask
	}
}
