#include "stdafx.h"
#include "PhysicsWorld.h"
#include "PhysicsDebugDrawer.h"
#include "BulletDynamics\MLCPSolvers\btDantzigSolver.h"
#include "BulletDynamics\MLCPSolvers\btSolveProjectedGaussSeidel.h"
#include "BulletDynamics\MLCPSolvers\btMLCPSolver.h"

using namespace DirectX;

PhysicsWorld::PhysicsWorld() :
debugDrawer( nullptr ),
useMCLPSolver( false ) {}

PhysicsWorld::~PhysicsWorld() {
	CleanUpDemo();
	//TODO -- Delete all the things
}

bool PhysicsWorld::Init( ID3D11DeviceContext* ctx ) {
#ifdef PHYSICS_DEBUG_MODE
	debugDrawer = new PhysicsDebugDrawer(); debugDrawer = new PhysicsDebugDrawer();
	debugDrawer->Init( ctx );
#endif
	return this->Init();
}

bool PhysicsWorld::Init() {
	collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	dispatcher = new btCollisionDispatcher( collisionConfiguration );

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	//overlappingPairCache = new btDbvtBroadphase();
	btVector3 worldMin( -100, -100, -100 );
	btVector3 worldMax( 100, 100, 100 );
	overlappingPairCache = new btAxisSweep3( worldMin, worldMax );

	if( useMCLPSolver ) {
		btDantzigSolver* mlcp = new btDantzigSolver();
		//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
		btMLCPSolver* sol = new btMLCPSolver( mlcp );
		solver = sol;
	} else {
		solver = new btSequentialImpulseConstraintSolver();
	}

	//m_dynamicsWorld->setConstraintSolver( m_solver );

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	solver = new btSequentialImpulseConstraintSolver;

	dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, overlappingPairCache, solver, collisionConfiguration );
	if( useMCLPSolver ) {
		dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 1;//for direct solver it is better to have a small A matrix
	} else {
		dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 128;//for direct solver, it is better to solve multiple objects together, small batches have high overhead
	}

	dynamicsWorld->setGravity( btVector3( btScalar( 0 ), btScalar( -9.8 ), btScalar( 0 ) ) );
	dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration = 0.0001f;

#ifdef PHYSICS_DEBUG_MODE
	dynamicsWorld->setDebugDrawer( debugDrawer );
#endif

	return true;
}

void PhysicsWorld::AddCollisionShape( btCollisionShape* shape ) {
	collisionShapes.push_back( shape ); 
} 

void PhysicsWorld::SetupDemo() {
	btCollisionShape* groundShape = new btBoxShape( btVector3( btScalar( 50. ), btScalar( 50. ), btScalar( 50. ) ) );

	AddCollisionShape( groundShape );
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin( btVector3( 0, -56, 0 ) );
	{
		btScalar mass( 0. );
		bool isDynamic = (mass!=0.f);
		btVector3 localInertia( 0, 0, 0 );
		if( isDynamic ) {
			groundShape->calculateLocalInertia( mass, localInertia );
		}

		//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState( groundTransform );
		btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, myMotionState, groundShape, localInertia );
		btRigidBody* body = new btRigidBody( rbInfo );

		dynamicsWorld->addRigidBody( body );
	}
	{
		//create a dynamic rigidbody
		btCollisionShape* colShape = new btSphereShape( btScalar( 1. ) );
		AddCollisionShape( colShape );

		btTransform startTransform;
		startTransform.setIdentity();
		btScalar mass( 1.f );
		bool isDynamic = (mass!=0.f);
		btVector3 localInertia( 0, 0, 0 );
		if( isDynamic ) {
			colShape->calculateLocalInertia( mass, localInertia );
			startTransform.setOrigin( btVector3( 2, 40, 0 ) );
			btDefaultMotionState* myMotionState = new btDefaultMotionState( startTransform );
			btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, myMotionState, colShape, localInertia );
			btRigidBody* body = new btRigidBody( rbInfo );
			short group = COLLIDE_MASK::FIRE_PLINTH;
			short mask = COLLIDE_MASK::PLAYER_CONTROLLER|COLLIDE_MASK::PLAYER_BODY|COLLIDE_MASK::GROUND;
			dynamicsWorld->addRigidBody( body );
		}
	}
}

void PhysicsWorld::Update( float dt ) {
	btScalar timeStepInSeconds( dt );
	dynamicsWorld->stepSimulation( timeStepInSeconds, 100, fixedTimeStep );
}

void PhysicsWorld::RunDemo() {
	for( int i = dynamicsWorld->getNumCollisionObjects()-1; i>=0; --i ) {
		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast( obj );
		btTransform trans;
		if( body && body->getMotionState() ) {
			body->getMotionState()->getWorldTransform( trans );
		} else {
			trans = obj->getWorldTransform();
		}
		//fprintf( stdout, "world pos object %d = %f,%f,%f\n", i, float( trans.getOrigin().getX() ), float( trans.getOrigin().getY() ), float( trans.getOrigin().getZ() ) );
	}
}

void PhysicsWorld::CleanUpDemo() {
	for( int i = dynamicsWorld->getNumCollisionObjects()-1; i>=0; --i ) {
		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast( obj );
		if( body && body->getMotionState() ) {
			delete body->getMotionState();
		}
		dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}
	for( int j = 0; j<collisionShapes.size(); ++j ) {
		btCollisionShape* shape = collisionShapes[j];
		collisionShapes[j] = 0;
		delete shape;
	}
	//delete dynamics world
	delete dynamicsWorld;

	//delete solver
	delete solver;

	//delete broadphase
	delete overlappingPairCache;

	//delete dispatcher
	delete dispatcher;

	delete collisionConfiguration;

	//next line is optional: it will be cleared by the destructor when the array goes out of scope
	collisionShapes.clear();
}

void XM_CALLCONV PhysicsWorld::DrawDebug( FXMMATRIX viewProj ) {
	assert( debugDrawer!=nullptr );
	debugDrawer->Begin( viewProj );
	dynamicsWorld->debugDrawWorld();
	debugDrawer->End();
}