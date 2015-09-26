#pragma once
#include "DirectX_11_1_Includes.h"
#include "btBulletDynamicsCommon.h"
#include "BulletDynamics\MLCPSolvers\btDantzigSolver.h"

//#define PHYSICS_DEBUG_MODE

class PhysicsDebugDrawer;

enum COLLIDE_MASK {
	NOTHING = 0,
	PLAYER_CONTROLLER = 1<<0,
	PLAYER_BODY = 1<<1,
	GROUND = 1<<2,
	FIRE_PLINTH = 1<<3
};

class PhysicsWorld {
public:
	PhysicsWorld();
	~PhysicsWorld();
	bool Init();
	bool Init( ID3D11DeviceContext* device );
	void SetupDemo();
	void RunDemo();
	void Update( float dt );
	inline btDiscreteDynamicsWorld* World() { return dynamicsWorld; };
	void AddCollisionShape( btCollisionShape* shape );
	void XM_CALLCONV DrawDebug( DirectX::FXMMATRIX viewProj );
	inline btBroadphaseInterface* getPairCache() { return overlappingPairCache; };
	const float								fixedTimeStep = 1.f/60.f;
private:
	void CleanUpDemo();
	btDefaultCollisionConfiguration*		collisionConfiguration;
	btCollisionDispatcher*					dispatcher;
	btBroadphaseInterface*					overlappingPairCache;
	btSequentialImpulseConstraintSolver*	solver;
	btDiscreteDynamicsWorld*				dynamicsWorld;

	//keep track of the shapes, we release memory at exit.
	//make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray<btCollisionShape*> collisionShapes;
	PhysicsDebugDrawer*						debugDrawer;
	bool									useMCLPSolver;
};

