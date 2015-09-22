#pragma once
#include "DirectX_11_1_Includes.h"
#include "Lighting.h"
#include "Keyboard.h"
#include "GamePad.h"
#include "FSM.h"
#include "AnimationController.h"

struct ID3DX11Effect;
struct ID3DX11EffectTechnique;
struct ID3DX11EffectMatrixVariable;
struct ID3D11ShaderResourceView;

class ModelLoader;
class Mesh;
class Skeleton;
struct ID3D11DeviceContext;
struct ID3D11Device;

class PhysicsWorld;
class btKinematicCharacterController;
class btPairCachingGhostObject;

class Goblin {
public:
	enum PLAYER {
		PLAYER_1 = 0,
		PLAYER_2 = 1
	};

	Goblin();
	~Goblin();
	bool Init( ModelLoader* modelLoader, ID3D11Device* device, DirectX::Keyboard::KeyboardStateTracker* kb, DirectX::GamePad* gamepad, PLAYER player, PhysicsWorld* physicsWorld );
	void Update( float dt );
	void XM_CALLCONV Draw( DirectX::FXMMATRIX viewProj, DirectX::FXMVECTOR cameraPos, std::vector<PointLight> pointLights, ID3D11DeviceContext* context );
	void XM_CALLCONV SetPos( DirectX::FXMVECTOR pos );
	DirectX::FXMVECTOR XM_CALLCONV getPos();
	void XM_CALLCONV SetRot( DirectX::FXMVECTOR rot );
	DirectX::FXMMATRIX XM_CALLCONV GetRot();
	DirectX::FXMMATRIX XM_CALLCONV GetWorld();
	void ResetActions();
private:
	struct Actions {
		bool Forward;
		bool Back;
		bool Left;
		bool Right;
		bool Attack;
		bool Jump;
		bool Duck;
	};
	void UpdateActions();
	void UpdateController( float dtS );
	void UpdateModelTransforms();
	void UpdateWalkDirection();

	// Finite State Machine Functions
	void InitFSM();
	void Idle_Before( float dt );
	void Idle_Update( float dt );
	void Idle_After( float dt );
	void Forward_Before( float dt );
	void Forward_Update( float dt );
	void Forward_After( float dt );
	void Turn_Right_Before( float dt );
	void Turn_Right_Update( float dt );
	void Turn_Right_After( float dt );
	void Turn_Left_Before( float dt );
	void Turn_Left_Update( float dt );
	void Turn_Left_After( float dt );
	void Backward_Before( float dt );
	void Backward_Update( float dt );
	void Backward_After( float dt );
	void Jump_Before( float dt );
	void Jump_Update( float dt );
	void Jump_After( float dt );
	void Fall_Before( float dt );
	void Fall_Update( float dt );
	void Fall_After( float dt );
	void Die_Before( float dt );
	void Die_Update( float dt );
	void Die_After( float dt );
	void Duck_Before( float dt );
	void Duck_Update( float dt );
	void Duck_After( float dt );
	void Attack_Before( float dt );
	void Attack_Update( float dt );
	void Attack_After( float dt );
	void Attack_Left_Before( float dt );
	void Attack_Left_Update( float dt );
	void Attack_Left_After( float dt );
	void Attack_Right_Before( float dt );
	void Attack_Right_Update( float dt );
	void Attack_Right_After( float dt );
	void Attack_Jump_Before( float dt );
	void Attack_Jump_Update( float dt );
	void Attack_Jump_After( float dt );

	PLAYER										player;
	Mesh*										mesh;
	Skeleton*									skeleton;
	ID3D11ShaderResourceView*					diffuseView;
	Material									mat;
	DirectX::XMMATRIX							pos;
	//DirectX::XMMATRIX							importRot;
	DirectX::XMMATRIX							rot;
	DirectX::XMMATRIX							scale;
	DirectX::XMMATRIX							world;
	DirectX::XMMATRIX							modelControllerOffset;
	DirectX::Keyboard::KeyboardStateTracker*	kb;
	DirectX::GamePad*							gamePad;
	Actions										action;
	PhysicsWorld*								physicsWorld;
	btKinematicCharacterController*				controller;
	btPairCachingGhostObject*					ghostObject;
	FSM<Goblin>*								fsm;
	float										movementBearing;
	AnimationController							animController;

	//Player movement
	DirectX::XMFLOAT2							moveDir;
	DirectX::XMFLOAT2							moveVel;
	float										maxVel;
	float										moveAccel; // m/s/s
	float										turnAccel; // rad/s/s
	float										moveDecel; // m/s/s							
	float										fallSpeed;
	float										jumpSpeed;
	float										maxJumpHeight;
	const float									forwardAngle = DirectX::XM_PIDIV4/2.f;			// DELETEME
	const float									backwardAngle = DirectX::XM_PI-DirectX::XM_PIDIV4/2.f;	// DELETEME
	float										jumpTimer;
	float										attackTimer;
};

