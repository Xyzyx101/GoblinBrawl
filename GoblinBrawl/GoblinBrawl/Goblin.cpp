#include "stdafx.h"
#include "Goblin.h"
#include "ModelLoader.h"
#include "Vertex.h"
#include "Mesh.h"
#include "MyEffects.h"
#include "MathUtils.h"
#include "Skeleton.h"
#include "WICTextureLoader.h"
#include "PhysicsWorld.h"
#include "Bullet/BulletDynamics/Character/btKinematicCharacterController.h"
#include "Bullet/BulletCollision/CollisionDispatch/btGhostObject.h"

Goblin::Goblin() :
mesh( nullptr ),
diffuseView( nullptr ),
ghostObject( nullptr ),
controller( nullptr ),
forwardAmount( 0 ),
turnAmount( 0 ),
strafeAmount( 0 ),
forwardSpeed( 1.f ),
turnSpeed( 2.f ),
strafeSpeed( 1.f ),
fallSpeed( 20.f ),
jumpSpeed( 10.f ),
maxJumpHeight( 1.75f )
{}

Goblin::~Goblin() {
	delete skeleton;
	delete mesh;
}

bool Goblin::Init( ModelLoader* modelLoader, ID3D11Device* device, Keyboard::KeyboardStateTracker* kb, GamePad* gamePad, PLAYER player, PhysicsWorld* physicsWorld ) {
	// Model
	modelLoader->Load( "Goblin4.fbx", Vertex::CHARACTER_SKINNED );
	mesh = modelLoader->GetMesh();
	if( mesh->VB()==nullptr ) {
		return false;
	}
	skeleton = modelLoader->GetSkeleton();
	if( skeleton==nullptr ) {
		return false;
	}

	//Texture
	HR( CreateWICTextureFromFile( device, L"./art/textures/goblin_color.tif", NULL, &diffuseView, NULL ) );
	mat.Ambient = XMFLOAT4( 0.02f, 0.3f, 0.5f, 1.0f );
	mat.Diffuse = XMFLOAT4( 0.8f, 0.8f, 0.8f, 1.0f );
	mat.Specular = XMFLOAT4( 0.3f, 0.3f, 0.3f, 32.0f );

	// Start Position
	XMFLOAT4 goblinPos;
	if( player==PLAYER_1 ) {
		goblinPos = XMFLOAT4( 0.f, 3.f, 0.f, 1.0f );
	} else {
		goblinPos = XMFLOAT4( 20.f, 5.f, 10.f, 1.0f );
	}
	XMVECTOR xmVectorPos = XMLoadFloat4( &goblinPos );
	SetPos( xmVectorPos );
	rot = XMMatrixIdentity();
	scale = XMMatrixScaling( 0.01f, 0.01f, 0.01f ); //FBX scale

	// Keyboard Controller
	this->kb = kb;
	this->player = player;

	// GamePad
	this->gamePad = gamePad;

	// Physics
	this->physicsWorld = physicsWorld;
	btScalar controllerWidth( 0.2 );
	btScalar controllerHeight( 1.5 );
	controllerHeight -= controllerWidth*2;
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin( btVector3( goblinPos.x, goblinPos.y, goblinPos.z ) );
	ghostObject = new btPairCachingGhostObject();
	ghostObject->setWorldTransform( startTransform );
	physicsWorld->getPairCache()->getOverlappingPairCache()->setInternalGhostPairCallback( new btGhostPairCallback() );
	btConvexShape* capsule = new btCapsuleShape( controllerWidth, controllerHeight );
	physicsWorld->AddCollisionShape( capsule );
	ghostObject->setCollisionShape( capsule );
	ghostObject->setCollisionFlags( btCollisionObject::CF_CHARACTER_OBJECT );
	btScalar stepHeight = btScalar( 0.35 );
	controller = new btKinematicCharacterController( ghostObject, capsule, stepHeight );
	controller->setFallSpeed( btScalar( fallSpeed ) );
	controller->setJumpSpeed( btScalar( jumpSpeed ) );
	controller->setMaxJumpHeight( btScalar( maxJumpHeight ) );
	//physicsWorld->World()->addCollisionObject( ghostObject, btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter|btBroadphaseProxy::DefaultFilter );
	physicsWorld->World()->addCollisionObject( ghostObject, COLLIDE_MASK::PLAYER_CONTROLLER, COLLIDE_MASK::GROUND );
	physicsWorld->World()->addAction( controller );

	modelControllerOffset = XMMatrixTranslation( 0.f, -(controllerHeight*0.5f+controllerWidth), 0.f ); //offset y by height and width because width is the sphere on the end of the capsule

	// Create Physics Skelton
	XMMATRIX goblinTransform = GetWorld();
	skeleton->SetRootTransform(goblinTransform );
	skeleton->InitPhysics( physicsWorld );

	// Animations
	animController.SetSkeleton( skeleton );
	std::vector<Anim*> anims = modelLoader->GetAnimations();
	for( Anim* anim : anims ) {
		animController.AddAnim( anim );
	}
	skeleton->SetAnimationController( &animController );

	// Finite State Machine
	InitFSM();

	return true;
}

void XM_CALLCONV Goblin::Draw( FXMMATRIX viewProj, FXMVECTOR cameraPos, std::vector<PointLight> pointLights, ID3D11DeviceContext* context ) {
	context->IASetInputLayout( InputLayouts::CharacterSkinned );
	context->IASetPrimitiveTopology( D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST );
	UINT stride = sizeof( Vertex::CharacterSkinnedVertex );
	UINT offset = 0;
	ID3D11Buffer* buffers[1] = { mesh->VB() };
	context->IASetVertexBuffers( 0, 1, &buffers[0], &stride, &offset );
	context->IASetIndexBuffer( mesh->IB(), mesh->IndexFormat(), 0 );

	XMMATRIX world = scale * rot * pos;
	XMMATRIX worldInvTranspose = MathUtils::InverseTranspose( world );
	XMMATRIX worldViewProj = world*viewProj;

	MyEffects::CharacterSkinnedFX->SetWorld( world );
	MyEffects::CharacterSkinnedFX->SetWorldInvTranspose( worldInvTranspose );
	MyEffects::CharacterSkinnedFX->SetWorldViewProj( worldViewProj );
	MyEffects::CharacterSkinnedFX->SetDiffuseMap( diffuseView );
	MyEffects::CharacterSkinnedFX->SetEyePosW( cameraPos );
	MyEffects::CharacterSkinnedFX->SetPointLights( pointLights.data() );
	MyEffects::CharacterSkinnedFX->SetMaterial( mat );
	auto a = skeleton->GetFinalTransforms();
	auto b = skeleton->BoneCount();
	MyEffects::CharacterSkinnedFX->SetBoneTransforms( a, b );

	ID3DX11EffectTechnique* tech = MyEffects::CharacterSkinnedFX->characterSkinnedLight5Tech;
	D3DX11_TECHNIQUE_DESC td;
	tech->GetDesc( &td );
	for( UINT p = 0; p<td.Passes; ++p ) {
		tech->GetPassByIndex( p )->Apply( 0, context );
		context->DrawIndexed( mesh->IndexCount(), 0, 0 );
	}
}

void Goblin::Update( float dt ) {
	fprintf( stdout, "DT : %f, Pos: %3.2f %3.2f %3.2f", dt, pos.r[3].m128_f32[0], pos.r[3].m128_f32[1], pos.r[3].m128_f32[2] );
	UpdateActions();
	DebugActionDisplay();
	fsm->Update( dt );
	animController.Interpolate( dt );
	UpdateModelTransforms();
}

void Goblin::UpdateController( float dt ) {
	btTransform controllerTransform;
	controllerTransform = ghostObject->getWorldTransform();

	btVector3 forwardDir = controllerTransform.getBasis()[2];
	fprintf( stdout, "forwardDir=%f,%f,%f\n", forwardDir[0], forwardDir[1], forwardDir[2] );
	btVector3 upDir = controllerTransform.getBasis()[1];
	btVector3 sideDir = controllerTransform.getBasis()[0];
	forwardDir.normalize();
	upDir.normalize();
	sideDir.normalize();

	btScalar walkSpeed = btScalar( forwardSpeed ) * forwardAmount;
	fprintf( stdout, "walkSpeed=%f\n", walkSpeed );

	btScalar sideWalkSpeed = btScalar( strafeSpeed )* strafeAmount;
	fprintf( stdout, "sideSpeed=%f\n", sideWalkSpeed );

	if( turnAmount!=0.f ) {
		btMatrix3x3 orn = ghostObject->getWorldTransform().getBasis();
		btQuaternion rotQuat = btQuaternion( btVector3( 0, 1, 0 ), turnSpeed*turnAmount*dt ); // negative to convert right handed to left handed
		rotQuat.normalize();
		orn *= btMatrix3x3( rotQuat );
		ghostObject->getWorldTransform().setBasis( orn );
	}
	forwardDir *= btVector3( 1.0, 1.0, -1.0 );
	sideDir *= btVector3( 1.0, 1.0, -1.0 );
	btVector3 btWalkVector = forwardDir*walkSpeed+sideDir*sideWalkSpeed;
	movementBearing = forwardDir.angle( btWalkVector );
	controller->setWalkDirection( btWalkVector*dt );
}

void Goblin::UpdateModelTransforms() {
	btTransform controllerTransform = ghostObject->getWorldTransform();
	btVector3 btPos = controllerTransform.getOrigin();
	XMVECTOR dxPos = XMLoadFloat4( &XMFLOAT4( btPos.x(), btPos.y(), btPos.z(), 1.f ) );
	dxPos = XMVector3Transform( dxPos, modelControllerOffset );
	SetPos( dxPos );
	btMatrix3x3 btRot = controllerTransform.getBasis().transpose();
	XMMATRIX dxMat = XMMATRIX(
		btRot[0].x(), btRot[0].y(), btRot[0].z(), 0,
		btRot[1].x(), btRot[1].y(), btRot[1].z(), 0,
		btRot[2].x(), btRot[2].y(), btRot[2].z(), 0,
		0, 0, 0, 1.f );
	rot = dxMat;
}

void XM_CALLCONV Goblin::SetPos( FXMVECTOR _pos ) {
	pos = XMMatrixTranslationFromVector( _pos );
}

FXMVECTOR XM_CALLCONV Goblin::getPos() {
	XMVECTOR outScale;
	XMVECTOR outRotQuat;
	XMVECTOR outTrans;
	XMMatrixDecompose( &outScale, &outRotQuat, &outTrans, pos );
	return outTrans;
}

void XM_CALLCONV Goblin::SetRot( FXMVECTOR _rot ) {
	rot = XMMatrixRotationRollPitchYawFromVector( _rot );
}

FXMMATRIX XM_CALLCONV Goblin::GetRot() {
	return rot;
}

FXMMATRIX XM_CALLCONV Goblin::GetWorld() {
	return scale * rot * pos;
}

void Goblin::UpdateActions() {
	ResetActions();
	auto gpState = gamePad->GetState( player ); // PLAYER_1 == gamepad 0
	if( gpState.IsConnected() ) {
		strafeAmount = gpState.thumbSticks.leftX;
		forwardAmount = gpState.thumbSticks.leftY;
		turnAmount = -gpState.thumbSticks.rightX;
		if( forwardAmount>0 ) {
			action.Forward = true;
		} else if( forwardAmount<0 ) {
			action.Back = true;
		}
		if( turnAmount<0||strafeAmount<0 ) {
			action.Left = true;
		} else if( turnAmount>0||strafeAmount>0 ) {
			action.Right = true;
		}
		action.Attack = gpState.IsBPressed();
		action.Jump = gpState.IsAPressed();
		action.Duck = gpState.IsXPressed();
	} else {
		if( player==PLAYER_1 ) {
			// Player 1 keys
			action.Forward = kb->lastState.W;
			action.Back = kb->lastState.S;
			action.Left = kb->lastState.A;
			action.Right = kb->lastState.D;
			action.Attack = kb->lastState.V;
			action.Jump = kb->lastState.B;
			action.Duck = kb->lastState.N;
		} else {
			// Player 2 keys
			action.Forward = kb->lastState.Up;
			action.Back = kb->lastState.Down;
			action.Left = kb->lastState.Left;
			action.Right = kb->lastState.Right;
			action.Attack = kb->lastState.NumPad0;
			action.Jump = kb->lastState.NumPad2;
			action.Duck = kb->lastState.Decimal;
		}

		// This turn amount and forward amount will be overridden if using gamepad
		if (action.Forward) {
			forwardAmount = 1.f;
		}
		else if (action.Back) {
			forwardAmount = -1.f;
		}
		else if (!action.Back && !action.Forward)
		{
			forwardAmount = 0.f;
		}

		if (action.Left) {
			turnAmount = 1.f;
		}
		else if (action.Right) {
			turnAmount = -1.f;
		}
		else if (!action.Left && !action.Right)
		{
			turnAmount = 0.f;
		}
	}
}

void Goblin::ResetActions() {
	action.Forward = false;
	action.Back = false;
	action.Left = false;
	action.Right = false;
	action.Attack = false;
	action.Jump = false;
	action.Duck = false;
}

void Goblin::DebugActionDisplay() {
	fprintf( stdout, "\n\nForward:%i\nBack:%i\nRight:%i\nLeft:%i\nAttack:%i\nJump:%i\nDuck:%i\n",
		action.Forward,
		action.Back,
		action.Left,
		action.Right,
		action.Attack,
		action.Jump,
		action.Duck );
}

void Goblin::InitFSM() {

	fsm = new FSM<Goblin>( this );
	FSM<Goblin>::StateData idleStateData;
	idleStateData.Before = &Goblin::Idle_Before;
	idleStateData.Update = &Goblin::Idle_Update;
	idleStateData.After = &Goblin::Idle_After;
	fsm->AddState( FSM_STATE::IDLE, idleStateData );

	FSM<Goblin>::StateData forwardStateData;
	forwardStateData.Before = &Goblin::Forward_Before;
	forwardStateData.Update = &Goblin::Forward_Update;
	forwardStateData.After = &Goblin::Forward_After;
	fsm->AddState( FSM_STATE::FORWARD, forwardStateData );

	FSM<Goblin>::StateData turnRightStateData;
	turnRightStateData.Before = &Goblin::Turn_Right_Before;
	turnRightStateData.Update = &Goblin::Turn_Right_Update;
	turnRightStateData.After = &Goblin::Turn_Right_After;
	fsm->AddState( FSM_STATE::TURN_RIGHT, turnRightStateData);

	FSM<Goblin>::StateData turnLeftStateData;
	turnLeftStateData.Before = &Goblin::Turn_Left_Before;
	turnLeftStateData.Update = &Goblin::Turn_Left_Update;
	turnLeftStateData.After = &Goblin::Turn_Left_After;
	fsm->AddState( FSM_STATE::TURN_LEFT, turnLeftStateData );

	FSM<Goblin>::StateData backwardStateData;
	backwardStateData.Before = &Goblin::Backward_Before;
	backwardStateData.Update = &Goblin::Backward_Update;
	backwardStateData.After = &Goblin::Backward_After;
	fsm->AddState( FSM_STATE::BACKWARD, backwardStateData );

	FSM<Goblin>::StateData jumpStateData;
	jumpStateData.Before = &Goblin::Jump_Before;
	jumpStateData.Update = &Goblin::Jump_Update;
	jumpStateData.After = &Goblin::Jump_After;
	fsm->AddState( FSM_STATE::JUMP, jumpStateData );

	FSM<Goblin>::StateData fallStateData;
	fallStateData.Before = &Goblin::Fall_Before;
	fallStateData.Update = &Goblin::Fall_Update;
	fallStateData.After = &Goblin::Fall_After;
	fsm->AddState( FSM_STATE::FALL, fallStateData );

	FSM<Goblin>::StateData dieStateData;
	dieStateData.Before = &Goblin::Die_Before;
	dieStateData.Update = &Goblin::Die_Update;
	dieStateData.After = &Goblin::Die_After;
	fsm->AddState( FSM_STATE::DIE, dieStateData );

	FSM<Goblin>::StateData duckStateData;
	duckStateData.Before = &Goblin::Duck_Before;
	duckStateData.Update = &Goblin::Duck_Update;
	duckStateData.After = &Goblin::Duck_After;
	fsm->AddState( FSM_STATE::DUCK, duckStateData );

	FSM<Goblin>::StateData attackStateData;
	attackStateData.Before = &Goblin::Attack_Before;
	attackStateData.Update = &Goblin::Attack_Update;
	attackStateData.After = &Goblin::Attack_After;
	fsm->AddState( FSM_STATE::ATTACK, attackStateData );

	FSM<Goblin>::StateData attackLeftStateData;
	attackLeftStateData.Before = &Goblin::Attack_Left_Before;
	attackLeftStateData.Update = &Goblin::Attack_Left_Update;
	attackLeftStateData.After = &Goblin::Attack_Left_After;
	fsm->AddState( FSM_STATE::ATTACK_LEFT, attackLeftStateData );

	FSM<Goblin>::StateData attackRightStateData;
	attackRightStateData.Before = &Goblin::Attack_Right_Before;
	attackRightStateData.Update = &Goblin::Attack_Right_Update;
	attackRightStateData.After = &Goblin::Attack_Right_After;
	fsm->AddState( FSM_STATE::ATTACK_RIGHT, attackRightStateData );

	FSM<Goblin>::StateData attackJumpStateData;
	attackJumpStateData.Before = &Goblin::Attack_Jump_Before;
	attackJumpStateData.Update = &Goblin::Attack_Jump_Update;
	attackJumpStateData.After = &Goblin::Attack_Jump_After;
	fsm->AddState( FSM_STATE::ATTACK_JUMP, attackJumpStateData );

	fsm->ChangeState( FSM_STATE::IDLE );
}

void Goblin::UpdateWalkDirection() {
	if( abs( turnAmount )>0.01 ) {
		if( turnAmount>0 ) {
			fsm->ChangeState( TURN_LEFT );
		} else {
			fsm->ChangeState( TURN_RIGHT );
		}
		return;
	}
	if( movementBearing!=movementBearing ) { // check for undefined
		fsm->ChangeState( IDLE );
		return;
	}
	fprintf( stdout, "Bearing:%f\n", movementBearing );
	if( movementBearing>backwardAngle ) {
		fsm->ChangeState( BACKWARD );
	} else if( movementBearing>forwardAngle ) {
		if(strafeAmount>0) {
			fsm->ChangeState( TURN_RIGHT );
		} else {
			fsm->ChangeState( TURN_LEFT );
		}
	} else {
		fsm->ChangeState( FORWARD );
	}
}

void Goblin::Idle_Before( float dt ) {
	fprintf( stdout, "Idle_Before\n" );
	animController.ChangeAnim( ANIM_IDLE );
}

void Goblin::Idle_Update( float dt ) {
	fprintf( stdout, "Idle_Update\n" );
	UpdateController( dt );
	if( action.Jump ) {
		if( controller->canJump() ) {
			fsm->ChangeState( JUMP );
			controller->jump();
		}
	} else {
		UpdateWalkDirection();
	}
	if( action.Attack ) {
		fsm->ChangeState( ATTACK );
	}
}

void Goblin::Idle_After( float dt ) {
	fprintf( stdout, "Idle_After\n" );
}

void Goblin::Forward_Before( float dt ) {
	fprintf( stdout, "Forward_Before\n" );
	animController.ChangeAnim( ANIM_WALK );
}

void Goblin::Forward_Update( float dt ) {
	fprintf( stdout, "Forward_Update\n" );
	UpdateController( dt );
	if( action.Jump ) {
		if( controller->canJump() ) {
			fsm->ChangeState( JUMP );
			controller->jump();
		}
	} else {
		UpdateWalkDirection();
	}
	if( action.Attack ) {
		fsm->ChangeState( ATTACK );
	}
}

void Goblin::Forward_After( float dt ) {
	fprintf( stdout, "Forward_After\n" );
}

void Goblin::Turn_Right_Before( float dt ) {
	fprintf( stdout, "Turn_Right_Before\n" );
}

void Goblin::Turn_Right_Update( float dt ) {
	fprintf( stdout, "Turn_Right_Update\n" );
	UpdateController( dt );
	if( action.Jump ) {
		if( controller->canJump() ) {
			fsm->ChangeState( JUMP );
			controller->jump();
		}
	} else {
		UpdateWalkDirection();
	}
	if( action.Attack ) {
		fsm->ChangeState( ATTACK );
	}
}

void Goblin::Turn_Right_After( float dt ) {
	fprintf( stdout, "Turn_Right_After\n" );
}

void Goblin::Turn_Left_Before( float dt ) {
	fprintf( stdout, "Turn_Left_Before\n" );
}

void Goblin::Turn_Left_Update( float dt ) {
	fprintf( stdout, "Turn_Left_Update\n" );
	UpdateController( dt );
	if( action.Jump ) {
		if( controller->canJump() ) {
			fsm->ChangeState( JUMP );
			controller->jump();
		}
	} else {
		UpdateWalkDirection();
	}
	if( action.Attack ) {
		fsm->ChangeState( ATTACK );
	}
}

void Goblin::Turn_Left_After( float dt ) {
	fprintf( stdout, "Turn_Left_After\n" );
}

void Goblin::Backward_Before( float dt ) {
	fprintf( stdout, "Backward_Before\n" );
}

void Goblin::Backward_Update( float dt ) {
	fprintf( stdout, "Backward_Update\n" );
	UpdateController( dt );
	if( action.Jump ) {
		if( controller->canJump() ) {
			fsm->ChangeState( JUMP );
			controller->jump();
		}
	} else {
		UpdateWalkDirection();
	}
	if( action.Attack ) {
		fsm->ChangeState( ATTACK );
	}
}

void Goblin::Backward_After( float dt ) {
	fprintf( stdout, "Backward_After\n" );
}
void Goblin::Jump_Before( float dt ) {
	fprintf( stdout, "Jump_Before\n" );
	jumpTimer = 0.5f;
}

void Goblin::Jump_Update( float dt ) {
	fprintf( stdout, "Jump_Update\n" );
	jumpTimer -= dt;
	if( jumpTimer<0 ) {
		fsm->ChangeState( FALL );
	}
}

void Goblin::Jump_After( float dt ) {
	fprintf( stdout, "Jump_After\n" );
}
void Goblin::Fall_Before( float dt ) {
	fprintf( stdout, "Fall_Before\n" );
}

void Goblin::Fall_Update( float dt ) {
	fprintf( stdout, "Fall_Update\n" );
	if( controller->canJump() ) {
		fsm->ChangeState( IDLE );
	}
}

void Goblin::Fall_After( float dt ) {
	fprintf( stdout, "Fall_After\n" );
}
void Goblin::Die_Before( float dt ) {
	fprintf( stdout, "Die_Before\n" );
}

void Goblin::Die_Update( float dt ) {
	fprintf( stdout, "Die_Update\n" );
}

void Goblin::Die_After( float dt ) {
	fprintf( stdout, "Die_After\n" );
}

void Goblin::Duck_Before( float dt ) {
	fprintf( stdout, "Duck_Before\n" );
}

void Goblin::Duck_Update( float dt ) {
	fprintf( stdout, "Duck_Update\n" );
}

void Goblin::Duck_After( float dt ) {
	fprintf( stdout, "Duck_After\n" );
}

void Goblin::Attack_Before( float dt ) {
	fprintf( stdout, "Attack_Before\n" );
	attackTimer = animController.GetAnimTime( ANIM_ATTACK );
	animController.ChangeAnim( ANIM_ATTACK );
}

void Goblin::Attack_Update( float dt ) {
	fprintf( stdout, "Attack_Update\n" );
	attackTimer -= dt;
	if( attackTimer<0 ) {
		fsm->ChangeState( IDLE );
	}
}

void Goblin::Attack_After( float dt ) {
	fprintf( stdout, "Attack_After\n" );
}
void Goblin::Attack_Left_Before( float dt ) {
	fprintf( stdout, "Attack_Left_Before\n" );
}

void Goblin::Attack_Left_Update( float dt ) {
	fprintf( stdout, "Attack_Left_Update\n" );
}

void Goblin::Attack_Left_After( float dt ) {
	fprintf( stdout, "Attack_Left_After\n" );
}
void Goblin::Attack_Right_Before( float dt ) {
	fprintf( stdout, "Attack_Right_Before\n" );
}

void Goblin::Attack_Right_Update( float dt ) {
	fprintf( stdout, "Attack_Right_Update\n" );
}

void Goblin::Attack_Right_After( float dt ) {
	fprintf( stdout, "Attack_Right_After\n" );
}

void Goblin::Attack_Jump_Before( float dt ) {
	fprintf( stdout, "Attack_Jump_Before\n" );
}

void Goblin::Attack_Jump_Update( float dt ) {
	fprintf( stdout, "Attack_Jump_Update\n" );
}

void Goblin::Attack_Jump_After( float dt ) {
	fprintf( stdout, "Attack_Jump_After\n" );
}