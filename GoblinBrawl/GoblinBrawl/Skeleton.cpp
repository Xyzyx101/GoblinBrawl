#include "stdafx.h"
#include "Skeleton.h"
#include <vector>
#include "DirectX_11_1_Includes.h"
#include "PhysicsWorld.h"
#include "MathUtils.h"
#include "AnimationController.h"
#include "Bullet\BulletDynamics\ConstraintSolver\btTypedConstraint.h"

using namespace DirectX;

const int DEBUG_JOINT_COUNT_LIMIT = Skeleton::JOINT_COUNT; //delete this once the skeleton is working

Skeleton::Skeleton() : numBones( 0 ), useRagdoll( true ) {}

Skeleton::~Skeleton() {
	for( auto it = idxBones.begin(); it!=idxBones.end(); ++it ) {
		delete it->second;
	}
}

void Skeleton::SetAnimationController( AnimationController* animationController ) {
	this->animationController = animationController;
}

void Skeleton::AddBone( Bone* newBone ) {
	++numBones;
	idxBones.insert( std::pair<int, Bone*>( newBone->idx, newBone ) );
	nameBones.insert( std::pair<std::string, Bone*>( newBone->name, newBone ) );
}

Bone* Skeleton::GetBoneByName( std::string name ) {
	auto nameIt = nameBones.find( name );
	if( nameIt==nameBones.end() ) {
		return nullptr;
	}
	return nameIt->second;
}

Bone* Skeleton::GetBoneByIndex( int index ) {
	auto indexIt = idxBones.find( index );
	if( indexIt==idxBones.end() ) {
		return nullptr;
	}
	return indexIt->second;
}

void XM_CALLCONV Skeleton::UpdateTransformByName( FXMVECTOR translate, FXMVECTOR rotQuat, FXMVECTOR scale, std::string name ) {
	Bone* bone = nameBones[name];
	XMVECTOR zeroVec = DirectX::XMLoadFloat4( &XMFLOAT4( 0.f, 0.f, 0.f, 1.f ) );
	bone->localTransform = XMMatrixAffineTransformation(
		scale,
		zeroVec,
		rotQuat,
		translate
		);
}

void XM_CALLCONV Skeleton::UpdateTransformByIndex( FXMVECTOR translate, FXMVECTOR rotQuat, FXMVECTOR scale, int index ) {
	Bone* bone = idxBones[index];
	XMVECTOR zeroVec = DirectX::XMLoadFloat4( &XMFLOAT4( 0.f, 0.f, 0.f, 1.f ) );
	bone->localTransform = XMMatrixAffineTransformation(
		scale,
		zeroVec,
		rotQuat,
		translate
		);
}

DirectX::XMFLOAT4X4* Skeleton::GetFinalTransforms() {
	DirtyBones();
	toRoot.resize( numBones );
	if( useRagdoll ) {
		UpdateLocalTransformsFromRagdoll();
		Bone* root = GetBoneByName( "Skeleton_Root" );
		UpdateTransforms( root );
	} else {
		UpdateLocalTransformsFromAnimation();
		Bone* root = GetBoneByName( "Skeleton_Root" );
		UpdateTransforms( root );
	}
	return finalTransformData;
}

void Skeleton::Update( float dt ) {
	Bone* root = GetBoneByName( "Skeleton_Root" );
	XMMATRIX rootXform = toRoot[root->idx];
	XMMATRIX worldXform = XMLoadFloat4x4( &rootTransform );
	XMMATRIX finalXform = rootXform*worldXform;
	root->body->setWorldTransform( XMMatrixToBTTransform( finalXform, false ) );
	UpdateMotorData();
	SetAllMotors( dt );
}

void Skeleton::UpdateTransforms( Bone* bone ) {
	XMMATRIX localTransform = bone->localTransform;
	XMMATRIX offset = bone->offset;
	XMMATRIX finalTransform;
	if( bone->dirty ) {
		if( bone->parentIdx==-1 ) {
			// The root bone
			toRoot[bone->idx] = localTransform;
			finalTransform = offset*localTransform;
		} else {
			XMMATRIX parentToRoot;
			XMMATRIX boneToRoot;
			parentToRoot = toRoot[bone->parentIdx];
			boneToRoot = localTransform*parentToRoot;
			toRoot[bone->idx] = boneToRoot;
			finalTransform = offset*boneToRoot;
		}
		bone->dirty = false;
		DirectX::XMStoreFloat4x4( &finalTransformData[bone->idx], finalTransform );
	}
	if( bone->children.size()==0 ) { return; }
	for( Bone* childBone:bone->children ) {
		UpdateTransforms( childBone );
	}
}

void Skeleton::UpdateLocalTransformsFromAnimation() {
	for( auto it:nameBones ) {
		Bone* bone = it.second;
		XMMATRIX newBoneTransform = XMLoadFloat4x4( &animationController->GetBoneTransform( bone ) );
		if( XMMatrixIsIdentity( newBoneTransform ) ) {
			return; //This means there is no animation channel for this bone
		}
		bone->localTransform = newBoneTransform;
	}
}

void Skeleton::UpdateLocalTransformsFromRagdoll() {
	for( int i = 0; i<DEBUG_JOINT_COUNT_LIMIT /*FIXME -- JOINT_COUNT*/; ++i ) {
		MotorData* motor = motors[i];
		btConeTwistConstraint* joint = static_cast<btConeTwistConstraint*>(joints[i]);
		Bone* bone = GetBoneByIndex( motor->boneIdx );
		XMMATRIX localXForm = bone->localTransform;
		XMVECTOR outScale;
		XMVECTOR outRotQuat;
		XMVECTOR outTrans;
		XMMatrixDecompose( &outScale, &outRotQuat, &outTrans, localXForm );

		btDefaultMotionState* motionState = (btDefaultMotionState*)(joint->getRigidBodyB().getMotionState());
		btTransform jointXform;
		motionState->getWorldTransform( jointXform );
		btTransform bodyToBoneOffset = bone->bodyToBoneOffset;
		btTransform offsetJointXform = jointXform*bodyToBoneOffset;
		btVector3 jointTranspose = offsetJointXform.getOrigin();
		btQuaternion jointRot = offsetJointXform.getRotation();

		XMVECTOR translateVector = XMVectorSet( jointTranspose.x(), jointTranspose.y(), jointTranspose.z(), 1.f );
		XMMATRIX translate = XMMatrixTranslationFromVector( translateVector );

		XMVECTOR quat = XMVectorSet( jointRot.x(), jointRot.y(), jointRot.z(), jointRot.w() );
		XMMATRIX rot = XMMatrixRotationQuaternion( quat );
		XMMATRIX scale = XMMatrixScaling( 0.01f, 0.01f, 0.01f );
		XMMATRIX boneWorld = scale*rot*translate;

		XMMATRIX modelXform = XMLoadFloat4x4( &rootTransform );
		Bone* rootBone = GetBoneByIndex( 0 );
		XMVECTOR det = XMMatrixDeterminant( modelXform );
		XMMATRIX modelInverseWorld = XMMatrixInverse( &det, modelXform );
		XMMATRIX boneModelSpace = boneWorld*modelInverseWorld;

		XMMATRIX rootBoneXform = rootBone->localTransform;
		XMMATRIX boneToRoot = boneModelSpace*rootBoneXform;
		toRoot[bone->idx] = boneModelSpace;
		XMMATRIX offset = bone->offset;
		XMMATRIX finalTransform = offset*boneModelSpace;
		DirectX::XMStoreFloat4x4( &finalTransformData[bone->idx], finalTransform );
		bone->dirty = false;
	}
}

void XM_CALLCONV Skeleton::SetRootTransform( FXMMATRIX transform ) {
	DirectX::XMStoreFloat4x4( &rootTransform, transform );
}

void Skeleton::InitPhysics( PhysicsWorld* _physicsWorld ) {
	physicsWorld = _physicsWorld;
	Bone* root = GetBoneByName( "Skeleton_Root" );
	toRoot.resize( numBones );
	UpdateTransforms( root );

	btConvexShape* shape = new btSphereShape( 0.1 );
	XMMATRIX rootTransformMatrix = XMLoadFloat4x4( &rootTransform );

	XMMATRIX boneLocal = root->localTransform;
	XMMATRIX boneWorld = boneLocal*rootTransformMatrix;
	XMVECTOR junk, boneRotQuat;
	XMMatrixDecompose( &junk, &boneRotQuat, &junk, boneWorld );
	XMFLOAT4 f_boneRotQuat;
	XMStoreFloat4( &f_boneRotQuat, boneRotQuat );
	root->initialRotQuat = XMFLOAT4( f_boneRotQuat.x, f_boneRotQuat.y, f_boneRotQuat.y, f_boneRotQuat.w );
	XMFLOAT4X4 f_boneRot;
	XMMATRIX boneRot = XMMatrixRotationQuaternion( boneRotQuat );
	XMStoreFloat4x4( &f_boneRot, boneRot );
	root->initialRot = f_boneRot;

	btTransform btTran = MathUtils::XMMatrixTransformToBTTransform( boneWorld );

	root->btInitialBody = btTran;
	root->btInitialJointW = btTran;

	btDefaultMotionState* myMotionState = new btDefaultMotionState( btTran );
	btRigidBody::btRigidBodyConstructionInfo rbInfo( 0., myMotionState, shape, btVector3( 0, 0, 0 ) );
	btRigidBody* body = new btRigidBody( rbInfo );
	body->setCollisionFlags( btCollisionObject::CF_NO_CONTACT_RESPONSE );
	body->setActivationState( DISABLE_DEACTIVATION ); // this need to happen when the object is moved
	short group = COLLIDE_MASK::NOTHING;
	short mask = COLLIDE_MASK::NOTHING;
	physicsWorld->World()->addRigidBody( body, group, mask );
	root->body = body;
	
	CreateAllShapes();
	CreateAllBodies();
	CreateAllJoints();
	InitMotorData();
	//CreateDemoRagDoll();
}

void Skeleton::CreateAllShapes() {
	Bone* boneTarget;
	boneTarget = GetBoneByName( "Skeleton_Lower_Spine" );
	CreateBoneShape( S_HIPS, boneTarget, btScalar( 0.03 ) );

	boneTarget = GetBoneByName( "Skeleton_Upper_Spine" );
	CreateBoneShape( S_LOWER_SPINE, boneTarget, btScalar( 0.03 ) );

	boneTarget = GetBoneByName( "Skeleton_Neck" );
	CreateBoneShape( S_UPPER_SPINE, boneTarget, btScalar( 0.03 ) );

	boneTarget = GetBoneByName( "Skeleton_Head" );
	CreateBoneShape( S_NECK, boneTarget, btScalar( 0.03 ) );

	boneTarget = GetBoneByName( "Skeleton_LowerLeg_R" );
	CreateBoneShape( S_UPPER_LEG, boneTarget, btScalar( 0.03 ) );

	boneTarget = GetBoneByName( "Skeleton_Ankle_R" );
	CreateBoneShape( S_LOWER_LEG, boneTarget, btScalar( 0.03 ) );

	boneTarget = GetBoneByName( "Skeleton_Shoulder_R" );
	CreateBoneShape( S_CLAVICLE, boneTarget, btScalar( 0.03 ) );

	boneTarget = GetBoneByName( "Skeleton_Elbow_R" );
	CreateBoneShape( S_UPPER_ARM, boneTarget, btScalar( 0.03 ) );

	boneTarget = GetBoneByName( "Skeleton_Wrist_R" );
	CreateBoneShape( S_LOWER_ARM, boneTarget, btScalar( 0.03 ) );

	boneTarget = GetBoneByName( "Skeleton_Foot_R" );
	CreateBoneShape( S_FOOT, boneTarget, btScalar( 0.03 ) );

	boneTarget = GetBoneByName( "Skeleton_Head_Target" );
	CreateBoneShape( S_HEAD, boneTarget, btScalar( 0.03 ) );

	boneTarget = GetBoneByName( "Skeleton_Hand_R" );
	CreateBoneShape( S_HAND, boneTarget, btScalar( 0.03 ) );

	boneTarget = GetBoneByName( "Skeleton_Club" );
	CreateBoneShape( S_CLUB, boneTarget, btScalar( 0.03 ) );
}

void Skeleton::CreateBoneShape( SHAPE shapeName, Bone* target, float radius ) {
	XMMATRIX transform = target->localTransform;
	XMVECTOR junk, outTrans;
	XMMatrixDecompose( &junk, &junk, &outTrans, transform );
	XMVECTOR totalLength = XMVector3Length( outTrans );
	float length;
	XMStoreFloat( &length, totalLength );
	length *= 0.01; // fbx scale

	btScalar btLength( length );
	shapeLengths[shapeName] = length;
	btConvexShape* shape = new btCapsuleShapeX( radius, length );
	physicsWorld->AddCollisionShape( shape );
	shapes[shapeName] = shape;
}

void Skeleton::CreateAllBodies() {
	Bone* bone, *to;
	btRigidBody* body;

	bone = GetBoneByName( "Skeleton_Hips" );
	to = GetBoneByName( "Skeleton_Lower_Spine" );
	body = CreateBoneBody( bone, to, shapes[S_HIPS], btScalar( 20.0 ) );
	bone->body = bodies[HIPS] = body;

	bone = GetBoneByName( "Skeleton_Lower_Spine" );
	to = GetBoneByName( "Skeleton_Upper_Spine" );
	body = CreateBoneBody( bone, to, shapes[S_LOWER_SPINE], btScalar( 8.0 ) );
	bone->body = bodies[LOWER_SPINE] = body;

	bone = GetBoneByName( "Skeleton_Upper_Spine" );
	to = GetBoneByName( "Skeleton_Neck" );
	body = CreateBoneBody( bone, to, shapes[S_UPPER_SPINE], btScalar( 12.0 ) );
	bone->body = bodies[UPPER_SPINE] = body;

	bone = GetBoneByName( "Skeleton_Neck" );
	to = GetBoneByName( "Skeleton_Head" );
	body = CreateBoneBody( bone, to, shapes[S_NECK], btScalar( 1.5 ) );
	bone->body = bodies[NECK] = body;

	bone = GetBoneByName( "Skeleton_Head" );
	to = GetBoneByName( "Skeleton_Head_Target" );
	body = CreateBoneBody( bone, to, shapes[S_HEAD], btScalar( 3.0 ) );
	bone->body = bodies[HEAD] = body;

	bone = GetBoneByName( "Skeleton_Clavicle_L" );
	to = GetBoneByName( "Skeleton_Shoulder_L" );
	body = CreateBoneBody( bone, to, shapes[S_CLAVICLE], btScalar( 1.5 ) );
	bone->body = bodies[CLAVICLE_L] = body;

	bone = GetBoneByName( "Skeleton_Clavicle_R" );
	to = GetBoneByName( "Skeleton_Shoulder_R" );
	body = CreateBoneBody( bone, to, shapes[S_CLAVICLE], btScalar( 1.5 ) );
	bone->body = bodies[CLAVICLE_R] = body;

	bone = GetBoneByName( "Skeleton_Shoulder_R" );
	to = GetBoneByName( "Skeleton_Elbow_R" );
	body = CreateBoneBody( bone, to, shapes[S_UPPER_ARM], btScalar( 3.5 ) );
	bone->body = bodies[UPPER_ARM_R] = body;

	bone = GetBoneByName( "Skeleton_Shoulder_L" );
	to = GetBoneByName( "Skeleton_Elbow_L" );
	body = CreateBoneBody( bone, to, shapes[S_UPPER_ARM], btScalar( 3.5 ) );
	bone->body = bodies[UPPER_ARM_L] = body;

	bone = GetBoneByName( "Skeleton_Elbow_R" );
	to = GetBoneByName( "Skeleton_Wrist_R" );
	body = CreateBoneBody( bone, to, shapes[S_LOWER_ARM], btScalar( 2.0 ) );
	bone->body = bodies[LOWER_ARM_R] = body;

	bone = GetBoneByName( "Skeleton_Elbow_L" );
	to = GetBoneByName( "Skeleton_Wrist_L" );
	body = CreateBoneBody( bone, to, shapes[S_LOWER_ARM], btScalar( 2.0 ) );
	bone->body = bodies[LOWER_ARM_L] = body;

	bone = GetBoneByName( "Skeleton_Wrist_L" );
	to = GetBoneByName( "Skeleton_Hand_L" );
	body = CreateBoneBody( bone, to, shapes[S_HAND], btScalar( 1.0 ) );
	bone->body = bodies[HAND_L] = body;

	bone = GetBoneByName( "Skeleton_Wrist_R" );
	to = GetBoneByName( "Skeleton_Hand_R" );
	body = CreateBoneBody( bone, to, shapes[S_HAND], btScalar( 1.0 ) );
	bone->body = bodies[HAND_R] = body;

	bone = GetBoneByName( "Skeleton_UpperLeg_R" );
	to = GetBoneByName( "Skeleton_LowerLeg_R" );
	body = CreateBoneBody( bone, to, shapes[S_UPPER_LEG], btScalar( 6.0 ) );
	bone->body = bodies[UPPER_LEG_R] = body;

	bone = GetBoneByName( "Skeleton_UpperLeg_L" );
	to = GetBoneByName( "Skeleton_LowerLeg_L" );
	body = CreateBoneBody( bone, to, shapes[S_UPPER_LEG], btScalar( 6.0 ) );
	bone->body = bodies[UPPER_LEG_L] = body;

	bone = GetBoneByName( "Skeleton_LowerLeg_R" );
	to = GetBoneByName( "Skeleton_Ankle_R" );
	body = CreateBoneBody( bone, to, shapes[S_LOWER_LEG], btScalar( 4.0 ) );
	bone->body = bodies[LOWER_LEG_R] = body;

	bone = GetBoneByName( "Skeleton_LowerLeg_L" );
	to = GetBoneByName( "Skeleton_Ankle_L" );
	body = CreateBoneBody( bone, to, shapes[S_LOWER_LEG], btScalar( 4.0 ) );
	bone->body = bodies[LOWER_LEG_L] = body;

	bone = GetBoneByName( "Skeleton_Ankle_R" );
	to = GetBoneByName( "Skeleton_Foot_R" );
	body = CreateBoneBody( bone, to, shapes[S_FOOT], btScalar( 1.0 ) );
	bone->body = bodies[FOOT_R] = body;

	bone = GetBoneByName( "Skeleton_Ankle_L" );
	to = GetBoneByName( "Skeleton_Foot_L" );
	body = CreateBoneBody( bone, to, shapes[S_FOOT], btScalar( 1.0 ) );
	bone->body = bodies[FOOT_L] = body;

	bone = GetBoneByName( "Skeleton_Hand_R" );
	to = GetBoneByName( "Skeleton_Club" );
	body = CreateBoneBody( bone, to, shapes[S_CLUB], btScalar( 3.0 ) );
	bone->body = bodies[CLUB] = body;

	// Setup some damping on the m_bodies
	for( int i = 0; i<BODY_COUNT; ++i ) {
		if( bodies[i]==nullptr ) {
			continue; //this check is only required during debug/building the skeleton
		}
		bodies[i]->setDamping( 0.1, 0.85 );
		bodies[i]->setDeactivationTime( 0.8 );
		bodies[i]->setSleepingThresholds( 1.6, 2.5 );
	}
}

btRigidBody* Skeleton::CreateBoneBody( Bone* bone, Bone* target, btConvexShape* shape, float mass ) {
	bool isDynamic = (mass!=0.f);
	btVector3 localInertia( 0, 0, 0 );
	if( isDynamic ) {
		shape->calculateLocalInertia( mass, localInertia );
	}

	XMMATRIX boneLocal = bone->localTransform;
	XMMATRIX rootXform = XMLoadFloat4x4( &rootTransform );
	XMMATRIX boneToRoot = toRoot[bone->parentIdx];
	XMMATRIX boneWorld = boneLocal*boneToRoot*rootXform;

	XMMATRIX targetLocal = target->localTransform;
	XMMATRIX targetRoot = toRoot[target->parentIdx];
	XMMATRIX targetWorld = targetLocal*targetRoot*rootXform;

	XMVECTOR boneScale;
	XMVECTOR boneRotQuat;
	XMVECTOR boneTranslate;
	XMMatrixDecompose( &boneScale, &boneRotQuat, &boneTranslate, boneWorld );

	XMVECTOR targetScale;
	XMVECTOR targetRotQuat;
	XMVECTOR targetTranslate;
	XMMatrixDecompose( &targetScale, &targetRotQuat, &targetTranslate, targetWorld );

	XMFLOAT3 head, tail, center;
	XMStoreFloat3( &head, boneTranslate );
	XMStoreFloat3( &tail, targetTranslate );
	center.x = (head.x+tail.x)/2;
	center.y = (head.y+tail.y)/2;
	center.z = (head.z+tail.z)/2;

	XMMATRIX translate = XMMatrixTranslation( center.x, center.y, center.z );
	XMMATRIX rot = XMMatrixRotationQuaternion( boneRotQuat );
	XMMATRIX scale = XMMatrixScaling( 1.f, 1.f, 1.f );
	XMMATRIX bodyTransform = scale*rot*translate;

	XMVECTOR halfLengthVec = XMVector3Length( XMVectorSubtract( targetTranslate, boneTranslate ) );
	float halfLength;
	XMStoreFloat( &halfLength, halfLengthVec );
	halfLength *= 0.5f;

	btTransform bodyToBone;
	bodyToBone.setIdentity();
	bodyToBone.setOrigin( btVector3( btScalar( -halfLength ), btScalar( 0.f ), btScalar( 0.f ) ) );
	bone->bodyToBoneOffset = bodyToBone;
	XMVECTOR headL = XMLoadFloat3( &XMFLOAT3( -halfLength, 0.f, 0.f ) );
	XMVECTOR headW = XMVector3Transform( headL, bodyTransform );
	XMFLOAT3 f_headW;
	XMStoreFloat3( &f_headW, headW );

	XMStoreFloat4x4( &(bone->initialRot), rot );
	XMStoreFloat4( &(bone->initialRotQuat), boneRotQuat );
	XMFLOAT4 f_boneRotQuat;
	XMStoreFloat4( &f_boneRotQuat, boneRotQuat );

	bone->btInitialBody = MathUtils::XMMatrixTransformToBTTransform( bodyTransform );
	bone->btInitialJointW.setOrigin( btVector3( btScalar( f_headW.x ), btScalar( f_headW.y ), btScalar( f_headW.z ) ) );
	btQuaternion btRotQuat = btQuaternion( btScalar( f_boneRotQuat.x ), btScalar( f_boneRotQuat.y ), btScalar( f_boneRotQuat.z ), btScalar( f_boneRotQuat.w ));
	bone->btInitialJointW.setRotation( btRotQuat );

	btDefaultMotionState* myMotionState = new btDefaultMotionState( bone->btInitialBody );
	btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, myMotionState, shape, localInertia );
	btRigidBody* body = new btRigidBody( rbInfo );
	short group = COLLIDE_MASK::PLAYER_BODY;
	short mask = COLLIDE_MASK::GROUND|COLLIDE_MASK::FIRE_PLINTH;
	physicsWorld->World()->addRigidBody( body, group, mask );
	return body;
}

btConeTwistConstraint* Skeleton::CreateConstraint( Bone* bone, Bone* target ) {
	XMMATRIX boneLocal = bone->localTransform;
	XMMATRIX rootXform = XMLoadFloat4x4( &rootTransform );
	XMMATRIX boneToRoot;
	if( bone->parentIdx==-1 ) {
		// The root bone
		boneToRoot = XMMatrixIdentity();
	} else {
		boneToRoot = toRoot[bone->parentIdx];
	}
	/*XMMATRIX boneWorld = boneLocal*boneToRoot*rootXform;
	XMVECTOR boneDet = XMMatrixDeterminant( boneWorld );
	XMMATRIX boneWorldInverse = XMMatrixInverse( &boneDet, boneWorld );

	XMMATRIX targetLocal = target->localTransform;
	XMMATRIX targetRoot = toRoot[target->parentIdx];
	XMMATRIX targetWorld = targetLocal*targetRoot*rootXform;
	XMVECTOR targetDet = XMMatrixDeterminant( targetWorld );
	XMMATRIX targetWorldInverse = XMMatrixInverse( &targetDet, targetWorld );

	XMVECTOR targetHeadWorld = XMLoadFloat3( &(target->headW) );
	XMVECTOR constraintBoneSpace = XMVector3Transform( targetHeadWorld, boneWorldInverse );
	XMVECTOR constraintTargetSpace = XMVector3Transform( targetHeadWorld, targetWorldInverse );
	float fbxScale = 0.01f;
	constraintBoneSpace = XMVectorScale( constraintBoneSpace, fbxScale );
	constraintTargetSpace = XMVectorScale( constraintTargetSpace, fbxScale );

	XMFLOAT3 f_constraintBoneSpace, f_constraintTargetSpace;
	XMStoreFloat3( &f_constraintBoneSpace, constraintBoneSpace );
	XMStoreFloat3( &f_constraintTargetSpace, constraintTargetSpace );*/

	btTransform boneWorldInverse = bone->btInitialBody.inverse();
	btTransform targetWorldInverse = target->btInitialBody.inverse();
	btTransform constraintWorld = target->btInitialJointW;

	btTransform constraintBoneSpace = boneWorldInverse*constraintWorld;
	btTransform constraintTargetSpace = targetWorldInverse*constraintWorld;

	btTransform localBone, localTarget;
	localBone.setIdentity(); localTarget.setIdentity();
	localBone = constraintBoneSpace;
	//localBone.setOrigin( btVector3( btScalar( f_constraintBoneSpace.x ), btScalar( f_constraintBoneSpace.y ), btScalar( f_constraintBoneSpace.z ) ) );
	//localBone.setRotation( btQuaternion( btScalar( f_targetRotBoneSpaceQuat.x ), btScalar( f_targetRotBoneSpaceQuat.y ), btScalar( f_targetRotBoneSpaceQuat.z ), btScalar( f_targetRotBoneSpaceQuat.w ) ) );
	//localBone.setRotation( btQuaternion::getIdentity() );
	//localBone.getRotation().setEulerZYX( btScalar( 0. ), btScalar( 0. ), btScalar( 0. ) );

	localTarget = constraintTargetSpace;
	//localTarget.setOrigin( btVector3( btScalar( f_constraintTargetSpace.x ), btScalar( f_constraintTargetSpace.y ), btScalar( f_constraintTargetSpace.z ) ) );
	//localTarget.setRotation( btQuaternion( btScalar( f_boneRotTargetSpaceQuat.x ), btScalar( f_boneRotTargetSpaceQuat.y ), btScalar( f_boneRotTargetSpaceQuat.z ), btScalar( f_boneRotTargetSpaceQuat.w ) ) );
	//localTarget.setRotation( btQuaternion( btScalar( f_diff.x ), btScalar( f_diff.y ), btScalar( f_diff.z ), btScalar( f_diff.w ) ) );
	//localTarget.getRotation().setEulerZYX( btScalar( 0. ), btScalar( 0. ), btScalar( 0. ) );

	btConeTwistConstraint* c = new btConeTwistConstraint( *(bone->body), *(target->body), localBone, localTarget );
	return c;
}

void Skeleton::CreateAllJoints() {
	btScalar debugSwingSpan1( 0.001 );// XM_PIDIV4;
	btScalar debugSwingSpan2( 0.001 );// XM_PIDIV2;
	btScalar debugTwistSpan( 0.003 );// XM_PIDIV2;
	btScalar debugSoftness( 1.f );
	btScalar debugBiasFactor( 0.3f );
	btScalar debugRelaxationFactor( 1.0f );

	Bone* bone, *target;
	bone = GetBoneByName( "Skeleton_Root" );
	target = GetBoneByName( "Skeleton_Hips" );
	btConeTwistConstraint* c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 0.25 );
	joints[J_ROOT_HIP] = c;
	physicsWorld->World()->addConstraint( joints[J_ROOT_HIP], true );

	bone = GetBoneByName( "Skeleton_Hips" );
	target = GetBoneByName( "Skeleton_Lower_Spine" );
	c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 0.25 );
	joints[J_HIP_LOWER_SPINE] = c;
	physicsWorld->World()->addConstraint( joints[J_HIP_LOWER_SPINE], true );

	bone = GetBoneByName( "Skeleton_Lower_Spine" );
	target = GetBoneByName( "Skeleton_Upper_Spine" );
	c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 0.25 );
	joints[J_LOWER_UPPER_SPINE] = c;
	physicsWorld->World()->addConstraint( joints[J_LOWER_UPPER_SPINE], true );

	bone = GetBoneByName( "Skeleton_Upper_Spine" );
	target = GetBoneByName( "Skeleton_Neck" );
	c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 0.25 );
	joints[J_SPINE_NECK] = c;
	physicsWorld->World()->addConstraint( joints[J_SPINE_NECK], true );

	bone = GetBoneByName( "Skeleton_Neck" );
	target = GetBoneByName( "Skeleton_Head" );
	c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 0.25 );
	joints[J_NECK_HEAD] = c;
	physicsWorld->World()->addConstraint( joints[J_NECK_HEAD], true );

	bone = GetBoneByName( "Skeleton_Upper_Spine" );
	target = GetBoneByName( "Skeleton_Clavicle_R" );
	c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 0.5 );
	joints[J_CLAVICLE_R] = c;
	physicsWorld->World()->addConstraint( joints[J_CLAVICLE_R], true );

	bone = GetBoneByName( "Skeleton_Upper_Spine" );
	target = GetBoneByName( "Skeleton_Clavicle_L" );
	c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 0.5 );
	joints[J_CLAVICLE_L] = c;
	physicsWorld->World()->addConstraint( joints[J_CLAVICLE_L], true );

	bone = GetBoneByName( "Skeleton_Clavicle_R" );
	target = GetBoneByName( "Skeleton_Shoulder_R" );
	c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 1. );
	joints[J_SHOULDER_R] = c;
	physicsWorld->World()->addConstraint( joints[J_SHOULDER_R], true );

	bone = GetBoneByName( "Skeleton_Clavicle_L" );
	target = GetBoneByName( "Skeleton_Shoulder_L" );
	c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 0.25 );
	joints[J_SHOULDER_L] = c;
	physicsWorld->World()->addConstraint( joints[J_SHOULDER_L], true );

	bone = GetBoneByName( "Skeleton_Shoulder_R" );
	target = GetBoneByName( "Skeleton_Elbow_R" );
	c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 0.25 );
	joints[J_ELBOW_R] = c;
	physicsWorld->World()->addConstraint( joints[J_ELBOW_R], true );

	bone = GetBoneByName( "Skeleton_Shoulder_L" );
	target = GetBoneByName( "Skeleton_Elbow_L" );
	c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 0.25 );
	joints[J_ELBOW_L] = c;
	physicsWorld->World()->addConstraint( joints[J_ELBOW_L], true );

	bone = GetBoneByName( "Skeleton_Elbow_R" );
	target = GetBoneByName( "Skeleton_Wrist_R" );
	c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 0.25 );
	joints[J_WRIST_R] = c;
	physicsWorld->World()->addConstraint( joints[J_WRIST_R], true );

	bone = GetBoneByName( "Skeleton_Elbow_L" );
	target = GetBoneByName( "Skeleton_Wrist_L" );
	c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 0.25 );
	joints[J_WRIST_L] = c;
	physicsWorld->World()->addConstraint( joints[J_WRIST_L], true );

	bone = GetBoneByName( "Skeleton_Wrist_R" );
	target = GetBoneByName( "Skeleton_Hand_R" );
	c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 0.25 );
	joints[J_HAND_CLUB] = c;
	physicsWorld->World()->addConstraint( joints[J_HAND_CLUB], true );

	bone = GetBoneByName( "Skeleton_Hips" );
	target = GetBoneByName( "Skeleton_UpperLeg_R" );
	c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 0.25 );
	joints[J_HIP_R] = c;
	physicsWorld->World()->addConstraint( joints[J_HIP_R], true );

	bone = GetBoneByName( "Skeleton_Hips" );
	target = GetBoneByName( "Skeleton_UpperLeg_L" );
	c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 0.25 );
	joints[J_HIP_L] = c;
	physicsWorld->World()->addConstraint( joints[J_HIP_L], true );

	bone = GetBoneByName( "Skeleton_UpperLeg_R" );
	target = GetBoneByName( "Skeleton_LowerLeg_R" );
	c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 0.25 );
	joints[J_KNEE_R] = c;
	physicsWorld->World()->addConstraint( joints[J_KNEE_R], true );

	bone = GetBoneByName( "Skeleton_UpperLeg_L" );
	target = GetBoneByName( "Skeleton_LowerLeg_L" );
	c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 0.25 );
	joints[J_KNEE_L] = c;
	physicsWorld->World()->addConstraint( joints[J_KNEE_L], true );

	bone = GetBoneByName( "Skeleton_LowerLeg_R" );
	target = GetBoneByName( "Skeleton_Ankle_R" );
	c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 0.25 );
	joints[J_ANKLE_R] = c;
	physicsWorld->World()->addConstraint( joints[J_ANKLE_R], true );

	bone = GetBoneByName( "Skeleton_LowerLeg_L" );
	target = GetBoneByName( "Skeleton_Ankle_L" );
	c = CreateConstraint( bone, target );
	c->setLimit(
		debugSwingSpan1,		// Swing Span 1
		debugSwingSpan2,		// Swing Span 2
		debugTwistSpan,			// Twist Span
		debugSoftness,			// Softness
		debugBiasFactor,		// Bias Factor
		debugRelaxationFactor	// Relaxation Factor 
		);
	c->setDbgDrawSize( 0.25 );
	joints[J_ANKLE_L] = c;
	physicsWorld->World()->addConstraint( joints[J_ANKLE_L], true );
}

btRigidBody* Skeleton::localCreateRigidBody( float mass, const btTransform& startTransform, btCollisionShape* shape ) {
	bool isDynamic = (mass!=0.f);
	btVector3 localInertia( 0, 0, 0 );
	if( isDynamic )
		shape->calculateLocalInertia( mass, localInertia );

	btDefaultMotionState* myMotionState = new btDefaultMotionState( startTransform );
	btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, myMotionState, shape, localInertia );
	btRigidBody* body = new btRigidBody( rbInfo );
	physicsWorld->World()->addRigidBody( body );
	return body;
}

void Skeleton::CreateDemoRagDoll() {
	float M_PI = XM_PI;
	float M_PI_2 = XM_PIDIV2;
	float M_PI_4 = XM_PIDIV4;
	float CONSTRAINT_DEBUG_SIZE = 0.2;
	enum {
		BODYPART_PELVIS = 0,
		BODYPART_SPINE,
		BODYPART_HEAD,

		BODYPART_LEFT_UPPER_LEG,
		BODYPART_LEFT_LOWER_LEG,

		BODYPART_RIGHT_UPPER_LEG,
		BODYPART_RIGHT_LOWER_LEG,

		BODYPART_LEFT_UPPER_ARM,
		BODYPART_LEFT_LOWER_ARM,

		BODYPART_RIGHT_UPPER_ARM,
		BODYPART_RIGHT_LOWER_ARM,

		BODYPART_COUNT
	};

	enum {
		JOINT_PELVIS_SPINE = 0,
		JOINT_SPINE_HEAD,

		JOINT_LEFT_HIP,
		JOINT_LEFT_KNEE,

		JOINT_RIGHT_HIP,
		JOINT_RIGHT_KNEE,

		JOINT_LEFT_SHOULDER,
		JOINT_LEFT_ELBOW,

		JOINT_RIGHT_SHOULDER,
		JOINT_RIGHT_ELBOW,

		TEST_JOINT_COUNT
	};
	btDynamicsWorld* m_ownerWorld = physicsWorld->World();
	btCollisionShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];
	btTypedConstraint* m_joints[TEST_JOINT_COUNT];

	// Setup the geometry
	m_shapes[BODYPART_PELVIS] = new btCapsuleShape( btScalar( 0.15 ), btScalar( 0.20 ) );
	m_shapes[BODYPART_SPINE] = new btCapsuleShape( btScalar( 0.15 ), btScalar( 0.28 ) );
	m_shapes[BODYPART_HEAD] = new btCapsuleShape( btScalar( 0.10 ), btScalar( 0.05 ) );
	m_shapes[BODYPART_LEFT_UPPER_LEG] = new btCapsuleShape( btScalar( 0.07 ), btScalar( 0.45 ) );
	m_shapes[BODYPART_LEFT_LOWER_LEG] = new btCapsuleShape( btScalar( 0.05 ), btScalar( 0.37 ) );
	m_shapes[BODYPART_RIGHT_UPPER_LEG] = new btCapsuleShape( btScalar( 0.07 ), btScalar( 0.45 ) );
	m_shapes[BODYPART_RIGHT_LOWER_LEG] = new btCapsuleShape( btScalar( 0.05 ), btScalar( 0.37 ) );
	m_shapes[BODYPART_LEFT_UPPER_ARM] = new btCapsuleShape( btScalar( 0.05 ), btScalar( 0.33 ) );
	m_shapes[BODYPART_LEFT_LOWER_ARM] = new btCapsuleShape( btScalar( 0.04 ), btScalar( 0.25 ) );
	m_shapes[BODYPART_RIGHT_UPPER_ARM] = new btCapsuleShape( btScalar( 0.05 ), btScalar( 0.33 ) );
	m_shapes[BODYPART_RIGHT_LOWER_ARM] = new btCapsuleShape( btScalar( 0.04 ), btScalar( 0.25 ) );

	// Setup all the rigid bodies
	btTransform offset; offset.setIdentity();
	offset.setOrigin( btVector3( 0., 5., 0. ) );

	btTransform transform;
	transform.setIdentity();
	transform.setOrigin( btVector3( btScalar( 0. ), btScalar( 1. ), btScalar( 0. ) ) );
	m_bodies[BODYPART_PELVIS] = localCreateRigidBody( btScalar( 1. ), offset*transform, m_shapes[BODYPART_PELVIS] );

	transform.setIdentity();
	transform.setOrigin( btVector3( btScalar( 0. ), btScalar( 1.2 ), btScalar( 0. ) ) );
	m_bodies[BODYPART_SPINE] = localCreateRigidBody( btScalar( 1. ), offset*transform, m_shapes[BODYPART_SPINE] );

	transform.setIdentity();
	transform.setOrigin( btVector3( btScalar( 0. ), btScalar( 1.6 ), btScalar( 0. ) ) );
	m_bodies[BODYPART_HEAD] = localCreateRigidBody( btScalar( 1. ), offset*transform, m_shapes[BODYPART_HEAD] );

	transform.setIdentity();
	transform.setOrigin( btVector3( btScalar( -0.18 ), btScalar( 0.65 ), btScalar( 0. ) ) );
	m_bodies[BODYPART_LEFT_UPPER_LEG] = localCreateRigidBody( btScalar( 1. ), offset*transform, m_shapes[BODYPART_LEFT_UPPER_LEG] );

	transform.setIdentity();
	transform.setOrigin( btVector3( btScalar( -0.18 ), btScalar( 0.2 ), btScalar( 0. ) ) );
	m_bodies[BODYPART_LEFT_LOWER_LEG] = localCreateRigidBody( btScalar( 1. ), offset*transform, m_shapes[BODYPART_LEFT_LOWER_LEG] );

	transform.setIdentity();
	transform.setOrigin( btVector3( btScalar( 0.18 ), btScalar( 0.65 ), btScalar( 0. ) ) );
	m_bodies[BODYPART_RIGHT_UPPER_LEG] = localCreateRigidBody( btScalar( 1. ), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_LEG] );

	transform.setIdentity();
	transform.setOrigin( btVector3( btScalar( 0.18 ), btScalar( 0.2 ), btScalar( 0. ) ) );
	m_bodies[BODYPART_RIGHT_LOWER_LEG] = localCreateRigidBody( btScalar( 1. ), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_LEG] );

	transform.setIdentity();
	transform.setOrigin( btVector3( btScalar( -0.35 ), btScalar( 1.45 ), btScalar( 0. ) ) );
	transform.getBasis().setEulerZYX( 0, 0, M_PI_2 );
	m_bodies[BODYPART_LEFT_UPPER_ARM] = localCreateRigidBody( btScalar( 1. ), offset*transform, m_shapes[BODYPART_LEFT_UPPER_ARM] );

	transform.setIdentity();
	transform.setOrigin( btVector3( btScalar( -0.7 ), btScalar( 1.45 ), btScalar( 0. ) ) );
	transform.getBasis().setEulerZYX( 0, 0, M_PI_2 );
	m_bodies[BODYPART_LEFT_LOWER_ARM] = localCreateRigidBody( btScalar( 1. ), offset*transform, m_shapes[BODYPART_LEFT_LOWER_ARM] );

	transform.setIdentity();
	transform.setOrigin( btVector3( btScalar( 0.35 ), btScalar( 1.45 ), btScalar( 0. ) ) );
	transform.getBasis().setEulerZYX( 0, 0, -M_PI_2 );
	m_bodies[BODYPART_RIGHT_UPPER_ARM] = localCreateRigidBody( btScalar( 1. ), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_ARM] );

	transform.setIdentity();
	transform.setOrigin( btVector3( btScalar( 0.7 ), btScalar( 1.45 ), btScalar( 0. ) ) );
	transform.getBasis().setEulerZYX( 0, 0, -M_PI_2 );
	m_bodies[BODYPART_RIGHT_LOWER_ARM] = localCreateRigidBody( btScalar( 1. ), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_ARM] );

	// Setup some damping on the m_bodies
	for( int i = 0; i<BODYPART_COUNT; ++i ) {
		m_bodies[i]->setDamping( 0.05, 0.85 );
		m_bodies[i]->setDeactivationTime( 0.8 );
		m_bodies[i]->setSleepingThresholds( 1.6, 2.5 );
	}

	// Now setup the constraints
	btHingeConstraint* hingeC;
	btConeTwistConstraint* coneC;

	btTransform localA, localB;

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX( 0, M_PI_2, 0 ); localA.setOrigin( btVector3( btScalar( 0. ), btScalar( 0.15 ), btScalar( 0. ) ) );
	localB.getBasis().setEulerZYX( 0, M_PI_2, 0 ); localB.setOrigin( btVector3( btScalar( 0. ), btScalar( -0.15 ), btScalar( 0. ) ) );
	hingeC = new btHingeConstraint( *m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_SPINE], localA, localB );
	hingeC->setLimit( btScalar( -M_PI_4 ), btScalar( M_PI_2 ) );
	m_joints[JOINT_PELVIS_SPINE] = hingeC;
	hingeC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	m_ownerWorld->addConstraint( m_joints[JOINT_PELVIS_SPINE], true );


	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX( 0, 0, M_PI_2 ); localA.setOrigin( btVector3( btScalar( 0. ), btScalar( 0.30 ), btScalar( 0. ) ) );
	localB.getBasis().setEulerZYX( 0, 0, M_PI_2 ); localB.setOrigin( btVector3( btScalar( 0. ), btScalar( -0.14 ), btScalar( 0. ) ) );
	coneC = new btConeTwistConstraint( *m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_HEAD], localA, localB );
	coneC->setLimit( M_PI_4, M_PI_4, M_PI_2 );
	m_joints[JOINT_SPINE_HEAD] = coneC;
	coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	m_ownerWorld->addConstraint( m_joints[JOINT_SPINE_HEAD], true );


	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX( 0, 0, -M_PI_4*5 ); localA.setOrigin( btVector3( btScalar( -0.18 ), btScalar( -0.10 ), btScalar( 0. ) ) );
	localB.getBasis().setEulerZYX( 0, 0, -M_PI_4*5 ); localB.setOrigin( btVector3( btScalar( 0. ), btScalar( 0.225 ), btScalar( 0. ) ) );
	coneC = new btConeTwistConstraint( *m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_LEFT_UPPER_LEG], localA, localB );
	coneC->setLimit( M_PI_4, M_PI_4, 0 );
	m_joints[JOINT_LEFT_HIP] = coneC;
	coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	m_ownerWorld->addConstraint( m_joints[JOINT_LEFT_HIP], true );

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX( 0, M_PI_2, 0 ); localA.setOrigin( btVector3( btScalar( 0. ), btScalar( -0.225 ), btScalar( 0. ) ) );
	localB.getBasis().setEulerZYX( 0, M_PI_2, 0 ); localB.setOrigin( btVector3( btScalar( 0. ), btScalar( 0.185 ), btScalar( 0. ) ) );
	hingeC = new btHingeConstraint( *m_bodies[BODYPART_LEFT_UPPER_LEG], *m_bodies[BODYPART_LEFT_LOWER_LEG], localA, localB );
	hingeC->setLimit( btScalar( 0 ), btScalar( M_PI_2 ) );
	m_joints[JOINT_LEFT_KNEE] = hingeC;
	hingeC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	m_ownerWorld->addConstraint( m_joints[JOINT_LEFT_KNEE], true );


	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX( 0, 0, M_PI_4 ); localA.setOrigin( btVector3( btScalar( 0.18 ), btScalar( -0.10 ), btScalar( 0. ) ) );
	localB.getBasis().setEulerZYX( 0, 0, M_PI_4 ); localB.setOrigin( btVector3( btScalar( 0. ), btScalar( 0.225 ), btScalar( 0. ) ) );
	coneC = new btConeTwistConstraint( *m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_RIGHT_UPPER_LEG], localA, localB );
	coneC->setLimit( M_PI_4, M_PI_4, 0 );
	m_joints[JOINT_RIGHT_HIP] = coneC;
	coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	m_ownerWorld->addConstraint( m_joints[JOINT_RIGHT_HIP], true );

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX( 0, M_PI_2, 0 ); localA.setOrigin( btVector3( btScalar( 0. ), btScalar( -0.225 ), btScalar( 0. ) ) );
	localB.getBasis().setEulerZYX( 0, M_PI_2, 0 ); localB.setOrigin( btVector3( btScalar( 0. ), btScalar( 0.185 ), btScalar( 0. ) ) );
	hingeC = new btHingeConstraint( *m_bodies[BODYPART_RIGHT_UPPER_LEG], *m_bodies[BODYPART_RIGHT_LOWER_LEG], localA, localB );
	hingeC->setLimit( btScalar( 0 ), btScalar( M_PI_2 ) );
	m_joints[JOINT_RIGHT_KNEE] = hingeC;
	hingeC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	m_ownerWorld->addConstraint( m_joints[JOINT_RIGHT_KNEE], true );


	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX( 0, 0, M_PI ); localA.setOrigin( btVector3( btScalar( -0.2 ), btScalar( 0.15 ), btScalar( 0. ) ) );
	localB.getBasis().setEulerZYX( 0, 0, M_PI_2 ); localB.setOrigin( btVector3( btScalar( 0. ), btScalar( -0.18 ), btScalar( 0. ) ) );
	coneC = new btConeTwistConstraint( *m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_LEFT_UPPER_ARM], localA, localB );
	coneC->setLimit( M_PI_2, M_PI_2, 0 );
	coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	m_joints[JOINT_LEFT_SHOULDER] = coneC;
	m_ownerWorld->addConstraint( m_joints[JOINT_LEFT_SHOULDER], true );

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX( 0, M_PI_2, 0 ); localA.setOrigin( btVector3( btScalar( 0. ), btScalar( 0.18 ), btScalar( 0. ) ) );
	localB.getBasis().setEulerZYX( 0, M_PI_2, 0 ); localB.setOrigin( btVector3( btScalar( 0. ), btScalar( -0.14 ), btScalar( 0. ) ) );
	hingeC = new btHingeConstraint( *m_bodies[BODYPART_LEFT_UPPER_ARM], *m_bodies[BODYPART_LEFT_LOWER_ARM], localA, localB );
	//		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
	hingeC->setLimit( btScalar( 0 ), btScalar( M_PI_2 ) );
	m_joints[JOINT_LEFT_ELBOW] = hingeC;
	hingeC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	m_ownerWorld->addConstraint( m_joints[JOINT_LEFT_ELBOW], true );



	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX( 0, 0, 0 ); localA.setOrigin( btVector3( btScalar( 0.2 ), btScalar( 0.15 ), btScalar( 0. ) ) );
	localB.getBasis().setEulerZYX( 0, 0, M_PI_2 ); localB.setOrigin( btVector3( btScalar( 0. ), btScalar( -0.18 ), btScalar( 0. ) ) );
	coneC = new btConeTwistConstraint( *m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_RIGHT_UPPER_ARM], localA, localB );
	coneC->setLimit( M_PI_2, M_PI_2, 0 );
	m_joints[JOINT_RIGHT_SHOULDER] = coneC;
	coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	m_ownerWorld->addConstraint( m_joints[JOINT_RIGHT_SHOULDER], true );

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX( 0, M_PI_2, 0 ); localA.setOrigin( btVector3( btScalar( 0. ), btScalar( 0.18 ), btScalar( 0. ) ) );
	localB.getBasis().setEulerZYX( 0, M_PI_2, 0 ); localB.setOrigin( btVector3( btScalar( 0. ), btScalar( -0.14 ), btScalar( 0. ) ) );
	hingeC = new btHingeConstraint( *m_bodies[BODYPART_RIGHT_UPPER_ARM], *m_bodies[BODYPART_RIGHT_LOWER_ARM], localA, localB );
	//		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
	hingeC->setLimit( btScalar( 0 ), btScalar( M_PI_2 ) );
	m_joints[JOINT_RIGHT_ELBOW] = hingeC;
	hingeC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	m_ownerWorld->addConstraint( m_joints[JOINT_RIGHT_ELBOW], true );
}

btTransform XM_CALLCONV Skeleton::XMMatrixToBTTransform( FXMMATRIX m, bool fbxCorrection ) {
	XMVECTOR outScale;
	XMVECTOR outRotQuat;
	XMVECTOR outTrans;
	XMMatrixDecompose( &outScale, &outRotQuat, &outTrans, m );
	btTransform t = btTransform();
	t.setIdentity();
	if( fbxCorrection ) {
		outTrans = XMVectorScale( outTrans, 0.01f ); //FBX scale
	}
	t.setOrigin( btVector3( btScalar( outTrans.m128_f32[0] ), btScalar( outTrans.m128_f32[1] ), btScalar( outTrans.m128_f32[2] ) ) );
	btQuaternion rotQuat( btScalar( outRotQuat.m128_f32[0] ), btScalar( outRotQuat.m128_f32[1] ), btScalar( outRotQuat.m128_f32[2] ), btScalar( outRotQuat.m128_f32[3] ) );
	t.setRotation( rotQuat );
	return t;
}

void Skeleton::InitMotorData() {

	float debugMotorImpulse = 10.f;

	void* ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	MotorData* motor = new(ptr)MotorData();
	Bone* bone = GetBoneByName( "Skeleton_Hips" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_ROOT_HIP] = motor;

	ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_Lower_Spine" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_HIP_LOWER_SPINE] = motor;

	ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_Upper_Spine" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_LOWER_UPPER_SPINE] = motor;

	ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_Neck" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_SPINE_NECK] = motor;

	ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_Head" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_NECK_HEAD] = motor;

	ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_Clavicle_R" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_CLAVICLE_R] = motor;

	ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_Clavicle_L" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_CLAVICLE_L] = motor;

	ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_Shoulder_R" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_SHOULDER_R] = motor;

	ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_Shoulder_L" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_SHOULDER_L] = motor;

	ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_Elbow_R" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_ELBOW_R] = motor;

	ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_Elbow_L" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_ELBOW_L] = motor;

	ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_Wrist_R" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_WRIST_R] = motor;

	ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_Wrist_L" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_WRIST_L] = motor;

	ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_Hand_R" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_HAND_CLUB] = motor;

	ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_UpperLeg_L" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_HIP_L] = motor;

	ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_UpperLeg_R" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_HIP_R] = motor;

	ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_LowerLeg_L" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_KNEE_L] = motor;

	ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_LowerLeg_R" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_KNEE_R] = motor;

	ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_Ankle_L" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_ANKLE_L] = motor;

	ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_Ankle_R" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_ANKLE_R] = motor;

	/*ptr = btAlignedAlloc( sizeof( MotorData ), 16 );
	motor = new(ptr)MotorData();
	bone = GetBoneByName( "Skeleton_Club" );
	motor->boneIdx = bone->idx;
	motor->motorEnabled = true;
	motor->maxMotorImpulse = debugMotorImpulse;
	motor->currentMotorImpulse = 0.f;
	motor->rotTarget = btQuaternion();
	motor->rotFactor = 1.f;
	motor->velocityFactor = 1.f;
	motors[J_CLUB] = motor;*/
}

void Skeleton::UpdateMotorData() {
	for( int i = 0; i<DEBUG_JOINT_COUNT_LIMIT /*FIXME -- JOINT_COUNT*/; ++i ) {
		// Set Target
		MotorData* motor = motors[i];
		Bone* bone = GetBoneByIndex( motor->boneIdx );
		//XMMATRIX localTransform = bone->localTransform;
		//btTransform btLocal = XMMatrixToBTTransform( localTransform, false );

		XMFLOAT4 animQuat = animationController->GetBoneRotation( bone );
		btQuaternion btQuat( animQuat.x, animQuat.y, animQuat.z, animQuat.w );

		motor->rotTarget = btQuat;

		btScalar jointVelocity = joints[i]->getRigidBodyB().getAngularVelocity().length();
		btScalar animVelocity = animationController->GetBoneVelocity( bone );

		btScalar velFactor = btFabs( animVelocity-jointVelocity );

		btQuaternion rotA = joints[i]->getRigidBodyB().getCenterOfMassTransform().getRotation();
		XMFLOAT4X4 boneXform = finalTransformData[bone->idx];
		XMMATRIX xmBoneTransform = XMLoadFloat4x4( &boneXform );
		btTransform boneTransform = XMMatrixToBTTransform( xmBoneTransform, false );
		btScalar angleFactor = rotA.dot( boneTransform.getRotation() );
		angleFactor = 1./angleFactor;

		btScalar finalVelFactor = velFactor * motor->velocityFactor;
		btScalar finalAngleFactor = angleFactor * motor->rotFactor;
		btScalar totalFactor = btClamped( btScalar( (finalVelFactor+finalAngleFactor) * motor->maxMotorImpulse ), btScalar( 0. ), motor->maxMotorImpulse );

		motor->currentMotorImpulse = totalFactor;
	}
}

void Skeleton::SetAllMotors( float dt ) {
	for( int i = 0; i<DEBUG_JOINT_COUNT_LIMIT /*FIXME -- JOINT_COUNT*/; ++i ) {
		MotorData* motor = motors[i];
		btConeTwistConstraint* joint = static_cast<btConeTwistConstraint*>(joints[i]);
		joint->setMaxMotorImpulseNormalized( motor->currentMotorImpulse );
		joint->setMotorTargetInConstraintSpace( motor->rotTarget );
		//joint->enableMotor( motor->motorEnabled );
	}
}


void Skeleton::DirtyBones() {
	for( auto boneIt = idxBones.begin(); boneIt!=idxBones.end(); ++boneIt ) {
		boneIt->second->dirty = true;
	}
}
/*
btVector3 RagdollDemo::pointWorldToLocal( int bodyIndex, btVector3 worldPoint ) {
return body[bodyIndex]->getCenterOfMassTransform().inverse()(worldPoint);
}*/