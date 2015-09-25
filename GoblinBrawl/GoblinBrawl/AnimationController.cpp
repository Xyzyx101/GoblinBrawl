#include "stdafx.h"
#include "AnimationController.h"
#include "Skeleton.h"

using namespace DirectX;

AnimationController::AnimationController() {}

AnimationController::~AnimationController() {}

void AnimationController::SetSkeleton( Skeleton* skeleton ) {
	this->skeleton = skeleton;
}

void AnimationController::AddAnim( Anim* anim ) {
	std::string name = anim->name;
	if( name=="Test" ) {
		anims[ANIM_TEST] = anim;
	} else if( name=="Idle" ) {
		anims[ANIM_IDLE] = anim;
	} else if( name=="Walk" ) {
		anims[ANIM_WALK] = anim;
	} else if( name=="Attack" ) {
		anims[ANIM_ATTACK] = anim;
	} else if( name=="Jump" ) {
		anims[ANIM_JUMP] = anim;
	} else {
		fprintf( stderr, "Bad Animation name : %s", name );
	}
}

void AnimationController::ChangeAnim( ANIM_NAME newAnim ) {
	if( currentAnim==newAnim ) {
		return;
	}
	timeSinceStart = 0.f;
	currentAnim = newAnim;
}


float AnimationController::GetAnimTime( ANIM_NAME anim ) {
	return anims[anim]->totalTime;
}

void AnimationController::Interpolate( float dt ) {
	timeSinceStart += dt;
	Anim* anim = anims[currentAnim];
	float animCurrentTime = fmod( timeSinceStart, anim->totalTime );
	for( Bone* bone:anim->boneSet ) {
		XMMATRIX rotMat, scaleMat, translateMat;

		// Interpolate Rotation
		auto rotSetIt = anim->rotChannels.find( bone );
		if( rotSetIt==anim->rotChannels.end() ) {
			rotMat = XMMatrixIdentity();
		} else {
			keySet_t rotKeySet = rotSetIt->second;
			auto itLow = rotKeySet.lower_bound( animCurrentTime );
			if( itLow==rotKeySet.begin() ) {
				itLow = rotKeySet.end();
			}
			--itLow;
			auto itHigh = rotKeySet.upper_bound( animCurrentTime );
			if( itHigh==rotKeySet.end() ) {
				itHigh = rotKeySet.begin();
			}
			float factor = (animCurrentTime-itLow->first)/(itHigh->first-itLow->first);
			XMVECTOR low = XMLoadFloat4( &itLow->second );
			XMVECTOR high = XMLoadFloat4( &itHigh->second );
			XMVECTOR interp = XMQuaternionSlerp( low, high, factor );
			XMVECTOR normalized = XMQuaternionNormalize( interp );
			rotMat = XMMatrixRotationQuaternion( interp );


			XMVECTOR xmVelocity = XMQuaternionDot( low, high );
			float time = itHigh->first-itLow->first;
			float velocityFactor;
			if( time>0 ) {
				XMStoreFloat( &velocityFactor, xmVelocity );
				velocityFactor = (1.f-velocityFactor)/time;

			} else {
				velocityFactor = 0.f;
			}
			finalVelocityFactor[bone] = velocityFactor;
		}

		// Interpolate Scale
		auto scaleSetIt = anim->scaleChannels.find( bone );
		if( scaleSetIt==anim->scaleChannels.end() ) {
			scaleMat = XMMatrixIdentity();
		} else {
			keySet_t scaleKeySet = scaleSetIt->second;
			auto itLow = scaleKeySet.lower_bound( animCurrentTime );
			if( itLow==scaleKeySet.begin() ) {
				itLow = scaleKeySet.end();
			}
			--itLow;
			auto itHigh = scaleKeySet.upper_bound( animCurrentTime );
			if( itHigh==scaleKeySet.end() ) {
				itHigh = scaleKeySet.begin();
			}
			float factor = (animCurrentTime-itLow->first)/(itHigh->first-itLow->first);
			XMFLOAT4 lowVec = itLow->second;
			XMFLOAT4 highVec = itHigh->second;
			scaleMat = XMMatrixScaling(
				lowVec.x+factor*(highVec.x-lowVec.x),
				lowVec.y+factor*(highVec.y-lowVec.y),
				lowVec.z+factor*(highVec.z-lowVec.z) );
		}

		// Interpolate Position
		auto posSetIt = anim->posChannels.find( bone );
		if( posSetIt==anim->posChannels.end() ) {
			translateMat = XMMatrixIdentity();
		} else {
			keySet_t posKeySet = posSetIt->second;
			auto itLow = posKeySet.lower_bound( animCurrentTime );
			if( itLow==posKeySet.begin() ) {
				itLow = posKeySet.end();
			}
			--itLow;
			auto itHigh = posKeySet.upper_bound( animCurrentTime );
			if( itHigh==posKeySet.end() ) {
				itHigh = posKeySet.begin();
			}
			float factor = (animCurrentTime-itLow->first)/(itHigh->first-itLow->first);
			XMFLOAT4 lowVec = itLow->second;
			XMFLOAT4 highVec = itHigh->second;
			translateMat = XMMatrixTranslation(
				lowVec.x+factor*(highVec.x-lowVec.x),
				lowVec.y+factor*(highVec.y-lowVec.y),
				lowVec.z+factor*(highVec.z-lowVec.z) );
		}

		// Conversion Matrix - this converts imported coords to DirectX
		XMMATRIX reflectX = XMMatrixReflect( XMLoadFloat3( &XMFLOAT3( 1.f, 0.f, 0.f ) ) );
		XMMATRIX reflectY = XMMatrixReflect( XMLoadFloat3( &XMFLOAT3( 0.f, 1.f, 0.f ) ) );
		XMMATRIX reflectZ = XMMatrixReflect( XMLoadFloat3( &XMFLOAT3( 0.f, 0.f, 1.f ) ) );
		XMMATRIX rotX = XMMatrixRotationX( XM_PIDIV2 );
		XMMATRIX rotY = XMMatrixRotationY( XM_PIDIV2 );
		XMMATRIX rotZ = XMMatrixRotationZ( XM_PIDIV2 );
		int foo = 17; //DELETEME
		XMMATRIX flip = XMLoadFloat4x4( &XMFLOAT4X4(
			1.f, 0.f, 0.f, 0.f,
			0.f, 1.f, 0.f, 0.f,
			0.f, 0.f, 1.f, 0.f,
			0.f, 0.f, 0.f, 1.f
			) );

		XMMATRIX finalMat = scaleMat * rotMat * translateMat;
		XMFLOAT4X4 transform;
		XMStoreFloat4x4( &transform, finalMat );
		finalTransform[bone] = transform;
	}
}

DirectX::XMFLOAT4X4 AnimationController::GetBoneTransform( Bone* bone ) {
	auto it = finalTransform.find( bone );
	if( it==finalTransform.end() ) {
		fprintf( stderr, "Bone Rotation not found : %s\n", bone->name.c_str() );
		return XMFLOAT4X4(
			1.f, 0.f, 0.f, 0.f,
			0.f, 1.f, 0.f, 0.f,
			0.f, 0.f, 1.f, 0.f,
			0.f, 0.f, 0.f, 1.f );
	}
	return it->second;
}

DirectX::XMFLOAT4 AnimationController::GetBoneRotation( Bone* bone ) {
	XMFLOAT4X4 xform = GetBoneTransform( bone );
	XMMATRIX xmXform = XMLoadFloat4x4( &xform );

	DirectX::XMVECTOR junk;
	DirectX::XMVECTOR outRotQuat;
	XMMatrixDecompose( &junk, &outRotQuat, &junk, xmXform );
	XMFLOAT4 quat;
	XMStoreFloat4( &quat, outRotQuat );
	return quat;
}

float AnimationController::GetBoneVelocity( Bone* bone ) {
	auto it = finalVelocityFactor.find( bone );
	if( it==finalVelocityFactor.end() ) {
		return 0.f;
	}
	return it->second;
}