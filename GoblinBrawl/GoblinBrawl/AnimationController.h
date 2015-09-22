#pragma once
#include <map>
#include <unordered_set>
#include "DirectX_11_1_Includes.h"

struct Anim;
struct Bone;
class Skeleton;

enum ANIM_NAME {
	ANIM_TEST,
	ANIM_IDLE,
	ANIM_WALK,
	ANIM_JUMP,
	ANIM_ATTACK,
	_ANIM_COUNT
};

class AnimationController {
public:
	AnimationController();
	~AnimationController();
	void SetSkeleton( Skeleton* skeleton );
	void AddAnim(Anim* anim);
	void ChangeAnim( ANIM_NAME anim );
	void Interpolate( float dt );
	float GetAnimTime(ANIM_NAME anim);
	DirectX::XMFLOAT4 GetBoneRotation( Bone* bone);
	DirectX::XMFLOAT4X4 GetBoneTransform( Bone* bone );
	float GetBoneVelocity( Bone* bone );
private:
	float									timeSinceStart;
	Anim*									anims[_ANIM_COUNT];
	ANIM_NAME								currentAnim;
	std::map<Bone*, DirectX::XMFLOAT4X4>	finalTransform;
	std::map<Bone*, float>					finalVelocityFactor;
	Skeleton*								skeleton;
};

typedef std::map<float, DirectX::XMFLOAT4> keySet_t; // This is <Time, Quaternion>

struct Anim {
public:
	std::unordered_set<Bone*>		boneSet;
	std::string						name;
	float							totalTime;
	std::map<Bone*, keySet_t>		rotChannels;
	std::map<Bone*, keySet_t>		posChannels;
	std::map<Bone*, keySet_t>		scaleChannels;
};