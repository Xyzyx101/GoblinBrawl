#pragma once
#include <string>
#include <map>
#include <vector>
#include "DirectX_11_1_Includes.h"
#include "btBulletDynamicsCommon.h"


class PhysicsWorld;
class btCapsuleShape;
class btRigidBody;
class btConvexShape;
class btTypedConstraint;
class btCollisionShape;
class btTransform;
class AnimationController;

__declspec(align(16)) struct Bone {
	Bone::Bone() {
		localTransform = DirectX::XMMatrixIdentity();
	}
	int									idx;
	int									parentIdx;
	DirectX::XMMATRIX					offset;
	DirectX::XMMATRIX					localTransform;
	std::vector<Bone*>					children;
	btRigidBody*						body;
	std::vector<btTypedConstraint*>		joints;
	std::string							name;
};


// TODO I think I can scrap this struct
__declspec(align(16)) struct JointInfo {
	btVector3	fromOffset;
	btVector3	toOffset;
	btScalar	aRotX;
	btScalar	aRotY;
	btScalar	aRotZ;
	btScalar	bRotX;
	btScalar	bRotY;
	btScalar	bRotZ;
	btScalar	swingLimit1;
	btScalar	swingLimit2;
	btScalar	twistLimit;
};

__declspec(align(16)) struct MotorData {
	int				boneIdx;
	bool			motorEnabled;
	btScalar		maxMotorImpulse;
	btScalar		currentMotorImpulse;
	btQuaternion	rotTarget;
	float			rotFactor;
	float			velocityFactor;
};

class Skeleton {
public:
	enum SHAPE {
		S_HIPS,
		S_LOWER_SPINE,
		S_UPPER_SPINE,
		S_NECK,
		S_HEAD,
		S_UPPER_LEG,
		S_LOWER_LEG,
		S_FOOT,
		S_CLAVICLE,
		S_UPPER_ARM,
		S_LOWER_ARM,
		S_HAND,
		S_CLUB,
		SHAPE_COUNT
	};
	enum BODY {
		HIPS,
		LOWER_SPINE,
		UPPER_SPINE,
		NECK,
		HEAD,
		UPPER_LEG_R,
		UPPER_LEG_L,
		LOWER_LEG_R,
		LOWER_LEG_L,
		FOOT_R,
		FOOT_L,
		CLAVICLE_R,
		CLAVICLE_L,
		UPPER_ARM_R,
		UPPER_ARM_L,
		LOWER_ARM_R,
		LOWER_ARM_L,
		HAND_R,
		HAND_L,
		CLUB,
		BODY_COUNT
	};
	enum JOINT {
		J_ROOT_HIP,
		J_HIP_LOWER_SPINE,
		J_LOWER_UPPER_SPINE,
		J_SPINE_NECK,
		J_NECK_HEAD,
		J_CLAVICLE_R,
		J_CLAVICLE_L,
		J_SHOULDER_R,
		J_SHOULDER_L,
		J_ELBOW_R,
		J_ELBOW_L,
		J_WRIST_R,
		J_WRIST_L,
		J_HAND_CLUB,
		J_HIP_L,
		J_HIP_R,
		J_KNEE_L,
		J_KNEE_R,
		J_ANKLE_L,
		J_ANKLE_R,
		J_CLUB,
		JOINT_COUNT
	};

	Skeleton();
	~Skeleton();
	void AddBone( Bone* newBone );
	void XM_CALLCONV UpdateTransformByName( DirectX::FXMVECTOR translate, DirectX::FXMVECTOR rotQuat, DirectX::FXMVECTOR scale, std::string name );
	void XM_CALLCONV UpdateTransformByIndex( DirectX::FXMVECTOR translate, DirectX::FXMVECTOR rotQuat, DirectX::FXMVECTOR scale, int index );
	DirectX::XMFLOAT4X4* XM_CALLCONV GetFinalTransforms();
	Bone* GetBoneByName( std::string name );
	Bone* GetBoneByIndex( int index );
	int BoneCount() { return numBones; };
	void XM_CALLCONV InitPhysics( PhysicsWorld* physicsWorld );
	void XM_CALLCONV SetRootTransform( DirectX::FXMMATRIX transform );
	void CreateDemoRagDoll();
	btRigidBody* localCreateRigidBody( float mass, const btTransform& startTransform, btCollisionShape* shape );
	void SetAnimationController( AnimationController* animationController );
	void Update( float dt );
private:
	void UpdateLocalTransforms();
	void UpdateLocalTransformsFromRagdoll();
	void UpdateTransforms( Bone* bone );
	void CreateAllShapes();
	void CreateAllBodies();
	VOID CreateAllJoints();
	void CreateBoneShape( SHAPE shape, Bone* target, float radius );
	btRigidBody* CreateBoneBody( Bone* bone, btConvexShape* shape, float mass, btScalar xRot, btScalar yRot, btScalar zRot );
	//void CreateConstraint( JOINT joint, Bone* from, Bone* to, btVector3 fromOffset, btVector3 toOffset, btScalar aRotX, btScalar aRotY, btScalar aRotZ, btScalar bRotX, btScalar bRotY, btScalar bRotZ, btScalar swingLimit1, btScalar swingLimit2, btScalar twistLimit );
	void CreateConstraint( JOINT joint, Bone* from, Bone* to, const JointInfo &jointInfo );
	float GetBoneLength( Bone* bone );
	void InitMotorData();
	void UpdateMotorData();
	void SetAllMotors(float dt);
	btTransform XM_CALLCONV XMMatrixToBTTransform( DirectX::FXMMATRIX m, bool fbxCorrection );
	int									numBones;
	std::map<int, Bone*>				idxBones;
	std::map<std::string, Bone*>		nameBones;
	DirectX::XMFLOAT4X4					finalTransformData[96];
	std::vector<DirectX::XMMATRIX>		toRoot;
	PhysicsWorld*						physicsWorld;
	DirectX::XMFLOAT4X4					rootTransform;
	btConvexShape*						shapes[SHAPE_COUNT];
	btScalar							shapeLengths[SHAPE_COUNT];
	btRigidBody*						bodies[BODY_COUNT];
	btTypedConstraint*					joints[JOINT_COUNT];
	MotorData*							motors[JOINT_COUNT];
	AnimationController*				animationController;
	bool								useRagdoll;
};