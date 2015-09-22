#pragma once
#include "DirectX_11_1_Includes.h"
#include "Lighting.h"
#include "btBulletDynamicsCommon.h"

struct ID3DX11Effect;
struct ID3DX11EffectTechnique;
struct ID3DX11EffectMatrixVariable;
struct ID3D11ShaderResourceView;

class ModelLoader;
class Mesh;
struct ID3D11DeviceContext;
struct ID3D11Device;

class PhysicsWorld;

class Floor {
public:
	Floor();
	~Floor();
	bool Init( ModelLoader* modelLoader, ID3D11Device* device, PhysicsWorld* physicsWorld );
	void XM_CALLCONV Draw( DirectX::FXMMATRIX viewProj, DirectX::FXMVECTOR cameraPos, std::vector<PointLight> pointLights, ID3D11DeviceContext* context );
private:
	void CreateHeightfield( float minX, float maxX, float minY, float maxY, float minZ, float maxZ );
	BYTE* GetRawHeightData( int gridSize, float heightScale, btScalar gridSpacing, PHY_ScalarType type );
	Mesh*							mesh;
	ID3D11Device*					device;
	ID3D11ShaderResourceView*		diffuseView;
	Material						mat;
	PhysicsWorld*					physicsWorld;
	struct Pixel {
		uint16_t r;
		uint16_t g;
		uint16_t b;
		uint16_t a;
	};
};