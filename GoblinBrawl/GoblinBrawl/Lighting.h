#pragma once
#include <vector>
#include "DirectX_11_1_Includes.h"

#define POINTLIGHT_COUNT 5

class ModelLoader;

struct PointLight {
	DirectX::XMFLOAT4 Ambient;
	DirectX::XMFLOAT4 Diffuse;
	DirectX::XMFLOAT4 Specular;

	//packed into 4D vector (Position, Range)
	DirectX::XMFLOAT3 Position;
	float Range;

	// Packed into 4D vector (A0, A1, A2, Pad)
	DirectX::XMFLOAT3 Att;
	float Pad;

};

struct Material {
	Material() { ZeroMemory( this, sizeof( this ) ); }
	DirectX::XMFLOAT4 Ambient;
	DirectX::XMFLOAT4 Diffuse;
	DirectX::XMFLOAT4 Specular; // w = SpecPower
};


class Lighting {
public:
	Lighting();
	~Lighting();
	bool Init( ModelLoader* modelLoader );
	std::vector<PointLight> GetPointLights();
private:
	std::vector<PointLight>	pointLights;
};

