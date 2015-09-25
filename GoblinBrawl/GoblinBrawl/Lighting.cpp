#include "stdafx.h"
#include "Lighting.h"
#include "ModelLoader.h"

using namespace DirectX;

Lighting::Lighting() {}

Lighting::~Lighting() {}

bool Lighting::Init( ModelLoader* modelLoader ) {
	pointLights = std::vector<PointLight>( POINTLIGHT_COUNT );
	
	XMFLOAT3 firePlinthAtt = XMFLOAT3( 0.0f, 0.0f, 0.005f );

	// point light 1
	pointLights[0].Ambient = XMFLOAT4( 0.2f, 0.2f, 0.2f, 1.0f );
	pointLights[0].Diffuse = XMFLOAT4( 1.0f, 0.6136f, 0.2069f, 1.0f );
	pointLights[0].Specular = XMFLOAT4( 1.0f, 0.5136f, 0.1069f, 1.0f );
	pointLights[0].Position = XMFLOAT3( 16.4867f, 5.9049f, 15.8f );
	pointLights[0].Range = 300.0f;
	pointLights[0].Att = firePlinthAtt;

	// point light 2
	pointLights[1].Ambient = XMFLOAT4( 0.2f, 0.2f, 0.2f, 1.0f );
	pointLights[1].Diffuse = XMFLOAT4( 1.0f, 0.6136f, 0.2069f, 1.0f );
	pointLights[1].Specular = XMFLOAT4( 1.0f, 0.5136f, 0.1069f, 1.0f );
	pointLights[1].Position = XMFLOAT3( 9.3847f, 4.9689f, -11.4676f );
	pointLights[1].Range = 300.0f;
	pointLights[1].Att = firePlinthAtt;

	// point light 3
	pointLights[2].Ambient = XMFLOAT4( 0.2f, 0.2f, 0.2f, 1.0f );
	pointLights[2].Diffuse = XMFLOAT4( 1.0f, 0.6136f, 0.2069f, 1.0f );
	pointLights[2].Specular = XMFLOAT4( 1.0f, 0.5136f, 0.1069f, 1.0f );
	pointLights[2].Position = XMFLOAT3( -6.7762f, 6.2829f, 20.5063f );
	pointLights[2].Range = 300.0f;
	pointLights[2].Att = firePlinthAtt;

	// point light 4
	pointLights[3].Ambient = XMFLOAT4( 0.2f, 0.2f, 0.2f, 1.0f );
	pointLights[3].Diffuse = XMFLOAT4( 1.0f, 0.6136f, 0.2069f, 1.0f );
	pointLights[3].Specular = XMFLOAT4( 1.0f, 0.5136f, 0.1069f, 1.0f );
	pointLights[3].Position = XMFLOAT3( -23.6346f, 7.4317f, 4.4564f );
	pointLights[3].Range = 300.0f;
	pointLights[3].Att = firePlinthAtt;

	// point light 5
	pointLights[4].Ambient = XMFLOAT4( 0.2f, 0.2f, 0.2f, 1.0f );
	pointLights[4].Diffuse = XMFLOAT4( 1.0f, 0.6136f, 0.2069f, 1.0f );
	pointLights[4].Specular = XMFLOAT4( 1.0f, 0.5136f, 0.1069f, 1.0f );
	pointLights[4].Position = XMFLOAT3( -11.4568f, 5.9013f, -17.961f );
	pointLights[4].Range = 300.0f;
	pointLights[4].Att = firePlinthAtt;
	return true;
}

std::vector<PointLight> Lighting::GetPointLights( ) {
	return pointLights;
}