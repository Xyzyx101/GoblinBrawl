#include "stdafx.h"
#include "MyEffects.h"
#include <iostream>
#include <fstream>
#include <vector>

MyEffect::MyEffect( ID3D11Device* device, const std::wstring& filename ) :
fx( nullptr ) {
	std::ifstream fin( filename, std::ios::binary );
	fin.seekg( 0, std::ios_base::end );
	int size = (int)fin.tellg();
	fin.seekg( 0, std::ios_base::beg );
	std::vector<char> compiledShader( size );
	fin.read( &compiledShader[0], size );
	fin.close();
	HR( D3DX11CreateEffectFromMemory( &compiledShader[0], size, 0, device, &fx ) );
}

MyEffect::~MyEffect() {
	ReleaseCOM( fx );
}

SimpleEffect::SimpleEffect( ID3D11Device* device, const std::wstring& filename ) :
MyEffect( device, filename ) {
	simpleTechnique = fx->GetTechniqueByName( "SimpleTech" );
	worldViewProj = fx->GetVariableByName( "gWorldViewProj" )->AsMatrix();
}
SimpleEffect::~SimpleEffect() {}

LavaEffect::LavaEffect( ID3D11Device* device, const std::wstring& filename ) :
MyEffect( device, filename ) {
	lavaTechnique = fx->GetTechniqueByName( "TerrainTech" );
	world = fx->GetVariableByName( "gWorld" )->AsMatrix();
	worldInvTranspose = fx->GetVariableByName( "gWorldInvTranspose" )->AsMatrix();
	worldViewProj = fx->GetVariableByName( "gWorldViewProj" )->AsMatrix();
	diffuseMap = fx->GetVariableByName( "gDiffuseMap" )->AsShaderResource();
}
LavaEffect::~LavaEffect() {}

StaticGeomEffect::StaticGeomEffect( ID3D11Device* device, const std::wstring& filename ) :
MyEffect( device, filename ) {
	staticGeomLight5Tech = fx->GetTechniqueByName( "StaticGeomLight5" );
	world = fx->GetVariableByName( "gWorld" )->AsMatrix();
	worldInvTranspose = fx->GetVariableByName( "gWorldInvTranspose" )->AsMatrix();
	worldViewProj = fx->GetVariableByName( "gWorldViewProj" )->AsMatrix();
	diffuseMap = fx->GetVariableByName( "gDiffuseMap" )->AsShaderResource();
	ambientMap = fx->GetVariableByName( "gAmbientMap" )->AsShaderResource();
	eyePosW = fx->GetVariableByName( "gEyePosW" )->AsVector();
	pointLights = fx->GetVariableByName( "gPointLights" );
	mat = fx->GetVariableByName( "gMaterial" );
}
StaticGeomEffect::~StaticGeomEffect() {}

CharacterEffect::CharacterEffect( ID3D11Device* device, const std::wstring& filename ) :
MyEffect( device, filename ) {
	characterLight5Tech = fx->GetTechniqueByName( "CharacterLight5" );
	world = fx->GetVariableByName( "gWorld" )->AsMatrix();
	worldInvTranspose = fx->GetVariableByName( "gWorldInvTranspose" )->AsMatrix();
	worldViewProj = fx->GetVariableByName( "gWorldViewProj" )->AsMatrix();
	diffuseMap = fx->GetVariableByName( "gDiffuseMap" )->AsShaderResource();
	eyePosW = fx->GetVariableByName( "gEyePosW" )->AsVector();
	pointLights = fx->GetVariableByName( "gPointLights" );
	mat = fx->GetVariableByName( "gMaterial" );
}
CharacterEffect::~CharacterEffect() {}

CharacterSkinnedEffect::CharacterSkinnedEffect( ID3D11Device* device, const std::wstring& filename ) :
MyEffect( device, filename ) {
	characterSkinnedLight5Tech = fx->GetTechniqueByName( "CharacterSkinnedLight5" );
	world = fx->GetVariableByName( "gWorld" )->AsMatrix();
	worldInvTranspose = fx->GetVariableByName( "gWorldInvTranspose" )->AsMatrix();
	worldViewProj = fx->GetVariableByName( "gWorldViewProj" )->AsMatrix();
	diffuseMap = fx->GetVariableByName( "gDiffuseMap" )->AsShaderResource();
	ambientMap = fx->GetVariableByName( "gAmbientMap" )->AsShaderResource();
	eyePosW = fx->GetVariableByName( "gEyePosW" )->AsVector();
	pointLights = fx->GetVariableByName( "gPointLights" );
	mat = fx->GetVariableByName( "gMaterial" );
	boneTransforms = fx->GetVariableByName( "gBoneTransforms" )->AsMatrix();
}
CharacterSkinnedEffect::~CharacterSkinnedEffect() {}

SimpleEffect* MyEffects::SimpleFX = 0;
LavaEffect* MyEffects::LavaFX = 0;
StaticGeomEffect* MyEffects::StaticGeomFX = 0;
CharacterEffect* MyEffects::CharacterFX = 0;
CharacterSkinnedEffect* MyEffects::CharacterSkinnedFX = 0;

void MyEffects::InitAll( ID3D11Device* device ) {
	SimpleFX = new SimpleEffect( device, L"fx/simple.fxo" );
	LavaFX = new LavaEffect( device, L"fx/lava.fxo" );
	StaticGeomFX = new StaticGeomEffect( device, L"fx/staticGeom.fxo" );
	CharacterFX = new CharacterEffect( device, L"fx/character.fxo" );
	CharacterSkinnedFX = new CharacterSkinnedEffect( device, L"fx/characterSkinned.fxo" );
}

void MyEffects::DestroyAll() {
	SafeDelete( SimpleFX );
	SafeDelete( LavaFX );
	SafeDelete( StaticGeomFX );
	SafeDelete( CharacterFX );
	SafeDelete( CharacterSkinnedFX );
}