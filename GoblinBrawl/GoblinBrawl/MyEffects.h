#pragma once
#include <string>
#include "DirectX_11_1_Includes.h"
#include "FX11/d3dx11effect.h"
#include "Lighting.h"

struct PointLight;

class MyEffect {
public:
	MyEffect( ID3D11Device* device, const std::wstring& filename );
	virtual ~MyEffect();

private:
	MyEffect( const MyEffect& rhs );
	MyEffect& operator=(const MyEffect& rhs);

protected:
	ID3DX11Effect* fx;
};

class SimpleEffect : public MyEffect {
public:
	SimpleEffect( ID3D11Device* device, const std::wstring& filename );
	~SimpleEffect();
	void XM_CALLCONV SetWorldViewProj( DirectX::FXMMATRIX m ) { worldViewProj->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	ID3DX11EffectTechnique*			simpleTechnique;
	ID3DX11EffectMatrixVariable*	worldViewProj;
};

class LavaEffect : public MyEffect {
public:
	LavaEffect( ID3D11Device* device, const std::wstring& filename );
	~LavaEffect();
	void XM_CALLCONV SetWorld( DirectX::FXMMATRIX m ) { world->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	void XM_CALLCONV SetWorldInvTranspose( DirectX::FXMMATRIX m ) { worldInvTranspose->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	void XM_CALLCONV SetWorldViewProj( DirectX::FXMMATRIX m ) { worldViewProj->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	void SetDiffuseMap( ID3D11ShaderResourceView* srv ) { diffuseMap->SetResource( srv ); }
	ID3DX11EffectTechnique*					lavaTechnique;
	ID3DX11EffectMatrixVariable*			world;
	ID3DX11EffectMatrixVariable*			worldInvTranspose;
	ID3DX11EffectMatrixVariable*			worldViewProj;
	ID3DX11EffectShaderResourceVariable*	diffuseMap;
};

class StaticGeomEffect : public MyEffect {
public:
	StaticGeomEffect( ID3D11Device* device, const std::wstring& filename );
	~StaticGeomEffect();
	void XM_CALLCONV SetWorld( DirectX::FXMMATRIX m ) { world->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	void XM_CALLCONV SetWorldInvTranspose( DirectX::FXMMATRIX m ) { worldInvTranspose->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	void XM_CALLCONV SetWorldViewProj( DirectX::FXMMATRIX m ) { worldViewProj->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	void SetDiffuseMap( ID3D11ShaderResourceView* srv ) { diffuseMap->SetResource( srv ); }
	void XM_CALLCONV SetEyePosW( const DirectX::FXMVECTOR v ) { eyePosW->SetRawValue( &v, 0, sizeof( DirectX::XMFLOAT3 ) ); }
	void SetPointLights( const PointLight* lights ) { pointLights->SetRawValue( lights, 0, POINTLIGHT_COUNT *sizeof( PointLight ) ); }
	void SetMaterial( const Material& m ) { mat->SetRawValue( &m, 0, sizeof( Material ) ); }
	ID3DX11EffectTechnique*					staticGeomLight5Tech;
	ID3DX11EffectMatrixVariable*			world;
	ID3DX11EffectMatrixVariable*			worldInvTranspose;
	ID3DX11EffectMatrixVariable*			worldViewProj;
	ID3DX11EffectShaderResourceVariable*	diffuseMap;
	ID3DX11EffectVectorVariable*			eyePosW;
	ID3DX11EffectVariable*					pointLights;
	ID3DX11EffectVariable*					mat;
};

class CharacterEffect : public MyEffect {
public:
	CharacterEffect( ID3D11Device* device, const std::wstring& filename );
	~CharacterEffect();
	void XM_CALLCONV SetWorld( DirectX::FXMMATRIX m ) { world->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	void XM_CALLCONV SetWorldInvTranspose( DirectX::FXMMATRIX m ) { worldInvTranspose->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	void XM_CALLCONV SetWorldViewProj( DirectX::FXMMATRIX m ) { worldViewProj->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	void SetDiffuseMap( ID3D11ShaderResourceView* srv ) { diffuseMap->SetResource( srv ); }
	void XM_CALLCONV SetEyePosW( const DirectX::FXMVECTOR v ) { eyePosW->SetRawValue( &v, 0, sizeof( DirectX::XMFLOAT3 ) ); }
	void SetPointLights( const PointLight* lights ) { pointLights->SetRawValue( lights, 0, POINTLIGHT_COUNT *sizeof( PointLight ) ); }
	void SetMaterial( const Material& m ) { mat->SetRawValue( &m, 0, sizeof( Material ) ); }
	ID3DX11EffectTechnique*					characterLight5Tech;
	ID3DX11EffectMatrixVariable*			world;
	ID3DX11EffectMatrixVariable*			worldInvTranspose;
	ID3DX11EffectMatrixVariable*			worldViewProj;
	ID3DX11EffectShaderResourceVariable*	diffuseMap;
	ID3DX11EffectVectorVariable*			eyePosW;
	ID3DX11EffectVariable*					pointLights;
	ID3DX11EffectVariable*					mat;
};

class CharacterSkinnedEffect : public MyEffect {
public:
	CharacterSkinnedEffect( ID3D11Device* device, const std::wstring& filename );
	~CharacterSkinnedEffect();
	void XM_CALLCONV SetWorld( DirectX::FXMMATRIX m ) { world->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	void XM_CALLCONV SetWorldInvTranspose( DirectX::FXMMATRIX m ) { worldInvTranspose->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	void XM_CALLCONV SetWorldViewProj( DirectX::FXMMATRIX m ) { worldViewProj->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	void SetDiffuseMap( ID3D11ShaderResourceView* srv ) { diffuseMap->SetResource( srv ); }
	void XM_CALLCONV SetEyePosW( const DirectX::FXMVECTOR v ) { eyePosW->SetRawValue( &v, 0, sizeof( DirectX::XMFLOAT3 ) ); }
	void SetPointLights( const PointLight* lights ) { pointLights->SetRawValue( lights, 0, POINTLIGHT_COUNT *sizeof( PointLight ) ); }
	void SetMaterial( const Material& m ) { mat->SetRawValue( &m, 0, sizeof( Material ) ); }
	void XM_CALLCONV SetBoneTransforms( DirectX::XMFLOAT4X4* m, int cnt ) { boneTransforms->SetMatrixArray( reinterpret_cast<const float*>(m), 0, cnt ); }
	ID3DX11EffectTechnique*					characterSkinnedLight5Tech;
	ID3DX11EffectMatrixVariable*			world;
	ID3DX11EffectMatrixVariable*			worldInvTranspose;
	ID3DX11EffectMatrixVariable*			worldViewProj;
	ID3DX11EffectShaderResourceVariable*	diffuseMap;
	ID3DX11EffectVectorVariable*			eyePosW;
	ID3DX11EffectVariable*					pointLights;
	ID3DX11EffectVariable*					mat;
	ID3DX11EffectMatrixVariable*			boneTransforms;
};

class MyEffects {
public:
	static void InitAll( ID3D11Device* device );
	static void DestroyAll();

	static SimpleEffect*			SimpleFX;
	static LavaEffect*				LavaFX;
	static StaticGeomEffect*		StaticGeomFX;
	static CharacterEffect*			CharacterFX;
	static CharacterSkinnedEffect*	CharacterSkinnedFX;
};

