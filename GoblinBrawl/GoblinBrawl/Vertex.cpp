#include "stdafx.h"
#include "Vertex.h"
#include "MyEffects.h"

const D3D11_INPUT_ELEMENT_DESC InputLayoutDesc::SimpleVertexDesc[] =
{
	{ "POSITION", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 }
};
const D3D11_INPUT_ELEMENT_DESC InputLayoutDesc::StaticGeomVertexDesc[] =
{
	{ "POSITION", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "NORMAL", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "TEXCOORD", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0 }
};
const D3D11_INPUT_ELEMENT_DESC InputLayoutDesc::LavaVertexDesc[] =
{
	{ "POSITION", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "NORMAL", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "TEXCOORD", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0 }
};
const D3D11_INPUT_ELEMENT_DESC InputLayoutDesc::CharacterVertexDesc[] =
{
	{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0 }
};
const D3D11_INPUT_ELEMENT_DESC InputLayoutDesc::CharacterSkinnedVertexDesc[] =
{
	{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "WEIGHTS", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 32, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "BONEINDICES", 0, DXGI_FORMAT_R8G8B8A8_UINT, 0, 48, D3D11_INPUT_PER_VERTEX_DATA, 0 }
};

ID3D11InputLayout* InputLayouts::Simple = 0;
ID3D11InputLayout* InputLayouts::StaticGeom = 0;
ID3D11InputLayout* InputLayouts::Lava = 0;
ID3D11InputLayout* InputLayouts::Character = 0;
ID3D11InputLayout* InputLayouts::CharacterSkinned = 0;

void InputLayouts::InitAll( ID3D11Device* device ) {
	D3DX11_PASS_DESC passDesc;

	//Simple
	MyEffects::SimpleFX->simpleTechnique->GetPassByIndex( 0 )->GetDesc( &passDesc );
	HR( device->CreateInputLayout( InputLayoutDesc::SimpleVertexDesc, 2, passDesc.pIAInputSignature, passDesc.IAInputSignatureSize, &Simple ) );

	//Static Geometry
	MyEffects::StaticGeomFX->staticGeomLight5Tech->GetPassByIndex( 0 )->GetDesc( &passDesc );
	HR( device->CreateInputLayout( InputLayoutDesc::StaticGeomVertexDesc, 3, passDesc.pIAInputSignature, passDesc.IAInputSignatureSize, &StaticGeom ) );

	//Lava
	MyEffects::LavaFX->lavaTechnique->GetPassByIndex( 0 )->GetDesc( &passDesc );
	HR( device->CreateInputLayout( InputLayoutDesc::LavaVertexDesc, 3, passDesc.pIAInputSignature, passDesc.IAInputSignatureSize, &Lava ) );

	//Character
	MyEffects::CharacterFX->characterLight5Tech->GetPassByIndex( 0 )->GetDesc( &passDesc );
	HR( device->CreateInputLayout( InputLayoutDesc::CharacterVertexDesc, 3, passDesc.pIAInputSignature, passDesc.IAInputSignatureSize, &Character ) );

	//Character Skinned
	MyEffects::CharacterSkinnedFX->characterSkinnedLight5Tech->GetPassByIndex( 0 )->GetDesc( &passDesc );
	HR( device->CreateInputLayout( InputLayoutDesc::CharacterSkinnedVertexDesc, 5, passDesc.pIAInputSignature, passDesc.IAInputSignatureSize, &CharacterSkinned ) );
}

void InputLayouts::DestroyAll() {
	ReleaseCOM( Simple );
	ReleaseCOM( StaticGeom );
	ReleaseCOM( Lava );
	ReleaseCOM( Character );
	ReleaseCOM( CharacterSkinned );
}