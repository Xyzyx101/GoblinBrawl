#pragma once
#include "DirectX_11_1_Includes.h"

namespace Vertex {
	enum VERTEX_TYPE {
		SIMPLE,
		STATIC_GEOMETRY,
		LAVA,
		CHARACTER,
		CHARACTER_SKINNED
	};
	struct SimpleVertex {
		DirectX::XMFLOAT3	Pos;
		DirectX::XMFLOAT4	Color;
	};
	struct StaticGeomVertex {
		XMFLOAT3	Pos;
		XMFLOAT3	Normal;
		XMFLOAT2	Tex;
	};
	struct LavaVertex {
		XMFLOAT3	Pos;
		XMFLOAT3	Normal;
		XMFLOAT2	Tex;
	};
	struct CharacterVertex {
		DirectX::XMFLOAT3	Pos;
		DirectX::XMFLOAT3	Normal;
		DirectX::XMFLOAT2	Tex;
	};
	struct CharacterSkinnedVertex {
		DirectX::XMFLOAT3	Pos;
		DirectX::XMFLOAT3	Normal;
		DirectX::XMFLOAT2	Tex;
		DirectX::XMFLOAT4	Weights;
		BYTE		BoneIndicies[4];
	};
}

class InputLayoutDesc {
public:
	static const D3D11_INPUT_ELEMENT_DESC SimpleVertexDesc[2];
	static const D3D11_INPUT_ELEMENT_DESC StaticGeomVertexDesc[3];
	static const D3D11_INPUT_ELEMENT_DESC LavaVertexDesc[3];
	static const D3D11_INPUT_ELEMENT_DESC CharacterVertexDesc[3];
	static const D3D11_INPUT_ELEMENT_DESC CharacterSkinnedVertexDesc[5];
};

class InputLayouts {
public:
	static void InitAll( ID3D11Device* device );
	static void DestroyAll();
	static ID3D11InputLayout* Simple;
	static ID3D11InputLayout* StaticGeom;
	static ID3D11InputLayout* Lava;
	static ID3D11InputLayout* Character;
	static ID3D11InputLayout* CharacterSkinned;
};
