#pragma once
#include "DirectX_11_1_Includes.h"

using namespace DirectX;

namespace Vertex {
	enum VERTEX_TYPE {
		SIMPLE,
		STATIC_GEOMETRY,
		LAVA,
		CHARACTER,
		CHARACTER_SKINNED
	};
	struct SimpleVertex {
		XMFLOAT3	Pos;
		XMFLOAT4	Color;
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
		XMFLOAT3	Pos;
		XMFLOAT3	Normal;
		XMFLOAT2	Tex;
	};
	struct CharacterSkinnedVertex {
		XMFLOAT3	Pos;
		XMFLOAT3	Normal;
		XMFLOAT2	Tex;
		XMFLOAT4	Weights;
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
