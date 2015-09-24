#pragma once
#include "DirectX_11_1_Includes.h"

class SharedResources {
public:
	SharedResources();
	~SharedResources();
	static void Init( ID3D11Device* device );

	static ID3D11ShaderResourceView*		directionalAmbientView;
};

