#include "stdafx.h"
#include "SharedResources.h"
#include "WICTextureLoader.h"

SharedResources::SharedResources() {}

SharedResources::~SharedResources() {}

ID3D11ShaderResourceView* SharedResources::directionalAmbientView = nullptr;

void SharedResources::Init( ID3D11Device* device ) {
	HR( DirectX::CreateWICTextureFromFile( device, L"./art/textures/ToonRamp.png", NULL, &directionalAmbientView, NULL ) );
}