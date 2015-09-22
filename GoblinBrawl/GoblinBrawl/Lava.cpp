#include "stdafx.h"
#include "Lava.h"
#include "ModelLoader.h"
#include "Vertex.h"
#include "Mesh.h"
#include "MyEffects.h"
#include "MathUtils.h"
#include "WICTextureLoader.h"

Lava::Lava() :
mesh( nullptr ),
diffuseView( nullptr ) {}

Lava::~Lava() {}

bool Lava::Init( ModelLoader* modelLoader, ID3D11Device* device ) {
	modelLoader->Load( "lava.lxo", Vertex::LAVA );
	mesh = modelLoader->GetMesh();
	if( !mesh ) {
		return false;
	}
	HR( CreateWICTextureFromFile( device, L"./art/textures/lava_color.tif",  NULL, &diffuseView, NULL ) );
	return true;
}

void XM_CALLCONV Lava::Draw( FXMMATRIX viewProj, ID3D11DeviceContext* context ) {
	context->IASetInputLayout( InputLayouts::Lava );
	context->IASetPrimitiveTopology( D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST );
	UINT stride = sizeof( Vertex::LavaVertex );
	UINT offset = 0;
	ID3D11Buffer* buffers[1] = { mesh->VB() };
	context->IASetVertexBuffers( 0, 1, &buffers[0], &stride, &offset );
	context->IASetIndexBuffer( mesh->IB(), mesh->IndexFormat(), 0 );

	XMMATRIX world = XMMatrixIdentity();
	XMMATRIX worldInvTranspose = MathUtils::InverseTranspose( world );
	XMMATRIX worldViewProj = world*viewProj;

	MyEffects::LavaFX->SetWorld( world );
	MyEffects::LavaFX->SetWorldInvTranspose( worldInvTranspose );
	MyEffects::LavaFX->SetWorldViewProj( worldViewProj );
	MyEffects::LavaFX->SetDiffuseMap( diffuseView );

	ID3DX11EffectTechnique* tech = MyEffects::LavaFX->lavaTechnique;
	D3DX11_TECHNIQUE_DESC td;
	tech->GetDesc( &td );
	for( UINT p = 0; p<td.Passes; ++p ) {
		tech->GetPassByIndex( p )->Apply( 0, context );
		context->DrawIndexed( mesh->IndexCount(), 0, 0 );
	}
}
