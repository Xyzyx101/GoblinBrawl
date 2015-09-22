#include "stdafx.h"
#include "FirePlinth.h"
#include "ModelLoader.h"
#include "Vertex.h"
#include "Mesh.h"
#include "MyEffects.h"
#include "MathUtils.h"
#include "WICTextureLoader.h"

FirePlinth::FirePlinth() : 
mesh (nullptr),
diffuseView(nullptr)
{}

FirePlinth::~FirePlinth() {}

bool FirePlinth::Init( ModelLoader* modelLoader, ID3D11Device* device ) {
	modelLoader->Load( "firePlinth.lxo", Vertex::STATIC_GEOMETRY );
	mesh = modelLoader->GetMesh();
	if( !mesh ) {
		return false;
	}
	HR( CreateWICTextureFromFile( device, L"./art/textures/fire_plinth_color.tif", NULL, &diffuseView, NULL ) );
	mat.Ambient = XMFLOAT4( 0.02f, 0.3f, 0.5f, 1.0f );
	mat.Diffuse = XMFLOAT4( 0.8f, 0.8f, 0.8f, 1.0f );
	mat.Specular = XMFLOAT4( 0.3f, 0.3f, 0.3f, 32.0f );
	return true;
}

void XM_CALLCONV FirePlinth::Draw( FXMMATRIX viewProj, FXMVECTOR cameraPos, std::vector<PointLight> pointLights, ID3D11DeviceContext* context ) {
	context->IASetInputLayout( InputLayouts::StaticGeom );
	context->IASetPrimitiveTopology( D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST );
	UINT stride = sizeof( Vertex::StaticGeomVertex );
	UINT offset = 0;
	ID3D11Buffer* buffers[1] = { mesh->VB() };
	context->IASetVertexBuffers( 0, 1, &buffers[0], &stride, &offset );
	context->IASetIndexBuffer( mesh->IB(), mesh->IndexFormat(), 0 );

	XMMATRIX world = XMMatrixIdentity();
	XMMATRIX worldInvTranspose = MathUtils::InverseTranspose( world );
	XMMATRIX worldViewProj = world*viewProj;

	MyEffects::StaticGeomFX->SetWorld( world );
	MyEffects::StaticGeomFX->SetWorldInvTranspose( worldInvTranspose );
	MyEffects::StaticGeomFX->SetWorldViewProj( worldViewProj );
	MyEffects::StaticGeomFX->SetDiffuseMap( diffuseView );
	MyEffects::StaticGeomFX->SetEyePosW( cameraPos );
	MyEffects::StaticGeomFX->SetPointLights( pointLights.data() );
	MyEffects::StaticGeomFX->SetMaterial( mat );

	ID3DX11EffectTechnique* tech = MyEffects::StaticGeomFX->staticGeomLight5Tech;
	D3DX11_TECHNIQUE_DESC td;
	tech->GetDesc( &td );
	for( UINT p = 0; p<td.Passes; ++p ) {
		tech->GetPassByIndex( p )->Apply( 0, context );
		context->DrawIndexed( mesh->IndexCount(), 0, 0 );
	}
}