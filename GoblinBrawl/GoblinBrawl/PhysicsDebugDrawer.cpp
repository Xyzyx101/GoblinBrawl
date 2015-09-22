#include "stdafx.h"
#include "PhysicsDebugDrawer.h"
#include "MyEffects.h"

using namespace DirectX;

PhysicsDebugDrawer::PhysicsDebugDrawer() : ctx(nullptr) 
{}

PhysicsDebugDrawer::~PhysicsDebugDrawer() {}

bool PhysicsDebugDrawer::Init( ID3D11DeviceContext* device ) {
	this->ctx = device;
	primitiveBatch = std::unique_ptr<PrimitiveBatch<Vertex::SimpleVertex>>( new PrimitiveBatch<Vertex::SimpleVertex>( ctx ) );
	setDebugMode( DBG_DrawConstraints|DBG_DrawConstraintLimits );
	return true;
}

void XM_CALLCONV PhysicsDebugDrawer::Begin(FXMMATRIX viewProj ) {
	ctx->OMGetBlendState(&oldBlendState, oldBlendFactor,  NULL);
	ctx->OMGetDepthStencilState(&oldStencilState, &oldStencilRef);
	ctx->RSGetState( &oldRasterizerState );

	XMMATRIX world = XMMatrixIdentity();
	XMMATRIX worldViewProj = world*viewProj;

	MyEffects::SimpleFX->SetWorldViewProj( worldViewProj );

	ctx->IASetInputLayout( InputLayouts::Simple );
	ID3DX11EffectTechnique* tech = MyEffects::SimpleFX->simpleTechnique;
	tech->GetPassByIndex( 0 )->Apply( 0, ctx );
	primitiveBatch->Begin();
}

void PhysicsDebugDrawer::End() {
	primitiveBatch->End();
	ctx->OMSetBlendState( oldBlendState, oldBlendFactor, 0xffffffff );
	ctx->OMSetDepthStencilState( oldStencilState, oldStencilRef );
	ctx->RSSetState( oldRasterizerState );
}

void PhysicsDebugDrawer::setDebugMode( int debugMode ) {
	//TODO -- make this actually do something
}

int PhysicsDebugDrawer::getDebugMode() const {
	//TODO -- make this actually do something
	return -1;
}

void PhysicsDebugDrawer::drawLine( const btVector3 &from, const btVector3 &to, const btVector3 &color ) {
	XMFLOAT3 dxFrom = XMFLOAT3( (float)from.getX(), (float)from.getY(), (float)from.getZ() );
	XMFLOAT3 dxTo = XMFLOAT3( (float)to.getX(), (float)to.getY(), (float)to.getZ() );
	XMFLOAT4 dxColor = XMFLOAT4( (float)color.getX(), (float)color.getY(), (float)color.getZ(), 1.f );
	drawLine( dxFrom, dxTo, dxColor );
}

void PhysicsDebugDrawer::drawLine( XMFLOAT3 from, XMFLOAT3 to, XMFLOAT4 color ) {
	Vertex::SimpleVertex vertFrom, vertTo;
	vertFrom.Pos = from;
	vertFrom.Color = color;
	vertTo.Pos = to;
	vertTo.Color = color;
	primitiveBatch->DrawLine( vertFrom, vertTo );
}

void PhysicsDebugDrawer::drawContactPoint( const btVector3 &PointOnB, const btVector3 &normalOnB, btScalar distance, int lifeTime, const btVector3 &color ) {
	//TODO -- make this actually do something
}

void PhysicsDebugDrawer::reportErrorWarning( const char *warningString ) {
	fprintf( stderr, warningString );
}

void PhysicsDebugDrawer::draw3dText( const btVector3 &location, const char *textString ) {
	//TODO -- make this actually do something
}