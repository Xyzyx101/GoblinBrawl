#pragma once
#include "DirectX_11_1_Includes.h"
#include <PrimitiveBatch.h>
#include "Vertex.h"
#include "LinearMath\btIDebugDraw.h"

class PhysicsDebugDrawer : public btIDebugDraw {
public:
	PhysicsDebugDrawer();
	~PhysicsDebugDrawer();
	bool Init( ID3D11DeviceContext* device );
	void XM_CALLCONV Begin( DirectX::FXMMATRIX viewProj );
	void End();
	virtual void setDebugMode( int debugMode );
	virtual int getDebugMode() const;
	virtual void drawLine( const btVector3 &from, const btVector3 &to, const btVector3 &color );
	void drawLine( DirectX::XMFLOAT3 from, DirectX::XMFLOAT3 to, DirectX::XMFLOAT4 color );
	virtual void drawContactPoint( const btVector3 &PointOnB, const btVector3 &normalOnB, btScalar distance, int lifeTime, const btVector3 &color );
	virtual void reportErrorWarning( const char *warningString );
	virtual void draw3dText( const btVector3 &location, const char *textString );
private:
	ID3D11DeviceContext*					ctx;
	ID3D11BlendState*						oldBlendState;
	float									oldBlendFactor[4];
	ID3D11DepthStencilState*				oldStencilState;
	UINT									oldStencilRef;
	ID3D11RasterizerState*					oldRasterizerState;
	std::unique_ptr<
		DirectX::PrimitiveBatch<
		Vertex::SimpleVertex>>				primitiveBatch;
};

