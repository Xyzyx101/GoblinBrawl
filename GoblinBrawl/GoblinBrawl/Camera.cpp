#include "stdafx.h"
#include "Camera.h"

Camera::Camera() :
nearZ( 1.f ),
farZ( 10000.f ),
camType(0),
pos( XMVectorSet( 0.f, 10.f, 20.1f, 1.0f ) ),
target( XMVectorSet( 0.f, 1.f, 0.f, 1.0f ) ),
right( XMVectorSet( 1.0f, 0.0f, 0.0f, 0.0f ) ),
look( XMVectorSet( 0.0f, 0.0f, 1.0f, 0.0f) ),
fovAngleY( XM_PIDIV4 ) {}

Camera::~Camera() {}

void Camera::Init( float aspectRatio ) {
	view = XMMatrixIdentity();
	up = XMVectorSet( 0.f, 1.f, 0.f, 0.f );
	proj = XMMatrixPerspectiveFovRH( fovAngleY, aspectRatio, nearZ, farZ );
}

void XM_CALLCONV Camera::Update() {
	
	if( GetCamType()==1 ) {
	} else {
		// default, everything else (GAME MODE)
		pos = XMVectorSet( 0.f, 10.f, 20.1f, 1.f );
		target = XMVectorSet( 0.f, 1.f, 0.f, 1.0f );
	}
	
	view = XMMatrixLookAtRH( pos, target, up );
	viewProj = view*proj;
}

XMMATRIX XM_CALLCONV Camera::GetViewProj() {
	return viewProj;
}

XMVECTOR XM_CALLCONV Camera::GetPos() {
	return pos;
}

void XM_CALLCONV Camera::SetPos( float x, float y, float z, float w) {
	float currX = XMVectorGetX( pos );
	float currY = XMVectorGetY( pos );
	float currZ = XMVectorGetZ( pos );
	float currW = XMVectorGetW( pos );

	pos = XMVectorSetX( pos, ( currX + x ));
	pos = XMVectorSetY( pos, ( currY + y ));
	pos = XMVectorSetZ( pos, ( currZ + z ));
	pos = XMVectorSetW( pos, ( currW + w ));
	Update();
}

UINT XM_CALLCONV Camera::GetCamType() {
	return camType;
}

void XM_CALLCONV Camera::SetCamType( UINT incTypeNum ) {
	// used int so we can have any number of different settings
	// 0 = default, normal game camera the way it should be played
	// 1 = dev view, move camera around freely using keyboard arrow keys
	// toggle through settings, increase maxCamTypes here if more are made
	
	if( incTypeNum >= MAXCAMTYPES ) {
		camType = 0;
	} else {
		camType++;
	}
}

void XM_CALLCONV Camera::Strafe( float distance ) {
	XMVECTOR s = XMVectorReplicate( distance );
	XMVECTOR r = right;
	XMVECTOR p = pos;
	pos = XMVectorMultiplyAdd( s, r, p );
}

void XM_CALLCONV Camera::Walk( float distance ) {
	XMVECTOR s = XMVectorReplicate( distance );
	XMVECTOR l = look;
	XMVECTOR p = pos;
	pos = XMVectorMultiplyAdd( s, l, p );
}