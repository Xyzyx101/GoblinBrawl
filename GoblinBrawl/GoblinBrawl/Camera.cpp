#include "stdafx.h"
#include "Camera.h"

Camera::Camera() :
nearZ( 1.f ),
farZ( 10000.f ),
camType(0),
fovAngleY( XM_PIDIV4 ) {}

Camera::~Camera() {}

void Camera::Init( float aspectRatio ) {
	view = XMMatrixIdentity();
	up = XMVectorSet( 0.f, 1.f, 0.f, 0.f );
	proj = XMMatrixPerspectiveFovRH( fovAngleY, aspectRatio, nearZ, farZ );
}

void XM_CALLCONV Camera::Update( FXMVECTOR _pos, FXMVECTOR target ) {
	pos = _pos;
	view = XMMatrixLookAtRH( pos, target, up );
	viewProj = view*proj;
}

/*
void XM_CALLCONV Camera::UpdateFollow( FXMMATRIX world ) {
	if( camType==1 ) {
		XMVECTOR target = XMVector3Transform( XMLoadFloat4( &XMFLOAT4( 0.f, 0.f, -100.f, 1.f ) ), world );
		XMVECTOR pos = XMVector3Transform( XMLoadFloat4( &XMFLOAT4( 0.f, 200.f, 200.f, 1.f ) ), world );
		
		Update( pos, target );
	} else {
		// anything else is default
		XMVECTOR target = XMVector3Transform( XMLoadFloat4( &XMFLOAT4( 0.f, 0.f, -800.f, 1.f ) ), world );
		XMVECTOR pos = XMVector3Transform( XMLoadFloat4( &XMFLOAT4( 0.f, 200.f, 500.f, 1.f ) ), world );
		Update( pos, target );
	}
}
*/

XMMATRIX XM_CALLCONV Camera::GetViewProj() {
	return viewProj;
}

XMVECTOR XM_CALLCONV Camera::GetPos() {
	return pos;
}

UINT XM_CALLCONV Camera::GetCamType() {
	return camType;
}

void XM_CALLCONV Camera::SetCamType( UINT incTypeNum ) {
	// used int so we can have any number of different settings
	// 0 = default, normal game camera the way it should be played
	// 1 = dev view, move camera around freely using keyboard arrow keys
	// toggle through settings, increase maxCamTypes here if more are made
	UINT maxCamTypes = 1; // 0 and 1
	if( incTypeNum >= maxCamTypes ) {
		camType = 0;
	} else {
		camType++;
	}
}