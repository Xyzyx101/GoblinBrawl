#include "stdafx.h"
#include "Camera.h"

using namespace DirectX;

Camera::Camera() :
nearZ( 1.f ),
farZ( 10000.f ),
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

void XM_CALLCONV Camera::UpdateFollow( FXMMATRIX world ) {
	XMVECTOR target = XMVector3Transform( XMLoadFloat4( &XMFLOAT4( 0.f, 0.f, 800.f, 1.f )), world);
	XMVECTOR pos = XMVector3Transform( XMLoadFloat4( &XMFLOAT4( 0.f, 200.f, -500.f, 1.f ) ), world );
	Update( pos, target );
}

XMMATRIX XM_CALLCONV Camera::GetViewProj() {
	return viewProj;
}

XMVECTOR XM_CALLCONV Camera::GetPos() {
	return pos;
}