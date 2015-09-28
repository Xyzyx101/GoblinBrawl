#include "stdafx.h"
#include "Camera.h"
#include "Goblin.h"
#include "Game.h"

/* 
Toggle DEV Camera on or off with HOME key
DEV Camera controls are;
Arrow Up		- move forward
Arrow Down		- move back
Arrow Left		- turn left
Arrow Right		- turn right
Page Up			- turn up
Page Down		- turn down
<, Key			- strafe left
>. Key			- strafe right
*/

Camera::Camera() :
camType(0),
pos( 0.0f, 8.0f, 0.0f ),
right( 1.0f, 0.0f, 0.0f ),
up( 0.0f, 1.0f, 0.0f ),
look( 0.0f, 0.0f, 1.0f )
{
}

Camera::~Camera() {}

void Camera::Init(float aspectRatio) {
	SetAspect( aspectRatio );
	
	SetLens( 0.25 * 3.14, aspect, 1.0f, 1000.0f );
	UpdateViewMatrix();
}

void XM_CALLCONV Camera::Update( float deltaTime ) {

	if( GetAsyncKeyState( VK_HOME ) ) {
		SetCamType();
	}
	
	if( GetCamType()==1 ) {

		if( GetAsyncKeyState( VK_LEFT ) ) {
			RotateY( 5.0f * deltaTime );
		}
		if( GetAsyncKeyState( VK_RIGHT ) ) {
			RotateY( -5.0f * deltaTime );
		}
		if( GetAsyncKeyState( VK_UP ) ) {
			Walk( -15.0f * deltaTime );
		}
		if( GetAsyncKeyState( VK_DOWN ) ) {
			Walk( 15.0f * deltaTime );
		}
		if( GetAsyncKeyState( VK_PRIOR ) ) {
			Pitch( 1.0f * deltaTime );
		}
		if( GetAsyncKeyState( VK_NEXT ) ) {
			Pitch( -1.0f * deltaTime );
		}
		if( GetAsyncKeyState( VK_OEM_COMMA ) ) {
			Strafe( -15.0f * deltaTime );
		}
		if( GetAsyncKeyState( VK_OEM_PERIOD ) ) {
			Strafe( 15.0f * deltaTime );
		}
	} else {

		// default camera
		
		//XMStoreFloat3( &target1, goblin1Pos);
		//XMVECTOR posV = XMLoadFloat3( &pos );
		//XMVECTOR targetV = goblin1Pos;
		//XMVECTOR upV = GetUpXM();
		//LookAt( posV, targetV, upV );
	}

	UpdateViewMatrix();
}

XMVECTOR Camera::GetPosXM() const {
	return XMLoadFloat3( &pos );
}
XMFLOAT3 Camera::GetPos() const {
	return pos;
}
void Camera::SetPos( float x, float y, float z ) {
	pos = XMFLOAT3( x, y, z );
}
void Camera::SetPos( const XMFLOAT3& v ) {
	pos = v;
}
XMVECTOR Camera::GetRightXM() const {
	return XMLoadFloat3( &right );
}
XMFLOAT3 Camera::GetRight() const {
	return right;
}
XMVECTOR Camera::GetUpXM() const {
	return XMLoadFloat3( &up );
}
XMFLOAT3 Camera::GetUp() const {
	return up;
}
XMVECTOR Camera::GetLookXM() const {
	return XMLoadFloat3( &look );
}
XMFLOAT3 Camera::GetLook() const {
	return look;
}
float Camera::GetNearZ() const {
	return nearZ;
}
float Camera::GetFarZ() const {
	return farZ;
}
float Camera::GetAspect() const {
	return aspect;
}
float Camera::GetFovY() const {
	return fovY;
}
float Camera::GetFovX() const {
	float halfWidth = 0.5f * GetNearWindowWidth();
	return 2.0f * atan( halfWidth/nearZ );
}
float Camera::GetNearWindowWidth() const {
	return aspect * nearWindowHeight;
}
float Camera::GetNearWindowHeight() const {
	return nearWindowHeight;
}
float Camera::GetFarWindowWidth() const {
	return aspect * farWindowHeight;
}
float Camera::GetFarWindowHeight() const {
	return farWindowHeight;
}

void Camera::SetLens(float ifovAngleY, float iaspect, float inear, float ifar) {
	fovY = ifovAngleY;
	aspect = iaspect;
	nearZ = inear;
	farZ = ifar;
	nearWindowHeight = 2.0f * nearZ * tanf( 0.5f * fovY );
	farWindowHeight = 2.0f * farZ * tanf( 0.5f * fovY );

	XMMATRIX P = XMMatrixPerspectiveFovRH( fovY, aspect, nearZ, farZ );
	XMStoreFloat4x4( &proj, P );
}

void XM_CALLCONV Camera::LookAt( FXMVECTOR iPos, FXMVECTOR iTarget, FXMVECTOR iUp ) {
	XMVECTOR L = XMVector3Normalize( XMVectorSubtract( iTarget, iPos ) );
	XMVECTOR R = XMVector3Normalize( XMVector3Cross( iUp, L ) );
	XMVECTOR U = XMVector3Cross( L, R );

	XMStoreFloat3( &pos, iPos );
	XMStoreFloat3( &look, L );
	XMStoreFloat3( &right, R );
	XMStoreFloat3( &up, U );
}
void XM_CALLCONV Camera::LookAt( const XMFLOAT3& iPos, const XMFLOAT3& iTarget, const XMFLOAT3& iUp ) {
	XMVECTOR P = XMLoadFloat3( &iPos );
	XMVECTOR T = XMLoadFloat3( &iTarget );
	XMVECTOR U = XMLoadFloat3( &iUp );
	LookAt( P, T, U );
}
XMMATRIX XM_CALLCONV Camera::View() const {
	return XMLoadFloat4x4( &view );
}
XMMATRIX XM_CALLCONV Camera::Proj() const {
	return XMLoadFloat4x4( &proj );
}
XMMATRIX XM_CALLCONV Camera::ViewProj() const {
	return XMMatrixMultiply( View(), Proj() );
}

void XM_CALLCONV Camera::Strafe( float distance ) {
	XMVECTOR s = XMVectorReplicate( distance );
	XMVECTOR r = XMLoadFloat3( &right );
	XMVECTOR p = XMLoadFloat3( &pos );
	XMStoreFloat3( &pos, XMVectorMultiplyAdd( s, r, p ) );
}

void XM_CALLCONV Camera::Walk( float distance ) {
	XMVECTOR s = XMVectorReplicate( distance );
	XMVECTOR l = XMLoadFloat3( &look );
	XMVECTOR p = XMLoadFloat3( &pos );
	XMStoreFloat3( &pos, XMVectorMultiplyAdd( s, l, p ));
}

void XM_CALLCONV Camera::Pitch( float angle ) {
	XMMATRIX R = XMMatrixRotationAxis( XMLoadFloat3( &right ), angle );
	XMStoreFloat3( &up, XMVector3TransformNormal( XMLoadFloat3( &up ), R ) );
	XMStoreFloat3( &look, XMVector3TransformNormal( XMLoadFloat3( &look ), R ) );
}

void XM_CALLCONV Camera::RotateY( float angle ) {
	XMMATRIX R = XMMatrixRotationY( angle );
	XMStoreFloat3( &right, XMVector3TransformNormal( XMLoadFloat3( &right ), R ) );
	XMStoreFloat3( &up, XMVector3TransformNormal( XMLoadFloat3( &up ), R ) );
	XMStoreFloat3( &look, XMVector3TransformNormal( XMLoadFloat3( &look ), R ) );
}

void XM_CALLCONV Camera::UpdateViewMatrix() {
	XMVECTOR R = XMLoadFloat3( &right );
	XMVECTOR U = XMLoadFloat3( &up );
	XMVECTOR L = XMLoadFloat3( &look );
	XMVECTOR P = XMLoadFloat3( &pos );
	L = XMVector3Normalize( L );
	U = XMVector3Normalize( XMVector3Cross( L, R ) );
	R = XMVector3Cross( U, L );
	float x = -XMVectorGetX( XMVector3Dot( P, R ) );
	float y = -XMVectorGetX( XMVector3Dot( P, U ) );
	float z = -XMVectorGetX( XMVector3Dot( P, L ) );
	XMStoreFloat3( &right, R );
	XMStoreFloat3( &up, U );
	XMStoreFloat3( &look, L );

	view( 0, 0 ) = right.x;
	view( 1, 0 ) = right.y;
	view( 2, 0 ) = right.z;
	view( 3, 0 ) = x;
	
	view( 0, 1 ) = up.x;
	view( 1, 1 ) = up.y;
	view( 2, 1 ) = up.z;
	view( 3, 1 ) = y;

	view( 0, 2 ) = look.x;
	view( 1, 2 ) = look.y;
	view( 2, 2 ) = look.z;
	view( 3, 2 ) = z;

	view( 0, 3 ) = 0.0f;
	view( 1, 3 ) = 0.0f;
	view( 2, 3 ) = 0.0f;
	view( 3, 3 ) = 1.0f;

}

UINT XM_CALLCONV Camera::GetCamType() const {
	return camType;
}

void XM_CALLCONV Camera::SetCamType() {
	// used int so we can have any number of different settings
	// 0 = default, normal game camera the way it should be played
	// 1 = dev view, move camera around freely using keyboard arrow keys
	// toggle through settings, increase maxCamTypes here if more are made
	camType++;
	if( camType > MAXCAMTYPES ) {
		camType = 0;
	} 
}

void Camera::SetAspect( float iAspect ) {
	aspect = iAspect;
}

void XM_CALLCONV Camera::SetGoblin1Pos( FXMVECTOR iGob1PosMatrix ) {
	goblin1Pos = iGob1PosMatrix;
}