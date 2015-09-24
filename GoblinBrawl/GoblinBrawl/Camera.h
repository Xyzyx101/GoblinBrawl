#pragma once
#include "DirectX_11_1_Includes.h"

const UINT MAXCAMTYPES = 1;

using namespace DirectX;
class Camera {
public:
	Camera();
	~Camera();

	void XM_CALLCONV Update( float deltaTime );
	
	// Get/Set world camera position
	XMVECTOR XM_CALLCONV GetPosXM() const;
	XMFLOAT3 GetPos() const;
	void SetPos( float x, float y, float z );
	void XM_CALLCONV SetPos( const XMFLOAT3& v );

	// camera basis vectors
	XMVECTOR XM_CALLCONV GetRightXM() const;
	XMFLOAT3 GetRight() const;
	XMVECTOR XM_CALLCONV GetUpXM() const;
	XMFLOAT3 GetUp() const;
	XMVECTOR XM_CALLCONV GetLookXM() const;
	XMFLOAT3 GetLook() const;

	// Get frustum properties.
	float GetNearZ() const;
	float GetFarZ() const;
	float GetAspect() const;
	float GetFovY() const;
	float GetFovX() const;

	// Get near and far - in view space coordinates.
	float GetNearWindowWidth() const;
	float GetNearWindowHeight() const;
	float GetFarWindowWidth() const;
	float GetFarWindowHeight() const;

	void SetLens( float iFovAngleY, float iAspect, float iNear, float iFar );

	void XM_CALLCONV LookAt( FXMVECTOR iPos, FXMVECTOR iTarget, FXMVECTOR iUp );
	void XM_CALLCONV LookAt( const XMFLOAT3& ipos, const XMFLOAT3& iTarget, const XMFLOAT3& iUp );

	XMMATRIX XM_CALLCONV View() const;
	XMMATRIX XM_CALLCONV Proj() const;
	XMMATRIX XM_CALLCONV ViewProj() const;

	void XM_CALLCONV Strafe( float distance );
	void XM_CALLCONV Walk( float distance );
	void XM_CALLCONV Pitch( float angle );
	void XM_CALLCONV RotateY( float angle );

	void XM_CALLCONV UpdateViewMatrix();

	UINT XM_CALLCONV GetCamType() const;
	void XM_CALLCONV SetCamType(UINT typeNum);
	void SetAspect( float iAspect );

private:
	UINT camType;
	// co-ord system - relative to world space
	XMFLOAT3 pos;
	XMFLOAT3 right;
	XMFLOAT3 up;
	XMFLOAT3 look;
	// view/proj matrices
	XMFLOAT4X4 view;
	XMFLOAT4X4 proj;
	// frustum properties
	float aspect;
	float nearZ;
	float farZ;
	float fovY;
	float nearWindowHeight;
	float farWindowHeight;

};