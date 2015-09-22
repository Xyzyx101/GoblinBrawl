#pragma once
#include "DirectX_11_1_Includes.h"

const UINT MAXCAMTYPES = 1;

using namespace DirectX;
class Camera {
public:
	Camera();
	~Camera();
	void Init(float aspectRatio);

	// Update expects the charactor pos and dir. The camera offset and
	// target will be calculated by the Update function.
	void XM_CALLCONV Update();
	XMMATRIX XM_CALLCONV GetViewProj();
	XMVECTOR XM_CALLCONV GetPos();
	void XM_CALLCONV SetPos( float x, float y, float z, float w );
	UINT XM_CALLCONV GetCamType();
	void XM_CALLCONV SetCamType(UINT typeNum);
	void XM_CALLCONV Strafe( float distance );
	void XM_CALLCONV Walk( float distance );
private:
	XMVECTOR pos; 
	XMVECTOR target;
	XMVECTOR right;
	XMVECTOR up;
	XMVECTOR look;
	XMMATRIX view;
	XMMATRIX proj;
	XMMATRIX viewProj;
	FLOAT nearZ;
	FLOAT farZ;
	FLOAT fovAngleY;
	UINT camType;
};