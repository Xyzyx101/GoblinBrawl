#pragma once
#include "DirectX_11_1_Includes.h"

using namespace DirectX;
class Camera {
public:
	Camera();
	~Camera();
	void Init(float aspectRatio);

	// Update expects the charactor pos and dir. The camera offset and
	// target will be calculated by the Update function.
	void XM_CALLCONV Update( FXMVECTOR pos, FXMVECTOR target);
	XMMATRIX XM_CALLCONV GetViewProj();
	XMVECTOR XM_CALLCONV GetPos();
private:
	XMVECTOR pos;
	XMMATRIX view;
	XMMATRIX proj;
	XMVECTOR up;
	XMMATRIX viewProj;
	FLOAT nearZ;
	FLOAT farZ;
	FLOAT fovAngleY;
};