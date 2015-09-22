#pragma once
#include "DirectX_11_1_Includes.h"

class Camera {
public:
	Camera();
	~Camera();
	void Init(float aspectRatio);

	// Update expects the charactor pos and dir. The camera offset and
	// target will be calculated by the Update function.
	void XM_CALLCONV Update( DirectX::FXMVECTOR pos, DirectX::FXMVECTOR target );
	void XM_CALLCONV UpdateFollow( DirectX::FXMMATRIX world );
	DirectX::XMMATRIX XM_CALLCONV GetViewProj();
	DirectX::XMVECTOR XM_CALLCONV GetPos();
private:
	DirectX::XMVECTOR pos;
	DirectX::XMMATRIX view;
	DirectX::XMMATRIX proj;
	DirectX::XMVECTOR up;
	DirectX::XMMATRIX viewProj;
	FLOAT nearZ;
	FLOAT farZ;
	FLOAT fovAngleY;
};