#pragma once
#include "DirectX_11_1_Includes.h"
#include "LinearMath\btVector3.h"
#include "LinearMath\btTransform.h"
#include "LinearMath\btQuaternion.h"

class MathUtils {
public:
	static DirectX::XMMATRIX XM_CALLCONV InverseTranspose( DirectX::FXMMATRIX M ) {
		// Inverse-transpose is just applied to normals.  So zero out 
		// translation row so that it doesn't get into our inverse-transpose
		// calculation--we don't want the inverse-transpose of the translation.
		DirectX::XMMATRIX A = M;
		A.r[3] = DirectX::XMVectorSet( 0.0f, 0.0f, 0.0f, 1.0f );

		DirectX::XMVECTOR det = XMMatrixDeterminant( A );
		return DirectX::XMMatrixTranspose( XMMatrixInverse( &det, A ) );
	}

	static void XM_CALLCONV GetEulerRotFromMatrix( DirectX::FXMMATRIX _m, float& outYaw, float& outPitch, float& outRoll ) {
		DirectX::XMFLOAT4X4 m;
		XMStoreFloat4x4( &m, _m );
		if( m._11==1.0f ) {
			outYaw = atan2f( m._13, m._34 );
			outPitch = 0;
			outRoll = 0;

		} else if( m._11==-1.0f ) {
			outYaw = atan2f( m._13, m._34 );
			outPitch = 0;
			outRoll = 0;
		} else {
			outYaw = atan2( -m._31, m._11 );
			outPitch = asin( m._21 );
			outRoll = atan2( -m._23, m._22 );
		}
	}

	static btTransform XM_CALLCONV XMMatrixTransformToBTTransform( DirectX::FXMMATRIX xm ) {
		DirectX::XMVECTOR outScale;
		DirectX::XMVECTOR outRotQuat;
		DirectX::XMVECTOR outTrans;
		XMMatrixDecompose( &outScale, &outRotQuat, &outTrans, xm );
		btVector3 trans( outTrans.m128_f32[0], outTrans.m128_f32[1], outTrans.m128_f32[2] );
		btQuaternion rotQuat( outRotQuat.m128_f32[0], outRotQuat.m128_f32[1], outRotQuat.m128_f32[2], outRotQuat.m128_f32[3] );
		btQuaternion myTestQuat = btQuaternion::getIdentity();
		myTestQuat.setEulerZYX(0.,0.,0.);
		return btTransform( rotQuat, trans );
	}
};

