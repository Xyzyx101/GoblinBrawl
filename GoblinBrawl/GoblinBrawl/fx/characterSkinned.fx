#include "lightHelper.fx"

cbuffer cbPerFrame {
	PointLight	gPointLights[5];
	float3		gEyePosW;
};

cbuffer cbPerObject {
	float4x4	gWorld;
	float4x4	gWorldInvTranspose;
	float4x4	gWorldViewProj;
	Material	gMaterial;
};

cbuffer cbSkinned {
	float4x4 gBoneTransforms[96]; // limited to 96 bone max
};

Texture2D	gDiffuseMap;
Texture2D	gAmbientMap;

SamplerState samAnisotropic {
	Filter = ANISOTROPIC;
	MaxAnisotropy = 4;
	AddressU = WRAP;
	AddressV = WRAP;
};

SamplerState samAmbient {
	Filter = MIN_MAG_MIP_LINEAR;
	AddressU = CLAMP;
	AddressV = CLAMP;
};

struct VertexIn {
	float3 PosL			:	POSITION;
	float3 NormalL		:	NORMAL0;
	float2 Tex			:	TEXCOORD;
	float4 Weights		:	WEIGHTS;
	uint4 BoneIndices	:	BONEINDICES;
};

struct VertexOut {
	float4 PosH       : SV_POSITION;
	float3 PosW       : POSITION;
	float3 NormalW    : NORMAL;
	float2 Tex      : TEXCOORD0;
	float4 Debug	: COLOR1;
};

VertexOut VS( VertexIn vin ) {
	VertexOut vout;

	// Init array or else we get strange warnings about SV_POSITION
	float weights[4] = { 0.f, 0.f, 0.f, 0.f };
	weights[0] = vin.Weights.x;
	weights[1] = vin.Weights.y;
	weights[2] = vin.Weights.z;
	weights[3] = 1.0f-weights[0]-weights[1]-weights[2];

	vin.NormalL = normalize( vin.NormalL.xyz );

	// Vertex blending
	float3 posL = float3(0.f, 0.f, 0.f);
	float3 normalL = float3(0.f, 0.f, 0.f);
	//[unroll]
	for( int i = 0; i<4; ++i ) {
		posL += weights[i]*mul( float4(vin.PosL, 1.0f), gBoneTransforms[vin.BoneIndices[i]] ).xyz;
		normalL += weights[i]*mul( vin.NormalL, (float3x3)gBoneTransforms[vin.BoneIndices[i]] );
	}
	
	vout.Debug = float4(vin.NormalL, 1.f);

	// Transform to world space
	vout.PosW = mul( float4(posL, 1.0f), gWorld ).xyz;
	vout.NormalW = mul( normalL, (float3x3)gWorldInvTranspose );

	// Transform to homogeneous clip space.
	vout.PosH = mul( float4(posL, 1.0f), gWorldViewProj );

	// Output vertex attributes for interpolation across triangle
	vout.Tex = vin.Tex;

	return vout;
}

float4 PS( VertexOut pin, uniform int gLightCount ) : SV_Target{

	//return pin.DebugCol;

	// Interpolating normal can unnormalize it
	pin.NormalW = normalize( pin.NormalW );

	float4 texColor = gDiffuseMap.Sample( samAnisotropic, pin.Tex );

	float3 toEye = gEyePosW-pin.PosW;

	float distToEye = length( toEye );
	toEye /= distToEye;
		
		///
		// Lighting
		///

	float4 ambient = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 diffuse = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 spec = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 dirAmbient = float4(0.0f, 0.0f, 0.0f, 0.0f);
	[unroll]
	for( int i = 0; i<gLightCount; ++i ) {
		float4 A, D, S, DA;
		ComputePointLight( gMaterial, gPointLights[i], pin.PosW, pin.NormalW, toEye, A, D, S, DA );
		ambient += A;
		diffuse += D;
		spec += S;
		dirAmbient += DA;
	}
	diffuse = saturate( diffuse );
	float4 ambientColor = gAmbientMap.Sample( samAmbient, float2(dirAmbient.r, 0.f) );
	ambientColor = clamp( ambientColor, ambient, float4(1.f, 1.f, 1.f, 1.f) );
	float4 litColor = (texColor*diffuse*ambientColor)+spec;

	litColor.a = gMaterial.Diffuse.a;

	return litColor;
}

technique11 CharacterSkinnedLight5 {
	pass P0 {
		SetVertexShader( CompileShader(vs_5_0, VS()) );
		SetGeometryShader( NULL );
		SetPixelShader( CompileShader(ps_5_0, PS(5)) );
	}
}