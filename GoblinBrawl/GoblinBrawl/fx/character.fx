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

Texture2D	gDiffuseMap;

SamplerState samAnisotropic {
	Filter = ANISOTROPIC;
	MaxAnisotropy = 4;
	AddressU = WRAP;
	AddressV = WRAP;
};

struct VertexIn {
	float3 PosL		:	POSITION;
	float3 NormalL	:	NORMAL;
	float2 Tex		:	TEXCOORD;
};

struct VertexOut {
	float4 PosH		:	SV_POSITION;
	float3 PosW		:	POSITION;
	float3 NormalW	:	NORMAL;
	float2 Tex		:	TEXCOORD;
};

VertexOut VS( VertexIn vin ) {
	VertexOut vout;

	// Transform to world space
	vout.PosW = mul( float4(vin.PosL, 1.0f), gWorld ).xyz;
	vout.NormalW = vin.NormalL;// mul( vin.NormalL, (float3x3)gWorldInvTranspose );

	// Transform to homogeneous clip space.
	vout.PosH = mul( float4(vin.PosL, 1.0f), gWorldViewProj );

	// Output vertex attributes for interpolation across triangle
	vout.Tex = vin.Tex;

	return vout;
}

float4 PS( VertexOut pin, uniform int gLightCount ) : SV_Target{

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
	[unroll]
	for( int i = 0; i<gLightCount; ++i ) {
		float4 A, D, S, DA;
		ComputePointLight( gMaterial, gPointLights[i], pin.PosW, pin.NormalW, toEye, A, D, S, DA );
		ambient += A;
		diffuse += D;
		spec += S;
	}
	
	float4 litColor = texColor * (ambient + diffuse) + spec;
	litColor.a = gMaterial.Diffuse.a;

	//return float4(pin.NormalW, 1.0f);
	return litColor;
}

technique11 CharacterLight5 {
	pass P0 {
		SetVertexShader( CompileShader(vs_5_0, VS()) );
		SetGeometryShader( NULL );
		SetPixelShader( CompileShader(ps_5_0, PS(5)) );
	}
}