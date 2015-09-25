cbuffer cbPerObject {
	float4x4	gWorld;
	float4x4	gWorldInvTranspose;
	float4x4	gWorldViewProj;
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
	vout.NormalW = mul( vin.NormalL, (float3x3)gWorldInvTranspose );

	// Transform to homogeneous clip space.
	vout.PosH = mul( float4(vin.PosL, 1.0f), gWorldViewProj );

	// Output vertex attributes for interpolation across triangle
	vout.Tex = vin.Tex;

	return vout;
}

float4 PS( VertexOut pin ) : SV_Target {
	// Interpolating normal can unnormalize it
	pin.NormalW = normalize( pin.NormalW );

	float4 texColor = gDiffuseMap.Sample( samAnisotropic, pin.Tex );

	return texColor;
}

technique11 TerrainTech {
	pass P0 {
		SetVertexShader( CompileShader(vs_5_0, VS()) );
		SetGeometryShader( NULL );
		SetPixelShader( CompileShader(ps_5_0, PS()) );
	}
}