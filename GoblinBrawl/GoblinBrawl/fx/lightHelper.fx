struct Material {
	float4 Ambient;
	float4 Diffuse;
	float4 Specular; // w = SpecPower
};

struct PointLight {
	float4 Ambient;
	float4 Diffuse;
	float4 Specular;

	float3 Position;
	float Range;

	float3 Att;
	float Pad;
};

void ComputePointLight( Material mat, PointLight L, float3 pos, float3 normal, float3 toEye, out float4 ambient, out float4 diffuse, out float4 spec, out float4 dirAmbient ) {
	
	ambient = float4 (0.f, 0.f, 0.f, 0.f);
	diffuse = float4 (0.f, 0.f, 0.f, 0.f);
	spec = float4 (0.f, 0.f, 0.f, 0.f);
	dirAmbient = float4 (0.f, 0.f, 0.f, 0.f);

	float3 lightVec = L.Position-pos;
		float d = length( lightVec );

	if( d>L.Range ) {
		return;
	}

	lightVec /= d; //normalize

	ambient = mat.Ambient * L.Ambient;

	dirAmbient = dot( lightVec, normal );
	dirAmbient = dirAmbient*0.5f+0.5f;
	dirAmbient = dirAmbient*dirAmbient;

	float diffuseFactor = dot( lightVec, normal );
	diffuseFactor = diffuseFactor*0.5f+0.5f;
	diffuseFactor = diffuseFactor*diffuseFactor;

	[flatten]
	if( diffuseFactor>0.0f ) {
		float3 v = reflect( -lightVec, normal );
		float specFactor = pow( max( dot( v, toEye ), 0.0f ), mat.Specular.w );
		diffuse = diffuseFactor * mat.Diffuse * L.Diffuse;
		spec = specFactor * mat.Specular * L.Specular;
	}

	// attenuate
	float att = 1.0f/dot( L.Att, float3(1.0f, d, d * d) );
	diffuse *= att;
	spec *= att;

	//d *= 3.f;
	float dirAmbientAtt = att*att;// /2.f;// 1.0f/dot( L.Att, float3(d, d*d, d*d*d) );
	dirAmbient *= dirAmbientAtt;
}