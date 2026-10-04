// Ported from the FleX demo: reference/flex/demo/d3d/shaders/diffusePS.hlsl (the D3D11 / D3D12 renderers' shaders).
// Compiled at shader model 6.0 for DXIL and SPIR-V and at 5.0 for DXBC; the changes are listed in shaders.cfg.

#include "shaderCommon.h"

cbuffer constBuf : register(b0)
{
	DiffuseShaderConst gParams;
};

float sqr(float x) { return x * x; }

float4 diffusePS(DiffuseGeometryOut input) : SV_TARGET
{
	float attenuation = 1.0f;
	float lifeTime = input.worldPos.w;
	float lifeFade = min(1.0, lifeTime*0.125);
	float velocityFade = input.viewVel.w;

    // calculate normal from texture coordinates
    float3 normal = float3(input.uv * 2.0 + float2(-1.0, -1.0), 0.0);
    float mag = dot(normal.xy, normal.xy);
    
	// kill pixels outside circle
	if (mag > 1.0)
		discard;   

   	normal.z = 1.0-mag;

	float alpha = lifeFade*velocityFade*sqr(normal.z);
	return float4(alpha, alpha, alpha, alpha);
}
