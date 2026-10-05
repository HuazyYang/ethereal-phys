// Ported from the FleX demo: FleX/demo/d3d/shaders/pointShadowPS.hlsl (the D3D11 / D3D12 renderers' shaders).
// Compiled at shader model 6.0 for DXIL and SPIR-V and at 5.0 for DXBC; the changes are listed in shaders.cfg.

#include "shaderCommon.h"

cbuffer constBuf : register(b0)
{
	PointShaderConst gParams;
};

void pointShadowPS(PointGeoOut input) 
{
	// calculate normal from texture coordinates
	float2 normal = input.texCoord.xy - float2(0.5, 0.5);
	float mag = dot(normal.xy, normal.xy);
	if (mag > 0.5 * 0.5) 
	{
		discard;   // kill pixels outside circle
	}
}
