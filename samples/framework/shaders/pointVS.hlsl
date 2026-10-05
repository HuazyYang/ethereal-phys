// Ported from the FleX demo: FleX/demo/d3d/shaders/pointVS.hlsl (the D3D11 / D3D12 renderers' shaders).
// Compiled at shader model 6.0 for DXIL and SPIR-V and at 5.0 for DXBC; the changes are listed in shaders.cfg.

#include "shaderCommon.h"

cbuffer constBuf : register(b0)
{
	PointShaderConst gParams;
};

PointVertexOut pointVS(PointVertexIn input, uint instance : SV_VertexID)
{
	const float4 modelPosition = input.position;
	const float4x4 modelViewMatrix = gParams.modelView;

	float density = input.density;
	int phase = input.phase;

	// calculate window-space point size
	float4 viewPos = mul(modelViewMatrix, float4(modelPosition.xyz, 1.0));

	PointVertexOut output;
	output.viewPosition = viewPos;
	output.density = density;
	output.phase = phase;
	output.modelPosition = modelPosition;
	return output;
}
