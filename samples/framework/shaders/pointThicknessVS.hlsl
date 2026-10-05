// Ported from the FleX demo: FleX/demo/d3d/shaders/pointThicknessVS.hlsl (the D3D11 / D3D12 renderers' shaders).
// Compiled at shader model 6.0 for DXIL and SPIR-V and at 5.0 for DXBC; the changes are listed in shaders.cfg.

#include "shaderCommon.h"

#pragma warning (disable : 3578)

cbuffer constBuf : register(b0)
{
	FluidShaderConst gParams;
};

// The attribute of FluidVertexIn this shader reads (FleX's took the whole FluidVertexIn; see shaders.cfg).
struct PointThicknessVertexIn
{
	float4 position : POSITION;
};

FluidVertexOut pointThicknessVS(PointThicknessVertexIn input, uint instance : SV_VertexID)
{
	const float4 gl_Vertex = input.position;
	const float4x4 gl_ModelViewMatrix = gParams.modelView;

	// calculate window-space point size
	float4 viewPos = mul(gl_ModelViewMatrix, float4(gl_Vertex.xyz, 1.0));

	FluidVertexOut output;
	output.position = viewPos;
	output.bounds = gl_Vertex;	// store gl_Vertex in bounds

	return output;
}
