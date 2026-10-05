// Ported from the FleX demo: FleX/demo/d3d/shaders/passThroughVS.hlsl (the D3D11 / D3D12 renderers' shaders).
// Compiled at shader model 6.0 for DXIL and SPIR-V and at 5.0 for DXBC; the changes are listed in shaders.cfg.

#include "shaderCommon.h"

cbuffer constBuf : register(b0)
{
	FluidShaderConst gParams;
};

// The attributes of MeshVertexIn this shader reads (FleX's took the whole MeshVertexIn; see shaders.cfg).
struct PassthroughVertexIn
{
	float3 position : POSITION;
	float2 texCoord : TEXCOORD;
};

PassthroughVertexOut passThroughVS(PassthroughVertexIn input)
{
	PassthroughVertexOut output;
	output.position = float4(input.position, 1.0f);
	output.texCoord = input.texCoord;
	return output;
}
