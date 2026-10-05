// Ported from the FleX demo: FleX/demo/d3d/shaders/meshShadowPS.hlsl (the D3D11 / D3D12 renderers' shaders).
// Compiled at shader model 6.0 for DXIL and SPIR-V and at 5.0 for DXBC; the changes are listed in shaders.cfg.

#include "shaderCommon.h"

cbuffer constBuf : register(b0)
{
	MeshShaderConst gParams;
};


void meshPS_Shadow(MeshVertexOut input) 
{
}

