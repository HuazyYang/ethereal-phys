// Ported from the FleX demo: FleX/demo/d3d/shaders/debugLinePS.hlsl (the D3D11 / D3D12 renderers' shaders).
// Compiled at shader model 6.0 for DXIL and SPIR-V and at 5.0 for DXBC; the changes are listed in shaders.cfg.

struct Input
{
	float4 position : SV_POSITION;
	float4 color : COLOR;
};

float4 debugLinePS(Input input) : SV_TARGET
{
	return input.color;
}
