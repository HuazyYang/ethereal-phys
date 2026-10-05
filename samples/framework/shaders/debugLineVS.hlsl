// Ported from the FleX demo: FleX/demo/d3d/shaders/debugLineVS.hlsl (the D3D11 / D3D12 renderers' shaders).
// Compiled at shader model 6.0 for DXIL and SPIR-V and at 5.0 for DXBC; the changes are listed in shaders.cfg.


cbuffer params : register(b0)
{
	float4x4  projectionViewWorld;
};

struct Input
{
	float3 position : POSITION;
	float4 color : COLOR;
};

struct Output
{
	float4 position : SV_POSITION;
	float4 color : COLOR;
};

Output debugLineVS(Input input)
{
	Output output;
	output.position = mul(projectionViewWorld, float4(input.position, 1.0f));
	output.color = input.color;

	return output;
}
