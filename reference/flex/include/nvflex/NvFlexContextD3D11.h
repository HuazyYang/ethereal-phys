// This code contains NVIDIA Confidential Information and is disclosed to you
// under a form of NVIDIA software license agreement provided separately to you.
//
// Notice
// NVIDIA Corporation and its licensors retain all intellectual property and
// proprietary rights in and to this software and related documentation and
// any modifications thereto. Any use, reproduction, disclosure, or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA Corporation is strictly prohibited.
//
// ALL NVIDIA DESIGN SPECIFICATIONS, CODE ARE PROVIDED "AS IS.". NVIDIA MAKES
// NO WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ALL IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Information and code furnished is believed to be accurate and reliable.
// However, NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2014-2021 NVIDIA Corporation. All rights reserved.

#pragma once

// --------------------------- NvFlexContextD3D11 -------------------------------
///@defgroup NvFlexContextD3D11
///@{

struct NvFlexDepthStencilViewDescD3D11
{
	ID3D11DepthStencilView* dsv;
	ID3D11ShaderResourceView* srv;
	D3D11_VIEWPORT viewport;
};

struct NvFlexRenderTargetViewDescD3D11
{
	ID3D11RenderTargetView* rtv;
	D3D11_VIEWPORT viewport;
};

struct NvFlexContextDescD3D11
{
	ID3D11Device* device;
	ID3D11DeviceContext* deviceContext;
};

struct NvFlexResourceViewDescD3D11
{
	ID3D11ShaderResourceView* srv;
};

struct NvFlexResourceRWViewDescD3D11
{
	NvFlexResourceViewDescD3D11 resourceView;
	ID3D11UnorderedAccessView* uav;
};

/**
 * Creates a graphics/compute context for Flex.
 *
 * @param[in] version Should be set by app to NV_FLEX_VERSION.
 * @param[in] desc A graphics-API dependent structure containing data needed for a FlexContext to interoperate with the app.
 *
 * @return The created Flex context.
 */
NV_FLEX_API NvFlexContext* NvFlexCreateContextD3D11(NvFlexUint version, const NvFlexContextDescD3D11* desc);

/**
 * Creates a Flex depth stencil view based on information provided by the application.
 *
 * @param[in] context The Flex context to create and use the depth stencil view.
 * @param[in] desc The graphics API dependent description.
 *
 * @return The created Flex depth stencil view.
 */
NV_FLEX_API NvFlexDepthStencilView* NvFlexCreateDepthStencilViewD3D11(NvFlexContext* context, const NvFlexDepthStencilViewDescD3D11* desc);

/**
 * Creates a Flex render target view based on information provided by the application.
 *
 * @param[in] context The Flex context to create and use the render target view.
 * @param[in] desc The graphics API dependent description.
 *
 * @return The created Flex render target view.
 */
NV_FLEX_API NvFlexRenderTargetView* NvFlexCreateRenderTargetViewD3D11(NvFlexContext* context, const NvFlexRenderTargetViewDescD3D11* desc);

/**
 * Updates a Flex context with information provided by the application.
 *
 * @param[in] context The Flex context to update.
 * @param[in] desc The graphics API dependent description.
 */
NV_FLEX_API void NvFlexUpdateContextD3D11(NvFlexContext* context, const NvFlexContextDescD3D11* desc);

/**
 * Gets a Flex context description from a Flex context.
 *
 * @param[in] context The Flex context.
 * @param[out] desc The graphics API dependent description.
 */
NV_FLEX_API void NvFlexUpdateContextDescD3D11(NvFlexContext* context, NvFlexContextDescD3D11* desc);

/**
 * Updates a Flex depth stencil view with information provided by the application.
 *
 * @param[in] context The Flex context used to create the depth stencil view.
 * @param[in] view The Flex depth stencil view to update.
 * @param[in] desc The graphics API dependent description.
 */
NV_FLEX_API void NvFlexUpdateDepthStencilViewD3D11(NvFlexContext* context, NvFlexDepthStencilView* view, const NvFlexDepthStencilViewDescD3D11* desc);

/**
 * Updates a Flex render target view with information provided by the application.
 *
 * @param[in] context The Flex context used to create the render target view.
 * @param[in] view The Flex render target view to update.
 * @param[in] desc The graphics API dependent description.
 */
NV_FLEX_API void NvFlexUpdateRenderTargetViewD3D11(NvFlexContext* context, NvFlexRenderTargetView* view, const NvFlexRenderTargetViewDescD3D11* desc);

/**
 * Updates an application visible description with internal Flex resource information.
 *
 * @param[in] context The Flex context that created the resource.
 * @param[in] resource The Flex resource to describe.
 * @param[out] desc The graphics API dependent Flex resource description.
 */
NV_FLEX_API void NvFlexUpdateResourceViewDescD3D11(NvFlexContext* context, NvFlexResource* resource, NvFlexResourceViewDescD3D11* desc);

/**
 * Updates an application visible description with internal Flex resourceRW information.
 *
 * @param[in] context The Flex context that created the resourceRW.
 * @param[in] resourceRW The Flex resourceRW to describe.
 * @param[out] desc The graphics API dependent Flex resourceRW description.
 */
NV_FLEX_API void NvFlexUpdateResourceRWViewDescD3D11(NvFlexContext* context, NvFlexResourceRW* resourceRW, NvFlexResourceRWViewDescD3D11* desc);

///@}