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

// --------------------------- NvFlexContextD3D12 -------------------------------
///@defgroup NvFlexContextD3D12
///@{

struct NvFlexDepthStencilViewDescD3D12
{
	D3D12_CPU_DESCRIPTOR_HANDLE dsvHandle;
	D3D12_DEPTH_STENCIL_VIEW_DESC dsvDesc;
	ID3D12Resource* dsvResource;
	D3D12_RESOURCE_STATES dsvCurrentState;

	D3D12_CPU_DESCRIPTOR_HANDLE srvHandle;
	D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc;
	ID3D12Resource* srvResource;
	D3D12_RESOURCE_STATES srvCurrentState;

	D3D12_VIEWPORT viewport;
};

struct NvFlexRenderTargetViewDescD3D12
{
	D3D12_CPU_DESCRIPTOR_HANDLE rtvHandle;
	D3D12_RENDER_TARGET_VIEW_DESC rtvDesc;
	ID3D12Resource* resource;
	D3D12_RESOURCE_STATES currentState;
	D3D12_VIEWPORT viewport;
	D3D12_RECT scissor;
};

struct NvFlexDescriptorReserveHandleD3D12
{
	ID3D12DescriptorHeap* heap;
	NvFlexUint descriptorSize;
	D3D12_CPU_DESCRIPTOR_HANDLE cpuHandle;
	D3D12_GPU_DESCRIPTOR_HANDLE gpuHandle;
};

struct NvFlexDynamicDescriptorHeapD3D12
{
	void* userdata;
	NvFlexDescriptorReserveHandleD3D12 (*reserveDescriptors)(void* userdata, NvFlexUint numDescriptors, NvFlexUint64 lastFenceCompleted, NvFlexUint64 nextFenceValue);
};

struct NvFlexContextDescD3D12
{
	ID3D12Device* device;						//!< The desired d3d12 device to use
	ID3D12CommandQueue* commandQueue;			//!< The commandQueue commandList will be submit on
	ID3D12Fence* commandQueueFence;				//!< Fence marking events on this queue
	ID3D12GraphicsCommandList* commandList;		//!< The commandlist for recording
	UINT64 lastFenceCompleted;					//!< The last fence completed on commandQueue
	UINT64 nextFenceValue;						//!< The fence value signaled after commandList is submitted

	NvFlexDynamicDescriptorHeapD3D12 dynamicHeapCbvSrvUav; //!< Optional interface to share app descriptor heap with Flex
};

struct NvFlexResourceViewDescD3D12
{
	D3D12_CPU_DESCRIPTOR_HANDLE srvHandle;
	D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc;
	ID3D12Resource* resource;
	D3D12_RESOURCE_STATES* currentState;
};

struct NvFlexResourceRWViewDescD3D12
{
	NvFlexResourceViewDescD3D12 resourceView;
	D3D12_CPU_DESCRIPTOR_HANDLE uavHandle;
	D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc;
};

/**
 * Creates a graphics/compute context for Flex.
 *
 * @param[in] version Should be set by app to NV_FLEX_VERSION.
 * @param[in] desc A graphics-API dependent structure containing data needed for a FlexContext to interoperate with the app.
 *
 * @return The created Flex context.
 */
NV_FLEX_API NvFlexContext* NvFlexCreateContextD3D12(NvFlexUint version, const NvFlexContextDescD3D12* desc);

/**
 * Creates a Flex depth stencil view based on information provided by the application.
 *
 * @param[in] context The Flex context to create and use the depth stencil view.
 * @param[in] desc The graphics API dependent description.
 *
 * @return The created Flex depth stencil view.
 */
NV_FLEX_API NvFlexDepthStencilView* NvFlexCreateDepthStencilViewD3D12(NvFlexContext* context, const NvFlexDepthStencilViewDescD3D12* desc);

/**
 * Creates a Flex render target view based on information provided by the application.
 *
 * @param[in] context The Flex context to create and use the render target view.
 * @param[in] desc The graphics API dependent description.
 *
 * @return The created Flex render target view.
 */
NV_FLEX_API NvFlexRenderTargetView* NvFlexCreateRenderTargetViewD3D12(NvFlexContext* context, const NvFlexRenderTargetViewDescD3D12* desc);

/**
 * Updates a Flex context with information provided by the application.
 *
 * @param[in] context The Flex context to update.
 * @param[in] desc The graphics API dependent description.
 */
NV_FLEX_API void NvFlexUpdateContextD3D12(NvFlexContext* context, const NvFlexContextDescD3D12* desc);

/**
 * Gets a Flex context description from a Flex context.
 *
 * @param[in] context The Flex context.
 * @param[out] desc The graphics API dependent description.
 */
NV_FLEX_API void NvFlexUpdateContextDescD3D12(NvFlexContext* context, NvFlexContextDescD3D12* desc);

/**
 * Updates a Flex depth stencil view with information provided by the application.
 *
 * @param[in] context The Flex context used to create the depth stencil view.
 * @param[in] view The Flex depth stencil view to update.
 * @param[in] desc The graphics API dependent description.
 */
NV_FLEX_API void NvFlexUpdateDepthStencilViewD3D12(NvFlexContext* context, NvFlexDepthStencilView* view, const NvFlexDepthStencilViewDescD3D12* desc);

/**
 * Updates a Flex render target view with information provided by the application.
 *
 * @param[in] context The Flex context used to create the render target view.
 * @param[in] view The Flex render target view to update.
 * @param[in] desc The graphics API dependent description.
 */
NV_FLEX_API void NvFlexUpdateRenderTargetViewD3D12(NvFlexContext* context, NvFlexRenderTargetView* view, const NvFlexRenderTargetViewDescD3D12* desc);

/**
 * Updates an application visible description with internal Flex resource information.
 *
 * @param[in] context The Flex context that created the resource.
 * @param[in] resource The Flex resource to describe.
 * @param[out] desc The graphics API dependent Flex resource description.
 */
NV_FLEX_API void NvFlexUpdateResourceViewDescD3D12(NvFlexContext* context, NvFlexResource* resource, NvFlexResourceViewDescD3D12* desc);

/**
 * Updates an application visible description with internal Flex resourceRW information.
 *
 * @param[in] context The Flex context that created the resourceRW.
 * @param[in] buffer The Flex resourceRW to describe.
 * @param[out] desc The graphics API dependent Flex resourceRW description.
 */
NV_FLEX_API void NvFlexUpdateResourceRWViewDescD3D12(NvFlexContext* context, NvFlexResourceRW* resourceRW, NvFlexResourceRWViewDescD3D12* desc);

///@}