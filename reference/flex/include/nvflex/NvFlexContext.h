// This code contains NVIDIA Confidential Information and is disclosed to you

#ifndef NVFLEXCONTEXT_H
#define NVFLEXCONTEXT_H
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


#include "NvFlexTypes.h"

//! NvFlexContext: A framework for fluid simulation
struct NvFlexContext;

//! API type
enum NvFlexContextAPI
{
	eNvFlexContextD3D11 = 1,
	eNvFlexContextD3D12 = 2
};

//! import interop buffers
struct NvFlexDepthStencilView;
struct NvFlexRenderTargetView;

//! export interop buffers
struct NvFlexResource;
struct NvFlexResourceRW;
struct NvFlexBuffer;
struct NvFlexTexture3D;

// --------------------------- NvFlexContext -------------------------------
///@defgroup NvFlexContext
///@{

/**
 * Get the API type of the current context
 *
 * @param[in] context The Flex context to get the type of.
 *
 * @return context The Flex context to be released.
 */
NV_FLEX_API NvFlexContextAPI NvFlexContextGetContextType(NvFlexContext* context);

/**
 * Push a request for the Flex context to request a flush to queue
 *
 * @param[in] context The Flex context to make the request on.
 */
NV_FLEX_API void NvFlexContextFlushRequestPush(NvFlexContext* context);

/**
 * Pop any pending requests for the Flex context to flush to queue, resets the request state
 *
 * @param[in] context The Flex context to check for requests on.
 *
 * @return true if a flush is requested
 */
NV_FLEX_API bool NvFlexContextFlushRequestPop(NvFlexContext* context);

/**
 * Process pending GPU wait on fence, on deviceQueue associated with this context
 *
 * @param[in] context The Flex context to submit fence waits on.
 */
NV_FLEX_API void NvFlexContextProcessFenceWait(NvFlexContext* context);

/**
 * Process pending GPU fence signals, on deviceQueue associated with this context
 *
 * @param[in] context The Flex context to submit fence signals on.
 */
NV_FLEX_API void NvFlexContextProcessFenceSignal(NvFlexContext* context);

/**
 * Releases a Flex context.
 *
 * @param[in] context The Flex context to be released.
 */
NV_FLEX_API void NvFlexReleaseContext(NvFlexContext* context);

/**
 * Releases a Flex depth stencil view.
 *
 * @param[in] view The Flex depth stencil view to be released.
 */
NV_FLEX_API void NvFlexReleaseDepthStencilView(NvFlexDepthStencilView* view);

/**
 * Releases a Flex render target view.
 *
 * @param[in] view The Flex render target view to be released.
 */
NV_FLEX_API void NvFlexReleaseRenderTargetView(NvFlexRenderTargetView* view);

/**
 * Pushes graphics/compute pipeline state for later restoration by NvFlexContextPop.
 *
 * @param[in] context The Flex context to push.
 */
NV_FLEX_API void NvFlexContextPush(NvFlexContext* context);

/**
 * Restores graphics/compute pipeline state pushed by NvFlexContextPush.
 *
 * @param[in] context The Flex context to restore.
 */
NV_FLEX_API void NvFlexContextPop(NvFlexContext* context);

/**l
 * An optional callback to allow the application to control how Flex allocates CPU memory.
 *
 * @param[in] malloc The allocation function for Flex to use.
 */
NV_FLEX_API void NvFlexSetMallocFunc(void*(*malloc)(size_t size));

/**
 * An optional callback to allow the application to contro how Flex releases CPU memory.
 *
 * @param[in] free The free function for Flex to use.
 */
NV_FLEX_API void NvFlexSetFreeFunc(void(*free)(void* ptr));

/**
 * Should be called before DLL unload, to ensure complete cleanup.
 *
 * @param[in] timeoutMS Wait timeout, in milliseconds
 *
 * @return The current number of active deferred release units.
 */
NV_FLEX_API NvFlexUint NvFlexDeferredRelease(float timeoutMS);

///@}

#endif /* NVFLEXCONTEXT_H */
