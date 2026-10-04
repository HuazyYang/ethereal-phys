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

#include "NvFlexContext.h"

//! GPU Graphics and compute interface
struct NvFlexContext;

struct NvFlexConstantBuffer;
struct NvFlexVertexBuffer;
struct NvFlexIndexBuffer;
struct NvFlexResource;
struct NvFlexResourceRW;
struct NvFlexBuffer;
struct NvFlexTexture1D;
struct NvFlexTexture2D;
struct NvFlexTexture3D;

//! Interface to manage resource reference count and lifetime
struct NvFlexContextObject;

NV_FLEX_API NvFlexUint NvFlexContextObjectAddRef(NvFlexContextObject* object);

NV_FLEX_API NvFlexUint NvFlexContextObjectRelease(NvFlexContextObject* object);

NV_FLEX_API NvFlexUint64 NvFlexContextObjectGetGPUBytesUsed(NvFlexContextObject* object);

enum NvFlexMemoryType {
    eNvFlexMemoryType_default = 0,
    eNvFlexMemoryType_upload = 2,
    eNvFlexMemoryType_readback = 3
};

enum NvFlexCpuAccessMode {
    eNvFlexCpuAccessMode_none = 0,
    eNvFlexCpuAccessMode_read = 1,
    eNvFlexCpuAccessMode_write = 2,
    eNvFlexCpuAccessMode_readwrite = 3
};

enum NvFlexStagingCpuAccessFlags {
    eNvFlexStagingCpuAccess_none = 0,
    eNvFlexStagingCpuAccess_read = 1,
    eNvFlexStagingCpuAccess_write = 2,
    eNvFlexStagingCpuAccess_readwrite = 3
};

//! Handle for mapped pitched data
struct NvFlexMappedData
{
	void* data;
	NvFlexUint rowPitch;
	NvFlexUint depthPitch;
};

//! A constant buffer
struct NvFlexConstantBuffer;

struct NvFlexConstantBufferDesc
{
	NvFlexUint sizeInBytes;
	bool uploadAccess;
    const char* debugName;
};

NV_FLEX_API void NvFlexConstantBufferGetDesc(NvFlexConstantBuffer* buffer, NvFlexConstantBufferDesc* desc);

NV_FLEX_API NvFlexConstantBuffer* NvFlexCreateConstantBuffer(NvFlexContext* context, const NvFlexConstantBufferDesc* desc);

NV_FLEX_API void NvFlexReleaseConstantBuffer(NvFlexConstantBuffer* buffer);

NV_FLEX_API NvFlexContextObject* NvFlexConstantBufferGetContextObject(NvFlexConstantBuffer* buffer);

NV_FLEX_API void* NvFlexConstantBufferMap(NvFlexContext* context, NvFlexConstantBuffer* constantBuffer);

NV_FLEX_API void NvFlexConstantBufferUnmap(NvFlexContext* context, NvFlexConstantBuffer* constantBuffer);

//! A vertex buffer
struct NvFlexVertexBuffer;

struct NvFlexVertexBufferDesc
{
	const void* data;
	NvFlexUint sizeInBytes;
};

NV_FLEX_API void NvFlexVertexBufferGetDesc(NvFlexVertexBuffer* buffer, NvFlexVertexBufferDesc* desc);

NV_FLEX_API NvFlexVertexBuffer* NvFlexCreateVertexBuffer(NvFlexContext* context, const NvFlexVertexBufferDesc* desc);

NV_FLEX_API void NvFlexReleaseVertexBuffer(NvFlexVertexBuffer* vertexBuffer);

NV_FLEX_API NvFlexContextObject* NvFlexVertexBufferGetContextObject(NvFlexVertexBuffer* buffer);

NV_FLEX_API void* NvFlexVertexBufferMap(NvFlexContext* context, NvFlexVertexBuffer* vertexBuffer);

NV_FLEX_API void NvFlexVertexBufferUnmap(NvFlexContext* context, NvFlexVertexBuffer* vertexBuffer);

//! An index buffer
struct NvFlexIndexBuffer;

struct NvFlexIndexBufferDesc
{
	const void* data;
	NvFlexUint sizeInBytes;
	NvFlexFormat format;
};

NV_FLEX_API void NvFlexIndexBufferGetDesc(NvFlexIndexBuffer* index, NvFlexIndexBufferDesc* desc);

NV_FLEX_API NvFlexIndexBuffer* NvFlexCreateIndexBuffer(NvFlexContext* context, const NvFlexIndexBufferDesc* desc);

NV_FLEX_API void NvFlexReleaseIndexBuffer(NvFlexIndexBuffer* buffer);

NV_FLEX_API NvFlexContextObject* NvFlexIndexBufferGetContextObject(NvFlexIndexBuffer* buffer);

NV_FLEX_API void* NvFlexIndexBufferMap(NvFlexContext* context, NvFlexIndexBuffer* indexBuffer);

NV_FLEX_API void NvFlexIndexBufferUnmap(NvFlexContext* context, NvFlexIndexBuffer* indexBuffer);

//! A read only resource interface
struct NvFlexResource;

NV_FLEX_API NvFlexContextObject* NvFlexResourceGetContextObject(NvFlexResource* resource);

//! A read/write resource interface
struct NvFlexResourceRW;

NV_FLEX_API NvFlexContextObject* NvFlexResourceRWGetContextObject(NvFlexResourceRW* resourceRW);

NV_FLEX_API NvFlexResource* NvFlexResourceRWGetResource(NvFlexResourceRW* resourceRW);

//! A render target interface
struct NvFlexRenderTarget;

//! Viewport description for rendering
struct NvFlexViewport
{
	float topLeftX;
	float topLeftY;
	float width;
	float height;
	float minDepth;
	float maxDepth;
};

struct NvFlexRenderTargetDesc
{
	NvFlexViewport viewport;
	NvFlexFormat rt_format;
};

NV_FLEX_API void NvFlexRenderTargetGetDesc(NvFlexRenderTarget* rt, NvFlexRenderTargetDesc* desc);

NV_FLEX_API void NvFlexRenderTargetSetViewport(NvFlexRenderTarget* rt, const NvFlexViewport* viewport);

//! A depth stencil inteface
struct NvFlexDepthStencil;

struct NvFlexDepthStencilDesc
{
	NvFlexFormat ds_format;
	NvFlexViewport viewport;
	NvFlexUint width;
	NvFlexUint height;
};

NV_FLEX_API void NvFlexDepthStencilGetDesc(NvFlexDepthStencil* ds, NvFlexDepthStencilDesc* desc);

NV_FLEX_API void NvFlexDepthStencilSetViewport(NvFlexDepthStencil* ds, const NvFlexViewport* viewport);

//! A buffer
struct NvFlexBuffer;

struct NvFlexBufferDesc
{
    NvFlexFormat format; // For typed buffer
    NvFlexUint structStride; // For structured buffer
    NvFlexUint dim;
    NvFlexMemoryType memType;
    NvFlexCpuAccessMode cpuAccessMode;
    bool isIndirectArgs;
    const char* debugName;
};

struct NvFlexBufferViewDesc
{
	NvFlexFormat format;
};

NV_FLEX_API void NvFlexBufferGetDesc(NvFlexBuffer* buffer, NvFlexBufferDesc* desc);

NV_FLEX_API NvFlexBuffer* NvFlexCreateBuffer(NvFlexContext* context, const NvFlexBufferDesc* desc);

NV_FLEX_API NvFlexBuffer* NvFlexCreateBufferView(NvFlexContext* context, NvFlexBuffer* buffer, const NvFlexBufferViewDesc* desc);

NV_FLEX_API void NvFlexReleaseBuffer(NvFlexBuffer* buffer);

NV_FLEX_API NvFlexContextObject* NvFlexBufferGetContextObject(NvFlexBuffer* buffer);

NV_FLEX_API NvFlexResource* NvFlexBufferGetResource(NvFlexBuffer* buffer);

NV_FLEX_API NvFlexResourceRW* NvFlexBufferGetResourceRW(NvFlexBuffer* buffer);

//! A 1D Texture
struct NvFlexTexture1D;

struct NvFlexTexture1DDesc
{
	NvFlexFormat format;
	NvFlexUint dim;
	bool uploadAccess;
};

NV_FLEX_API void NvFlexTexture1DGetDesc(NvFlexTexture1D* tex, NvFlexTexture1DDesc* desc);

NV_FLEX_API NvFlexTexture1D* NvFlexCreateTexture1D(NvFlexContext* context, const NvFlexTexture1DDesc* desc);

NV_FLEX_API void NvFlexReleaseTexture1D(NvFlexTexture1D* tex);

NV_FLEX_API NvFlexContextObject* NvFlexTexture1DGetContextObject(NvFlexTexture1D* tex);

NV_FLEX_API NvFlexResource* NvFlexTexture1DGetResource(NvFlexTexture1D* tex);

NV_FLEX_API NvFlexResourceRW* NvFlexTexture1DGetResourceRW(NvFlexTexture1D* tex);

NV_FLEX_API void* NvFlexTexture1DMap(NvFlexContext* context, NvFlexTexture1D* tex);

NV_FLEX_API void NvFlexTexture1DUnmap(NvFlexContext* context, NvFlexTexture1D* tex);

//! A 2D Texture
struct NvFlexTexture2D;

struct NvFlexTexture2DDesc
{
	NvFlexFormat format;
	NvFlexUint width;
	NvFlexUint height;
};

NV_FLEX_API void NvFlexTexture2DGetDesc(NvFlexTexture2D* tex, NvFlexTexture2DDesc* desc);

NV_FLEX_API NvFlexTexture2D* NvFlexCreateTexture2D(NvFlexContext* context, const NvFlexTexture2DDesc* desc);

NV_FLEX_API NvFlexTexture2D* NvFlexShareTexture2D(NvFlexContext* context, NvFlexTexture2D* sharedTexture);

NV_FLEX_API NvFlexTexture2D* NvFlexCreateTexture2DCrossAPI(NvFlexContext* context, const NvFlexTexture2DDesc* desc);

NV_FLEX_API NvFlexTexture2D* NvFlexShareTexture2DCrossAPI(NvFlexContext* context, NvFlexTexture2D* sharedTexture);

NV_FLEX_API void NvFlexReleaseTexture2D(NvFlexTexture2D* tex);

NV_FLEX_API NvFlexContextObject* NvFlexTexture2DGetContextObject(NvFlexTexture2D* tex);

NV_FLEX_API NvFlexResource* NvFlexTexture2DGetResource(NvFlexTexture2D* tex);

NV_FLEX_API NvFlexResourceRW* NvFlexTexture2DGetResourceRW(NvFlexTexture2D* tex);

//! A 3D Texture
struct NvFlexTexture3D;

struct NvFlexTexture3DDesc
{
	NvFlexFormat format;
	NvFlexDim dim;
	bool uploadAccess;
	bool downloadAccess;
};

NV_FLEX_API void NvFlexTexture3DGetDesc(NvFlexTexture3D* tex, NvFlexTexture3DDesc* desc);

NV_FLEX_API NvFlexTexture3D* NvFlexCreateTexture3D(NvFlexContext* context, const NvFlexTexture3DDesc* desc);

NV_FLEX_API void NvFlexReleaseTexture3D(NvFlexTexture3D* tex);

NV_FLEX_API NvFlexContextObject* NvFlexTexture3DGetContextObject(NvFlexTexture3D* tex);

NV_FLEX_API NvFlexResource* NvFlexTexture3DGetResource(NvFlexTexture3D* tex);

NV_FLEX_API NvFlexResourceRW* NvFlexTexture3DGetResourceRW(NvFlexTexture3D* tex);

NV_FLEX_API NvFlexMappedData NvFlexTexture3DMap(NvFlexContext* context, NvFlexTexture3D* tex);

NV_FLEX_API void NvFlexTexture3DUnmap(NvFlexContext* context, NvFlexTexture3D* tex);

NV_FLEX_API void NvFlexTexture3DDownload(NvFlexContext* context, NvFlexTexture3D* tex);

NV_FLEX_API NvFlexMappedData NvFlexTexture3DMapDownload(NvFlexContext* context, NvFlexTexture3D* tex);

NV_FLEX_API void NvFlexTexture3DUnmapDownload(NvFlexContext* context, NvFlexTexture3D* tex);

//! A memory heap for 3D Hardware Sparse Texture
struct NvFlexHeapSparse;

struct NvFlexHeapSparseDesc
{
	NvFlexUint sizeInBytes;
};

NV_FLEX_API void NvFlexHeapSparseGetDesc(NvFlexHeapSparse* heap, NvFlexHeapSparseDesc* desc);

NV_FLEX_API NvFlexHeapSparse* NvFlexCreateHeapSparse(NvFlexContext* context, const NvFlexHeapSparseDesc* desc);

NV_FLEX_API void NvFlexReleaseHeapSparse(NvFlexHeapSparse* heap);

NV_FLEX_API NvFlexContextObject* NvFlexHeapSparseGetContextObject(NvFlexHeapSparse* heap);

//! A 3D Hardware Sparse Texture
struct NvFlexTexture3DSparse;

struct NvFlexTexture3DSparseDesc
{
	NvFlexFormat format;
	NvFlexDim dim;
};

NV_FLEX_API void NvFlexTexture3DSparseGetDesc(NvFlexTexture3DSparse* tex, NvFlexTexture3DSparseDesc* desc);

NV_FLEX_API NvFlexTexture3DSparse* NvFlexCreateTexture3DSparse(NvFlexContext* context, const NvFlexTexture3DSparseDesc* desc);

NV_FLEX_API void NvFlexReleaseTexture3DSparse(NvFlexTexture3DSparse* tex);

NV_FLEX_API NvFlexContextObject* NvFlexTexture3DSparseGetContextObject(NvFlexTexture3DSparse* tex);

NV_FLEX_API NvFlexResource* NvFlexTexture3DSparseGetResource(NvFlexTexture3DSparse* tex);

NV_FLEX_API NvFlexResourceRW* NvFlexTexture3DSparseGetResourceRW(NvFlexTexture3DSparse* tex);

//! A 2D texture with render target support
struct NvFlexColorBuffer;

struct NvFlexColorBufferDesc
{
	NvFlexFormat format;
	NvFlexUint width;
	NvFlexUint height;
};

NV_FLEX_API void NvFlexColorBufferGetDesc(NvFlexColorBuffer* tex, NvFlexColorBufferDesc* desc);

NV_FLEX_API NvFlexColorBuffer* NvFlexCreateColorBuffer(NvFlexContext* context, const NvFlexColorBufferDesc* desc);

NV_FLEX_API void NvFlexReleaseColorBuffer(NvFlexColorBuffer* tex);

NV_FLEX_API NvFlexContextObject* NvFlexColorBufferGetContextObject(NvFlexColorBuffer* tex);

NV_FLEX_API NvFlexResource* NvFlexColorBufferGetResource(NvFlexColorBuffer* tex);

NV_FLEX_API NvFlexResourceRW* NvFlexColorBufferGetResourceRW(NvFlexColorBuffer* tex);

NV_FLEX_API NvFlexRenderTarget* NvFlexColorBufferGetRenderTarget(NvFlexColorBuffer* tex);

//! A 2D texture with depth stencil support
struct NvFlexDepthBuffer;

struct NvFlexDepthBufferDesc
{
	NvFlexFormat format_resource;
	NvFlexFormat format_dsv;
	NvFlexFormat format_srv;
	NvFlexUint width;
	NvFlexUint height;
};

NV_FLEX_API void NvFlexDepthBufferGetDesc(NvFlexDepthBuffer* depthBuffer, NvFlexDepthBufferDesc* desc);

NV_FLEX_API NvFlexDepthBuffer* NvFlexCreateDepthBuffer(NvFlexContext* context, const NvFlexDepthBufferDesc* desc);

NV_FLEX_API void NvFlexReleaseDepthBuffer(NvFlexDepthBuffer* depthBuffer);

NV_FLEX_API NvFlexContextObject* NvFlexDepthBufferGetContextObject(NvFlexDepthBuffer* depthBuffer);

NV_FLEX_API NvFlexResource* NvFlexDepthBufferGetResource(NvFlexDepthBuffer* depthBuffer);

NV_FLEX_API NvFlexDepthStencil* NvFlexDepthBufferGetDepthStencil(NvFlexDepthBuffer* depthBuffer);

//! A depth stencil imported from the app
struct NvFlexDepthStencilView;

NV_FLEX_API NvFlexResource* NvFlexDepthStencilViewGetResource(NvFlexDepthStencilView* dsv);

NV_FLEX_API NvFlexDepthStencil* NvFlexDepthStencilViewGetDepthStencil(NvFlexDepthStencilView* dsv);

NV_FLEX_API void NvFlexDepthStencilViewGetDepthBufferDesc(NvFlexDepthStencilView* dsv, NvFlexDepthBufferDesc* desc);

//! A render target imported from the app
struct NvFlexRenderTargetView;

NV_FLEX_API NvFlexRenderTarget* NvFlexRenderTargetViewGetRenderTarget(NvFlexRenderTargetView* rtv);

//! Constants for dispatch and draw commands
#define NV_FLEX_DISPATCH_MAX_READ_TEXTURES ( 32u )
#define NV_FLEX_DISPATCH_MAX_WRITE_TEXTURES ( 8u )

#define NV_FLEX_DRAW_MAX_READ_TEXTURES ( 16u )
#define NV_FLEX_DRAW_MAX_WRITE_TEXTURES ( 1u )
#define NV_FLEX_MAX_RENDER_TARGETS ( 8u )

//! A compute shader
struct NvFlexComputeShader;

struct NvFlexComputeShaderDesc
{
	const void* cs;
	NvFlexUint64 cs_length;
	const wchar_t* label;
    int NVAPI_Slot;
};

struct NvFlexDispatchParams
{
	NvFlexComputeShader* shader;
	NvFlexDim gridDim;
	NvFlexConstantBuffer* rootConstantBuffer;
	NvFlexConstantBuffer* secondConstantBuffer;
	NvFlexResource* readOnly[NV_FLEX_DISPATCH_MAX_READ_TEXTURES];
	NvFlexResourceRW* readWrite[NV_FLEX_DISPATCH_MAX_WRITE_TEXTURES];
};

NV_FLEX_API NvFlexComputeShader* NvFlexCreateComputeShader(NvFlexContext* context, const NvFlexComputeShaderDesc* desc);

NV_FLEX_API void NvFlexReleaseComputeShader(NvFlexComputeShader* computeShader);

//! A graphics shader pipeline
struct NvFlexGraphicsShader;

struct NvFlexInputElementDesc
{
	const char* semanticName;
	NvFlexFormat format;
};

enum NvFlexBlendEnum
{
	eNvFlexBlend_Zero = 1,
	eNvFlexBlend_One = 2,
	eNvFlexBlend_SrcAlpha = 3,
	eNvFlexBlend_InvSrcAlpha = 4,
	eNvFlexBlend_DstAlpha = 5,
	eNvFlexBlend_InvDstAlpha = 6,

	eNvFlexBlend_EnumCount = 7,
};

enum NvFlexBlendOpEnum
{
	eNvFlexBlendOp_Add = 1,
	eNvFlexBlendOp_Subtract = 2,
	eNvFlexBlendOp_RevSubtract = 3,
	eNvFlexBlendOp_Min = 4,
	eNvFlexBlendOp_Max = 5,

	eNvFlexBlendOp_EnumCount = 6
};

enum NvFlexComparisonEnum
{
	eNvFlexComparison_Never = 1,
	eNvFlexComparison_Less = 2,
	eNvFlexComparison_Equal = 3,
	eNvFlexComparison_LessEqual = 4,
	eNvFlexComparison_Greater = 5,
	eNvFlexComparison_NotEqual = 6,
	eNvFlexComparison_GreaterEqual = 7,
	eNvFlexComparison_Always = 8,

	eNvFlexComparison_EnumCount = 9
};

struct NvFlexBlendStateDesc
{
	bool enable;
	NvFlexBlendEnum srcBlendColor;
	NvFlexBlendEnum dstBlendColor;
	NvFlexBlendOpEnum blendOpColor;
	NvFlexBlendEnum srcBlendAlpha;
	NvFlexBlendEnum dstBlendAlpha;
	NvFlexBlendOpEnum blendOpAlpha;
};

enum NvFlexDepthWriteMask
{
	eNvFlexDepthWriteMask_Zero = 0,
	eNvFlexDepthWriteMask_All = 1
};

struct NvFlexDepthStateDesc
{
	bool depthEnable;
	NvFlexDepthWriteMask depthWriteMask;
	NvFlexComparisonEnum depthFunc;
};

struct NvFlexGraphicsShaderDesc
{
	const void* vs;
	NvFlexUint64 vs_length;
	const void* ps;
	NvFlexUint64 ps_length;
	const wchar_t* label;

	NvFlexUint numInputElements;
	NvFlexInputElementDesc* inputElementDescs;

	NvFlexBlendStateDesc blendState;
	NvFlexDepthStateDesc depthState;
	NvFlexUint numRenderTargets;
	NvFlexFormat renderTargetFormat[NV_FLEX_MAX_RENDER_TARGETS];
	NvFlexFormat depthStencilFormat;

	bool uavTarget;
	bool depthClipEnable;
	bool lineList;
};

NV_FLEX_API void NvFlexGraphicsShaderGetDesc(NvFlexGraphicsShader* shader, NvFlexGraphicsShaderDesc* desc);

struct NvFlexDrawParams
{
	NvFlexGraphicsShader* shader;
	NvFlexConstantBuffer* rootConstantBuffer;
	NvFlexResource* vs_readOnly[NV_FLEX_DRAW_MAX_READ_TEXTURES];
	NvFlexResource* ps_readOnly[NV_FLEX_DRAW_MAX_READ_TEXTURES];
	NvFlexResourceRW* ps_readWrite[NV_FLEX_DRAW_MAX_WRITE_TEXTURES];
	bool frontCounterClockwise;
};

NV_FLEX_API NvFlexGraphicsShader* NvFlexCreateGraphicsShader(NvFlexContext* context, const NvFlexGraphicsShaderDesc* desc);

NV_FLEX_API void NvFlexReleaseGraphicsShader(NvFlexGraphicsShader* shader);

NV_FLEX_API void NvFlexGraphicsShaderSetFormats(NvFlexContext* context, NvFlexGraphicsShader* shader, NvFlexFormat renderTargetFormat, NvFlexFormat depthStencilFormat);

//! A timer for work submitted to a context
struct NvFlexContextTimer;

NV_FLEX_API NvFlexContextTimer* NvFlexCreateContextTimer(NvFlexContext* context);

NV_FLEX_API void NvFlexReleaseContextTimer(NvFlexContextTimer* timer);

//! A queue of context events
struct NvFlexContextEventQueue;

NV_FLEX_API NvFlexContextEventQueue* NvFlexCreateContextEventQueue(NvFlexContext* context);

NV_FLEX_API void NvFlexReleaseContextEventQueue(NvFlexContextEventQueue* eventQueue);

//! An interface that create resource and submit GPU work
struct NvFlexContext;

NV_FLEX_API void NvFlexContextCopyConstantBuffer(NvFlexContext* context, NvFlexConstantBuffer* dst, NvFlexBuffer* src);

NV_FLEX_API void* NvFlexContextMapBuffer(NvFlexContext* context, NvFlexBuffer* buffer,
                                         NvFlexStagingCpuAccessFlags cpuAccessMode, bool cpuNoWait);

NV_FLEX_API void NvFlexContextUnmapBuffer(NvFlexContext* context, NvFlexBuffer* buffer);

NV_FLEX_API void NvFlexContextCopyBuffer(NvFlexContext* context, NvFlexBuffer* dst,
                                         NvFlexUint dstOffset, NvFlexBuffer* src,
                                         NvFlexUint srcOffset, NvFlexUint numBytes);

NV_FLEX_API void NvFlexContextUploadBuffer(NvFlexContext* context, NvFlexBuffer* dst,
                                           NvFlexUint offsetInBytes, const void* data,
                                           NvFlexUint sizeInBytes);

NV_FLEX_API void NvFlexContextCopyTexture3D(NvFlexContext* context, NvFlexTexture3D* dst, NvFlexTexture3D* src);

NV_FLEX_API void NvFlexContextCopyResource(NvFlexContext* context, NvFlexResourceRW* resourceRW, NvFlexResource* resource);

NV_FLEX_API void NvFlexContextDispatch(NvFlexContext* context, const NvFlexDispatchParams* params);

NV_FLEX_API void NvFlexContextSetVertexBuffer(NvFlexContext* context, NvFlexVertexBuffer* vertexBuffer, NvFlexUint stride, NvFlexUint offset);

NV_FLEX_API void NvFlexContextSetIndexBuffer(NvFlexContext* context, NvFlexIndexBuffer* indexBuffer, NvFlexUint offset);

NV_FLEX_API void NvFlexContextDrawIndexedInstanced(NvFlexContext* context, NvFlexUint indicesPerInstance, NvFlexUint numInstances, const NvFlexDrawParams* params);

NV_FLEX_API void NvFlexContextSetRenderTarget(NvFlexContext* context, NvFlexRenderTarget* rt, NvFlexDepthStencil* ds);

NV_FLEX_API void NvFlexContextSetViewport(NvFlexContext* context, const NvFlexViewport* viewport);

NV_FLEX_API void NvFlexContextClearRenderTarget(NvFlexContext* context, NvFlexRenderTarget* rt, const NvFlexFloat4 color);

NV_FLEX_API void NvFlexContextClearDepthStencil(NvFlexContext* context, NvFlexDepthStencil* ds, const float depth);

NV_FLEX_API void NvFlexContextRestoreResourceState(NvFlexContext* context, NvFlexResource* resource);

NV_FLEX_API void NvFlexContextClearState(NvFlexContext* context);

NV_FLEX_API bool NvFlexContextIsSparseTextureSupported(NvFlexContext* context);

NV_FLEX_API void NvFlexContextUpdateSparseMapping(NvFlexContext* context, NvFlexTexture3DSparse* tex, NvFlexHeapSparse* heap, NvFlexUint* blockTableImage, NvFlexUint rowPitch, NvFlexUint depthPitch);

NV_FLEX_API void NvFlexContextTimerBegin(NvFlexContext* context, NvFlexContextTimer* timer);

NV_FLEX_API void NvFlexContextTimerEnd(NvFlexContext* context, NvFlexContextTimer* timer);

NV_FLEX_API NvFlexResult NvFlexContextTimerGetResult(NvFlexContext* context, NvFlexContextTimer* timer, float* timeGPU, float* timeCPU, NvFlexUint64 *gpuStartStamp, NvFlexUint64 *gpuEndStamp, NvFlexUint64 *gpuFreq);

NV_FLEX_API void NvFlexContextEventQueuePush(NvFlexContext* context, NvFlexContextEventQueue* eventQueue, NvFlexUint64 uid);

NV_FLEX_API NvFlexResult NvFlexContextEventQueuePop(NvFlexContext* context, NvFlexContextEventQueue* eventQueue, NvFlexUint64* pUid);

NV_FLEX_API void NvFlexContextProfileGroupBegin(NvFlexContext* context, const wchar_t* label);

NV_FLEX_API void NvFlexContextProfileGroupEnd(NvFlexContext* context);

NV_FLEX_API void NvFlexContextProfileItemBegin(NvFlexContext* context, const wchar_t* label);

NV_FLEX_API void NvFlexContextProfileItemEnd(NvFlexContext* context);

//! A fence for queue synchronization
struct NvFlexFence;

struct NvFlexFenceDesc
{
	bool crossAdapterShared;
};

NV_FLEX_API void NvFlexFenceGetDesc(NvFlexFence* fence, NvFlexFenceDesc* desc);

NV_FLEX_API NvFlexFence* NvFlexCreateFence(NvFlexContext* context, const NvFlexFenceDesc* desc);

NV_FLEX_API NvFlexFence* NvFlexShareFence(NvFlexContext* context, NvFlexFence* fence);

NV_FLEX_API void NvFlexReleaseFence(NvFlexFence* fence);

NV_FLEX_API void NvFlexContextWaitOnFence(NvFlexContext* context, NvFlexFence* fence, NvFlexUint64 fenceValue);

NV_FLEX_API void NvFlexContextSignalFence(NvFlexContext* context, NvFlexFence* fence, NvFlexUint64 fenceValue);

//! A cross adapter shared 2d texture
struct NvFlexTexture2DCrossAdapter;

NV_FLEX_API NvFlexTexture2DCrossAdapter* NvFlexCreateTexture2DCrossAdapter(NvFlexContext* context, const NvFlexTexture2DDesc* desc);

NV_FLEX_API NvFlexTexture2DCrossAdapter* NvFlexShareTexture2DCrossAdapter(NvFlexContext* context, NvFlexTexture2DCrossAdapter* sharedTexture);

NV_FLEX_API void NvFlexReleaseTexture2DCrossAdapter(NvFlexTexture2DCrossAdapter* tex);

NV_FLEX_API void NvFlexContextTransitionToCommonState(NvFlexContext* context, NvFlexResource* resource);

NV_FLEX_API void NvFlexContextCopyToTexture2DCrossAdapter(NvFlexContext* context, NvFlexTexture2DCrossAdapter* dst, NvFlexTexture2D* src, NvFlexUint height);

NV_FLEX_API void NvFlexContextCopyFromTexture2DCrossAdapter(NvFlexContext* context, NvFlexTexture2D* dst, NvFlexTexture2DCrossAdapter* src, NvFlexUint height);

//! An opaque reference to another resource, for proper interqueue lifetime
struct NvFlexResourceReference;

NV_FLEX_API NvFlexResourceReference* NvFlexShareResourceReference(NvFlexContext* context, NvFlexResource* resource);

NV_FLEX_API void NvFlexReleaseResourceReference(NvFlexResourceReference* resource);
