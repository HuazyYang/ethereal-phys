// This code contains NVIDIA Confidential Information and is disclosed to you

#ifndef NVFLEXTYPES_H
#define NVFLEXTYPES_H
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

#ifndef NV_FLEX_API
#define NV_FLEX_API extern "C" __declspec(dllexport)
#endif

enum NvFlexResult
{
	eNvFlexSuccess = 0,
	eNvFlexFail = 1
};

typedef int NvFlexInt;
typedef unsigned int NvFlexUint;
typedef float NvFlexFloat;
typedef unsigned long long NvFlexUint64;

struct NvFlexDim
{
	NvFlexUint x, y, z;
};

struct NvFlexUint2
{
	NvFlexUint x, y;
};

struct NvFlexUint3
{
	NvFlexUint x, y, z;
};

struct NvFlexUint4
{
	NvFlexUint x, y, z, w;
};

struct NvFlexInt2
{
	int x, y;
};

struct NvFlexInt3
{
	int x, y, z;
};

struct NvFlexInt4
{
	int x, y, z, w;
};

struct NvFlexFloat2
{
	float x, y;
};

struct NvFlexFloat3
{
	float x, y, z;
};

struct NvFlexFloat4
{
	float x, y, z, w;
};

struct NvFlexFloat4x4
{
	NvFlexFloat4 x, y, z, w;
};

enum NvFlexFormat {
    eNvFlexFormat_unknown = 0,

    eNvFlexFormat_r32_typeless = 1,

    eNvFlexFormat_r32_float = 2,
    eNvFlexFormat_r32g32_float = 3,
    eNvFlexFormat_r32g32b32a32_float = 4,

    eNvFlexFormat_r16_float = 5,
    eNvFlexFormat_r16g16_float = 6,
    eNvFlexFormat_r16g16b16a16_float = 7,

    eNvFlexFormat_r32_uint = 8,
    eNvFlexFormat_r32g32_uint = 9,
    eNvFlexFormat_r32g32b32a32_uint = 10,

    eNvFlexFormat_r8_unorm = 11,
    eNvFlexFormat_r8g8_unorm = 12,
    eNvFlexFormat_r8g8b8a8_unorm = 13,

    eNvFlexFormat_r16_unorm = 14,
    eNvFlexFormat_r16g16_unorm = 15,
    eNvFlexFormat_r16g16b16a16_unorm = 16,

    eNvFlexFormat_d32_float = 17,
    eNvFlexFormat_d24_unorm_s8_uint = 18,

    eNvFlexFormat_r8_snorm = 19,
    eNvFlexFormat_r8g8_snorm = 20,
    eNvFlexFormat_r8g8b8a8_snorm = 21,

    eNvFlexFormat_r24_unorm_x8_typeless = 23,
    eNvFlexFormat_r24g8_typeless = 24,

    eNvFlexFormat_r16_typeless = 25,
    eNvFlexFormat_d16_unorm = 26,

    eNvFlexFormat_max
};

#endif /* NVFLEXTYPES_H */
