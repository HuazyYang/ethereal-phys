cbuffer consts : register(b0) {
    int gNumBounds;
}

struct FlexConvexMeshDevice {
    float4 mLower;
    float4 mUpper;
    int mPlaneOffset;
    int mPlaneCount;
    int2 mPad;
};

struct FlexTriangleMeshDevice {
    float4 mLower;
    float4 mUpper;
    int mIndexStart;
    int mVertexStart;
    int mNodeStart;
    int mPad;
};

static const uint eNvFlexShapeSphere = 0;
static const uint eNvFlexShapeCapsule = 1;
static const uint eNvFlexShapeBox = 2;
static const uint eNvFlexShapeConvexMesh = 3;
static const uint eNvFlexShapeTriangleMesh = 4;
static const uint eNvFlexShapeSDF = 5;

StructuredBuffer<float4> positions : register(t0);
StructuredBuffer<float4> rotations : register(t1);
StructuredBuffer<float4> positionsPrev : register(t2);
StructuredBuffer<float4> rotationsPrev : register(t3);
StructuredBuffer<uint> flags : register(t4);
StructuredBuffer<float4> geometry : register(t5);
StructuredBuffer<FlexConvexMeshDevice> convexMeshes : register(t6);
StructuredBuffer<FlexTriangleMeshDevice> triangleMeshes : register(t7);
RWStructuredBuffer<float4> lowers : register(u0);
RWStructuredBuffer<float4> uppers : register(u1);

// cross(q.xyz, v) spelled component-wise: the DXBC emits a two-wide product pair
// plus a scalar, not the three-wide pack the cross() intrinsic produces.
float3 CrossSplit(float3 a, float3 b) {
    float2 t = a.zx * b.yz;
    float2 xy = a.yz * b.zx - t;
    float tw = a.y * b.x;
    float z = a.x * b.y - tw;
    return float3(xy, z);
}

float3 RotateVector(float4 q, float3 v) {
    return v * (2.0 * q.w * q.w - 1.0) + CrossSplit(q.xyz, v) * q.w * 2.0 + q.xyz * dot(q.xyz, v) * 2.0;
}

// Same rotation with cross(q.xyz, v) and dot(q.xyz, v) supplied pre-folded. The
// DXBC collapses each basis-axis cross into one masked mul, which is only
// reproducible when the folded form is written out.
float3 RotateBasis(float4 q, float3 v, float3 crossQV, float dotQV) {
    return v * (2.0 * q.w * q.w - 1.0) + crossQV * q.w * 2.0 + q.xyz * dotQV * 2.0;
}

void LocalShapeBounds(uint shapeType, float4 geom, out float3 lower, out float3 upper) {
    switch (shapeType) {
    case eNvFlexShapeSphere:
        lower = -geom.xxx;
        upper = geom.xxx;
        break;
    case eNvFlexShapeCapsule:
        lower = -geom.xxx - float3(geom.y, 0.0, 0.0);
        upper = geom.xxx + float3(geom.y, 0.0, 0.0);
        break;
    case eNvFlexShapeBox:
        lower = -geom.xyz;
        upper = geom.xyz;
        break;
    case eNvFlexShapeConvexMesh: {
        int meshIndex = asint(geom.w) - 1;
        lower = geom.xyz * convexMeshes[meshIndex].mLower.xyz;
        upper = geom.xyz * convexMeshes[meshIndex].mUpper.xyz;
        break;
    }
    case eNvFlexShapeTriangleMesh: {
        int meshIndex = asint(geom.w) - 1;
        lower = geom.xyz * triangleMeshes[meshIndex].mLower.xyz;
        upper = geom.xyz * triangleMeshes[meshIndex].mUpper.xyz;
        break;
    }
    case eNvFlexShapeSDF:
        lower = 0.0;
        upper = geom.xxx;
        break;
    default:
        break;
    }
}

void TransformBounds(float4 rotation, float3 lower, float3 upper, out float3 worldCenter, out float3 worldExtents) {
    float3 axisX = RotateBasis(rotation, float3(1.0, 0.0, 0.0),
                               float3(0.0, rotation.z, -rotation.y), rotation.x);
    float3 axisY = RotateBasis(rotation, float3(0.0, 1.0, 0.0),
                               float3(-rotation.z, 0.0, rotation.x), rotation.y);
    float3 axisZ = RotateBasis(rotation, float3(0.0, 0.0, 1.0),
                               float3(rotation.y, -rotation.x, 0.0), rotation.z);

    float3 edges = upper - lower;
    float3 ex = axisX * edges.x;
    float3 ey = axisY * edges.y;
    float3 ez = axisZ * edges.z;
    worldExtents = abs(ex) + abs(ey);
    worldExtents = worldExtents.xyz + abs(ez);

    float3 center = (lower + upper) * 0.5;
    worldCenter = RotateVector(rotation, center);
}

[numthreads(256, 1, 1)]
void TransformShapeBounds(uint idx : SV_DispatchThreadID) {
    if (int(idx) < gNumBounds) {
    uint shapeType = flags[idx] & 7;
    float3 localLower;
    float3 localUpper;
    LocalShapeBounds(shapeType, geometry[idx], localLower, localUpper);

    float3 cNow, eNow, cPrev, ePrev;
    TransformBounds(rotations[idx], localLower, localUpper, cNow, eNow);
    float3 wNow = cNow + positions[idx].xyz;
    float3 lowerNow = wNow - eNow * 0.5;
    float3 upperNow = wNow + eNow * 0.5;

    TransformBounds(rotationsPrev[idx], localLower, localUpper, cPrev, ePrev);
    float3 wPrev = cPrev + positionsPrev[idx].xyz;
    float3 lowerPrev = wPrev - ePrev * 0.5;
    float3 upperPrev = wPrev + ePrev * 0.5;

    lowers[idx] = float4(min(lowerNow, lowerPrev), 0.0);
    uppers[idx] = float4(max(upperNow, upperPrev), 0.0);
    }
}
