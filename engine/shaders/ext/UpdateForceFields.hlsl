// ethereal-phys kernel source. Origin: FleX extensions/dx/flexExt.hlsl (namespace UpdateForceFields) and
// extensions/dx/flexExt_dx_common.h (FlexExtConstParams, FlexExtForceFieldD3D) at commit 2eed78e (branch
// feat-xpbd), the kernel of NvFlexExtForceFieldCallback (ephys::IForceFieldSet, ext/force_field.h).
// Changes made by the SM 6.0 port are recorded in engine/shaders/PORTING.md: the entry point is a global
// function; the field position is a float3 (FleX: float[3], the same 12 bytes in a structured buffer); the
// mode is an int and the falloff flag a uint of which only the low byte counts (FleX declared a 4-byte HLSL
// bool over a 1-byte C++ bool and 3 bytes of padding); an unknown mode changes nothing (FleX left the
// multiplier undefined).

#define kNumThreadsPerBlock 256

#define eNvFlexExtModeForce 0
#define eNvFlexExtModeImpulse 1
#define eNvFlexExtModeVelocityChange 2

cbuffer consts : register(b0) {
    int kNumParticles;    // the active particles of the step
    int kNumForceFields;
    float kDt;            // the length of the step
    float _pad;
};

// ephys::ForceField, 28 bytes.
struct ForceField {
    float3 mPosition;
    float mRadius;
    float mStrength;
    int mMode;
    uint mLinearFalloff;
};

StructuredBuffer<float4> positions : register(t0);       // the solver's predicted positions, x y z 1/m
StructuredBuffer<ForceField> forceFields : register(t1);
RWStructuredBuffer<float4> velocities : register(u0);    // the solver's velocities

[numthreads(kNumThreadsPerBlock, 1, 1)]
void UpdateForceFields(uint3 globalIdx : SV_DispatchThreadID) {
    const int i = globalIdx.x;
    const int numParticles = kNumParticles;
    const int numForceFields = kNumForceFields;
    const float dt = kDt;

    for (int f = 0; f < numForceFields; f++) {
        const ForceField forceField = forceFields[f];

        if (i < numParticles) {
            const int index = i;

            float4 p = positions[index];
            float3 v = velocities[index].xyz;

            float3 localPos = float3(p.x, p.y, p.z) - forceField.mPosition;

            float dist = length(localPos);
            if (dist >= forceField.mRadius) {
                continue;
            }

            float3 fieldDir;
            if (dist > 0.0f) {
                fieldDir = localPos / dist;
            } else {
                fieldDir = localPos;
            }

            // If using linear falloff, scale with distance.
            float fieldStrength = forceField.mStrength;
            if ((forceField.mLinearFalloff & 0xffu) != 0) {
                fieldStrength *= (1.0f - (dist / forceField.mRadius));
            }

            float unitMultiplier = 0.0f;
            if (forceField.mMode == eNvFlexExtModeForce) {
                unitMultiplier = dt * p.w;  // time/mass
            } else if (forceField.mMode == eNvFlexExtModeImpulse) {
                unitMultiplier = p.w;  // 1/mass
            } else if (forceField.mMode == eNvFlexExtModeVelocityChange) {
                unitMultiplier = 1.0f;
            }

            float3 deltaVelocity = fieldDir * fieldStrength * unitMultiplier;
            velocities[index] = float4(v + deltaVelocity, 0.0f);
        }
    }
}
