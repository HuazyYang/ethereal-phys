// ethereal-phys kernel source. Origin: FleX src/shaders/Utils.hlsli at commit 2eed78e (branch feat-xpbd).
// Changes made by the SM 6.0 port are recorded in engine/shaders/PORTING.md.

#ifndef UTILS_HLSLI
#define UTILS_HLSLI

// [allow_uav_condition] lets FXC and DXC compile a loop whose exit depends on a UAV read. The SPIR-V
// backend of DXC has no such attribute and warns that it ignores it; there the loops compile without it.
#if SPIRV
#define EPHYS_ALLOW_UAV_CONDITION
#else
#define EPHYS_ALLOW_UAV_CONDITION [allow_uav_condition]
#endif

void InterlockedAddFp32(RWByteAddressBuffer accum, uint addr, float value) // Works perfectly!
{
    uint i_val = asuint(value);
    uint tmp0 = 0;
    uint tmp1;
    EPHYS_ALLOW_UAV_CONDITION
    while (true) {
        accum.InterlockedCompareExchange(addr, tmp0, i_val, tmp1);
        if (tmp1 == tmp0)
            break;
        tmp0 = tmp1;
        i_val = asuint(value + asfloat(tmp1));
    }
}

void InterlockedAddFloatV2(RWByteAddressBuffer accum, uint addr, float value)
{
    uint comp, orig = accum.Load(addr);
    EPHYS_ALLOW_UAV_CONDITION
    do {
        accum.InterlockedCompareExchange(addr, comp = orig, asuint(asfloat(orig) + value), orig);
    } while (orig != comp);
}

#endif /* UTILS_HLSLI */
