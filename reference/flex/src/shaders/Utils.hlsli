#ifndef UTILS_HLSLI
#define UTILS_HLSLI

void InterlockedAddFp32(RWByteAddressBuffer accum, uint addr, float value) // Works perfectly!
{
    uint i_val = asuint(value);
    uint tmp0 = 0;
    uint tmp1;
    [allow_uav_condition]
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
    [allow_uav_condition]
    do {
        accum.InterlockedCompareExchange(addr, comp = orig, asuint(asfloat(orig) + value), orig);
    } while (orig != comp);
}

#endif /* UTILS_HLSLI */
