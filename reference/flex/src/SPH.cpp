#include "SPH.h"
#include "FlexMath.h"
#include "VectorCached.h"
#include "Random.h"

namespace NvFlex {

namespace {

float W(float r, float h) {
    return 15.f / (pi() * h * h * h) * sqr(1.f - r / h);
}

float dWdr(float r, float h) {
    return -30.f / (pi() * h * h * h * h) * (1.f - r / h);
}

}

static NvFlexUint TightPack3D(float radius, float separation, NvFlexFloat3 *points, NvFlexUint maxPoints) {
   int dim = (int)ceil(radius / separation);
   NvFlexUint c = 0;
   for (int z = -dim; z <= dim; ++z) {
       for (int y = -dim; y <= dim; ++y) {
           for (int x = -dim; x <= dim; ++x) {
               float offset;
               if((y + z) & 1)
                   offset = separation * 0.5f;
               else
                   offset = 0.f;
               float ypos = y * sqrt(0.75f) * separation;
               float zpos = z * sqrt(0.75f) * separation;
               auto pos = make_float3(x * separation + offset, ypos, zpos);
               auto len = length(pos);
               if(len != 0.f && c < maxPoints && radius >= len)
                   points[c++] = pos;
           }
       }
   }

   return c;
}

void SPHCalculateRestDensity(float restDistance, float h, float* rho, float* rhoDeriv,
                             float* surfaceDeriv) {
    ResetRandomSeeds(39384234, 481343);
    VectorCached<NvFlexFloat3, 1> samples((size_t)0x800);
    auto n = TightPack3D(h, restDistance, samples.data(), (NvFlexUint)samples.size());
    float rho_ = 0.f;
    float rhoDeriv_ = 0.f;
    float a = 0.f, b = 0.f;
    for (NvFlexUint i = 0; i < n; ++i) {
        NvFlexFloat3 sample = samples[i];
        float r = length(sample);
        rho_ += W(r, h);
        float dwdr = dWdr(r, h);
        rhoDeriv_ += sqr(dwdr);

        if(sample.y <= 0.f) {
            float cosTheta = sample.y / r;
            a += dwdr * cosTheta;
            b -= r * cosTheta;
        }
    }

    *rho = rho_;
    *rhoDeriv = rhoDeriv_;
    *surfaceDeriv = a / b;
}

}  // namespace NvFlex
