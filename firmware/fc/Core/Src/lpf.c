/* firmware/fc/Core/Src/lpf.c */
#include "lpf.h"
#include <math.h>
#include <stdint.h>

void LPF_Init(LPF_t *f, float alpha) {
    f->alpha = alpha;
    f->state = 0.0f;
    f->seeded = 0;
}

float LPF_Update(LPF_t *f, float x) {
    if (!f->seeded) {
        /*
         * On the very first sample, seed the filter state with the input
         * directly. Without this, the filter output ramps from 0 toward the
         * true value over several samples, causing a transient spike in the
         * D-term on boot — exactly what we're trying to avoid.
         */
        f->state = x;
        f->seeded = 1;
        return x;
    }

    f->state = f->alpha * x + (1.0f - f->alpha) * f->state;
    return f->state;
}

float LPF_ComputeAlpha(float dt, float fc_hz) {
    float rc = 1.0f / (2.0f * (float)M_PI * fc_hz);
    return dt / (dt + rc);
}