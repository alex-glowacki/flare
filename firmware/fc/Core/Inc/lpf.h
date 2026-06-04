/* firmware/fc/Core/Inc/lpf.h */
#ifndef LPF_H
#define LPF_H
#include <stdint.h>

typedef struct {
    float alpha;    /* Filter coefficient [0,1]. Lower = more filtering, more lag. */
    float state;    /* Last output sample (y[n-1]). */
    uint8_t seeded; /* 0 until first sample - prevents cold-start transient. */
} LPF_t;

/*
 * Initialise a first-order IIR low-pass filter.
 *
 *   alpha = (2π · dt · fc) / (2π · dt · fc + 1)
 *
 *   fc   — cutoff frequency in Hz
 *   dt   — sample period in seconds (1/loop_rate)
 *
 * Lower alpha → more filtering, more phase lag.
 * Higher alpha → less filtering, faster response.
 */
void LPF_Init(LPF_t *f, float alpha);

/*
 * Update the filter with a new input sample, return the filtered output.
 * y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 */
float LPF_Update(LPF_t *f, float x);

/*
 * Convenience: compute alpha from physical parameters.
 * Use this at init time rather than hard-coding the magic number.
 *
 *   alpha = (2π · dt · fc) / (2π · dt · fc + 1)
 */
float LPF_ComputeAlpha(float dt, float fc_hz);

#endif /* LPF_H */