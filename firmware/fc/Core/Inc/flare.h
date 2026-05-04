/* firmware/fc/Core/Inc/flare.h */
#ifndef FLARE_H
#define FLARE_H

#include <stdint.h>

typedef struct {
    float roll_sp;
    float pitch_sp;
    float yaw_rate_sp;
    uint16_t throttle;
    uint16_t armed;
} FLARE_State;

/* Motor commands written by FLARE_Update, read by TIM6 ISR at 1kHz */
extern volatile uint16_t dshot_m1;
extern volatile uint16_t dshot_m2;
extern volatile uint16_t dshot_m3;
extern volatile uint16_t dshot_m4;

void FLARE_Init(void);
void FLARE_Update(float roll, float pitch, float gx, float gy, float gz, float dt);
void FLARE_SetArmed(uint8_t armed);
void FLARE_SetThrottle(uint16_t throttle);
void FLARE_SetRollSP(float deg);
void FLARE_SetPitchSP(float deg);
void FLARE_SetYawRateSP(float dps);

#endif /* FLARE_H */