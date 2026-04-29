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

void FLARE_Init(void);

void FLARE_Update(float roll, float pitch, float gx, float gy, float gz, float dt);

void FLARE_SetArmed(uint8_t armed);
void FLARE_SetThrottle(uint16_t throttle);

/* Setpoint setters - called each loop from main.c with mapped RC values */
void FLARE_SetRollSP(float deg);
void FLARE_SetPitchSP(float deg);
void FLARE_SetYawRateSP(float dps);

#endif /* FLARE_H */