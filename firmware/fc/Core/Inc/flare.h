#ifndef FLARE_H
#define FLARE_H

#include <stdint.h>

typedef struct {
  /* Setpoints (degrees) */
  float roll_sp;
  float pitch_sp;
  float yaw_rate_sp; /* yaw controlled as rate, not angle */

  /* Base throttle (DSHOT units: 48-2047) */
  uint16_t throttle;

  /* Arming */
  uint8_t armed;
} FLARE_State;

void FLARE_Init(void);
void FLARE_Update(float roll, float pitch, float gx, float gy, float gz,
                  float dt);

#endif /* FLARE_H */