/*
The MIT License (MIT)

Copyright (c) 2018 Wolfgang Hoenig and James Alan Preiss

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
This controller is based on the following publication:

Daniel Mellinger, Vijay Kumar:
Minimum snap trajectory generation and control for quadrotors.
IEEE International Conference on Robotics and Automation (ICRA), 2011.

We added the following:
 * Integral terms (compensates for: battery voltage drop over time, unbalanced center of mass due to asymmmetries, and uneven wear on propellers and motors)
 * D-term for angular velocity
 * Support to use this controller as an attitude-only controller for manual flight
*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "controller_mellinger.h"
#include "physicalConstants.h"

static const float MASS    = 0.0315f;     // kg
static const float GRAVITY = 9.81f;     // m / s^2
static const float L       = 0.046f;    // m
static float K_F     = 1.8221205267714052e-06f;  //1.938952e-6f; // N / "PWM"
static float K_M     = 4.4732910188601685e-08f; // Nm / "PWM"

// K_LQR
static float K[4][12] = {
  {0.0f, 0.0f, -0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.4f, 0.0f, 0.0f, 0.0f},
  {0.0f, -5.8e-4f, 0.0f -1.7e-3f, 0.0f, 0.0f, 0.0f, -7e-4f, 0.0f, -5e-4f, 0.0f, 0.0f},
  {5.8e-4f, 0.0f, 0.0f, 0.0f, -1.7e-3f, 0.0f, 7e-4f, 0.0f, 0.0f, 0.0f, -5e-4f, 0.0f},
  {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1e-2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -7.1e-3}};

static float state_vec[12];
static float setpt_vec[12];
static float state_error_vec[12];
static float input_vec[4];
static float pwm_vec[4];


// static void forcesToPwm(float *control_vec, float *pwm_vec) {
//   for (int i = 0; i < 4; i++){
//     pwm_vec[i] = -1.4e6f*control_vec[i]*control_vec[i] + 6.1e5f*control_vec[i] + 2.1e3f;
//   }
// }

static inline void forcesToPwm(float *control_vec, float *pwm_vec) {
  // May need to negate pwm_vec[1], pwm_vec[2]. Sums in
  // power_distribution_stock.c seem to have negatives cf. Mellinger paper.
  pwm_vec[0] = control_vec[0] / (4 * K_F);
  pwm_vec[1] = sqrtf(2) * control_vec[1] / (4 * L * K_F);
  pwm_vec[2] = sqrtf(2) * control_vec[2] / (4 * L * K_F);
  pwm_vec[3] = control_vec[3] / (4 * K_M);
}

void controllerMellingerReset(void)
{
 
}


void controllerMellingerInit(void)
{
  controllerMellingerReset();
}


bool controllerMellingerTest(void)
{
  return true;
}


void controllerMellinger(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

  // CHANGED
  state_vec[0]  = state->position.x;
  state_vec[1]  = -state->position.y;

  state_vec[2]  = state->position.z;

  state_vec[3]  = radians(state->attitude.roll);
  state_vec[4]  = radians(state->attitude.pitch);
  state_vec[5]  = radians(state->attitude.yaw);

  // CHANGED
  state_vec[6]  = state->velocity.x;
  state_vec[7]  = -state->velocity.y;

  state_vec[8]  = state->velocity.z;

  state_vec[9]  = radians(sensors->gyro.x);
  state_vec[10] = -radians(sensors->gyro.y);
  state_vec[11] = radians(sensors->gyro.z);

  setpt_vec[0]  = 0.0f;//-setpoint->position.x;
  setpt_vec[1]  = 0.0f;//setpoint->position.y;
  setpt_vec[2]  = setpoint->position.z;

  setpt_vec[3]  = 0.0f;//radians(setpoint->attitude.roll);
  setpt_vec[4]  = 0.0f;//radians(setpoint->attitude.pitch);
  setpt_vec[5]  = 0.0f;//radians(setpoint->attitude.yaw);

  setpt_vec[6]  = 0.0f;//-0.5582;//-setpoint->velocity.x;
  setpt_vec[7]  = 0.0f;//0.1708f;//setpoint->velocity.y;
  setpt_vec[8]  = 0.0f;//setpoint->velocity.z;

  setpt_vec[9]  = 0.0f;//radians(setpoint->attitudeRate.roll);
  setpt_vec[10] = 0.0f;//-radians(setpoint->attitudeRate.pitch);
  setpt_vec[11] = 0.0f;//radians(setpoint->attitudeRate.yaw);

  for (int i = 0; i < 12; i++) {
    state_error_vec[i] = state_vec[i] - setpt_vec[i];
  }
  
  // u = K * (r - x)  in Newton
  for (int i = 0; i < 4; i++){
    input_vec[i] = 0;

    for (int j = 0; j < 12; j++) {
      input_vec[i] += K[i][j] * state_error_vec[j];
    }
  }

  input_vec[0] += setpoint->thrust + MASS * GRAVITY;  // gravity compensation in Newton

  // input_vec[0] = input_vec[0];
  // input_vec[1] = input_vec[1]*7.6859f*2.0f;
  // input_vec[2] = input_vec[2]*7.6859f*2.0f;
  // input_vec[3] = input_vec[3]*(-5.5f);

  forcesToPwm(input_vec, pwm_vec);  // from Newton to PWM

  // control->thrust = pwm_vec[0];
  // control->roll   = pwm_vec[1];
  // control->pitch  = pwm_vec[2];
  // control->yaw    = pwm_vec[3];  

  control->thrust = pwm_vec[0];
  control->roll   = 2.0f * pwm_vec[1];
  control->pitch  = 2.0f * pwm_vec[2];
  control->yaw    = -1.0f  * pwm_vec[3];  

  controllerMellingerReset();
  
}



PARAM_GROUP_START(ctrlLQR)

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k11, &K[0][0])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k21, &K[1][0])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k31, &K[2][0])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k41, &K[3][0])

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k12, &K[0][1])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k22, &K[1][1])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k32, &K[2][1])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k42, &K[3][1])

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k13, &K[0][2])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k23, &K[1][2])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k33, &K[2][2])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k43, &K[3][2])

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k14, &K[0][3])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k24, &K[1][3])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k34, &K[2][3])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k44, &K[3][3])

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k15, &K[0][4])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k25, &K[1][4])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k35, &K[2][4])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k45, &K[3][4])

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k16, &K[0][5])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k26, &K[1][5])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k36, &K[2][5])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k46, &K[3][5])

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k17, &K[0][6])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k27, &K[1][6])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k37, &K[2][6])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k47, &K[3][6])

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k18, &K[0][7])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k28, &K[1][7])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k38, &K[2][7])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k48, &K[3][7])

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k19, &K[0][8])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k29, &K[1][8])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k39, &K[2][8])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k49, &K[3][8])

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k110, &K[0][9])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k210, &K[1][9])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k310, &K[2][9])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k410, &K[3][9])

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k111, &K[0][10])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k211, &K[1][10])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k311, &K[2][10])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k411, &K[3][10])

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k112, &K[0][11])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k212, &K[1][11])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k312, &K[2][11])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, k412, &K[3][11])

PARAM_GROUP_STOP(ctrlLQR)

LOG_GROUP_START(ctrlLQR)
LOG_ADD(LOG_FLOAT, e_x, &state_error_vec[0])
LOG_ADD(LOG_FLOAT, e_y, &state_error_vec[1])
LOG_ADD(LOG_FLOAT, e_z, &state_error_vec[2])

LOG_ADD(LOG_FLOAT, e_roll,  &state_error_vec[3])
LOG_ADD(LOG_FLOAT, e_pitch, &state_error_vec[4])
LOG_ADD(LOG_FLOAT, e_yaw,   &state_error_vec[5])

LOG_ADD(LOG_FLOAT, e_vx, &state_error_vec[6])
LOG_ADD(LOG_FLOAT, e_vy, &state_error_vec[7])
LOG_ADD(LOG_FLOAT, e_vz, &state_error_vec[8])

LOG_ADD(LOG_FLOAT, e_vroll,  &state_error_vec[9])
LOG_ADD(LOG_FLOAT, e_vpitch, &state_error_vec[10])
LOG_ADD(LOG_FLOAT, e_vyaw,   &state_error_vec[11])

LOG_ADD(LOG_FLOAT, u1,   &input_vec[0])
LOG_ADD(LOG_FLOAT, u2,   &input_vec[1])
LOG_ADD(LOG_FLOAT, u3,   &input_vec[2])
LOG_ADD(LOG_FLOAT, u4,   &input_vec[3])

LOG_GROUP_STOP(ctrlLQR)