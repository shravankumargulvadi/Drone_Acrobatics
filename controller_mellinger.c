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
#include "position_controller.h"
#include "controller_mellinger.h"
#include "physicalConstants.h"

static float g_vehicleMass = CF_MASS;
// static float massThrust = 132000;

// K_LQR
const float scale = 500;
static float k_arr[4][12] = {{0,0,0.005f*scale,0,0,0.166375f*scale, 0,0,0,0,0,0},
                               {0,-0.00071f*scale, 0,0,-0.003f*scale, 0,0.0243f*scale,0,0,0.00316f*scale, 0,0},
                               {0.00071f*scale, 0,0,0.003f*scale, 0,0,0,0.0243f*scale, 0,0,0.00316f*scale,0},
                               {0,0,0,0,0,0,0,0,0.0067f*scale, 0,0,0.00316f*scale}};

float res[4][1] = {0};

static void matrix_multiply(float A[4][12], float B[12][1], float res[4][1]){
  for (int i= 0; i<4; ++i){
    for(int j=0;j<1;++j){
      for(int k=0; k<12;++k){
        res[i][j] += A[i][k]*B[k][j];
      }
    }
  }
}

static float posS_x, posS_y, posS_z;			// Current position
static float velS_x, velS_y, velS_z;			// Current velocity
static float attS_r, attS_p, attS_y;      // Current Euler attitude
static float rateS_r, rateS_p, rateS_y;      // Current angular rate

static float posR_x, posR_y, posR_z;
static float velR_x, velR_y, velR_z;
static float attR_r, attR_p, attR_y;
static float rateR_r, rateR_p, rateR_y;

// Logging variables
static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;

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
  float difff_arr[12][1];

  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }


  
  if (setpoint->mode.x == modeAbs) {
    posR_x = setpoint->position.x;
  } else {
    posR_x = 0.0;
  }

  if (setpoint->mode.y == modeAbs) {
    posR_y = setpoint->position.y;
  } else {
    posR_y = 0.0;
  }

  if (setpoint->mode.z == modeAbs) {
    posR_z = setpoint->position.z;
  } else {
    posR_z = 0.5;
  }

  posS_x = state->position.x;
  posS_y = state->position.y;  
  posS_z = state->position.z; 
  velS_x = state->velocity.x;  velR_x = setpoint->velocity.x;
  velS_y = state->velocity.y;  velR_y = setpoint->velocity.y;
  velS_z = state->velocity.z;  velR_y = setpoint->velocity.z;
  attS_r = radians(state->attitude.roll);  attR_r = radians(setpoint->attitude.roll);
  attS_p = -radians(state->attitude.pitch);  attR_p = -radians(setpoint->attitude.pitch);
  attS_y = radians(state->attitude.yaw);  attR_y = radians(setpoint->attitude.yaw);
  rateS_r = radians(sensors->gyro.x);  rateR_r = radians(setpoint->attitudeRate.roll);
  rateS_p = -radians(sensors->gyro.y);  rateR_p = -radians(setpoint->attitudeRate.pitch);
  rateS_y = radians(sensors->gyro.z); rateR_y = radians(setpoint->attitudeRate.yaw);

  attR_y = 0.0;
  rateR_y = 0.0;

  difff_arr[0][0] = posR_x - posS_x;  
  difff_arr[1][0] = posR_y - posS_y;
  difff_arr[2][0] = posR_z - posS_z;
  difff_arr[3][0] = velR_x - velS_x;
  difff_arr[4][0] = velR_y - velS_y;
  difff_arr[5][0] = velR_z - velS_z;
  difff_arr[6][0] = attR_r - attS_r;  // in rad
  difff_arr[7][0] = attR_p - attS_p;  // in rad
  difff_arr[8][0] = attR_y - attS_y;  // in rad
  difff_arr[9][0] = rateR_r - rateS_r;  // in rad/s
  difff_arr[10][0] = rateR_p - rateS_p; // in rad/s
  difff_arr[11][0] = rateR_y - rateS_y; // in rad/s

  matrix_multiply(k_arr, difff_arr, res);

  // Output
  if (setpoint->mode.z == modeDisable) {
    control->thrust = setpoint->thrust;
  } else {
    control->thrust = res[0][0] + g_vehicleMass * GRAVITY_MAGNITUDE;
  }
  
  cmd_thrust = control->thrust;
  r_roll = radians(sensors->gyro.x);
  r_pitch = -radians(sensors->gyro.y);
  r_yaw = radians(sensors->gyro.z);

  if (control->thrust > 0) {
    control->roll = clamp(res[1][0], -32000, 32000);
    control->pitch = clamp(res[2][0], -32000, 32000);
    control->yaw = clamp(-res[3][0], -32000, 32000);

    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

  } else {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    controllerMellingerReset();
  }
}



LOG_GROUP_START(ctrlMel)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
LOG_GROUP_STOP(ctrlMel)



























// #include "controller_lqr.h"
// #include "param.h"
// #include "log.h"
// #include "math3d.h"
// #include "num.h"
// //#include "eeprom.h"
// #include "console.h"

// #include <string.h>
// #include <math.h>

// //#define __DEBUG_COORDS 0

// #ifndef __DEBUG_COORDS
// static const float MASS    = 0.0313f;     // kg
// static const float GRAVITY = 9.81f;     // m / s^2
// #endif

// static const float L       = 0.046f;    // m


// //static float K[4][12]={{0,0,0.500000000000000,0,0,0.166375000000000,0,0,0,0,0,0},{0,-0.000711512473537874,0,0,-0.00298424248869664,0,0.0242813448457065,0,0,0.00316284410733867,0,0},{0.000711512473537946,0,0,0.00298424248869694,0,0,0,0.0242813448457095,0,0,0.00316284410733867,0},{0,0,0,0,0,0,0,0,0.00666666666666733,0,0,0.00316320407182333}};
// static float K[4][12]={{-5.53357820e-15,  0.00000000e+00, -1.00000000e+01,
//          0.00000000e+00,  9.55426549e-17,  0.00000000e+00,
//         -3.67076657e-14,  0.00000000e+00, -2.36643191e+00,
//          0.00000000e+00, -1.03084507e-15,  0.00000000e+00},
//        { 0.00000000e+00, -4.08248290e-02,  0.00000000e+00,
//         -6.78743424e-02,  0.00000000e+00, -1.46127134e-18,
//          0.00000000e+00, -2.54722408e-02,  0.00000000e+00,
//         -9.23364230e-03,  0.00000000e+00,  9.10294382e-21},
//        { 9.12870929e-03,  0.00000000e+00, -2.91274729e-15,
//          0.00000000e+00, -2.19983075e-02,  0.00000000e+00,
//          1.11497185e-02,  0.00000000e+00, -3.65833299e-17,
//          0.00000000e+00, -2.18781511e-03,  0.00000000e+00},
//        { 0.00000000e+00,  1.02095012e-12,  0.00000000e+00,
//          1.67584629e-12,  0.00000000e+00, -7.07106781e+00,
//          0.00000000e+00,  8.06281899e-13,  0.00000000e+00,
//          2.60664775e-15,  0.00000000e+00, -7.07404129e-01}};
// static float K_F     = 1.8221205267714052e-06f;  //1.938952e-6f; // N / "PWM"
// static float K_M     = 4.4732910188601685e-08f; // Nm / "PWM"

// static float state_vec[12];
// static float setpt_vec[12];
// static float state_error_vec[12];
// static float input_vec[4];
// static float pwm_vec[4];
// static int16_t pwm_total;
// static int16_t pwm_int[3];
// static float force_total;
// static float thrust;
// static float set_pitch;
// static uint8_t idx = 0;

// /**
// * Converts forces to "PWM" units.
// *
// * This function converts from forces (specified in Newtons) to PWM units, which
// * are proportional to motor RPM ^ 2, and ranges from 0 to 65536. The PWM units
// * corresponds to the motor duty cycle.
// *
// * Essentially, this function is half of the inversion of (2) in [1].
// * Specifically, the entries of `pwm_vec` returned are those of `control_vec`
// * multiplied by the coefficient from the inversion of (2). The actual summing
// * of these values is done in lines 84-90 of `power_distribution_stock.c`.
// *
// * [1]: "Minimum  Snap  Trajectory  Generation  and  Control  for  Quadrotor"
// *      by Mellinger and Kumar.
// *
// * @param[in] control_vec The desired inputs in Newtons and Newton-meters. Order
// *            is the same as in [1].
// *
// * @param[out] pwm_vec The inputs converted into PWM units as described above.
// */
// static inline void forcesToPwm(float *control_vec, float *pwm_vec) {
//   // May need to negate pwm_vec[1], pwm_vec[2]. Sums in
//   // power_distribution_stock.c seem to have negatives cf. Mellinger paper.
//   pwm_vec[0] = control_vec[0] / (4 * K_F);
//   pwm_vec[1] = sqrtf(2) * control_vec[1] / (4 * L * K_F);
//   pwm_vec[2] = sqrtf(2) * control_vec[2] / (4 * L * K_F);
//   pwm_vec[3] = control_vec[3] / (4 * K_M);
// }


// /**
// * Converts a float to an int16_t without overflow. Instead, either INT16_MIN or
// * INT16_MAX are returned if the input exceeds the limits of the int16_t type.
// *
// * @param[in] in The float that will be converted.
// *
// * @return The converted value.
// */
// static inline int16_t saturateSignedInt16(float in)
// {
//   // Don't use INT16_MIN, because later we may negate it,
//   // which won't work for that value.
//   if (in > INT16_MAX)
//     return INT16_MAX;
//   else if (in < -INT16_MAX)
//     return -INT16_MAX;
//   else
//     return (int16_t) in;
// }


// /**
// * Initialize the LQR controller.
// *
// * Called once when switching to the the LQR controller. Useful for initializing
// * static variables used by the module.
// */
// void controllerMellingerInit(void) //LQR in reality
// {
//   //memset(pwm_int, 0, 3 * sizeof(int16_t));
// }

// /**
// * Test the LQR controller initialization.
// *
// * Called by the system to check that the controller is properly initialized.
// */
// bool controllerMellingerTest(void) //LQR
// {
//   return true;
// }



// void controllerMellinger(control_t *control, setpoint_t *setpoint,
//                                          const sensorData_t *sensors,
//                                          const state_t *state,
//                                          const uint32_t tick)     //LQR
// {
//   if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
//     return;
//   }

//   // CHANGED
//   state_vec[0]  = state->position.x;
//   state_vec[1]  = -state->position.y;

//   state_vec[2]  = state->position.z;

//   state_vec[3]  = radians(state->attitude.roll); //state->velocity.x;
//   state_vec[4]  = radians(state->attitude.pitch);//-state->velocity.y;
//   state_vec[5]  = radians(state->attitude.yaw); //state->velocity.z;

//   // CHANGED
//   state_vec[6]  = state->velocity.x;//radians(state->attitude.roll);//state->velocity.x;
//   state_vec[7]  = -state->velocity.y;//radians(state->attitude.pitch);//-state->velocity.y;

//   state_vec[8]  = state->velocity.z;//radians(state->attitude.yaw);//state->velocity.z;

//   state_vec[9]  = radians(sensors->gyro.x);
//   state_vec[10] = -radians(sensors->gyro.y);
//   state_vec[11] = radians(sensors->gyro.z);

//   setpt_vec[0]  = 0.0f;//-setpoint->position.x;
//   setpt_vec[1]  = 0.0f;//setpoint->position.y;
//   setpt_vec[2]  = setpoint->position.z;

//   setpt_vec[3]  = 0.0f;//radians(setpoint->attitude.roll);
//   setpt_vec[4]  = 0.0f;//radians(setpoint->attitude.pitch);
//   setpt_vec[5]  = 0.0f;//radians(setpoint->attitude.yaw);

//   setpt_vec[6]  = 0.0f;//-0.5582;//-setpoint->velocity.x;
//   setpt_vec[7]  = 0.0f;//0.1708f;//setpoint->velocity.y;
//   setpt_vec[8]  = 0.0f;//setpoint->velocity.z;

//   setpt_vec[9]  = 0.0f;//radians(setpoint->attitudeRate.roll);
//   setpt_vec[10] = 0.0f;//-radians(setpoint->attitudeRate.pitch);
//   setpt_vec[11] = 0.0f;//radians(setpoint->attitudeRate.yaw);

//   #ifdef __DEBUG_COORDS
//   for (int i = 0; i < 12; i++) {
//     state_error_vec[i] = 0;
//   }
//   #else
//   for (int i = 0; i < 12; i++) {
//     state_error_vec[i] = state_vec[i] - setpt_vec[i];
//   }
//   #endif

//   state_error_vec[idx] = state_vec[idx] - setpt_vec[idx];
//   //state_error_vec[0] = state_error_vec[1] = 0.0f;

//   // Matrix multiplication!
//   for (int i = 0; i < 4; i++){
//     input_vec[i] = 0;

//     for (int j = 0; j < 12; j++) {
//       input_vec[i] += K[i][j] * state_error_vec[j];
//     }
//   }

//   #ifndef __DEBUG_COORDS
//   input_vec[0] += setpoint->thrust + MASS * GRAVITY;
//   #endif

//   force_total = input_vec[0];
//   forcesToPwm(input_vec, pwm_vec);

//   control->thrust = pwm_vec[0];

//   #ifndef __DEBUG_COORDS
//   control->roll   = saturateSignedInt16(2.0f * pwm_vec[1]);
//   control->pitch  = saturateSignedInt16(2.0f * pwm_vec[2]);
//   control->yaw    = saturateSignedInt16(-1.0f  * pwm_vec[3]);
//   #else
//   control->roll   = saturateSignedInt16(10.0f * pwm_vec[1]);
//   control->pitch  = saturateSignedInt16(10.0f * pwm_vec[2]);
//   control->yaw    = saturateSignedInt16(-1.0f  * pwm_vec[3]);
//   #endif

//   pwm_int[0] = control->roll;
//   pwm_int[1] = control->pitch;
//   pwm_int[2] = control->yaw;

//   //memset(control, 0, sizeof(control_t));

//   pwm_total = (int16_t) (force_total / (4 * K_F));
//   thrust = setpoint->thrust;
//   set_pitch = setpoint->velocity.x;
// }


// PARAM_GROUP_START(ctrlMel)
// PARAM_ADD(PARAM_FLOAT, k_f, &K_F)
// PARAM_ADD(PARAM_FLOAT, k_m, &K_M)

// PARAM_ADD(PARAM_FLOAT, k11, &K[0][0])
// PARAM_ADD(PARAM_FLOAT, k21, &K[1][0])
// PARAM_ADD(PARAM_FLOAT, k31, &K[2][0])
// PARAM_ADD(PARAM_FLOAT, k41, &K[3][0])

// PARAM_ADD(PARAM_FLOAT, k12, &K[0][1])
// PARAM_ADD(PARAM_FLOAT, k22, &K[1][1])
// PARAM_ADD(PARAM_FLOAT, k32, &K[2][1])
// PARAM_ADD(PARAM_FLOAT, k42, &K[3][1])

// PARAM_ADD(PARAM_FLOAT, k13, &K[0][2])
// PARAM_ADD(PARAM_FLOAT, k23, &K[1][2])
// PARAM_ADD(PARAM_FLOAT, k33, &K[2][2])
// PARAM_ADD(PARAM_FLOAT, k43, &K[3][2])

// PARAM_ADD(PARAM_FLOAT, k14, &K[0][3])
// PARAM_ADD(PARAM_FLOAT, k24, &K[1][3])
// PARAM_ADD(PARAM_FLOAT, k34, &K[2][3])
// PARAM_ADD(PARAM_FLOAT, k44, &K[3][3])

// PARAM_ADD(PARAM_FLOAT, k15, &K[0][4])
// PARAM_ADD(PARAM_FLOAT, k25, &K[1][4])
// PARAM_ADD(PARAM_FLOAT, k35, &K[2][4])
// PARAM_ADD(PARAM_FLOAT, k45, &K[3][4])

// PARAM_ADD(PARAM_FLOAT, k16, &K[0][5])
// PARAM_ADD(PARAM_FLOAT, k26, &K[1][5])
// PARAM_ADD(PARAM_FLOAT, k36, &K[2][5])
// PARAM_ADD(PARAM_FLOAT, k46, &K[3][5])

// PARAM_ADD(PARAM_FLOAT, k17, &K[0][6])
// PARAM_ADD(PARAM_FLOAT, k27, &K[1][6])
// PARAM_ADD(PARAM_FLOAT, k37, &K[2][6])
// PARAM_ADD(PARAM_FLOAT, k47, &K[3][6])

// PARAM_ADD(PARAM_FLOAT, k18, &K[0][7])
// PARAM_ADD(PARAM_FLOAT, k28, &K[1][7])
// PARAM_ADD(PARAM_FLOAT, k38, &K[2][7])
// PARAM_ADD(PARAM_FLOAT, k48, &K[3][7])

// PARAM_ADD(PARAM_FLOAT, k19, &K[0][8])
// PARAM_ADD(PARAM_FLOAT, k29, &K[1][8])
// PARAM_ADD(PARAM_FLOAT, k39, &K[2][8])
// PARAM_ADD(PARAM_FLOAT, k49, &K[3][8])

// PARAM_ADD(PARAM_FLOAT, k110, &K[0][9])
// PARAM_ADD(PARAM_FLOAT, k210, &K[1][9])
// PARAM_ADD(PARAM_FLOAT, k310, &K[2][9])
// PARAM_ADD(PARAM_FLOAT, k410, &K[3][9])

// PARAM_ADD(PARAM_FLOAT, k111, &K[0][10])
// PARAM_ADD(PARAM_FLOAT, k211, &K[1][10])
// PARAM_ADD(PARAM_FLOAT, k311, &K[2][10])
// PARAM_ADD(PARAM_FLOAT, k411, &K[3][10])

// PARAM_ADD(PARAM_FLOAT, k112, &K[0][11])
// PARAM_ADD(PARAM_FLOAT, k212, &K[1][11])
// PARAM_ADD(PARAM_FLOAT, k312, &K[2][11])
// PARAM_ADD(PARAM_FLOAT, k412, &K[3][11])

// PARAM_ADD(PARAM_UINT8, idx, &idx)
// PARAM_GROUP_STOP(ctrlLQR)

// LOG_GROUP_START(ctrlLQR)
// LOG_ADD(LOG_FLOAT, e_x, &state_error_vec[0])
// LOG_ADD(LOG_FLOAT, e_y, &state_error_vec[1])
// LOG_ADD(LOG_FLOAT, e_z, &state_error_vec[2])

// LOG_ADD(LOG_FLOAT, e_roll,  &state_error_vec[3])
// LOG_ADD(LOG_FLOAT, e_pitch, &state_error_vec[4])
// LOG_ADD(LOG_FLOAT, e_yaw,   &state_error_vec[5])

// LOG_ADD(LOG_FLOAT, e_vx, &state_error_vec[6])
// LOG_ADD(LOG_FLOAT, e_vy, &state_error_vec[7])
// LOG_ADD(LOG_FLOAT, e_vz, &state_error_vec[8])

// LOG_ADD(LOG_FLOAT, e_vroll,  &state_error_vec[9])
// LOG_ADD(LOG_FLOAT, e_vpitch, &state_error_vec[10])
// LOG_ADD(LOG_FLOAT, e_vyaw,   &state_error_vec[11])

// LOG_ADD(LOG_FLOAT, u1,   &input_vec[0])
// LOG_ADD(LOG_FLOAT, u2,   &input_vec[1])
// LOG_ADD(LOG_FLOAT, u3,   &input_vec[2])
// LOG_ADD(LOG_FLOAT, u4,   &input_vec[3])

// LOG_ADD(LOG_INT16, u2_pwm,   &pwm_int[0])
// LOG_ADD(LOG_INT16, u3_pwm,   &pwm_int[1])
// LOG_ADD(LOG_INT16, u4_pwm,   &pwm_int[2])


// LOG_ADD(LOG_FLOAT, set_pitch, &set_pitch)
// LOG_ADD(LOG_FLOAT, thrust, &thrust)
// LOG_GROUP_STOP(ctrlMel)
