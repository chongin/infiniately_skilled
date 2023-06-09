#include <time.h>

#ifndef _COMMON_H
#define _COMMON_H

// #define FFT_SAMPLES 8

struct twig_struct
{
  uint8_t twig_data;
  long ts_micro; // microseconds

  // int recenter_counter;
  float accel[3];
  float gyro[3];
  float mag[3];
  float quat[4];

  float yaw_filtered;
  float yaw_ref;
  uint8_t yaw;

  float pitch_filtered;
  float pitch_ref;
  uint8_t pitch;

  float roll_filtered;
  float roll_ref;
  uint8_t roll;

  double heading;

  // uint8_t step_count;

  // uint8_t tap_event;
  // uint8_t tap_event_counter;

  uint32_t msg_time;
  uint8_t msg_count;

  int fps;
};

#endif
