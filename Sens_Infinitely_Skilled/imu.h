#ifndef _IMU_H
#define _IMU_H
#include <Arduino.h>
#include <Wire.h>
#include <time.h>
#include "SparkFun_ISM330DHCX.h"
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include "Fusion.h" // absolute lifesaver!
#include "common.h"

/* Constants */
static constexpr double DEGREES_PER_RADIAN =
    (180.0 / 3.141592653589793238463);         ///< Degrees per radian for conversion
static constexpr double GRAVITY_EARTH = 9.807; ///< Standard Earth Gravity

void imu_init(void);
void imu_update(struct twig_struct *twig_data);
void imu_print(struct twig_struct *twig_data);
void imu_print_motion_cal(struct twig_struct *twig_data);
void imu_print_x_imu3_gui(struct twig_struct *twig_data);
 
#endif
