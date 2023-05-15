#include <bluefruit.h>
#include <U8x8lib.h>
#include <Metro.h>
#include "common.h"
#include "imu.h"
#include "ble.h"
// #include "uwb.h"
// #include "vibe.h"
// #include "gesture.h"

#define OLED_DISPLAY true

Metro fps_timer = Metro(1000); // 1hz - 1 cycle per second
Metro metro_20hz = Metro(50);  // 20hz - 20 cycles per second
Metro imu_timer = Metro(20);   // 50hz - 50 cycles per second

// uint8_t wing_data[60] = {0};

struct twig_struct twig = {0};
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/PIN_WIRE_SCL, /* data=*/PIN_WIRE_SDA, /* reset=*/U8X8_PIN_NONE); // OLEDs without Reset of the Display

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10); // for nrf52840 with native usb

  imu_init();
  // ble_init(&twig);
  // uwb_init(&twig);

  if (OLED_DISPLAY)
  {
    u8x8.begin();
    u8x8.setFlipMode(1); // set number from 1 to 3, the screen word will rotary 180
  }
}

void oled_display_imu(struct twig_struct *twig_data)
{
  // @NOTE: Ideally we also include RTC timestamp logging here as well :)

  // https://x-io.co.uk/downloads/x-IMU3-User-Manual-v1.0.pdf (page #23-24)

  /*
  The following message examples are for a timestamp of 1 second (1,000,000 microseconds) and argument
  values of:
    1. Gyroscope X axis = 0
    2. Gyroscope Y axis = 0
    3. Gyroscope Z axis = 0
    4. Accelerometer X axis = 0
    5. Accelerometer Y axis = 0
    6. Accelerometer Z axis = 1

    ASCII example: I,1000000,0.0000,0.0000,0.0000,0.0000,0.0000,1.0000\r\n
  */
  u8x8.setCursor(0, 0);
  u8x8.print("I");
  u8x8.print(",");
  u8x8.print(twig_data->gyro[0], 4);
  u8x8.print(",");
  u8x8.print(twig_data->gyro[1], 4);
  u8x8.print(",");
  u8x8.print(twig_data->gyro[2], 4);
  u8x8.print(",");
  u8x8.print(twig_data->accel[0], 4);
  u8x8.print(",");
  u8x8.print(twig_data->accel[1], 4);
  u8x8.print(",");
  u8x8.print(twig_data->accel[2], 4);
  u8x8.println("");

  /*
    The following message examples are for a timestamp of 1 second (1,000,000 microseconds) and argument
    values of:
      1. Magnetometer X axis = 1
      2. Magnetometer Y axis = 0
      3. Magnetometer Z axis = 0

      ASCII example: M,1000000,1.0000,0.0000,0.0000\r\n
  */
  u8x8.setCursor(0, 1);
  u8x8.print("M");
  u8x8.print(",");
  u8x8.print(twig_data->mag[0], 4);
  u8x8.print(",");
  u8x8.print(twig_data->mag[1], 4);
  u8x8.print(",");
  u8x8.print(twig_data->mag[2], 4);
  u8x8.println("");

  /*
    The following message examples are for a timestamp of 1 second (1,000,000 microseconds) and argument
    values of:
      1. Quaternion W element = 1
      2. Quaternion X element = 0
      3. Quaternion Y element = 0
      4. Quaternion Z element = 0
    ASCII example: Q,1000000,1.0000,0.0000,0.0000,0.0000\r\n
  */
  u8x8.setCursor(0, 2);
  u8x8.print("Q");
  u8x8.print(",");
  u8x8.print(twig_data->quat[3], 4);
  u8x8.print(",");
  u8x8.print(twig_data->quat[0], 4);
  u8x8.print(",");
  u8x8.print(twig_data->quat[1], 4);
  u8x8.print(",");
  u8x8.print(twig_data->quat[2], 4);
  u8x8.println("");
}

void loop()
{
  static int notify_fps = 0;
  static int led_fps = 0;
  twig.fps++;

  // if (metro_20hz.check())
  // {
  //   vibe_update(&cpu_left.vibe, &cpu_right.vibe);
  //   ble_notify(cpu_left.vibe, cpu_right.vibe);
  //   notify_fps++;

  //   gesture_check(&cpu_left, &cpu_right, &cpu_head);
  // }

  if (imu_timer.check())
  {
    imu_update(&twig); // 4 to 9 ms
    imu_print_x_imu3_gui(&twig);

    // we probably should move this to another timer.check() to manage output refresh
    oled_display_imu(&twig);

    //@TODO - we also want to log the imu details to SD card
    //        but we should track the type of motion so that
    //        we can properly classify actions into stick events
    //        for decision tree generation!
    // log_imu_sd()

    led_fps++;
  }

  // VIBE - PLACEHOLDER
  // static int left_tap_event_counter = 0;
  // static int right_tap_event_counter = 0;

  // if (left_tap_event_counter != cpu_left.tap_event_counter)
  // {
  //   vibe_add_quick_slow(VIBE_LEFT, 1, 0);
  //   left_tap_event_counter = cpu_left.tap_event_counter;
  // }

  // if (right_tap_event_counter != cpu_right.tap_event_counter)
  // {
  //   vibe_add_quick_slow(VIBE_RIGHT, 1, 0);
  //   right_tap_event_counter = cpu_right.tap_event_counter;
  // }

  // STATS
  if (fps_timer.check())
  {
    Serial.print("fps: ");
    Serial.print(twig.fps);
    Serial.println(" ");
    twig.fps = 0;
    notify_fps = 0;
    led_fps = 0;
    // cpu_right.fps = 0;
    // cpu_left.fps = 0;
  }
}
