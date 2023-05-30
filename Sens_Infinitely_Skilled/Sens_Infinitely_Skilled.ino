#include <U8x8lib.h>
#include <Metro.h>
#include "common.h"
#include "imu.h"
#include <SD.h>

Metro fps_timer = Metro(1000); // 1hz - 1 cycle per second
Metro metro_20hz = Metro(50);  // 20hz - 20 cycles per second
Metro imu_timer = Metro(20);   // 50hz - 50 cycles per second

struct twig_struct twig = {0};
//U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/PIN_WIRE_SCL, /* data=*/PIN_WIRE_SDA, /* reset=*/U8X8_PIN_NONE); 
const int sdPin = 4;
// File variable
File tempsFile;
void setup()
{
Serial.begin(115200);
  while (!Serial)
    delay(10); // for nrf52840 with native usb

imu_init();   
 Serial.print("Initializing SD card...");
  if(!SD.begin(sdPin)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("Initialization done.");

  tempsFile = SD.open("inf.txt", FILE_WRITE);

  if (tempsFile) {
    Serial.println("Printing temperatures");
    tempsFile.println("Printing temperatures:");
    tempsFile.close();
    Serial.println("Done.");
  } else {
    Serial.println("Error opening file in setup.");
    return;
  }
}

void loop(){

 static int notify_fps = 0;
  static int led_fps = 0;
  twig.fps++;

 if (imu_timer.check())
  {
    imu_update(&twig); // 4 to 9 ms
    imu_print_x_imu3_gui(&twig);
  saveData(&twig);
    // we probably should move this to another timer.check() to manage output refresh
    ///oled_display_imu(&twig);

    //@TODO - we also want to log the imu details to SD card
    //        but we should track the type of motion so that
    //        we can properly classify actions into stick events
    //        for decision tree generation!
    // log_imu_sd()

    led_fps++;
  }


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

void saveData(struct twig_struct *twig_data)
 {
    tempsFile.print("I");
    tempsFile.print(",");
    tempsFile.print(twig_data->ts_micro);
    tempsFile.print(",");
    tempsFile.print(twig_data->gyro[0], 4);
    tempsFile.print(",");
    tempsFile.print(twig_data->gyro[1], 4);
    tempsFile.print(",");
    tempsFile.print(twig_data->gyro[2], 4);
    tempsFile.print(",");
    tempsFile.print(twig_data->accel[0], 4);
    tempsFile.print(",");
    tempsFile.print(twig_data->accel[1], 4);
    tempsFile.print(",");
    tempsFile.print(twig_data->accel[2], 4);
    tempsFile.println("");

    tempsFile.print("M");
    tempsFile.print(",");
    tempsFile.print(twig_data->ts_micro);
    tempsFile.print(",");
    tempsFile.print(twig_data->mag[0], 4);
    tempsFile.print(",");
    tempsFile.print(twig_data->mag[1], 4);
    tempsFile.print(",");
    tempsFile.print(twig_data->mag[2], 4);
    tempsFile.println("");

    tempsFile.print("Q");
    tempsFile.print(",");
    tempsFile.print(twig_data->ts_micro);
    tempsFile.print(",");
    tempsFile.print(twig_data->quat[3], 4);
    tempsFile.print(",");
    tempsFile.print(twig_data->quat[0], 4);
    tempsFile.print(",");
    tempsFile.print(twig_data->quat[1], 4);
    tempsFile.print(",");
    tempsFile.print(twig_data->quat[2], 4);
    tempsFile.println("");
 }
