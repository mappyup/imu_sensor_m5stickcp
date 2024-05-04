#include <M5Unified.h>
#include "imu_sensor.h"

#define BAUDRATE 19200
#define DATA_UPDATE_MS 100

// multi-task
void task0(void* arg);

// val
float roll = 0.0F;
float pitch = 0.0F;
float yaw = 0.0F;

IMUSensor imuS;

void setup() {
  // put your setup code here, to run once:
  auto cfg = M5.config();
  cfg.serial_baudrate = BAUDRATE;
  cfg.internal_imu = true;

  M5.begin(cfg);

  // LCD Screen setting ## M5StickC Plus2
  M5.Display.setRotation(1);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextSize(2);

  // multi-task
  imuS.initIMUSensor();
  xTaskCreatePinnedToCore(task0, "Sensor", 4096, NULL, 1, NULL, 1);

}

void loop() {
  // put your main code here, to run repeatedly:
  M5.update();

  // read sensor data
  roll = imuS.s_roll();
  pitch = imuS.s_pitch();
  yaw = imuS.s_yaw();

  // change format up to reveiver
  Serial.printf("%5.1f,%5.1f,%5.1f\n", roll, pitch, yaw);

  M5.Display.fillScreen(BLACK);
  M5.Display.setCursor(0, 20);
  M5.Display.printf(" R: %6.2f\n P: %6.2f\n Y: %6.2f\n", roll, pitch, yaw);

  delay(DATA_UPDATE_MS);
}

void task0(void* arg){
  imuS.taskRun();
}
