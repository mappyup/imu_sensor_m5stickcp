#ifndef _IMU_SENSOR_M5SCP_IMU_SENSOR_H_
#define _IMU_SENSOR_M5SCP_IMU_SENSOR_H_

#include <M5Unified.h>

#define MAX_QUEUE_LENGTH 10
#define GYRO_CAL_THRESHOLD 20
#define CAL_COUNT 500
#define YAW_DELTA_LIM 200
#define SENSOR_READ_TIMER_MS 10
#define TIMER_COUNT (1000 * SENSOR_READ_TIMER_MS)

class IMUSensor
{
private:
    // sensor data
    float accX = 0.0F;
    float accY = 0.0F;
    float accZ = 0.0F;

    float gyroX = 0.0F;
    float gyroY = 0.0F;
    float gyroZ = 0.0F;

    float pitch = 0.0F;
    float roll = 0.0F;
    float yaw = 0.0F;
    float sum = 0.0F;
    float ave_pitch = 0.0F;
    float ave_roll = 0.0F;
    float ave_yaw = 0.0F;

    // calibration
    uint8_t calibration = 0;
    uint16_t calibrationCount = 0;
    float pitchSum = 0.0F;
    float rollSum = 0.0F;
    float yawSum = 0.0F;
    float pitchOffset = 0.0F;
    float rollOffset = 0.0F;
    float yawOffset = 0.0F;
    float yawOld = 0.0F;
    float yaw2 = 0.0F;

    // array for SMA calculate
    float Qroll[MAX_QUEUE_LENGTH];
    float Qpitch[MAX_QUEUE_LENGTH];
    float Qyaw[MAX_QUEUE_LENGTH];

public:
    void initIMUSensor();
    void taskRun();
    float s_roll();
    float s_pitch();
    float s_yaw();
};


#endif  // _IMU_SENSOR_M5SCP_IMU_SENSOR_H_