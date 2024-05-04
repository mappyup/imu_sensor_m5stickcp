#include <M5Unified.h>
#include <MahonyAHRS.h>

#include "imu_sensor.h"

Mahony filter;

void IMUSensor::initIMUSensor() {
    // Mahony filter
    filter.begin(1000 / SENSOR_READ_TIMER_MS);
    // queue array initialize
    for(int i = 0; i < MAX_QUEUE_LENGTH; i++){
        Qpitch[i] = 0.0F;
        Qroll[i] = 0.0F;
        Qyaw[i] = 0.0F;
    }
}

void IMUSensor::taskRun() {
    while(1){
        // Stores the triaxial gyroscope data of the inertial sensor to the relevant
        M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
        // Stores the triaxial accelerometer
        M5.Imu.getAccelData(&accX, &accY, &accZ);
        // Stores the inertial sensor attitude
        filter.updateIMU(gyroX, gyroY, gyroZ, accX, accY, accZ);
        roll = filter.getRoll();
        pitch = filter.getPitch();
        yaw = filter.getYaw();

        // simple moving average process 
        for(int i = 0; i < MAX_QUEUE_LENGTH - 1; i++){
            Qroll[i] = Qroll[i+1];
            Qpitch[i] = Qpitch[i+1];
            Qyaw[i] = Qyaw[i+1];
        }
        // add latest data at the end of array
        Qroll[MAX_QUEUE_LENGTH - 1] = roll;
        Qpitch[MAX_QUEUE_LENGTH - 1] = pitch;
        Qyaw[MAX_QUEUE_LENGTH - 1] = yaw;

        // Calibration for roll, pitch, yaw
        if(calibration == 0){
            calibrationCount++;
            if(200 <= calibrationCount){
            calibration = 1;
            calibrationCount = 0;
            yawOld = yaw;
            }
        } else if(calibration == 1){
            float gyro = abs(gyroX) + abs(gyroY) + abs(gyroZ);
            if(GYRO_CAL_THRESHOLD < gyro){
            calibrationCount = 0;
            rollSum = 0;
            pitchSum = 0;
            yawSum = 0;
            Serial.println("##### Gyro cal");
            } else {
            rollSum += roll;
            pitchSum += pitch;
            yawSum += (yawOld - yaw);
            yawOld = yaw;
            calibrationCount++;
            if(CAL_COUNT <= calibrationCount){
                calibration = 2;
                rollOffset = rollSum / calibrationCount;
                pitchOffset = pitchSum / calibrationCount;
                yawOffset = yawSum / calibrationCount;
            }
            }
        } else {
            // adjust roll, pitch value by offset
            // yaw is not adjusted
            // TODO: substitute yaw by yaw2
            roll -= rollOffset;
            pitch -= pitchOffset;
            float yawDelta = yawOld - yaw;
            if (-YAW_DELTA_LIM < yawDelta && yawDelta < YAW_DELTA_LIM) {
            yaw2 -= yawDelta - yawOffset;
            yaw2 = yaw2 * 0.9;
            }
        }

        // calculate SMA respectively
        for(int i = 0; i < MAX_QUEUE_LENGTH - 1; i++) sum = sum + Qroll[i];
        ave_roll = sum / float(MAX_QUEUE_LENGTH);
        sum = 0.0F;
        for(int i = 0; i < MAX_QUEUE_LENGTH - 1; i++) sum = sum + Qpitch[i];
        ave_pitch = sum / float(MAX_QUEUE_LENGTH);
        sum = 0.0F;
        for(int i = 0; i < MAX_QUEUE_LENGTH - 1; i++) sum = sum + Qyaw[i];
        ave_yaw = sum / float(MAX_QUEUE_LENGTH);
        sum = 0.0F;

        delay(SENSOR_READ_TIMER_MS);
    }
}   // taskRun()

float IMUSensor::s_roll() {
    return ave_roll;
}

float IMUSensor::s_pitch() {
    return ave_pitch;
}

float IMUSensor::s_yaw() {
    return ave_yaw;
}
