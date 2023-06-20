#include <Arduino.h>
#include "MPU9250.h"
#include "eeprom_utils.h"
#include "BluetoothSerial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
//definitions of handler tasks
TaskHandle_t TaskCollectData_h;
TaskHandle_t TaskSendData_h;

//Prototipes
void TaskSendData( void * pvParameters );
void TaskCollectData( void * pvParameters );

MPU9250 mpu;
// BluetoothSerial SerialBT;
void print_roll_pitch_yaw();
void print_inertial_data();
void print_magnetometer_data();
void print_calibration();
unsigned long time_now;
unsigned long packet = 0;
#define LED_PIN 2 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
void setup() {
    Serial.begin(115200);
    // SerialBT.begin("Neuroprotese");


    

    delay(5000);
    Wire.begin();
    // delay(2000);
    pinMode(LED_PIN, OUTPUT);
    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // mpu.selectFilter(QuatFilterSel::MAHONY);

    mpu.setFilterIterations(20);
    // calibrate anytime you want to
    Serial.print("S,");
    // Serial.print(packet);
    Serial.print(time_now + millis());
    Serial.print(",");
    Serial.println(" Accel Gyro calibration will start in 5sec.");
    Serial.print("S,");
    // Serial.print(packet);
    Serial.print(time_now + millis());
    Serial.print(",");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

    Serial.print("S,");
    // Serial.print(packet);
    Serial.print(time_now + millis());
    Serial.print(",");
    Serial.println("Mag calibration will start in 5sec.");
    Serial.print("S,");
    // Serial.print(packet);
    Serial.print(time_now + millis());
    Serial.print(",");
    Serial.println(" Please Wave device in a figure eight until done.");
    delay(5000);
    mpu.calibrateMag();
    delay(1000);
    // // print_calibration();
    // mpu.verbose(false);
    
    // tasks begin 
    xTaskCreatePinnedToCore(
    TaskSendData, /* Task function. */
    "TaskSendData",   /* name of task. */
    2048,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &TaskSendData_h,    /* Task handle to keep track of created task */
    1);        /* pin task to core 0 */
  xTaskCreatePinnedToCore(
    TaskCollectData, /* Task function. */
    "TaskCollectData",   /* name of task. */
    2048,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &TaskCollectData_h,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */
    
    extern TaskHandle_t loopTaskHandle;
    vTaskDelete(loopTaskHandle);
}
unsigned long num_readings_last_second = 0;
unsigned long start_time = millis();

void loop() {
    //nothing here
    
}

void print_roll_pitch_yaw() {
    Serial.print("A,");
    // Serial.print(packet);
    Serial.print(time_now + millis());
    Serial.print(",");
    Serial.print(mpu.getRoll(), 4);
    Serial.print(",");
    Serial.print(mpu.getPitch(), 4);
    Serial.print(",");
    Serial.println(mpu.getYaw(), 4);
}

void print_quaternions_data() {
    Serial.print("Q,");
    // Serial.print(packet);
    Serial.print(time_now + millis());
    Serial.print(",");
    Serial.print(mpu.getQuaternionW(), 4);
    Serial.print(",");
    Serial.print(mpu.getQuaternionX(), 4);
    Serial.print(",");
    Serial.print(mpu.getQuaternionY(), 4);
    Serial.print(",");
    Serial.println(mpu.getQuaternionZ(), 4);
}
void print_magnetometer_data() {
    Serial.print("M,");
    // Serial.print(packet);
    Serial.print(time_now + millis());
    Serial.print(",");
    Serial.print(mpu.getMagX(), 4);
    Serial.print(",");
    Serial.print(mpu.getMagY(), 4);
    Serial.print(",");
    Serial.println(mpu.getMagZ(), 4);
}
void print_inertial_data() {
    // SerialBT.print("AI,");
    Serial.print("I,");
    // Serial.print(packet);
    Serial.print(time_now + millis());
    Serial.print(",");
    // SerialBT.print(packet);
    // SerialBT.print(",");
    Serial.print(mpu.getGyroX(), 4);
    Serial.print(",");
    Serial.print(mpu.getGyroY(), 4);
    Serial.print(",");
    Serial.print(mpu.getGyroZ(), 4);
    Serial.print(",");
    Serial.print(mpu.getAccX(), 4);
    Serial.print(",");
    Serial.print(mpu.getAccY(), 4);
    Serial.print(",");
    Serial.println(mpu.getAccZ(), 4);
    // SerialBT.print(",");
    // SerialBT.print(mpu.getMagX(), 4);
    // SerialBT.print(",");
    // SerialBT.print(mpu.getMagY(), 4);
    // SerialBT.print(",");
    // SerialBT.println(mpu.getMagZ(), 4);
}

void print_calibration() {
  Serial.print("S,");
    // Serial.print(packet);
    Serial.print(time_now + millis());
    Serial.print(",");
    Serial.println("< calibration parameters >");
    Serial.print("S,");
    // Serial.print(packet);
    Serial.print(time_now + millis());
    Serial.print(",");
    Serial.print("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.print("S,");
    // Serial.print(packet);
    Serial.print(time_now + millis());
    Serial.print(",");
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.print("S,");
    // Serial.print(packet);
    Serial.print(time_now + millis());
    Serial.print(",");
    Serial.print("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.print("S,");
    // Serial.print(packet);
    // Serial.print(packet);
    // Serial.print(",");
    Serial.print("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}

unsigned long prev_ms = micros();

void TaskSendData( void * pvParameters ) {
  

  Serial.print("Task SendData running on core ");
  Serial.println(xPortGetCoreID());
  for (;;) {
    unsigned long current_time = millis() - start_time;
        if (micros() >= prev_ms + 3000) { // maximo de 3000 microsegundos o que equivale a 310Hz
            print_inertial_data();
            print_magnetometer_data();
            print_roll_pitch_yaw();
            // print_quaternions_data();
            // blink LED to indicate activity
            blinkState = !blinkState;
            digitalWrite(LED_PIN, blinkState);
            prev_ms = micros();
            // delay(15);
            // packet++;
            if(current_time < 1000)
                num_readings_last_second ++;
            else{
                Serial.println("S,"+ String(time_now + millis()) +", Número de leituras no último segundo: " + String(num_readings_last_second));
                num_readings_last_second = 1;
                start_time = millis();
            }
        }
  }
  
}

void TaskCollectData( void * pvParameters ) {
  

  Serial.print("Task CollectData running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    
    
    mpu.update();
  }
  
}
