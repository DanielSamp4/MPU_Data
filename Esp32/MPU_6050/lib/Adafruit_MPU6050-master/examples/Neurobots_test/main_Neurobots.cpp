#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

//   Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    // Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
//   Serial.println("MPU6050 Found!");

  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
//   mpu.setMotionDetectionThreshold(1);
//   mpu.setMotionDetectionDuration(20);
//   mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
//   mpu.setInterruptPinPolarity(true);
//   mpu.setMotionInterrupt(true);

  // Serial.println("");
  Serial.println("N,0,Arduino Connected");
  // Serial.println("");

  delay(100);
}

void loop() {

//   if(mpu.getMotionInterruptStatus()) {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    // Serial.print("a&");
    Serial.print("I,");
    /* Print out the values */
    // Serial.print("AccelX:");
    // Serial.print(a.acceleration.x, 3);
    Serial.print(0 + millis());
    Serial.print(",");
    // Serial.print(":");
    // Serial.print("AccelY:");
    Serial.print(g.gyro.x, 3);
    Serial.print(",");
    Serial.print(g.gyro.y, 3);
    Serial.print(",");
    Serial.print(g.gyro.z, 3);
    Serial.print(",");
    Serial.print(a.acceleration.x, 3);
    Serial.print(",");
    Serial.print(a.acceleration.y, 3);
    Serial.print(",");
    Serial.println(a.acceleration.z, 3);
    // Serial.print(":");
    // Serial.print("AccelZ:");
    // Serial.print(",");
    // Serial.print(":");
    // Serial.print("GyroX:");
    // Serial.println(g.gyro.x, 3);
    // Serial.print(":");
    // // Serial.print("GyroY:");
    // Serial.print(g.gyro.y, 3);
    // Serial.print(":");
    // // Serial.print("GyroZ:");
    // Serial.print(g.gyro.z, 3);
    // Serial.println();
//   }

  // delay(200);
}