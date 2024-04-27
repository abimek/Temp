#include <Adafruit_MPU6050.h>
#include <Afafruit_Sensor.h>

Adafruit_MPU6050 mpu;
float rotationX;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  rotationX = 0;

  if(!mpu.begin()){
    Serial.println("Failed to Find MPU6050 chip");

    while (1) {
      delay(10);
    }
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}


void loop() {
  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp);

  rotationX = rotationX + g.gyro.x;

  Serial.print("Raw Rotation X": );
  Serial.println(g.gyro.x);
  Serial.print("Rotation X: ");
  Serial.println(rotationX);
}