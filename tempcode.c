#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

double z_setpoint, z_input, z_output;
PID ZPID (&z_input, &z_output, &z_setpoints, 1, 1, 0, DIRECT);

double y_setpoint, y_input, y_output;
PID YPID (&y_input, &y_output, &y_setpoints, 1, 1, 0, DIRECT);

double x_setpoint, x_input, x_output;
PID XPID (&x_input, &x_output, &x_setpoints, 1, 1, 0, DIRECT);

Adafruit_MPU6050 mpu;

Servo servo1;
Servo servo2;

Servo throttle3;

float rotationX;
float rotationY;
float rotationZ;
float altitude;

#define CHANNEL1_PIN 2
#define CHANNEL2_PIN 3
#define CHANNEL3_PIN 18
#define CHANNEL4_PIN 19
#define ULTRASONIC_PIN 9

// THe servos are not exactly center at 90 for their center, for servo one, the mid point is about 88
#define SERVO1_PIN 6
// center is 82 for servo 2
#define SERVO2_PIN 7
#define SERVO_THROTTLE3 10

#define FILTER_SIZE 10

volatile int channel1 = 0;
volatile int channel2 = 0;
volatile int channel3 = 0;
volatile int channel4 = 0;

float lastVelocity = 0;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  throttle3.attach(SERVO_THROTTLE3);

  if(!mpu.begin()){
    Serial.println("Failed to Find MPU6050 chip");

    while (1) {
      delay(10);
    }
  }

  pinMode(CHANNEL1_PIN, INPUT);
  pinMode(CHANNEL2_PIN, INPUT);
  pinMode(CHANNEL3_PIN, INPUT);
  pinMode(CHANNEL4_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(CHANNEL1_PIN), readChannel1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL2_PIN), readChannel2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL3_PIN), readChannel3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL4_PIN), readChannel4, CHANGE);

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  ZPID.SetMode(AUTOMATIC);
  YPID.SetMode(AUTOMATIC);
  XPID.SetMode(AUTOMATIC);


}

unsigned long time = 0;

// error for x is .07 (add)
// weoee z .015 (subtrCT)

// retu
float threshold(float value, float threshold){
  if(abs(value) > threshold){
    return value;
  }else{
    return 0.0;
  }
}

float thresholdFromCenter(float value, float center, float threshold){
  float difference = value - center;
  if (value > threshold){
    return value;
  } else {
    return center;
  }
}

float currentPosition = 0;

long ultrasonic_loop() {
    long duration, inches;

    pinMode(ULTRASONIC_PIN, OUTPUT);
    digitalWrite(ULTRASONIC_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(ULTRASONIC_PIN, LOW);

    pinMode(ULTRASONIC_PIN, INPUT);
    duration = pulseIn(ULTRASONIC_PIN, INPUT);

    inches = duration / 74 / 2;
    return inches;
}

int value_or_max(int value, int max){
  if(value > max){
    return max;
  }
  return value;
}


void loop() {
  sensors_event_t a, g, temp;
  long sonic_distance = ultrasonic_loop();

  unsigned long newTime = millis();
  float deltaTime = (newTime - time);
  //Serial.print("Check below: ");
  //Serial.println(deltaTime);
  deltaTime = deltaTime / 1000;

  time = newTime;

  mpu.getEvent(&a, &g, &temp);

  rotationZ = rotationZ + ((threshold(g.gyro.z - .015, .01)) * (180/3.141)) * deltaTime;
  rotationX = rotationX + ((threshold(g.gyro.x + .07, .01)) * (180/3.141)) * deltaTime;
  rotationY = rotationY + ((threshold(g.gyro.y - .01, .01)) * (180/3.141)) * deltaTime;
  float currentVelocity = lastVelocity + threshold(g.acceleration.y, .01) * deltaTime;

  currentPosition += currentVelocity * deltaTime;

  lastVelocity = currentVelocity;

 // altitude = altitiude * deltaTime;
  Serial.print("Raw Altitiude: ");
  Serial.println(g.acceleration.y);
  Serial.print("Altitiude: ");
  Serial.println(currentPosition);

  float complete_throttle = map(channel3, 1000, 2000, 0, 100);

  //whatever channel is forward ont he right stick
  y_setpoint = value_or_max(complete_throttle + map(channel2, 1000, 2000, -50, 50), 100);
  y_input = complete_throttle + .5 * rotationY;
  YPID.Compute();

  //whatever channel is left and right on the right stick
  x_setpoint = map(channel4, 1000, 2000, -45, 45);
  x_input = rotationX;
  XPID.Compute();

  throttle3.write(y_output);

  // leave x alone



  // use computed value to figure out how much to turn left




 // servo2.write(82);
  //Serial.print("Servo Value: ");
  //Serial.println(servo2.read());
  //delay(1000);

  float servo1Value = map(channel3, 1000, 2000, 0, 180) - 2;
  //float servo2Value = map(channel3)
}

void readChannel(volatile int &channel, int pin, unsigned long readings[], int &readIndex, unsigned long &total) {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = micros();

  if (digitalRead(pin) == HIGH) {
    lastInterruptTime = interruptTime;
  } else {
    // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = interruptTime - lastInterruptTime;
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;

    // if we're at the end of the array...
    if (readIndex >= FILTER_SIZE) {
      // ...wrap around to the beginning:
      readIndex = 0;
    }

    // calculate the average:
    channel = total / FILTER_SIZE;
  }
}

void readChannel1() {
  static unsigned long readings[FILTER_SIZE] = {0};
  static int readIndex = 0;
  static unsigned long total = 0;
  readChannel(channel1, CHANNEL1_PIN, readings, readIndex, total);
}

void readChannel2() {
  static unsigned long readings[FILTER_SIZE] = {0};
  static int readIndex = 0;
  static unsigned long total = 0;
  readChannel(channel2, CHANNEL2_PIN, readings, readIndex, total);
}

void readChannel3() {
  static unsigned long readings[FILTER_SIZE] = {0};
  static int readIndex = 0;
  static unsigned long total = 0;
  readChannel(channel3, CHANNEL3_PIN, readings, readIndex, total);
}

void readChannel4() {
  static unsigned long readings[FILTER_SIZE] = {0};
  static int readIndex = 0;
  static unsigned long total = 0;
  readChannel(channel4, CHANNEL4_PIN, readings, readIndex, total);
}