/*
 * Code for Space Tigers 2024/2025
 * - Rocksat C
 * 
 * By John (Yianni) Kiritsis
 * Created: 12/30/2024
 * Last Edited: 2/11/2025 Rev 1
 */


//Includes
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//Pin Defines
#define ESC_PIN 4
#define LASER_INTERRUPT_PIN 3

//Other Defines
#define ESC_MIN 1000
#define ESC_MAX 2000

#define FX29_I2C_ADDRESS  0x28
#define DEFAULT_TARE 860 //IDK?>>>>
#define FX29_LOAD_RANGE 111.2  // 25 lbf converted to Newtons

//Global Variables and Structures
Servo ESC;

Adafruit_MPU6050 MPU;
unsigned long lastAccelerometerCalculationTime = 0;
sensors_event_t a, g, temp;

float fx29Tare = 0; 
unsigned long lastForceCalculationTime = 0;
float force;

const int ringSlits = 50;
volatile unsigned int pulseCount = 0;
unsigned long lastRPMCalculationTime = 0;
float rpm = 0;

unsigned long lastPrintTime = 0;

//Functions
uint16_t readRawData();
float calibrateFX29();
void countPulse();

const bool useSerial = true;
void setup() {
  delay(2000);
  if(useSerial){
    Serial.begin(115200);
    delay(100);
    Serial.print("...STARTING...\n");
  }

  //Attatch ESC and set PWM pulse from 1000 to 2000mus
  ESC.attach(ESC_PIN, ESC_MIN, ESC_MAX); 
  ESC.writeMicroseconds(ESC_MIN);
  if(useSerial) Serial.print("SETUP: (PASS) ESC Intialized");

  //Begin I2C
  Wire.begin();
  if(useSerial) Serial.print("SETUP: (PASS) I2C Intialized\n");

  //Begin MPU
  if (!MPU.begin()) {
    if(useSerial) Serial.print("SETUP: (FAIL) Failed to find MPU6050 chip\n");
		while (1) {
		  delay(10);
		}
	}
  MPU.setAccelerometerRange(MPU6050_RANGE_8_G);
  MPU.setGyroRange(MPU6050_RANGE_500_DEG);
	MPU.setFilterBandwidth(MPU6050_BAND_21_HZ);
  MPU.getEvent(&a, &g, &temp);
  if(useSerial) Serial.print("SETUP: (PASS) MPU6050 Initialized\n");

  //Laser Interrupt
  pinMode(LASER_INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LASER_INTERRUPT_PIN), countPulse, FALLING);
  if(useSerial) Serial.print("SETUP: (PASS) Laser Interrupt Initialized\n");

  //Give 1 second1 for everything to wrap up then set tare
  delay(1000);
  
  //Calculate Tare
  fx29Tare = calibrateFX29();
  if(fx29Tare < 0) fx29Tare = DEFAULT_TARE;

  if(useSerial){
    Serial.print("Tare: ");
    Serial.print(fx29Tare);
    Serial.print("\n");
  }

  if(useSerial) Serial.print("*** END SETUP ***\n\n\n");

}

unsigned long lastMotorStepTime = 0;
unsigned int throttle = ESC_MIN;
bool goUp = true;
void loop() {
  //All Times
  unsigned long currentTime = millis();

  unsigned long timeSinceRPMCalculation = currentTime - lastRPMCalculationTime;
  unsigned long timeSinceForceCalculation = currentTime - lastForceCalculationTime;
  unsigned long timeSinceAccelerometerCalculation = currentTime - lastAccelerometerCalculationTime;
  unsigned long timeSincePrint = currentTime - lastPrintTime;
  
  unsigned long timeSinceMotorStep = currentTime - lastMotorStepTime;

  //TEMPORARY MOTOR STEP
  if(timeSinceMotorStep >= 2000){
    if(throttle <= 1900){
      ESC.writeMicroseconds(throttle);
      throttle+=50;
    }
     lastMotorStepTime = currentTime; 
  }

  //RPM Computation every 1 second
  if(timeSinceRPMCalculation >= 1000){
    //disable interrupts temporarily
    noInterrupts();

    //save pulse count
    unsigned int _count = pulseCount;
    pulseCount = 0;

    //enable interrupts
    interrupts(); 

    //compute rpm: ((_count pulse/sec) * (60 sec/min) )
      //        / (ringSlits pulse/revolution)
    rpm = (_count * 60.0) / ringSlits;

    if(useSerial) {
      Serial.print("RPM: ");
      Serial.print(rpm);
      Serial.print("\n");
    }
    lastRPMCalculationTime = currentTime;
  }

  //Calculate Force every 0.25 seconds
  if(timeSinceForceCalculation >= 250){
    uint16_t fx29RawData = readRawData();
    float oldForce = force;

    float fx29CorrectedData = fx29RawData - fx29Tare;
    force = ((fx29CorrectedData) / 14000.0) * FX29_LOAD_RANGE;

    //if rawdata is bad then just keep old force
    if (fx29RawData == 0) {
        if(useSerial) Serial.print("ERROR: READ DATA\n");
        force = oldForce; 
    }

    lastForceCalculationTime = currentTime;
  }

  //do accelerometer every 0.5 seconds
  if(timeSinceAccelerometerCalculation >= 500){
    MPU.getEvent(&a, &g, &temp);
  }

  //Print every 0.5 seconds
  if(timeSincePrint >= 1000 && useSerial){
    Serial.println("*** DATA CHUNK ***");
    
    Serial.print("RPM: ");
    Serial.println(rpm);

    Serial.print("FORCE: ");
    Serial.println(force);

    Serial.print("GYRO: ");
    Serial.print(a.acceleration.x);
    Serial.print(" (x), ");
    Serial.print(a.acceleration.y);
    Serial.print(" (y), ");
    Serial.print(a.acceleration.z);
    Serial.println(" (z)");
    
    Serial.print("ANGL: ");
    Serial.print(g.gyro.x);
    Serial.print(" (x), ");
    Serial.print(g.gyro.y);
    Serial.print(" (y), ");
    Serial.print(g.gyro.y);
    Serial.println(" (z)");
    lastPrintTime = currentTime;
  }
  
}

uint16_t readRawData() {
    uint8_t data[2];

    Wire.beginTransmission(FX29_I2C_ADDRESS);
    Wire.endTransmission();

    Wire.requestFrom(FX29_I2C_ADDRESS, 2);
    if (Wire.available() == 2) {
        data[0] = Wire.read();  // Read first byte
        data[1] = Wire.read();  // Read second byte

        // Mask status bits, keeping only the 14-bit data
        return ((data[0] & 0x3F) << 8) | data[1];
    } else {
        return 0; 
    }
}

float calibrateFX29() {
    float sum = 0;
    int samples = 20;

    for (int i = 0; i < samples; i++) {
        uint16_t reading = readRawData();
        if (reading == 0) {
            if(useSerial) Serial.print("Error: No valid data received during calibration.\n");
         
            return -1;  //Error
        }
        sum += reading;
        delay(50);
    }
    return sum / samples;
}

//ISR
void countPulse() {
    pulseCount++;
}
