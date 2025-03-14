/*
 * Code for Space Tigers 2024/2025
 * - Rocksat C
 * 
 * By John (Yianni) Kiritsis
 * Created: 12/30/2024
 * Last Edited: 3/13/2025 Rev 1

 21 Watts
 */


//Includes
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SD.h>

//Pin Defines
#define ESC_PIN 10
#define LASER_INTERRUPT_PIN 3
#define CHIP_SELECT 5

//Other Defines
#define ESC_MIN 1100
#define ESC_MAX 2000

#define FX29_I2C_ADDRESS  0x28
#define DEFAULT_TARE 860 //IDK?>>>>
#define FX29_LOAD_RANGE 111.2  // 25 lbf converted to Newtons

//Global Variables and Structures
Servo ESC;
/*File logFile;*/
File logFile;

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


unsigned long lastFileWriteTime = 0;
unsigned long lastSaveTime = 0;


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
  delay(1000);
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

//*****SD CARD

  /*if(useSerial) Serial.print("Initializing SD Card\n");
  SD.begin();

  if(useSerial)Serial.print("SD Card initialized\n");
  logFile = SD.open("log.txt", FILE_WRITE);
  */
  
  if(useSerial) Serial.print("Initializing SD Card\n");
  SD.begin(CHIP_SELECT);

  if(useSerial)Serial.print("SD Card initialized\n");
  logFile = SD.open("logf.txt", FILE_WRITE);
  

  if(useSerial) Serial.print("*** END SETUP ***\n\n\n");

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
    unsigned long timeSinceFileWrite = currentTime - lastFileWriteTime;
  unsigned long timeSinceSave = currentTime - lastSaveTime;


  //TEMPORARY MOTOR STEP
  if(timeSinceMotorStep >= 2000){
    //Motor starts spinning around 1180
    if(throttle <= 1200){
      throttle+=25;
    }
      ESC.writeMicroseconds(throttle);
     lastMotorStepTime = currentTime; 
  }
      ESC.writeMicroseconds(throttle);

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

 //Write to SD card every 1 second
  if(timeSinceFileWrite >= 500){
    // Open the log file
    
    // If the file is available, write to it
    if (logFile) {
      //Data string format: rpm,force, accelerationx,accelerationy,accelerationz,gyrox,gyroy,gyroz
      logFile.print(rpm);
      logFile.print(',');
      logFile.print(force);
      logFile.print(',');
      logFile.print(a.acceleration.x);
      logFile.print(',');
      logFile.print(a.acceleration.y);
      logFile.print(',');
      logFile.print(a.acceleration.z);
      logFile.print(',');
      logFile.print(g.gyro.x);
      logFile.print(',');
      logFile.print(g.gyro.y);
      logFile.print(',');
      logFile.print(g.gyro.z); // Fixed typo: was g.gyro.y
      logFile.println();
      
      
      // Optional: Also print to Serial for debugging
      if(useSerial) {
        Serial.println("Data written to SD card");
      }
    } else {
      // If the file isn't open, pop up an error
      if(useSerial) {
       // Serial.println("Error opening log.txt");
      }
    }

    lastFileWriteTime = currentTime;
  }

  //save SD card every 10 seconds
  if(timeSinceSave >= 10000){
    logFile.close();
    
    delay(10);

    logFile = SD.open("logf.txt", FILE_WRITE);
    lastSaveTime = currentTime;
  }
  //Print every 1 seconds
  if(timeSincePrint >= 1000 && useSerial){
    Serial.println("*** DATA CHUNK ***");
    
    Serial.print("PWM: ");
    Serial.println(throttle);
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
    Wire.requestFrom(FX29_I2C_ADDRESS, 2);
    
    if (Wire.available() == 2) {
        uint8_t byte1 = Wire.read();
        uint8_t byte2 = Wire.read();

        uint8_t statusBits = (byte1 & 0xC0) >> 6;  // Extract status bits (upper 2 bits)
        uint16_t rawValue = ((byte1 & 0x3F) << 8) | byte2;  // Keep only data bits

        return rawValue;
    } else {
        Serial.println("ERROR: No data received.");
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
              Serial.println("\n\n\n\n\n\n");
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

