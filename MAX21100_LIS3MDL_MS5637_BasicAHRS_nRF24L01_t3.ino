/* MAX21100_LIS3MDL_MS5637_t3 Basic Example Code
 by: Kris Winer
 date: September 1, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrates basic MAX21100 functionality including parameterizing the register addresses, 
 initializing the sensor, communinicating with slave magnetometer LIS3MDL and pressure sensor MS5637, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. 
 
 Added display functions to allow display to on breadboard monitor. 
 
 Addition of 9 DoF sensor fusion using open source Madgwick and Mahony filter algorithms. 
 Can compare results to hardware 9 DoF sensor fusion carried out on the MAX21100.
 Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 This sketch is intended specifically for the MAX21100+LIS3MDL+MS5637 Add-On Shield for the Teensy 3.1.
 It uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 
 The Add-on shield can also be used as a stand-alone breakout board for any Arduino, Teensy, or 
 other microcontroller by closing the solder jumpers on the back of the board.
 
 The LIS3MDL magnetometer is super small (2 mm x 2 mm) and ultra-low power, with 16-bit resolution and
 full scale selectable between +/- 4, 8, 12 and 16 Gauss. 
 
 The MS5637 is a simple but high resolution (24-bit) pressure sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.
 
 All sensors communicate via I2C at 400 Hz or higher.
 SDA and SCL should have external pull-up resistors (to 3.3V).
 4K7 resistors are on the MAX21100_LIS3MDL_MS5637 breakout board.
 
 Hardware setup:
 Breakout Board --------- Arduino/Teensy
 3V3 ---------------------- 3.3V
 SDA ----------------------- A4/17
 SCL ----------------------- A5/16
 GND ---------------------- GND
 
 Note: The MAX21100_LIS3MDL_MS5637 breakout board is an I2C sensor and uses the Arduino Wire or Teensy i2c_t3.h library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 The Teensy has no internal pullups and we are using the Wire.begin function of the i2c_t3.h library
 to select 400 Hz i2c speed.
 
 The Teensy 3.1 set up for nRF24L01+ is as follows:

Connection              Teensy 3.1 pin
+++++++++++++++++++++++++++++++++++
3.7 V LiPo battery     VIN
ground                 GND
nRF24L01+ CSN          10
nRF24L01+ MOSI         11
nRF24L01+ MISO         12
nRF24L01+ SCK          13
nRF24L01+ CE           14
Motor1 control          ?  PWM
Motor2 control          ?  PWM
Motor3 control          ?  PWM
Motor4 control          ?  PWM
Power all devices     3.3 V
Ground all devices    GND

Other
++++++++++++++++++++++++++++++++++++
nRF24L01+ IRQ           9 or 15   LOW for Receiver 

 */
//#include <Wire.h>   
#include <i2c_t3.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>


// Set up nRF24L01+ radio on SPI bus plus pins 14 & 10
RF24 radio(14, 10);
// sets the role of this unit in hardware.  Connect to GND to be the 'pong' receiver
// Leave open to be the 'ping' transmitter
const short role_pin = 20;

// Single radio pipe address for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

//
// Role management
//
// Set up role.  This sketch uses the same software for all the nodes in this
// system.  Doing so greatly simplifies testing.  The hardware itself specifies
// which node it is.
//
// This is done through the role_pin
//

// The various roles supported by this sketch
typedef enum { role_sender = 1, role_receiver } role_e;

// The debug-friendly names of those roles
const char* role_friendly_name[] = { "invalid", "Sender", "Receiver"};

// The role of the current running sketch
role_e role;

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 3 - LCD chip select (SCE)
// pin 4 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 3, 4);

// See MS5637-02BA03 Low Voltage Barometric Pressure Sensor Data Sheet
// http://www.meas-spec.com/product/pressure/MS5637-02BA03.aspx
//
#define MS5637_RESET      0x1E
#define MS5637_CONVERT_D1 0x40
#define MS5637_CONVERT_D2 0x50
#define MS5637_ADC_READ   0x00

// LIS3MDL Digital output magnetic sensor:
// http://www.st.com/web/catalog/sense_power/FM89/SC1449/PF255198
//
//Magnetometer Registers
#define LIS3MDL_ADDRESS      0x1C
#define LIS3MDL_WHO_AM_I     0x0F  // should return 0x3D
#define LIS3MDL_CTRL_REG1    0x20
#define LIS3MDL_CTRL_REG2    0x21
#define LIS3MDL_CTRL_REG3    0x22
#define LIS3MDL_CTRL_REG4    0x23
#define LIS3MDL_CTRL_REG5    0x24
#define LIS3MDL_STATUS_REG   0x27   
#define LIS3MDL_OUT_X_L	     0x28  // data
#define LIS3MDL_OUT_X_H	     0x29
#define LIS3MDL_OUT_Y_L	     0x2A
#define LIS3MDL_OUT_Y_H	     0x2B
#define LIS3MDL_OUT_Z_L	     0x2C
#define LIS3MDL_OUT_Z_H	     0x2D
#define LIS3MDL_TEMP_OUT_L   0x2E
#define LIS3MDL_TEMP_OUT_H   0x2F  // data
#define LIS3MDL_INT_CFG	     0x30
#define LIS3MDL_INT_SRC	     0x31
#define LIS3MDL_INT_THS_L    0x32
#define LIS3MDL_INT_THS_H    0x33

// MAX21100 Register Map
// http://www.maximintegrated.com/en/products/analog/sensors-and-sensor-interface/MAX21100.html
//
// MAX21100 Common Bank
#define MAX21100_WHO_AM_I      0x20    // should be 0xB2              
#define MAX21100_REVISION_ID   0x21                                                                          
#define MAX21100_BANK_SELECT   0x22
#define MAX21100_SYSTEM_STATUS 0x23
#define MAX21100_GYRO_X_H      0x24
#define MAX21100_GYRO_X_L      0x25
#define MAX21100_GYRO_Y_H      0x26
#define MAX21100_GYRO_Y_L      0x27
#define MAX21100_GYRO_Z_H      0x28
#define MAX21100_GYRO_Z_L      0x29
#define MAX21100_ACC_X_H       0x2A
#define MAX21100_ACC_X_L       0x2B
#define MAX21100_ACC_Y_H       0x2C
#define MAX21100_ACC_Y_L       0x2D
#define MAX21100_ACC_Z_H       0x2E
#define MAX21100_ACC_Z_L       0x2F
#define MAX21100_MAG_X_H       0x30
#define MAX21100_MAG_X_L       0x31
#define MAX21100_MAG_Y_H       0x32
#define MAX21100_MAG_Y_L       0x33
#define MAX21100_MAG_Z_H       0x34
#define MAX21100_MAG_Z_L       0x35
#define MAX21100_TEMP_H        0x36
#define MAX21100_TEMP_L        0x37
#define MAX21100_FIFO_COUNT    0x3C
#define MAX21100_FIFO_STATUS   0x3D
#define MAX21100_FIFO_DATA     0x3E
#define MAX21100_RST_REG       0x3F

// MAX21100 Bank 0
#define MAX21100_POWER_CFG     0x00
#define MAX21100_GYRO_CFG1     0x01
#define MAX21100_GYRO_CFG2     0x02
#define MAX21100_GYRO_CFG3     0x03
#define MAX21100_PWR_ACC_CFG   0x04
#define MAX21100_ACC_CFG1      0x05
#define MAX21100_ACC_CFG2      0x06
#define MAX21100_MAG_SLV_CFG   0x07
#define MAX21100_MAG_SLV_ADD   0x08
#define MAX21100_MAG_SLV_REG   0x09
#define MAX21100_MAG_MAP_REG   0x0A
#define MAX21100_I2C_MST_ADD   0x0B
#define MAX21100_I2C_MST_DATA  0x0C
#define MAX21100_MAG_OFS_X_MSB 0x0D
#define MAX21100_MAG_OFS_X_LSB 0x0E
#define MAX21100_MAG_OFS_Y_MSB 0x0F
#define MAX21100_MAG_OFS_Y_LSB 0x10
#define MAX21100_MAG_OFS_Z_MSB 0x11
#define MAX21100_MAG_OFS_Z_LSB 0x12
#define MAX21100_DR_CFG        0x13
#define MAX21100_IO_CFG        0x14
#define MAX21100_I2C_PAD       0x15
#define MAX21100_I2C_CFG       0x16
#define MAX21100_FIFO_TH       0x17
#define MAX21100_FIFO_CFG      0x18
#define MAX21100_DSYNC_CFG     0x1A
#define MAX21100_DSYNC_CNT     0x1B
#define MAX21100_ITF_OTP       0x1C

// MAX21100 Bank 1
#define MAX21100_INT_REF_X     0x00
#define MAX21100_INT_REF_Y     0x01
#define MAX21100_INT_REF_Z     0x02
#define MAX21100_INT_DEB_X     0x03
#define MAX21100_INT_DEB_Y     0x04
#define MAX21100_INT_DEB_Z     0x05
#define MAX21100_INT_MSK_X     0x06
#define MAX21100_INT_MSK_Y     0x07
#define MAX21100_INT_MSK_Z     0x08
#define MAX21100_INT_MSK_AQ    0x09
#define MAX21100_INT_CFG1      0x0A
#define MAX21100_INT_CFG2      0x0B
#define MAX21100_INT_TM0       0x0C
#define MAX21100_INT_STS_UL    0x0D
#define MAX21100_INT_STS       0x0E
#define MAX21100_INT_MSK       0x0F
#define MAX21100_INT_SRC_SEL   0x17
#define MAX21100_SERIAL_5      0x1A
#define MAX21100_SERIAL_4      0x1B
#define MAX21100_SERIAL_3      0x1C
#define MAX21100_SERIAL_2      0x1D
#define MAX21100_SERIAL_1      0x1E
#define MAX21100_SERIAL_0      0x1F

// MAX21100 Bank 2 (Bank select 0010)

#define MAX21100_QUAT0_H         0x00
#define MAX21100_QUAT0_L         0x01
#define MAX21100_QUAT1_H         0x02
#define MAX21100_QUAT1_L         0x03
#define MAX21100_QUAT2_H         0x04
#define MAX21100_QUAT2_L         0x05
#define MAX21100_QUAT3_H         0x06
#define MAX21100_QUAT3_L         0x07
#define MAX21100_BIAS_GYRO_X_H   0x13
#define MAX21100_BIAS_GYRO_X_L   0x14
#define MAX21100_BIAS_GYRO_Y_H   0x15
#define MAX21100_BIAS_GYRO_Y_L   0x16
#define MAX21100_BIAS_GYRO_Z_H   0x17
#define MAX21100_BIAS_GYRO_Z_L   0x18
#define MAX21100_BIAS_COMP_ACC_X 0x19
#define MAX21100_BIAS_COMP_ACC_Y 0x1A
#define MAX21100_BIAS_COMP_ACC_Z 0x1B
#define MAX21100_FUS_CFG0        0x1C
#define MAX21100_FUS_CFG1        0x1D
#define MAX21100_YR_ODR_TRIM     0x1F


// Using the MAX21100_LIS3MDL_MS5637 breakout board/Teensy 3.1 Add-On Shield, ADO is set to 0 
#define ADO 0
#if ADO
#define MAX21100_ADDRESS 0x59   //  Device address of MAX21100 when ADO = 1
#define LIS3MDL_ADDRESS  0x1C   //  Address of LIS3MDL magnetometer
#define MS5637_ADDRESS   0x76   //  Address of MS5637 altimeter
#else
#define MAX21100_ADDRESS 0x58   //  Device address of MAX21100 when ADO = 0
#define LIS3MDL_ADDRESS  0x1C   //  Address of LIS3MDL magnetometer
#define MS5637_ADDRESS   0x76   //  Address of MS5637 altimeter
#endif  

#define SerialDebug true      // set to true to get Serial output for debugging
#define MAX21100Bypass false  // operate magnetometer as slave to MAX21100 when false

// Set initial input parameters
enum Ascale {
  AFS_16G = 0,
  AFS_8G,
  AFS_4G,
  AFS_2G
};

enum Aodr { // gyro Output Data Rate
  AODR_2kHz = 0,
  AODR_1kHz,
  AODR_500Hz,
  AODR_250Hz,
  AODR_125Hz,   // default
  AODR_62_5Hz,
  AODR_31_25Hz    //0x0F
};

enum Abw { // accel bandwidth
  ABW_div48 = 0,  // default, accel bandwidth is 1/48 of the accel ODR
  ABW_div22,
  ABW_div9,
  ABW_div3        // 0x03
};

enum Gscale {  // gyro full scale
  GFS_2000DPS = 0,
  GFS_1000DPS,
  GFS_500DPS,
  GFS_250DPS         // 0x03
};

enum Godr { // gyro Output Data Rate
  GODR_8kHz = 0,
  GODR_4kHz,
  GODR_2kHz,
  GODR_1kHz,
  GODR_500Hz,   // default
  GODR_250Hz,
  GODR_125Hz,
  GODR_62_5Hz,  // 62.5 Hz
  GODR_31_25Hz,
  GODR_15_625Hz,
  GODR_7_8125Hz,   
  GODR_3_90625Hz

};

enum Gbw { // gyro bandwidth
  GBW_2Hz = 0,
  GBW_4Hz,
  GBW_6Hz,
  GBW_8Hz,
  GBW_10Hz,
  GBW_14Hz,
  GBW_22Hz,
  GBW_32Hz,
  GBW_50Hz,
  GBW_75Hz,
  GBW_100Hz,  // default = 0x0A
  GBW_150Hz,
  GBW_200Hz,
  GBW_250Hz,
  GBW_300Hz,
  GBW_400Hz  // 0x0F
};

enum powerMode {  // power modes without using DSYNC enable
  powerDownmode = 0, // 0x00
  gyrosleepmode,
  gyrospotmode,
  gyronormalmode,
  notused0,
  notused1,
  notused2,
  notused3,
  accelspotmode, // 0x08
  notused4,
  notused5,
  notused6,
  accelnormalmode,
  accelnormalgyrosleepmode,
  accelnormalgyrospotmode,
  accelnormalgyronormalmode  // 0x0F
};

enum Mscale {
  MFS_4Gauss = 0,  // 0.15 mG per LSB
  MFS_8Gauss,      // 0.30 mG per LSB
  MFS_12Gauss,     // 0.60 mG per LSB
  MFS_16Gauss      // 1.20 mG per LSB
};

enum Mopmode {
  MOM_lowpower = 0,   
  MOM_medperf,       
  MOM_hiperf,      
  MOM_ultrahiperf       
};

enum MSodr {        // magnetometer output data rate when slaved to the MAX21100
  MODR_div1 = 0,    // default, magnetometer ODR is 1/1 of the accel ODR
  MODR_div2,
  MODR_div4,
  MODR_div8,  
  MODR_div16,
  MODR_div32,
  MODR_div64, 
  MODR_div128 
};

enum Modr {         // magnetometer output data rate MAX21100 is bypassed
  MODR_0_625Hz = 0,     
  MODR_1_25Hz,
  MODR_2_5Hz,
  MODR_5Hz,  
  MODR_10Hz,
  MODR_20Hz,
  MODR_40Hz, 
  MODR_80Hz 
};

#define ADC_256  0x00 // define pressure and temperature conversion rates
#define ADC_512  0x02
#define ADC_1024 0x04
#define ADC_2048 0x06
#define ADC_4096 0x08
#define ADC_8192 0x0A
#define ADC_D1   0x40
#define ADC_D2   0x50

// Specify sensor full scale
uint8_t OSR = ADC_8192;       // set pressure amd temperature oversample rate
uint8_t Gscale = GFS_250DPS;  // Gyro full scale
uint8_t Godr = GODR_250Hz;    // Gyro sample rate
uint8_t Gbw = GBW_22Hz;       // Gyro bandwidth
uint8_t Ascale = AFS_2G;      // Accel full scale
uint8_t Aodr = AODR_250Hz;    // Accel sample rate
uint8_t Abw = ABW_div9;       // Accel bandwidth, accel sample rate divided by ABW_divx
uint8_t Mscale = MFS_4Gauss;  // Select magnetometer full-scale resolution
uint8_t Mopmode = MOM_hiperf; // Select magnetometer perfomance mode
uint8_t Modr = MODR_10Hz;     // Select magnetometer ODR when in MAX21100 bypass mode
uint8_t MSodr = MODR_div16;   // Select magnetometer ODR as Aodr/MODR_divx
uint8_t powerSelect = 0x00;   // no DSYNC enable or usage
uint8_t powerMode = accelnormalgyronormalmode;  // specify power mode for accel + gyro
uint8_t status;               // MAX21100 data status register
float aRes, gRes, mRes;       // scale resolutions per LSB for the sensors
  
// Pin definitions
int intPin = 8;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed = 13;

// Radio transceiver data
int16_t ThrustIn = 0;
int16_t RollIn = 0;
int16_t PitchIn = 0;
int16_t YawIn = 0;

int16_t ThrustOut = 0;
int16_t RollOut = 0;
int16_t PitchOut = 0;
int16_t YawOut = 0;

int16_t dataIn[] = {ThrustIn, RollIn, PitchIn, YawIn};
int16_t dataOut[] = {ThrustOut, RollOut, PitchOut, YawOut};
const uint8_t num_data = sizeof(dataIn);

// Pin definitions
int motorPin1 = 23;
int motorPin2 =  4;
int motorPin3 =  3;
int motorPin4 = 22;
int thrustPin = 23;

uint16_t Pcal[8];         // calibration constants from MS5637 PROM registers
unsigned char nCRC;       // calculated check sum to ensure PROM integrity
uint32_t D1 = 0, D2 = 0;  // raw MS5637 pressure and temperature data
double dT, OFFSET, SENS, T2, OFFSET2, SENS2;  // First order and second order corrections for raw S5637 temperature and pressure data
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempGCount, tempMCount;      // temperature raw count output of mag and gyro
float   Gtemperature, Mtemperature; // Stores the MAX21100 gyro and LIS3MDL mag internal chip temperatures in degrees Celsius
double Temperature, Pressure;       // stores MS5637 pressures sensor pressure and temperature
float SelfTest[12];                  // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method


void setup()
{
//  Wire.begin();
//  TWBR = 12;  // 400 kbit/sec I2C speed for Pro Mini
  // Setup for Master mode, pins 16/17, external pullups, 400kHz for Teensy 3.1
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(4000);
  Serial.begin(38400);

  
  // set up the role pin
  pinMode(role_pin, INPUT);
  digitalWrite(role_pin, LOW);
  delay(20); // Just to get a solid reading on the role pin

  // read the address pin, establish our role
  if ( digitalRead(role_pin) )
    role = role_sender;
  else
    role = role_receiver;
    
  delay(1000);
  Serial.begin(38400);
  delay(1000);
  Serial.print("ROLE: "); Serial.println(role_friendly_name[role]);
    
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  pinMode(motorPin1, OUTPUT); 
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(thrustPin, INPUT);
  
    
  //
  // Setup and configure rf radio
  //
  radio.begin();

  // We will be using the Ack Payload feature, so please enable it
//  radio.enableAckPayload();

  //
  // Open pipes to other nodes for communication
  //

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);

  // optionally, reduce the payload size.  seems to
  // improve reliability
  radio.setPayloadSize(8);

  //
  // Open pipes to other nodes for communication
  //

  // This simple sketch opens two pipes for these two nodes to communicate
  // back and forth.
  // Open 'our' pipe for writing
  // Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading

    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1,pipes[0]);

  // Start listening
    radio.startListening();

 // Dump the configuration of the rf unit for debugging
//    radio.printDetails(); Enable include printf.h for details

  display.begin(); // Initialize the display
  display.setContrast(58); // Set the contrast
  
// Start device display with ID of sensor
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0); display.print("MAX21100");
  display.setTextSize(1);
  display.setCursor(0, 20); display.print("6-DOF 16-bit");
  display.setCursor(0, 30); display.print("motion sensor");
  display.setCursor(20,40); display.print("60 ug LSB");
  display.display();
  delay(1000);

// Set up for data display
  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
  display.clearDisplay();   // clears the screen and buffer

/*
// scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 */ 
  
  // Read the WHO_AM_I register, this is a good test of communication
  Serial.println("MAX21100 6-axis motion sensor...");
  byte c = readByte(MAX21100_ADDRESS, MAX21100_WHO_AM_I);  // Read WHO_AM_I register for MAX21100
  Serial.print("MAX21100 Address = 0x"); Serial.println(MAX21100_ADDRESS, HEX);
  Serial.print("MAX21100 WHO_AM_I = 0x"); Serial.println(MAX21100_WHO_AM_I, HEX);
  Serial.print("MAX21100 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0xB2, HEX);
  byte r = readByte(MAX21100_ADDRESS, MAX21100_REVISION_ID);  // Read REVISION_ID register for MAX21100
  Serial.print("MAX21100 Revision ID = 0x"); Serial.println(r, HEX);
  display.setCursor(20,0); display.print("MAX21100");
  display.setCursor(0,10); display.print("I AM");
  display.setCursor(0,20); display.print(c, HEX);  
  display.setCursor(0,30); display.print("I Should Be");
  display.setCursor(0,40); display.print(0xB2, HEX); 
  display.display();
  delay(1000); 

  if (c == 0xB2) // MAX21100 WHO_AM_I should always be 0xB2
  {  
    Serial.println("MAX21100 is online...");
    
    selftestMAX21100(SelfTest); // Start by performing self test and reporting values
    Serial.print("x-axis self test +: gyration : "); Serial.print(SelfTest[0], 2); Serial.println(" should be +55");
    Serial.print("y-axis self test +: gyration : "); Serial.print(SelfTest[1], 2); Serial.println(" should be +55");
    Serial.print("z-axis self test +: gyration : "); Serial.print(SelfTest[2], 2); Serial.println(" should be +55");
    Serial.print("x-axis self test -: gyration : "); Serial.print(SelfTest[3], 2); Serial.println(" should be -55");
    Serial.print("y-axis self test -: gyration : "); Serial.print(SelfTest[4], 2); Serial.println(" should be -55");
    Serial.print("z-axis self test -: gyration : "); Serial.print(SelfTest[5], 2); Serial.println(" should be -55");
    Serial.print("x-axis self test +: acceleration : "); Serial.print(1000.*SelfTest[6]); Serial.println(" should be +300 mg");
    Serial.print("y-axis self test +: acceleration : "); Serial.print(1000.*SelfTest[7]); Serial.println(" should be +300 mg");
    Serial.print("z-axis self test +: acceleration : "); Serial.print(1000.*SelfTest[8]); Serial.println(" should be +300 mg");
    Serial.print("x-axis self test -: acceleration : "); Serial.print(1000.*SelfTest[9]); Serial.println(" should be -300 mg");
    Serial.print("y-axis self test -: acceleration : "); Serial.print(1000.*SelfTest[10]); Serial.println(" should be -300 mg");
    Serial.print("z-axis self test -: acceleration : "); Serial.print(1000.*SelfTest[11]); Serial.println(" should be -300 mg");
    delay(1000);
    
  calibrateMAX21100(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
  Serial.println(1000*accelBias[0]);  Serial.println(1000*accelBias[1]);  Serial.print(1000*accelBias[2]); Serial.println(" mg");
  Serial.println(gyroBias[0]);  Serial.println(gyroBias[1]);  Serial.print(gyroBias[2]); Serial.println(" dps");

  display.clearDisplay();
     
  display.setCursor(0, 0); display.print("MAX2110 bias");
  display.setCursor(0, 8); display.print(" x   y   z  ");

  display.setCursor(0,  16); display.print((int)(1000*accelBias[0])); 
  display.setCursor(24, 16); display.print((int)(1000*accelBias[1])); 
  display.setCursor(48, 16); display.print((int)(1000*accelBias[2])); 
  display.setCursor(72, 16); display.print("mg");
    
  display.setCursor(0,  24); display.print(gyroBias[0], 1); 
  display.setCursor(24, 24); display.print(gyroBias[1], 1); 
  display.setCursor(48, 24); display.print(gyroBias[2], 1); 
  display.setCursor(72, 24); display.print("dp");   
 
  display.display();
  delay(1000); 
 
  initbypassMAX21100(); // Treat MAX21100 and LIS3MDL and MS5637 as slaves to the Teensy 3.1 master
  Serial.println("MAX21100 initialized for bypass mode...."); // Initialize MAX21100 for direct read of magnetometer by microcontroller
 
   // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
  byte d = readByte(LIS3MDL_ADDRESS, LIS3MDL_WHO_AM_I);  // Read WHO_AM_I register for LIS3MDL
  Serial.print("LIS3MDL "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x3D, HEX);
  display.clearDisplay();
  display.setCursor(20,0); display.print("LIS3MDL");
  display.setCursor(0,10); display.print("I AM");
  display.setCursor(0,20); display.print(d, HEX);  
  display.setCursor(0,30); display.print("I Should Be");
  display.setCursor(0,40); display.print(0x3D, HEX);  
  display.display();
  delay(1000); 
  
  // Initialize and set up LIS3MDL magnetometer
  initLIS3MDL(); Serial.println("LIS3MDL initialized for active data mode...."); // Initialize device for active mode read of magnetometer
 
  // Reset the MS5637 pressure sensor
  MS5637Reset();
  delay(100);
  Serial.println("MS5637 pressure sensor reset...");
  // Read PROM data from MS5637 pressure sensor
  MS5637PromRead(Pcal);
  Serial.println("PROM data read:");
  Serial.print("C0 = "); Serial.println(Pcal[0]);
  unsigned char refCRC = Pcal[0] >> 12;
  Serial.print("C1 = "); Serial.println(Pcal[1]);
  Serial.print("C2 = "); Serial.println(Pcal[2]);
  Serial.print("C3 = "); Serial.println(Pcal[3]);
  Serial.print("C4 = "); Serial.println(Pcal[4]);
  Serial.print("C5 = "); Serial.println(Pcal[5]);
  Serial.print("C6 = "); Serial.println(Pcal[6]);
  
  nCRC = MS5637checkCRC(Pcal);  //calculate checksum to ensure integrity of MS5637 calibration data
  Serial.print("Checksum = "); Serial.print(nCRC); Serial.print(" , should be "); Serial.println(refCRC);  
  
  display.clearDisplay();
  display.setCursor(20,0); display.print("MS5637");
  display.setCursor(0,10); display.print("CRC is "); display.setCursor(50,10); display.print(nCRC);
  display.setCursor(0,20); display.print("Should be "); display.setCursor(50,30); display.print(refCRC);
  display.display();
  delay(1000);  

   if(!MAX21100Bypass) {
     initmasterMAX21100(); // Let the MAX21100 be master to the LIS3MDL slave
     Serial.println("MAX21100 initialized for master mode...."); // Initialize MAX21100 for read of magnetometer as master
   }
   
  // get sensor resolutions, only need to do this once
   getAres();
   getGres();
   getMres();
   magbias[0] = -237.;// User environmental x-axis correction in milliGauss, should be automatically calculated
   magbias[1] = +220.;// User environmental y-axis correction in milliGauss
   magbias[2] = -430.;// User environmental z-axis correction in milliGauss
 
   int16_t MBiasX = (int16_t) magbias[0]/mRes;  // convert estimated mag bias to 16-bit two's complement integer
   int16_t MBiasY = (int16_t) magbias[1]/mRes;
   int16_t MBiasZ = (int16_t) magbias[2]/mRes;
   
   writeByte(MAX21100_ADDRESS, MAX21100_MAG_OFS_X_MSB, ((MBiasX >> 8) & 0xFF));  // load bias into mag bias registers
   writeByte(MAX21100_ADDRESS, MAX21100_MAG_OFS_X_LSB, (MBiasX  & 0xFF));
   writeByte(MAX21100_ADDRESS, MAX21100_MAG_OFS_Y_MSB, ((MBiasY >> 8) & 0xFF));
   writeByte(MAX21100_ADDRESS, MAX21100_MAG_OFS_Y_LSB, (MBiasY  & 0xFF));
   writeByte(MAX21100_ADDRESS, MAX21100_MAG_OFS_Z_MSB, ((MBiasZ >> 8) & 0xFF));
   writeByte(MAX21100_ADDRESS, MAX21100_MAG_OFS_Z_LSB, (MBiasZ  & 0xFF));
   
  }
  else
  {
    Serial.print("Could not connect to MAX21100: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop()
{  
 // If intPin goes high, all data registers have new data
 // if (digitalRead(intPin)) {  // On interrupt, read data
   status = readByte(MAX21100_ADDRESS, MAX21100_SYSTEM_STATUS);
   if( (status & 0x04) && !(status & 0x08) ) {  // check if accel data ready and no accel data error
    readAccelData(accelCount);  // Read the x/y/z adc values
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - accelBias[1];   
    az = (float)accelCount[2]*aRes - accelBias[2]; 
   } 

   if( (status & 0x01) && !(status & 0x02) ) {  // check if gyro data ready  and no gyro data error
    readGyroData(gyroCount);  // Read the x/y/z adc values
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes; // - gyroBias[1];  
    gz = (float)gyroCount[2]*gRes; // - gyroBias[2]; 
   }  

    if( (!MAX21100Bypass && (status & 0x10) && !(status & 0x20)) || (MAX21100Bypass && (readByte(LIS3MDL_ADDRESS, LIS3MDL_STATUS_REG) & 0x08))) {  // if all three axes have new magnetometer data
    readMagData(magCount);  // Read the x/y/z adc values   
    // Calculate the magnetometer values in milliGauss
    mx = (float)magCount[0]*mRes; // - magbias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes; // - magbias[1];  
    mz = (float)magCount[2]*mRes; // - magbias[2];   
  }
  
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
  
  // Sensors x (y)-axis of the accelerometer opposite the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ up) is the same as the z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
  // For the MAX21100 + LIS3MDL, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  -mx,  -my,  -mz);
//  MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, -mx, -my, -mz);
//    MAX21100Quaternion();  // The MAX21100 does 9 DoF sensor fusion in hardware!
    
    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate

    if(SerialDebug) {
    Serial.print("ax = "); Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2); 
    Serial.print(" gy = "); Serial.print( gy, 2); 
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
    Serial.print("mx = "); Serial.print( (int)mx ); 
    Serial.print(" my = "); Serial.print( (int)my ); 
    Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");
    
    Serial.print("q0 = "); Serial.print(q[0]);
    Serial.print(" qx = "); Serial.print(q[1]); 
    Serial.print(" qy = "); Serial.print(q[2]); 
    Serial.print(" qz = "); Serial.println(q[3]); 
    } 
    
    tempGCount = readGyroTempData();  // Read the gyro adc values
    Gtemperature = ((float) tempGCount) / 256.; // Gyro chip temperature in degrees Centigrade
   // Print gyro die temperature in degrees Centigrade      
    Serial.print("Gyro temperature is ");  Serial.print(Gtemperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of a degree C
 
    if(!MAX21100Bypass) {
      // Temporarily enable MAX21100 bypass mode to read from pressure sensor
    uint8_t temp = readByte(MAX21100_ADDRESS, MAX21100_DR_CFG); // store existing register contents
    writeByte(MAX21100_ADDRESS, MAX21100_DR_CFG, temp | 0x80); // toggle on bypass mode
    delay(25);
    D1 = MS5637Read(ADC_D1, OSR);  // get raw pressure value
    D2 = MS5637Read(ADC_D2, OSR);  // get raw temperature value
    tempMCount = readMagTempData();  // Read the mag temperature adc values
    writeByte(MAX21100_ADDRESS, MAX21100_DR_CFG, temp & ~0x80); // toggle off bypass mode
    }
    
     Mtemperature = ((float) tempMCount) / 8. + 25.0; // Mag chip die temperature in degrees Centigrade
   // Print magnetometer die temperature in degrees Centigrade      
    Serial.print("Mag temperature is ");  Serial.print(Mtemperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of a degree C
   
    dT = D2 - Pcal[5]*pow(2,8);    // calculate temperature difference from reference
    OFFSET = Pcal[2]*pow(2, 17) + dT*Pcal[4]/pow(2,6);
    SENS = Pcal[1]*pow(2,16) + dT*Pcal[3]/pow(2,7);
 
    Temperature = (2000 + (dT*Pcal[6])/pow(2, 23))/100;           // First-order Temperature in degrees Centigrade
//
// Second order corrections
    if(Temperature > 20) 
    {
      T2 = 5*dT*dT/pow(2, 38); // correction for high temperatures
      OFFSET2 = 0;
      SENS2 = 0;
    }
    if(Temperature < 20)                   // correction for low temperature
    {
      T2      = 3*dT*dT/pow(2, 33); 
      OFFSET2 = 61*(Temperature - 2000)*(Temperature - 2000)/16;
      SENS2   = 29*(Temperature - 2000)*(Temperature - 2000)/16;
    } 
    if(Temperature < -15)                      // correction for very low temperature
    {
      OFFSET2 = OFFSET2 + 17*(Temperature + 1500)*(Temperature + 1500);
      SENS2 = SENS2 + 9*(Temperature + 1500)*(Temperature + 1500);
    }
 // End of second order corrections
 //
     Temperature = Temperature - T2;
     OFFSET = OFFSET - OFFSET2;
     SENS = SENS - SENS2;
 
     Pressure = (((D1*SENS)/pow(2, 21) - OFFSET)/pow(2, 15))/100;  // Pressure in mbar or kPa
  
    const int station_elevation_m = 1050.0*0.3048; // Accurate for the roof on my house; convert from feet to meters

    float baroin = Pressure; // pressure is now in millibars

    // Formula to correct absolute pressure in millbars to "altimeter pressure" in inches of mercury 
    // comparable to weather report pressure
    float part1 = baroin - 0.3; //Part 1 of formula
    const float part2 = 0.0000842288;
    float part3 = pow(part1, 0.190284);
    float part4 = (float)station_elevation_m / part3;
    float part5 = (1.0 + (part2 * part4));
    float part6 = pow(part5, 5.2553026);
    float altimeter_setting_pressure_mb = part1 * part6; // Output is now in adjusted millibars
    baroin = altimeter_setting_pressure_mb * 0.02953;

    float altitude = 145366.45*(1. - pow((Pressure/1013.25), 0.190284));
   
    if(SerialDebug) {
    Serial.print("Digital temperature value = "); Serial.print( (float)Temperature, 2); Serial.println(" C"); // temperature in degrees Celsius
    Serial.print("Digital temperature value = "); Serial.print(9.*(float) Temperature/5. + 32., 2); Serial.println(" F"); // temperature in degrees Fahrenheit
    Serial.print("Digital pressure value = "); Serial.print((float) Pressure, 2);  Serial.println(" mbar");// pressure in millibar
    Serial.print("Altitude = "); Serial.print(altitude, 2); Serial.println(" feet");
    }
    
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll  *= 180.0f / PI;
     
    if(SerialDebug) {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);
    
    Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
    }
   
    display.clearDisplay();    
 
    display.setCursor(0, 0); display.print(" x   y   z ");

    display.setCursor(0,  8); display.print((int)(1000*ax)); 
    display.setCursor(24, 8); display.print((int)(1000*ay)); 
    display.setCursor(48, 8); display.print((int)(1000*az)); 
    display.setCursor(72, 8); display.print("mg");
    
    display.setCursor(0,  16); display.print((int)(gx)); 
    display.setCursor(24, 16); display.print((int)(gy)); 
    display.setCursor(48, 16); display.print((int)(gz)); 
    display.setCursor(66, 16); display.print("o/s");    

    display.setCursor(0,  24); display.print((int)(mx)); 
    display.setCursor(24, 24); display.print((int)(my)); 
    display.setCursor(48, 24); display.print((int)(mz)); 
    display.setCursor(72, 24); display.print("mG");    
 
    display.setCursor(0,  32); display.print((int)(yaw)); 
    display.setCursor(24, 32); display.print((int)(pitch)); 
    display.setCursor(48, 32); display.print((int)(roll)); 
    display.setCursor(66, 32); display.print("ypr");  
  
    // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and 
    // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
    // The filter update rate is determined mostly by the mathematical steps in the respective algorithms, 
    // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
    // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
    // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively. 
    // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
    // This filter update rate should be fast enough to maintain accurate platform orientation for 
    // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
    // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
    // The 3.3 V 8 MHz Pro Mini is doing pretty well!
    
    display.setCursor(0, 40); display.print(altitude, 0); display.print("ft"); 
    display.setCursor(68, 0); display.print(9.*Temperature/5. + 32., 0); 
    display.setCursor(42, 40); display.print((float) sumCount / (1000.*sum), 2); display.print("kHz"); 
    display.display();



    // if there is data ready
    if ( radio.available() )
    {
      // Dump the payloads until we've gotten everything
      bool done = false;
      while (!done)
      {
        // Fetch the payload, and see if this was the last one.
        done = radio.read( dataIn, num_data );

        ThrustIn = dataIn[0];
        RollIn   = dataIn[1];  
        PitchIn  = dataIn[2];
        YawIn    = dataIn[3];

        // Spew it
      Serial.print("Got payload...");   
     Serial.print(" Thrust in = ");Serial.println(ThrustIn);
      Serial.print(" Roll in = ");Serial.println(RollIn);
      Serial.print(" Pitch in = ");Serial.println(PitchIn);
      Serial.print(" Yaw in = ");Serial.println(YawIn);

	// Delay just a little bit to let the other unit
	// make the transition to receiver
	delay(20);
      }
      constrain(ThrustIn, 0, 254); // Don't let the motor drive exceed the maximum
      analogWrite(motorPin1,  ThrustIn);
      analogWrite(motorPin2,  ThrustIn);
      analogWrite(motorPin3,  ThrustIn);
      analogWrite(motorPin4,  ThrustIn);     
   
     // First, stop listening so we can talk
      radio.stopListening();

      // Send the final one back.
      ThrustOut = analogRead(thrustPin);
      dataOut[0] = ThrustOut;
      dataOut[1] = roll;
      dataOut[2] = pitch;
      dataOut[3] = yaw;

      radio.write(dataOut, num_data );
      Serial.println("Sent response");

//      Serial.print(" Thrust out = ");Serial.println(ThrustOut);
//      Serial.print(" Roll out = ");Serial.println(RollOut);
//      Serial.print(" Pitch out = ");Serial.println(PitchOut);
//      Serial.print(" Yaw out = ");Serial.println(YawOut);
      
      // Now, resume listening so we catch the next packets.
      radio.startListening();
    
    }
    
    
    digitalWrite(myLed, !digitalRead(myLed));
    count = millis(); 
    sumCount = 0;
    sum = 0;    
    }

}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void getMres() {
  switch (Mscale)
  {
 	// Possible magnetometer scales (and their register bit settings)are:
    case MFS_4Gauss:
          mRes = 4.0f/32.768f; // Proper scale to return milliGauss
          break;
    case MFS_8Gauss:
          mRes = 8.0f/32.768f; // Proper scale to return milliGauss
          break;
    case MFS_12Gauss:
          mRes = 12.0f/32.768f; // Proper scale to return milliGauss
          break;
    case MFS_16Gauss:
          mRes = 16.0f/32.768f; // Proper scale to return milliGauss
          break;
  }
}

void getGres() {
  switch (Gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (11), 500 DPS (10), 1000 DPS (01), and 2000 DPS  (00). 
    case GFS_250DPS:
          gRes = 250.0f/32768.0f;
          break;
    case GFS_500DPS:
          gRes = 500.0f/32768.0f;
          break;
    case GFS_1000DPS:
          gRes = 1000.0f/32768.0f;
          break;
    case GFS_2000DPS:
          gRes = 2000.0f/32768.0f;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (11), 4 Gs (10), 8 Gs (01), and 16 Gs  (00). 
    case AFS_2G:
          aRes = 2.0f/32768.0f;
          break;
    case AFS_4G:
          aRes = 4.0f/32768.0f;
          break;
    case AFS_8G:
          aRes = 8.0f/32768.0f;
          break;
    case AFS_16G:
          aRes = 16.0f/32768.0f;
          break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MAX21100_ADDRESS, MAX21100_ACC_X_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;      // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MAX21100_ADDRESS, MAX21100_GYRO_X_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

int16_t readGyroTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MAX21100_ADDRESS, MAX21100_TEMP_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return ((int16_t)rawData[0] << 8) | rawData[1] ;               // Turn the MSB and LSB into a 16-bit value
}

void readMagData(int16_t * destination)
{
if (MAX21100Bypass) {
  uint8_t rawData[2];  // x/y/z mag register data, little Endian but not contiguous!
  readBytes(LIS3MDL_ADDRESS, LIS3MDL_OUT_X_L, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  readBytes(LIS3MDL_ADDRESS, LIS3MDL_OUT_Y_L, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    destination[1] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  readBytes(LIS3MDL_ADDRESS, LIS3MDL_OUT_Z_L, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    destination[2] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
}
else {
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MAX21100_ADDRESS, MAX21100_MAG_X_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

}

int16_t readMagTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(LIS3MDL_ADDRESS, LIS3MDL_TEMP_OUT_L, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a 16-bit value
}


void initLIS3MDL() {
// Configure magnetometer
// Choose device mode (bits 1:0 = 00 = continuous data read, 01 = single conversion, 10 & 11 = default power down)
writeByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG3, 0x00); // Enable continuous data read mode (bits 1:0 = 00)
// Enable temperature sensor (bit 7 = 1)
// Set magnetometer operative mode for x and y axes (bits 6:5)
// Set magnetometer ODR (bits 4:2)
writeByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG1, 0x80 | Mopmode << 5 | Modr << 2);
writeByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG2, Mscale << 5);  // Set magnetometer full scale range
writeByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG4, Mopmode << 2);  // Set magnetometer operative mode for z axis
writeByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG5, 0x40); // output registers not updated until both data bytes have been read
}


void selftestMAX21100(float * selfTest) {
   int16_t gyroSelfTestplus[3] = {0, 0, 0}, gyroSelfTestminus[3] = {0, 0, 0};
   int16_t accelSelfTestplus[3] = {0, 0, 0}, accelSelfTestminus[3] = {0, 0, 0};
   uint8_t rawData[6];
   
  // Enable power select to control power mode from DSYNC (enable = 0x80, disable = 0x00)
  // choose power mode (accelnormal_gyronormal = 0x0F in bits 6:3
  // Enable all axes (z = bit 2, y = bit 1, x = bit 0)
   writeByte(MAX21100_ADDRESS, MAX21100_POWER_CFG, 0x0F << 3 | 0x07);

  // Configure gyro
  // Select gyro bandwidth (bits 5:2) and gyro full scale (bits 1:0)
   float gyrosensitivity  = 131.0f;   // = 131 LSB/degrees/sec per data sheet
   writeByte(MAX21100_ADDRESS, MAX21100_GYRO_CFG1, 0x40 | GBW_14Hz << 2 | GFS_250DPS); // positive deflection
   delay(100);
   
   readBytes(MAX21100_ADDRESS, MAX21100_GYRO_X_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
   gyroSelfTestplus[0] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);   // Turn the MSB and LSB into a signed 16-bit value
   gyroSelfTestplus[1] = (int16_t) (((int16_t)rawData[2] << 8) | rawData[3]);  
   gyroSelfTestplus[2] = (int16_t) (((int16_t)rawData[4] << 8) | rawData[5]);  
   
   writeByte(MAX21100_ADDRESS, MAX21100_GYRO_CFG1, 0x80 | GBW_14Hz << 2 | GFS_250DPS); // negative deflection
   delay(100);
  
   readBytes(MAX21100_ADDRESS, MAX21100_GYRO_X_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
   gyroSelfTestminus[0] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
   gyroSelfTestminus[1] = (int16_t) (((int16_t)rawData[2] << 8) | rawData[3]);  
   gyroSelfTestminus[2] = (int16_t) (((int16_t)rawData[4] << 8) | rawData[5]); 
  
   selfTest[0] = (float) gyroSelfTestplus[0] / gyrosensitivity;
   selfTest[1] = (float) gyroSelfTestplus[1] / gyrosensitivity;
   selfTest[2] = (float) gyroSelfTestplus[2] / gyrosensitivity;
   selfTest[3] = (float) gyroSelfTestminus[0] / gyrosensitivity;
   selfTest[4] = (float) gyroSelfTestminus[1] / gyrosensitivity;
   selfTest[5] = (float) gyroSelfTestminus[2] / gyrosensitivity;

// disable gyro self test mode
   writeByte(MAX21100_ADDRESS, MAX21100_GYRO_CFG1, 0x00 | GBW_14Hz << 2 | GFS_250DPS);  
 
// Configure the accelerometer
// Select accel full scale (bits 7:6) and enable all three axes (bits 2:0)
   float accelsensitivity = 16384.0f;  // = 16384 LSB/g per data sheet

//  positive x-axis deflection
   writeByte(MAX21100_ADDRESS, MAX21100_PWR_ACC_CFG, AFS_2G << 6 | 0x08 | 0x07); // pos x axis
   delay(100);
   readBytes(MAX21100_ADDRESS, MAX21100_ACC_X_H, 2, &rawData[0]);    // Read the raw data registers into data array
   accelSelfTestplus[0] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value

//  negative x-axis deflection
   writeByte(MAX21100_ADDRESS, MAX21100_PWR_ACC_CFG, AFS_2G << 6 | 0x28 | 0x07); // neg x axis
   delay(100);
   readBytes(MAX21100_ADDRESS, MAX21100_ACC_X_H, 2, &rawData[0]);    // Read the raw data registers into data array
   accelSelfTestminus[0] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value

//  positive y-axis deflection
   writeByte(MAX21100_ADDRESS, MAX21100_PWR_ACC_CFG, AFS_2G << 6 | 0x10 | 0x07); // pos y axis
   delay(100);
   readBytes(MAX21100_ADDRESS, MAX21100_ACC_Y_H, 2, &rawData[0]);    // Read the raw data registers into data array
   accelSelfTestplus[1] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);   // Turn the MSB and LSB into a signed 16-bit value

//  negative y-axis deflection
   writeByte(MAX21100_ADDRESS, MAX21100_PWR_ACC_CFG, AFS_2G << 6 | 0x30 | 0x07); // neg y axis
   delay(100);
   readBytes(MAX21100_ADDRESS, MAX21100_ACC_Y_H, 2, &rawData[0]);    // Read the raw data registers into data array
   accelSelfTestminus[1] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value

//  positive z-axis deflection
   writeByte(MAX21100_ADDRESS, MAX21100_PWR_ACC_CFG, AFS_2G << 6 | 0x18 | 0x07); // pos z axis
   delay(100);
   readBytes(MAX21100_ADDRESS, MAX21100_ACC_Z_H, 2, &rawData[0]);    // Read the raw data registers into data array
   accelSelfTestplus[2] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value

//  negative z-axis deflection
   writeByte(MAX21100_ADDRESS, MAX21100_PWR_ACC_CFG, AFS_2G << 6 | 0x38 | 0x07); // neg z axis
   delay(100);
   readBytes(MAX21100_ADDRESS, MAX21100_ACC_Z_H, 2, &rawData[0]);    // Read the raw data registers into data array
   accelSelfTestminus[2] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value

   selfTest[6] = (float) accelSelfTestplus[0] / accelsensitivity;
   selfTest[7] = (float) accelSelfTestplus[1] / accelsensitivity;
   selfTest[8] = (float) accelSelfTestplus[2] / accelsensitivity;
   selfTest[9] = (float) accelSelfTestminus[0] / accelsensitivity;
   selfTest[10] = (float) accelSelfTestminus[1] / accelsensitivity;
   selfTest[11] = (float) accelSelfTestminus[2] / accelsensitivity;
  
   // disable accel self test
   writeByte(MAX21100_ADDRESS, MAX21100_PWR_ACC_CFG, AFS_2G << 6 | 0x00 | 0x07); // end accel self test
}


// Calibrate the MAX21100 gyro and accelerometer
void calibrateMAX21100(float * dest1, float * dest2) {

  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii = 0, fifo_count = 0, sample_count = 0;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
  // Choose normal power mode (accelnormalgyronormalmode = 0x0F in bits 6:3
  // Enable all axes (z = bit 2, y = bit 1, x = bit 0)
   writeByte(MAX21100_ADDRESS, MAX21100_POWER_CFG, accelnormalgyronormalmode << 3 | 0x07);
   delay(100);
   
// Configure gyro
  // Select 14 Hz gyro bandwidth (bits 5:2) and 250 dps gyro full scale (bits 1:0)
   writeByte(MAX21100_ADDRESS, MAX21100_GYRO_CFG1, GBW_14Hz << 2 | GFS_250DPS);
  // Select 125 Hz gyro ODR (bits 1:0)
   writeByte(MAX21100_ADDRESS, MAX21100_GYRO_CFG2, GODR_125Hz);
   delay(100);
   
// Configure the accelerometer
// Select accel full scale (bits 7:6) and enable all three axes (bits 2:0)
   writeByte(MAX21100_ADDRESS, MAX21100_PWR_ACC_CFG, AFS_2G << 6 | 0x07);
  // Select 14 Hz accel band width (bits 5:4) and 125 Hz accel ODR (bits 3:0)
   writeByte(MAX21100_ADDRESS, MAX21100_ACC_CFG1, ABW_div9 << 4 | AODR_125Hz);
   delay(100);
   
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec per data sheet
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g per data sheet
  
   // 128 bytes in the FIFO = 64 words; use 63 words to collect 63/3 axes = 21 3-axis gyro data samples
   // Use FIFO to collect and average 21 gyro data samples
   writeByte(MAX21100_ADDRESS, MAX21100_FIFO_TH,  0x3C); // Set word threshold to 63 = 21/125 Hz = 168 ms of data
   writeByte(MAX21100_ADDRESS, MAX21100_FIFO_CFG, 0x41); // Set FIFO normal mode and accumulate gyro data
   delay(200);  // Wait to collect gyro data in the FIFO
 
   if(readByte(MAX21100_ADDRESS, MAX21100_FIFO_STATUS) & 0x04) {  // Verify FIFO threshold reached
   
   fifo_count = readByte(MAX21100_ADDRESS, MAX21100_FIFO_COUNT);  // get number of samples in the FIFO
   sample_count = fifo_count/3; // should be 21 for 63 word fifo count
   for(ii = 0; ii < sample_count; ii++) {   
    int16_t gyro_temp[3] = {0, 0, 0};
    readBytes(MAX21100_ADDRESS, MAX21100_FIFO_DATA, 6, &data[0]);  // Read the six FIFO registers per sample
//    readBytes(MAX21100_ADDRESS, MAX21100_GYRO_X_H, 6, &data[0]);  // Read the six FIFO registers per sample
    gyro_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]) ; // Form signed 16-bit integer for each sample in FIFO
    gyro_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]) ;
    gyro_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]) ;
     
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
   }
   }
    gyro_bias[0]  /= (int32_t) sample_count;  // get average gyro bias
    gyro_bias[1]  /= (int32_t) sample_count;
    gyro_bias[2]  /= (int32_t) sample_count;
    
  // Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;
  
   // Now do the same for the accelerometer
   // 128 bytes in the FIFO = 64 words; use 63 words to collect 63/3 axes = 21 3-axis accel data samples
   // Use FIFO to collect and average 21 accel data samples
   writeByte(MAX21100_ADDRESS, MAX21100_FIFO_TH,  0x3C); // Set word threshold to 63 = 21/125 Hz = 168 ms of data
   writeByte(MAX21100_ADDRESS, MAX21100_FIFO_CFG, 0x42); // Set FIFO normal mode and accumulate accel data
   delay(200);  // Wait to collect accel data in the FIFO
 
   if(readByte(MAX21100_ADDRESS, MAX21100_FIFO_STATUS) & 0x04) {  // Verify FIFO threshold reached
   
   fifo_count = readByte(MAX21100_ADDRESS, MAX21100_FIFO_COUNT);  // get number of samples in the FIFO
   sample_count = fifo_count/3;
   for(ii = 0; ii < sample_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0};
    readBytes(MAX21100_ADDRESS, MAX21100_FIFO_DATA, 6, &data[0]);  // Read the six FIFO registers per sample
//    readBytes(MAX21100_ADDRESS, MAX21100_ACC_X_H, 6, &data[0]);  // Read the six FIFO registers per sample
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]) ; // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]) ;
     
    accel_bias[0]  += (int32_t) accel_temp[0];
    accel_bias[1]  += (int32_t) accel_temp[1];
    accel_bias[2]  += (int32_t) accel_temp[2];
   }
   }
    accel_bias[0]  /= (int32_t) sample_count;  // get average gyro bias
    accel_bias[1]  /= (int32_t) sample_count;
    accel_bias[2]  /= (int32_t) sample_count;
          
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
    
  // Output scaled gyro biases for display in the main program
  dest2[0] = (float) accel_bias[0]/(float) accelsensitivity;  
  dest2[1] = (float) accel_bias[1]/(float) accelsensitivity;
  dest2[2] = (float) accel_bias[2]/(float) accelsensitivity;
  
   writeByte(MAX21100_ADDRESS, MAX21100_FIFO_CFG, 0x00); // Turn off FIFO  
   
   writeByte(MAX21100_ADDRESS, MAX21100_BANK_SELECT, 0x02);  // select bank 2

   gyro_bias[0] *= -1;
   gyro_bias[1] *= -1;
   gyro_bias[2] *= -1;
   
   writeByte(MAX21100_ADDRESS, MAX21100_BIAS_GYRO_X_H, ((gyro_bias[0] >> 8) & 0x1F));  // load bias into gyro bias registers
   writeByte(MAX21100_ADDRESS, MAX21100_BIAS_GYRO_X_L, (gyro_bias[0]  & 0xFF));
   writeByte(MAX21100_ADDRESS, MAX21100_BIAS_GYRO_Y_H, ((gyro_bias[1] >> 8) & 0x1F));  // load bias into gyro bias registers
   writeByte(MAX21100_ADDRESS, MAX21100_BIAS_GYRO_Y_L, (gyro_bias[1]  & 0xFF));
   writeByte(MAX21100_ADDRESS, MAX21100_BIAS_GYRO_Z_H, ((gyro_bias[2] >> 8) & 0x1F));  // load bias into gyro bias registers
   writeByte(MAX21100_ADDRESS, MAX21100_BIAS_GYRO_Z_L, (gyro_bias[2]  & 0xFF));

   writeByte(MAX21100_ADDRESS, MAX21100_BANK_SELECT, 0x00);  // select bank 0
}


// Initialize the MAX21100 for bypass mode operations (read from magnetometer directly via microcontroller
void initbypassMAX21100() {
  // Enable power select to control power mode from DSYNC (enable = 0x80, disable = 0x00)
  // choose power mode (accelnormal_gyronormal = 0xFF in bits 6:3
  // Enable all axes (z = bit 2, y = bit 1, x = bit 0)
   writeByte(MAX21100_ADDRESS, MAX21100_POWER_CFG, powerSelect | powerMode << 3 | 0x07);

// Configure gyro
  // Select gyro bandwidth (bits 5:2) and gyro full scale (bits 1:0)
   writeByte(MAX21100_ADDRESS, MAX21100_GYRO_CFG1, Gbw << 2 | Gscale);
  // Select gyro ODR (bits 1:0)
   writeByte(MAX21100_ADDRESS, MAX21100_GYRO_CFG2, Godr);
   
// Configure the accelerometer
// Select accel full scale (bits 7:6) and enable all three axes (bits 2:0)
   writeByte(MAX21100_ADDRESS, MAX21100_PWR_ACC_CFG, Ascale << 6 | 0x07);
  // Select accel band width (bits 5:4) and accel ODR (bits 3:0)
   writeByte(MAX21100_ADDRESS, MAX21100_ACC_CFG1, Abw << 4 | Aodr);
 
  // Data ready configuration
  // Enable bypass mode to read magnetometer directly from microcontroller (bit 7 = 1)
  // Clear data ready bits when status register is read (bits 3:2 = 10)
  // Enable fine temperature mode, enable temperature sensor (bits 1:0 = 01)
  writeByte(MAX21100_ADDRESS, MAX21100_DR_CFG, 0x80 | 0x08 | 0x01);
 }


// Initialize the MAX21100 for master mode operations (read from magnetometer from MAX21100 master)
void initmasterMAX21100() {
  // Enable power select to control power mode from DSYNC (enable = 0x80, disable = 0x00)
  // choose power mode (accelnormal_gyronormal = 0xFF in bits 6:3
  // Enable all axes (z = bit 2, y = bit 1, x = bit 0)
   writeByte(MAX21100_ADDRESS, MAX21100_POWER_CFG, powerSelect | powerMode << 3 | 0x07);

// Configure gyro
  // Select gyro bandwidth (bits 5:2) and gyro full scale (bits 1:0)
   writeByte(MAX21100_ADDRESS, MAX21100_GYRO_CFG1, Gbw << 2 | Gscale);
  // Select gyro ODR (bits 1:0)
   writeByte(MAX21100_ADDRESS, MAX21100_GYRO_CFG2, Godr);
   
// Configure the accelerometer
// Select accel full scale (bits 7:6) and enable all three axes (bits 2:0)
   writeByte(MAX21100_ADDRESS, MAX21100_PWR_ACC_CFG, Ascale << 6 | 0x07);
  // Select accel band width (bits 5:4) and accel ODR (bits 3:0)
   writeByte(MAX21100_ADDRESS, MAX21100_ACC_CFG1, Abw << 4 | Aodr);

// Configure magnetometer in slave mode
   writeByte(MAX21100_ADDRESS, MAX21100_ACC_CFG2, MSodr << 1);           // slave magnetometer ODR
 // magnetometer slave enable (bit 7 = 1), byte order swap (LSB first, bit 6 = 1), byte length = 6
   writeByte(MAX21100_ADDRESS, MAX21100_MAG_SLV_CFG, 0x80 | 0x40 | 0x06);      
   writeByte(MAX21100_ADDRESS, MAX21100_MAG_SLV_ADD, LIS3MDL_ADDRESS);  // magnetometer slave address
   writeByte(MAX21100_ADDRESS, MAX21100_MAG_SLV_REG, LIS3MDL_OUT_X_L);  // magnetometer slave first data register
   writeByte(MAX21100_ADDRESS, MAX21100_MAG_MAP_REG, 0x00);             // magnetometer has inverted x,y axes wrt the MAX21100
      
  // Data ready configuration
  // Enable master mode to read magnetometer from MAX21100 (bit 7 = 0, default)
  // Clear data ready bits when status register is read (bits 3:2 = 10)
  // Enable fine temperature mode, enable temperature sensor (bits 1:0 = 01)
  writeByte(MAX21100_ADDRESS, MAX21100_DR_CFG, 0x08 | 0x01);
 }

 
  void MAX21100Quaternion() {

  uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};  
  int16_t quat[4] = {0, 0, 0, 0};
  float norm;
  
  writeByte(MAX21100_ADDRESS, MAX21100_BANK_SELECT, 0x02);  // select bank 2
  writeByte(MAX21100_ADDRESS, MAX21100_FUS_CFG0, 0x06 | 0x01);  // accel + mag + gyro, enable fusion engine
  writeByte(MAX21100_ADDRESS, MAX21100_FUS_CFG1, 0x02 << 5 | 0x03 << 3);  // adjust alpha trim

  readBytes(MAX21100_ADDRESS, MAX21100_QUAT0_H, 8, &data[0]);
  quat[0] = (int16_t) (((int16_t) data[0] << 8) | data[1]);
  quat[1] = (int16_t) (((int16_t) data[2] << 8) | data[3]);
  quat[2] = (int16_t) (((int16_t) data[4] << 8) | data[5]);
  quat[3] = (int16_t) (((int16_t) data[6] << 8) | data[7]);
  
  norm = sqrt(quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]);
  norm = 1.0f/norm;
  
  q[0] = (float) quat[0] * norm;
  q[1] = (float) quat[1] * norm;
  q[2] = (float) quat[2] * norm;
  q[3] = (float) quat[3] * norm;
  
  writeByte(MAX21100_ADDRESS, MAX21100_BANK_SELECT, 0x00);  // select bank 0
}


// I2C communication with the MS5637 is a little different from that with the MAX21100 and most other sensors
// For the MS5637, we write commands, and the MS5637 sends data in response, rather than directly reading
// MS5637 registers

        void MS5637Reset()
        {
        Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
	Wire.write(MS5637_RESET);                // Put reset command in Tx buffer
	Wire.endTransmission();                  // Send the Tx buffer
        }
        
        void MS5637PromRead(uint16_t * destination)
        {
        uint8_t data[2] = {0,0};
        for (uint8_t ii = 0; ii < 7; ii++) {
          Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
          Wire.write(0xA0 | ii << 1);              // Put PROM address in Tx buffer
          Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//          Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
	  uint8_t i = 0;
          Wire.requestFrom(MS5637_ADDRESS, 2);   // Read two bytes from slave PROM address 
	  while (Wire.available()) {
          data[i++] = Wire.read(); }               // Put read results in the Rx buffer
          destination[ii] = (uint16_t) (((uint16_t) data[0] << 8) | data[1]); // construct PROM data for return to main program
        }
        }

        uint32_t MS5637Read(uint8_t CMD, uint8_t OSR)  // temperature data read
        {
        uint8_t data[3] = {0,0,0};
        Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
        Wire.write(CMD | OSR);                  // Put pressure conversion command in Tx buffer
        Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//        Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
        
        switch (OSR)
        {
          case ADC_256: delay(1); break;  // delay for conversion to complete
          case ADC_512: delay(3); break;
          case ADC_1024: delay(4); break;
          case ADC_2048: delay(6); break;
          case ADC_4096: delay(10); break;
          case ADC_8192: delay(20); break;
        }
       
        Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
        Wire.write(0x00);                        // Put ADC read command in Tx buffer
//        Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
        Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
        Wire.requestFrom(MS5637_ADDRESS, 3);     // Read three bytes from slave PROM address 
	while (Wire.available()) {
        data[i++] = Wire.read(); }               // Put read results in the Rx buffer
        return (uint32_t) (((uint32_t) data[0] << 16) | (uint32_t) data[1] << 8 | data[2]); // construct PROM data for return to main program
        }



unsigned char MS5637checkCRC(uint16_t * n_prom)  // calculate checksum from PROM register contents
{
  int cnt;
  unsigned int n_rem = 0;
  unsigned char n_bit;
  
  n_prom[0] = ((n_prom[0]) & 0x0FFF);  // replace CRC byte by 0 for checksum calculation
  n_prom[7] = 0;
  for(cnt = 0; cnt < 16; cnt++)
  {
    if(cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
    else         n_rem ^= (unsigned short)  (n_prom[cnt>>1]>>8);
    for(n_bit = 8; n_bit > 0; n_bit--)
    {
        if(n_rem & 0x8000)    n_rem = (n_rem<<1) ^ 0x3000;
        else                  n_rem = (n_rem<<1);
    }
  }
  n_rem = ((n_rem>>12) & 0x000F);
  return (n_rem ^ 0x00);
}


// I2C read/write functions for the MAX21100 and LIS3MDL sensors

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

        uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.requestFrom(address, 1);  // Read one byte from slave register address 
	Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
//        Wire.requestFrom(address, count);  // Read bytes from slave register address 
        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

