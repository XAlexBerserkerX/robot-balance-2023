#include <Arduino.h>
#include "board_mapping.h"
#include "interfaces.h"
#include "FS.h"
#include <Wire.h>
#include <SPI.h>
#include "MXC6655.h"
#include "secrets.h"
#include <WiFi.h>




 int angle_index = 0;
 int angle_samples[10];
float angle_average = 0;

unsigned long previousMillisControlLoop1;
unsigned long previousMillisControlLoop2;

float angle_set_point = 5;
int angle = 0;


float absolute_position_set_point = 0;
int32_t absolute_position = 0;
float absolute_position_error = 0;

float angle_erreur = 0;
float angle_erreur_somme = 0;

int nReceived = 0;


uint8_t  compass_bearing_8b;
uint16_t compass_bearing_16b;
int angle_0_255 = 0;
int8_t   pitch_angle_90_90; 
int8_t   roll_angle_90_90; 

int16_t x_axis_magnetometre;
int16_t y_axis_magnetometre;
int16_t z_axis_magnetometre;

int16_t x_axis_accelerometre;
int16_t y_axis_accelerometre;
int16_t z_axis_accelerometre;

int16_t x_axis_gyro;
int16_t y_axis_gyro;
int16_t z_axis_gyro;
int16_t temp_cmps12;

int16_t horizontal_pitch_angle;
int16_t compass_bearing;





#define KP 75
#define KI 100
float dt = 0.02;


// Accéléromètre
MXC6655 accel;



// Fonction d'initialisation de l'UART
int initialisationUART(void)
{
  Serial.setPins(GPIO_UART_TX, GPIO_UART_RX);
  Serial.begin(115200);
  printf("Robot Balance 2023\r\n");
  return 0;
}



void setup()
{

  pinMode(GPIO_VPSI_CS1, OUTPUT);
  pinMode(GPIO_VPSI_SCK, OUTPUT);
  pinMode(GPIO_VPSI_MISO, INPUT);
  pinMode(GPIO_VPSI_MOSI, OUTPUT);
  digitalWrite(GPIO_VPSI_CS1, HIGH);

  // SPI
  pinMode(GPIO_I2C_SDA, OUTPUT);
  pinMode(GPIO_I2C_SCL, OUTPUT);

  // I2C
  Wire.begin(GPIO_I2C_SDA, GPIO_I2C_SCL);

  delay(500);

  Serial.begin(115200);
  Serial.println("Booting");


  // I2C
  Wire.begin(GPIO_I2C_SDA, GPIO_I2C_SCL);

  SPI.begin(GPIO_VPSI_SCK, GPIO_VPSI_MISO, GPIO_VPSI_MOSI, GPIO_VPSI_CS1);
}
 
  
void loop()
{

  ////////////// CAPTURE COMPASS AND ACCELERATION//////////////
  // mesure_cmps12();

  accel.begin();                       // Initialize the accelerometer
  float accX = accel.getAccel(0, 0);   // Get the X acceleration
  float accY = accel.getAccel(1, 0);   // Get the Y acceleration
  float accZ = accel.getAccel(2, 0);   // Get the Z acceleration
  float temp = accel.getTemp();        // Get the temperature


  Wire.beginTransmission(I2C_CMPS12_ADDRESS);   // Start communication with CMPS12
  Wire.write(0);
  Wire.endTransmission();
  // Request 31 bytes from CMPS12
  nReceived = Wire.requestFrom(I2C_CMPS12_ADDRESS, I2C_CMPS12_REGISTER_LENGTH);   // Request 31 bytes from CMPS12
  uint8_t data[I2C_CMPS12_REGISTER_LENGTH] = {0};                                // Store the received data in this array

  nReceived = Wire.readBytes(data, I2C_CMPS12_REGISTER_LENGTH);                  // Read the 31 bytes of data

  
  // If something has gone wrong
  if (nReceived != I2C_CMPS12_REGISTER_LENGTH)   
  {
    printf("Erreur de reception\r\n");
    //angle_0_255 = 1;
  }
  else
  {
    compass_bearing_8b =   data[1];
    compass_bearing_16b =  (data[2] << 8) |  data[3];
    angle_0_255 =          data[4];
    pitch_angle_90_90 =    data[5]; 
    roll_angle_90_90 =     data[6]; 

    x_axis_magnetometre =       (data[7] << 8) |  data[8];
    y_axis_magnetometre =       (data[9] << 8) |  data[10];
    z_axis_magnetometre =       (data[11] << 8) |  data[12];

    x_axis_accelerometre =      (data[13] << 8) |  data[14];
    y_axis_accelerometre =      (data[15] << 8) |  data[16];
    z_axis_accelerometre =      (data[17] << 8) |  data[18];

    x_axis_gyro =               (data[19] << 8) |  data[20];
    y_axis_gyro =               (data[21] << 8) |  data[22];
    z_axis_gyro =               (data[23] << 8) |  data[24];

    temp_cmps12 =               (data[25] << 8) |  data[26];
    horizontal_pitch_angle =    (data[27] << 8) |  data[28];
    compass_bearing =           (data[29] << 8) |  data[30];

  }
  



  // affiche toutes les valeurs du capteur MXC6655
  printf("Accelerations X:\t%5.2f,\tY:\t%5.2f,\tZ:\t%5.2f,\tTemps:\t%5.2f\r\n", accX, accY, accZ, temp);

    // affiche toutes les valeurs du capteur CMPS12
  printf("compass_bearing_8b:\t%d\tcompass_bearing_16b:\t%d\tangle_0_255:\t%d\tpitch_angle_90_90:\t%d\troll_angle_90_90:\t%d\tx_axis_magnetometre:\t%d\ty_axis_magnetometre:\t%d\tz_axis_magnetometre:\t%d\tx_axis_accelerometre:\t%d\ty_axis_accelerometre:\t%d\tz_axis_accelerometre:\t%d\tx_axis_gyro:\t%d\ty_axis_gyro:\t%d\tz_axis_gyro:\t%d\ttemp_cmps12:\t%d\thorizontal_pitch_angle:\t%d\tcompass_bearing:\t%d\r\n", compass_bearing_8b,compass_bearing_16b,angle_0_255,pitch_angle_90_90,roll_angle_90_90,x_axis_magnetometre,y_axis_magnetometre,z_axis_magnetometre,x_axis_accelerometre,y_axis_accelerometre,z_axis_accelerometre,x_axis_gyro,y_axis_gyro,z_axis_gyro,temp_cmps12,horizontal_pitch_angle,compass_bearing); 
 


  
}

// void mesure_cmps12(void)
// {
//   Wire.beginTransmission(I2C_CMPS12_ADDRESS);
//   Wire.write(0);
//   Wire.endTransmission();
//   // Request 31 bytes from CMPS12
//   nReceived = Wire.requestFrom(I2C_CMPS12_ADDRESS, I2C_CMPS12_REGISTER_LENGTH);
//   uint8_t data[I2C_CMPS12_REGISTER_LENGTH] = {0};

//   nReceived = Wire.readBytes(data, I2C_CMPS12_REGISTER_LENGTH);

  
//   // Something has gone wrong
//   if (nReceived != I2C_CMPS12_REGISTER_LENGTH)
//   {
//     printf("Erreur de reception\r\n");
//     angle_0_255 = 1;
//   }
//   else
//   {
//     compass_bearing_8b =   data[1];
//     compass_bearing_16b =  (data[2] << 8) |  data[3];
//     angle_0_255 =          data[4];
//     pitch_angle_90_90 =    data[5]; 
//     roll_angle_90_90 =     data[6]; 

//     x_axis_magnetometre =       (data[7] << 8) |  data[8];
//     y_axis_magnetometre =       (data[9] << 8) |  data[10];
//     z_axis_magnetometre =       (data[11] << 8) |  data[12];

//     x_axis_accelerometre =      (data[13] << 8) |  data[14];
//     y_axis_accelerometre =      (data[15] << 8) |  data[16];
//     z_axis_accelerometre =      (data[17] << 8) |  data[18];

//     x_axis_gyro =               (data[19] << 8) |  data[20];
//     y_axis_gyro =               (data[21] << 8) |  data[22];
//     z_axis_gyro =               (data[23] << 8) |  data[24];

//     temp_cmps12 =               (data[25] << 8) |  data[26];
//     horizontal_pitch_angle =    (data[27] << 8) |  data[28];
//     compass_bearing =           (data[29] << 8) |  data[30];

//   }

//   // if (angle_0_255 > 127)
//   // {
//   //   angle = angle_0_255 - 256;
//   // }
//   // else
//   // {
//   //   angle = angle_0_255;
//   // }

// }
