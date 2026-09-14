/*
  Code adapted from:
  ***************************************************
 HUSKYLENS An Easy-to-use AI Machine Vision Sensor
  <https://www.dfrobot.com/product-1922.html>
 
  This example shows the basic function of library for HUSKYLENS via I2c.
 
  Created 2020-03-13
  By [Angelo qiao](Angelo.qiao@dfrobot.com)
 
  GNU Lesser General Public License.
  See <http://www.gnu.org/licenses/> for details.
  All above must be included in any redistribution
 ****************************************************
 Reading lat and long via UBX binary commands - no more NMEA parsing!
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 3rd, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to query a u-blox module for its lat/long/altitude. We also
  turn off the NMEA output on the I2C port. This decreases the amount of I2C traffic 
  dramatically.
 ****************************************************
 SparkFun BMI270 Arduino Library: Example01_BasicReadingsI2C
*/

#include <Wire.h>
#include "HUSKYLENS.h"
#include "SparkFun_BMI270_Arduino_Library.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  // http://librarymanager/All#SparkFun_u-blox_GNSS
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"


const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640

#define TA_SHIFT 8 //Default shift for MLX90640 in open air

const int DATA_SIZE = 32 * 24; //variable for number of pixels

static float mlx90640To[768]; 

HUSKYLENS huskylens;
SFE_UBLOX_GNSS myGNSS;
BMI270 imu;
paramsMLX90640 mlx90640;

long lastTime = 0;                           // Simple local timer. Limits amount of I2C traffic to u-blox module
uint8_t i2cAddressImu = BMI2_I2C_PRIM_ADDR;  // 0x68

void printResultHusky(HUSKYLENSResult result);
void printPositionGNSS();

void setup() {
  Serial.begin(115200);
  Wire1.begin();

  // Set up HuskyLens
  Serial.println("Setting up HuskyLens...");
  while (!huskylens.begin(Wire1)) {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(3000);
  }

  // Set up GNSS
  Serial.println("Setting up GNSS...");
  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  while (!myGNSS.begin(Wire1))
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring."));
    delay(3000);
  }
  myGNSS.setI2COutput(COM_TYPE_UBX);                  // Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);  //Save (only) the communications port settings to flash and BBR

  // Set up IMU
  Serial.println("Setting up IMU...");
  // Check if sensor is connected and initialize
  // Address is optional (defaults to 0x68)
  while (imu.beginI2C(i2cAddressImu, Wire1) != BMI2_OK)  // added second parameter to prevent defaulting to Wire
  {
    // Not connected, inform user
    Serial.println("Error: BMI270 not connected, check wiring and I2C address!");
    // Wait a bit to see if connection is established
    delay(3000);
  }
  Serial.println("BMI270 connected!");

  //Set up IR Array
  if (isConnected() == false)
  {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    while (1);
  }
  Serial.println("MLX90640 online!");

  //Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)
    Serial.println("Failed to load system parameters");

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
    Serial.println("Parameter extraction failed");


}

void loop() {
  // Get result from HuskyLens
  if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
  else if (!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
  else if (!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
  else {
    Serial.println(F("###########"));
    while (huskylens.available()) {
      HUSKYLENSResult result = huskylens.read();
      printResultHusky(result);
    }
  }

  // Get position from GNSS
  printPositionGNSS();

  // Get measurements from the IMU sensor. This must be called before accessing
  // the sensor data, otherwise it will never update
  imu.getSensorData();

  //Get Temps from IR Array
  for (byte x = 0 ; x < 2 ; x++) //Read both subpages
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
  }

  int num_hot_pixels = 0;

  for (int x = 0 ; x < DATA_SIZE ; x++)
  {
    float pixel_temp = mlx90640To[x]; //Reads temps of all 768 pixels
    if (pixel_temp > 35) //If the temp of a pixel is greater than 35C...
      num_hot_pixels++; //...add it to the count of hot pixels
  }
  if (num_hot_pixels > 25) //If the count of hot pixels exceeds 25...
    Serial.println("hot!!"); //declare it hot
    
// communicate with the husky to confirm/deny a fire
  Serial.println(num_hot_pixels); 
  delay(500);
}

void printResultHusky(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK) {
    if (result.ID == 1) {
      Serial.println("Wildfire");
    } else if (result.ID == 2) {
      Serial.println("Normal");
    } else {
      Serial.println("Default");
    }
  } else {
    Serial.println("Object unknown!");
  }
}

void printPositionGNSS() {
  // Query module only every second. Doing it more often will just cause I2C traffic
  // The module only responds when a new position is available
  if (millis() - lastTime > 1000) {
    lastTime = millis();  // Update the timer

    long latitude = myGNSS.getLatitude();                   // latitude gives raw GPS reading (degrees * 10^-7)
    double actualLatitude = (double)latitude / 10000000.0;  // divide by 10,000,000 to get lat readable by google maps
    Serial.print(F("Lat: "));
    Serial.print(actualLatitude, 6);  // forces display of 6 decimals (or more)

    long longitude = myGNSS.getLongitude();                   // longitude gives raw GPS reading (degrees * 10^-7)
    double actualLongitude = (double)longitude / 10000000.0;  // divide by 10,000,000 to get long readable by google maps
    Serial.print(F(" Long: "));
    Serial.print(actualLongitude, 6);  // forces display of 6 decimals (or more)

    long altitude = myGNSS.getAltitudeMSL();  // changed from getAltitude() to getAltitudeMSL()
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    byte SIV = myGNSS.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(SIV);

    Serial.println();
  }
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
  Wire1.beginTransmission((uint8_t)MLX90640_address);
  if (Wire1.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}