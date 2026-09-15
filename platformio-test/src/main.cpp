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
 ****************************************************
 Read the temperature pixels from the MLX90640 IR array
  By: Nathan Seidle
  SparkFun Electronics
  Date: May 22nd, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example initializes the MLX90640 and outputs the 768 temperature values
  from the 768 pixels.
*/
#include <Arduino.h>

#include <Wire.h>
#include "HUSKYLENS.h"
#include "SparkFun_BMI270_Arduino_Library.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  // http://librarymanager/All#SparkFun_u-blox_GNSS
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

#define TA_SHIFT 8  // Default shift for MLX90640 in open air

HUSKYLENS huskylens;
SFE_UBLOX_GNSS myGNSS;
BMI270 imu;
paramsMLX90640 mlx90640;

uint8_t i2cAddressImu = BMI2_I2C_PRIM_ADDR;  // 0x68
const byte MLX90640_address = 0x33;          // Default 7-bit unshifted address of the MLX90640
const int DATA_SIZE_IR = 32 * 24;            // Variable for number of pixels
static float mlx90640To[768];
const int MIN_HOT_PIXELS = 25;
const int MIN_DEG_C = 35;
bool isSmokeDetected = false;
const int SMOKE_ID = 1;

bool detectSmoke(HUSKYLENSResult result);
void printPositionGNSS();
bool isConnectedMLX();
int detectHeat();


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
  while (!myGNSS.begin(Wire1)) {
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
    delay(3000);
  }
  Serial.println("BMI270 connected!");

  // Set up IR Array
  Serial.println("Setting up IR Array...");
  while (!isConnectedMLX()) {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring.");
    delay(3000);
  }
  Serial.println("MLX90640 online!");

  // Get device parameters for IR Array - We only have to do this once
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
  // Get measurements from the IMU sensor. This must be called before accessing
  // the sensor data, otherwise it will never update
  imu.getSensorData();

  // Get result from HuskyLens
  if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
  else if (!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
  else if (!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
  else {
    HUSKYLENSResult result = huskylens.read();
    isSmokeDetected = detectSmoke(result);
  }

  // Get result from IR Array
  int numHotPixels = detectHeat();

  if (isSmokeDetected && numHotPixels > MIN_HOT_PIXELS) {
    Serial.println("Smoke and heat detected");
    Serial.print("Number of hot pixels: ");
    Serial.println(numHotPixels);
    printPositionGNSS();
  }

  delay(1000);
}

bool detectSmoke(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK) {
    if (result.ID == SMOKE_ID) {
      return true;
    }
  }
  return false;
}

// Query module only every second. Doing it more often will just cause I2C traffic
// Check the delay where this function is called to do so
void printPositionGNSS() {
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

// Returns true if the MLX90640 is detected on the I2C bus
bool isConnectedMLX() {
  Wire1.beginTransmission((uint8_t)MLX90640_address);
  if (Wire1.endTransmission() != 0)
    return (false);  // Sensor did not ACK
  return (true);
}

int detectHeat() {
  // Get Temps from IR Array
  for (byte x = 0; x < 2; x++)  // Read both subpages
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0) {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT;  // Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
  }

  int num_hot_pixels = 0;

  for (int x = 0; x < DATA_SIZE_IR; x++) {
    float pixel_temp = mlx90640To[x];  // Reads temps of all 768 pixels
    if (pixel_temp > MIN_DEG_C)        // If the temp of a pixel is greater than the threshold...
      num_hot_pixels++;                // ...add it to the count of hot pixels
  }

  return num_hot_pixels;
}
