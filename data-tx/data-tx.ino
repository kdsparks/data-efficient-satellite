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
 SparkFun BMI270 Arduino Library: Example01_BasicReadingsI2C
*/

#include <Wire.h>
#include "HUSKYLENS.h"
#include "SparkFun_BMI270_Arduino_Library.h"

HUSKYLENS huskylens;
BMI270 imu;

uint8_t i2cAddressImu = BMI2_I2C_PRIM_ADDR;  // 0x68

void printResult(HUSKYLENSResult result);

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
      printResult(result);
    }
  }

  // Get measurements from the IMU sensor. This must be called before accessing
  // the sensor data, otherwise it will never update
  imu.getSensorData();
}

void printResult(HUSKYLENSResult result) {
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