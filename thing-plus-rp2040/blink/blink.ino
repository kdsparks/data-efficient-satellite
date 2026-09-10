// Adapted from:
/*
  SparkFun Inventor’s Kit
  Circuit 1A-Blink

  Turns an LED connected to pin 13 on and off. Repeats forever.

  This sketch was written by SparkFun Electronics, with lots of help from the Arduino community.
  This code is completely free for any use.

  View circuit diagram and instructions at: https://learn.sparkfun.com/tutorials/sparkfun-inventors-kit-experiment-guide---v41
  Download code at: https://github.com/sparkfun/SIK-Guide-Code
*/

void setup() {

  pinMode(25, OUTPUT);      // Set pin 25 (for RP2040) to output

}


void loop() {

  digitalWrite(25, HIGH);   // Turn on the LED

  delay(1000);              // Wait for two seconds

  digitalWrite(25, LOW);    // Turn off the LED

  delay(1000);              // Wait for two seconds

}

