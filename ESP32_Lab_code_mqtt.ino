#include <Wire.h>
#include <ESP32Encoder.h>
#include <ArduinoJson.h>

// Encoder pins - read rotation
#define CLK 18  // CLK ENCODER
#define DT 19   // DT ENCODER

// Control the motor
#define ENA 15  
#define IN1 2
#define IN2 4

// IMU - I2C address
#define Addr 0x1C

ESP32Encoder encoder;

char incomingByte;
int movement = 0;
boolean data_stream = false; // indicates whether the system is currently processing a command
int sign = 1;

// Function to move motor
void movemotor(int p1, int p2, int duration) {
  digitalWrite(IN1, p1);
  digitalWrite(IN2, p2);
  delay(duration);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void setup() {
  Serial.begin(115200);

  // Configure output pins for the motor and enable the motor
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(ENA, HIGH);

  // Start I2C communication
  Wire.begin(); 

  // Set the encoder to an initial count of 0
  encoder.attachHalfQuad(DT, CLK);
  encoder.setCount(0);

  // Setup accelerometer registers
  // Set the accelerometer to active mode by writing to the control register
  Wire.beginTransmission(Addr);
  Wire.write(0x2A); // Control register
  Wire.write(0x01); // Active mode
  Wire.endTransmission();
  delay(300);
}

void loop() {

  // Handle serial input for motor control
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    if (!data_stream) {
      sign = (incomingByte == '-') ? -1 : 1;
      movement = (sign == 1) ? incomingByte - '0' : 0;
      data_stream = true;
    } else if (isDigit(incomingByte)) {
      movement = movement * 10 + (incomingByte - '0');
    }
  } else if (data_stream) {
    data_stream = false;
    movemotor(sign == -1 ? HIGH : LOW, sign == -1 ? LOW : HIGH, movement);
  }

  // Read accelerometer data
  unsigned int data[7];
  Wire.requestFrom(Addr, 7);
  if (Wire.available() == 7) {
    for (int i = 0; i < 7; i++) {
      data[i] = Wire.read();
    }
  }

  // data[1] is the MSB (Most Significant Byte) and data[2] is the LSB
  int xAccl = ((data[1] * 256) + data[2]) / 16;
  if (xAccl > 2047) xAccl -= 4096;

  int yAccl = ((data[3] * 256) + data[4]) / 16;
  if (yAccl > 2047) yAccl -= 4096;

  int zAccl = ((data[5] * 256) + data[6]) / 16;
  if (zAccl > 2047) zAccl -= 4096;

  long position = encoder.getCount();

  // Publish data via MQTT using JSON format
  StaticJsonDocument<256> jsonDoc;
  jsonDoc["xAccl"] = xAccl;
  jsonDoc["yAccl"] = yAccl;
  jsonDoc["zAccl"] = zAccl;
  jsonDoc["encoderPosition"] = position;

  char payload[256];
  serializeJson(jsonDoc, payload);
 
  // Output data to serial monitor
  Serial.println(payload);
  delay(50); // Adjust loop delay as needed
}