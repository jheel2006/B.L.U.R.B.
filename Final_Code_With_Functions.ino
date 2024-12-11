#include <Servo.h>
#include <LiquidCrystal.h>
#include <Adafruit_NeoPixel.h>

// Servo declarations
Servo Servo1, Servo2;
int Servo1Start = 160, Servo1End = 15;
int Servo2Start = 120, Servo2End = 60;

// Motor control pins
const int ain1Pin = 3;
const int ain2Pin = 4;
const int pwmAPin = 5;
const int motorSpeed = 255; // Max speed for the motor
const int motorRunTime = 2000; // Motor run time in milliseconds

// LCD setup
const int rs = 13, en = 12, d4 = 11, d5 = 10, d6 = 9, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Timing variables
const int delayTime = 5000;

// Variables for heartbeat effect
unsigned long previousMillis = 0;
const unsigned long heartbeatInterval = 500; // 500 ms interval for heart toggle
bool heartState = false; // false = emptyHeart, true = fullHeart
bool readingTime = true;
bool initialStage = true;

// Piezo sensor pin
const int piezoPin = A0;

// NeoPixel setup
const int neoPixelPin = 15; // Pin connected to NeoPixel data
const int numPixels = 16;  // Number of LEDs in the NeoPixel ring
Adafruit_NeoPixel pixels(numPixels, neoPixelPin, NEO_GRB + NEO_KHZ800);

// Base lavender color
const int baseR = 115;
const int baseG = 216;
const int baseB = 250;

// Brightness scaling factor (0.0 to 1.0)
const float brightnessFactor = 0.1; // Reduces brightness

// Threshold for piezo sensor
const int threshold = 400;

// NeoPixel state tracker
bool isOn = false;  // Tracks whether LEDs are ON or OFF

// Debounce variables
bool triggered = false;


// Custom characters for LCD
byte smiley[8] = {
  B00000,
  B10001,
  B00000,
  B00000,
  B10001,
  B01110,
  B00000,
};

byte emptyHeart[8] = {
  B00000,
  B00000,
  B01010,
  B10101,
  B10001,
  B01010,
  B00100,
  B00000
};

byte fullHeart[8] = {
  B00000,
  B00000,
  B01010,
  B11111,
  B11111,
  B01110,
  B00100,
  B00000,
};

void setup() {
  Serial.begin(9600);
  // Servo setup
  Servo1.attach(6);
  Servo2.attach(7);
  Servo1.write(Servo1Start);
  Servo2.write((Servo2Start+ Servo2End)/2);
  delay(1000);

  // Motor setup
  pinMode(ain1Pin, OUTPUT);
  pinMode(ain2Pin, OUTPUT);
  pinMode(pwmAPin, OUTPUT);

  // LCD setup
  lcd.begin(16, 2);
  lcd.createChar(0, smiley);
  lcd.createChar(1, emptyHeart);
  lcd.createChar(2, fullHeart);
  lcd.clear();
  // lcd.print("System Ready");
  lcd.print("    Hellooo!");
  delay(2000);

  // Initialize NeoPixel ring
  pixels.begin();
  pixels.show(); // Turn off all LEDs
}

void loop() {
  // moveWheelAndServo1();
  // moveServo2();
  // displayLCDReadingMessage();
  // Check for incoming serial data
  if (Serial.available() > 0) {
    char command = Serial.read(); // Read the command character

    switch (command) {
      case 'A': // Trigger wheel and Servo1
        moveWheelAndServo1();
        break;

      case 'B': // Trigger Servo2
        Serial.println("helloooo");
        moveServo2();
        break;

      case 'C': // Display LCD message
        displayLCDReadingMessage();
        break;

      case 'D': // Display LCD message
        displayLCDBreakMessage();
        break;

      case 'E': // Display LCD message
        displayLCDHelloMessage();
        break;

      default:
        Serial.println("Invalid Command");
        break;
    }
  }

  // Read the piezo sensor value
  int sensorValue = analogRead(piezoPin);

  // Check if the threshold is exceeded
  if (sensorValue > threshold && !triggered) {
    // Toggle the NeoPixel state
    isOn = !isOn;

    // Update NeoPixel state
    if (isOn) {
      turnOnNeoPixels();
    } else {
      turnOffNeoPixels();
    }

    // Mark as triggered to avoid repeated toggling
    triggered = true;
    delay(1000);
  } 
  else if (sensorValue <= threshold) {
    // Reset triggered state when sensor value drops below the threshold
    triggered = false;
  }

  // Keep the heart beating
  if (readingTime && !initialStage){
    updateHeartbeat();
  }

  

  
}

void turnOnNeoPixels() {
  // Scale lavender color to reduce brightness
  int scaledR = baseR * brightnessFactor;
  int scaledG = baseG * brightnessFactor;
  int scaledB = baseB * brightnessFactor;

  // Set all NeoPixels to the scaled color
  for (int i = 0; i < numPixels; i++) {
    pixels.setPixelColor(i, pixels.Color(scaledR, scaledG, scaledB));
  }
  pixels.show();
}

void turnOffNeoPixels() {
  // Turn off all NeoPixels
  for (int i = 0; i < numPixels; i++) {
    pixels.setPixelColor(i, 0);
  }
  pixels.show();
}


void updateHeartbeat() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= heartbeatInterval) {
    previousMillis = currentMillis; // Update the time stamp

    lcd.setCursor(2, 0); // Position of the heart on the LCD

    if (heartState) {
      lcd.setCursor(14, 0);
      lcd.write(byte(1)); // Display empty heart
    } else {
      lcd.setCursor(14, 0);
      lcd.write(byte(2)); // Display full heart
    }

    heartState = !heartState; // Toggle heart state
  }
}


// Function to control the wheel and Servo1
void moveWheelAndServo1() {
  initialStage = false;
  // Motor operation
  analogWrite(pwmAPin, motorSpeed);
  digitalWrite(ain1Pin, LOW);
  digitalWrite(ain2Pin, HIGH);
  // lcd.clear();
  // lcd.print("Motor Running");
  delay(4000);
  Servo1.write(Servo1Start - 20);
  delay(motorRunTime);

  analogWrite(pwmAPin, 0);
  digitalWrite(ain1Pin, LOW);
  digitalWrite(ain2Pin, LOW);
  // lcd.clear();
  // lcd.print("Motor Stopped");

  // Servo1 operation
  Servo1.write(Servo1End);
  delay(2000);
  Servo1.write(Servo1Start);
  delay(1000);
  // Start motor in opp direction
  analogWrite(pwmAPin, motorSpeed);
  digitalWrite(ain1Pin, HIGH);
  digitalWrite(ain2Pin, LOW);
  delay(3000);
  analogWrite(pwmAPin, 0);
  digitalWrite(ain1Pin, LOW);
  digitalWrite(ain2Pin, LOW);
  // lcd.clear();
  // lcd.print("Motor Stopped");
}

// Function to control Servo2
void moveServo2() {
  
  Servo2.write(Servo2End);
  delay(1000);
  Servo2.write(Servo2Start);
  delay(1000);
  Servo2.write((Servo2Start+ Servo2End)/2);
  delay(delayTime);
}

// Function to display a custom message on the LCD with custom characters
void displayLCDReadingMessage() {
  initialStage = false;
  readingTime = true;
  lcd.clear();
  
  
  // Display message with custom characters
  lcd.setCursor(0, 0);
  lcd.print("Reading Time! ");
  lcd.write(byte(1)); // Empty heart
  // lcd.write(byte(0)); // Smiley face
  delay(1000);

  // Update the second line with a full heart
  lcd.setCursor(14, 0);
  lcd.write(byte(2)); // Full heart
  delay(1000);
}

// Function to display a custom message on the LCD with custom characters
void displayLCDBreakMessage() {
  initialStage = false;
  readingTime = false;
  lcd.clear();
  
  
  // Display message with custom characters
  lcd.setCursor(1, 0);
  lcd.print("Break Time!  ");
  // lcd.write(byte(1)); // Empty heart
  lcd.write(byte(0)); // Smiley face
  delay(1000);
}

// Function to display a custom message on the LCD with custom characters
void displayLCDHelloMessage() {
  initialStage = true;
  readingTime = false;
  lcd.clear();
  
  // Display message with custom characters
  lcd.setCursor(0, 0);
  lcd.print("    Hellooo!");
  delay(1000);
}