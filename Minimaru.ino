#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"

#if (defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO) || defined(ESP8266))   // Using a soft serial port
#include <SoftwareSerial.h>
SoftwareSerial softSerial(4, 5);  // RX, TX
#define FPSerial softSerial
#else
#define FPSerial Serial1
#endif

DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

// Define the pin for the LED
const int ledPin = 3;
// Define the pin for audio input
const int audioPin = A0;
// Define the pin for the button
const int buttonPin = 2;

// Noise threshold to filter out small values
const int threshold = 550;  // Adjusted based on your readout values

void setup() {
#if (defined ESP32)
  FPSerial.begin(9600, SERIAL_8N1, D3, D2);
#else
  FPSerial.begin(9600);
#endif

  Serial.begin(115200);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(FPSerial, true, true)) {  // Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while (true) {
      delay(0);  // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));
  
  myDFPlayer.volume(25);  // Set volume value. From 0 to 30
  myDFPlayer.play(1);     // Play the first mp3

  // Initialize the LED pin as output and button pin as input
  pinMode(ledPin, OUTPUT);
  pinMode(audioPin, INPUT); // Set audio pin as input
  pinMode(buttonPin, INPUT_PULLUP); // Enable internal pull-up resistor for button
}

void loop() {
  // Read the button state
  static bool lastButtonState = HIGH; // Start with the button unpressed
  bool currentButtonState = digitalRead(buttonPin);
  
  // Check if the button is pressed (LOW)
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    myDFPlayer.next();  // Play the next mp3 on button press
    Serial.println(F("Next track requested."));
    delay(200); // Debounce delay
  }
  lastButtonState = currentButtonState;

  // Read audio signal from the speaker
  int audioValue = analogRead(audioPin);  // Read the audio signal
  
  // Only process the audio signal if it's above the threshold
  if (audioValue > threshold) {
    int brightness = map(audioValue, threshold, 1023, 0, 255);  // Map the value to PWM range (with threshold)
    analogWrite(ledPin, brightness);  // Set the LED brightness
  } else {
    analogWrite(ledPin, 0);  // Turn off the LED if below threshold
  }

  // Print out the debug values
  Serial.print(F("Audio Value: "));
  Serial.print(audioValue);  // Print the analog audio value
  Serial.print(F(" | LED Brightness: "));
  Serial.println(audioValue > threshold ? map(audioValue, threshold, 1023, 0, 255) : 0);

  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read());  // Handle different errors and states.
  }
}

void printDetail(uint8_t type, int value) {
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}
