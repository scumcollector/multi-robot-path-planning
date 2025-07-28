#include <WiFiNINA.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "TRSensors.h"

// WiFi credentials
char ssid[] = "SemenSnorter";
char pass[] = "bruhbruh";

// Server setup
WiFiServer server(80);

// robot globals
String received_path = "";
String directions = "";
int robot_id = 0;
bool calibrated = false;
bool execute_flag = false;
int dir_index = 0;

// NeoPixel
#define PIN 7
Adafruit_NeoPixel RGB = Adafruit_NeoPixel(4, PIN, NEO_GRB + NEO_KHZ800);

// Line following
#define NUM_SENSORS 5
TRSensors trs;
unsigned int sensorValues[NUM_SENSORS];
unsigned int last_proportional = 0;
long integral = 0;
unsigned int position;

// Motor Pins
#define PWMA   6
#define AIN2   A0
#define AIN1   A1
#define PWMB   5
#define BIN1   A2
#define BIN2   A3

// ------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
  delay(1000);

  // Start RGB
  RGB.begin();
  RGB.show();

  // Init motor pins
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  // Connect to WiFi
  Serial.print("Connecting to SSID: "); Serial.println(ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected to WiFi!");
  Serial.print("Arduino IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
}

// ------------------------------------------------------------------------------------------------

void loop() {
  WiFiClient client = server.available();
  
  if (client) {
    Serial.println("Client connected");
    String input = client.readStringUntil('\n');
    input.trim();
    Serial.print("Received: "); Serial.println(input);

    // Parse commands
    if (input.startsWith("ID:")) {
      int colon = input.indexOf(':');
      robot_id = input.substring(3, colon).toInt();
      directions = input.substring(colon + 1);
      dir_index = 0;
      client.println("DIRECTION_OK");
    } 
    else if (input == "CALIBRATE") {
      linecalibration();
      RGB.setPixelColor(0, 0 ); 
      RGB.setPixelColor(1, 0 );
      RGB.setPixelColor(2, 0 );
      RGB.setPixelColor(3, 0 );
      RGB.show();  
      calibrated = true;
      client.println("CALIBRATION_DONE");
    } 
    else if (input == "EXECUTE") {
      if (calibrated) {
        execute_flag = true;
        client.println("EXECUTING");
      } else {
        client.println("NOT_CALIBRATED");
      }
    } 
    else {
      client.println("INVALID_COMMAND");
    }

    client.stop();
  }

  // Execution phase
  if (execute_flag && dir_index < directions.length()) {
    char dir = directions.charAt(dir_index);
    runDirection(dir);
    delay(2000);
    //followline();
    dir_index++;
  }
  
  if (execute_flag && dir_index >= directions.length()) {
    Serial.println("Execution completed. Resetting flags...");
    execute_flag = false;
    calibrated = false;
    directions = "";
    robot_id = 0;
    dir_index = 0;
    RGB.setPixelColor(0, 0 ); 
    RGB.setPixelColor(1, 0 );
    RGB.setPixelColor(2, 0 );
    RGB.setPixelColor(3, 0 );
    RGB.show(); 
  }
}

// ------------------------------------------------------------------------------------------------

void runDirection(char dir) {
  switch (dir) {
    case 'U':
      RGB.setPixelColor(0, 0x00FF00 ); //green
      RGB.setPixelColor(1, 0x00FF00 );
      RGB.setPixelColor(2, 0x00FF00 );
      RGB.setPixelColor(3, 0x00FF00 );
      RGB.show();
      break;
    case 'D':
      RGB.setPixelColor(0, 0xff0000 ); //red
      RGB.setPixelColor(1, 0xff0000 );
      RGB.setPixelColor(2, 0xff0000 );
      RGB.setPixelColor(3, 0xff0000 );
      RGB.show();
      break;
    case 'L':
      //left(); 
      delay(370); 
      RGB.setPixelColor(0, 0x0000ff ); //blue
      RGB.setPixelColor(1, 0x0000ff );
      RGB.setPixelColor(2, 0x0000ff );
      RGB.setPixelColor(3, 0x0000ff );
      RGB.show();
      break;
    case 'R':
      //right(); 
      delay(340); 
      RGB.setPixelColor(0, 0xff00ff ); //pink
      RGB.setPixelColor(1, 0xff00ff );
      RGB.setPixelColor(2, 0xff00ff );
      RGB.setPixelColor(3, 0xff00ff );
      RGB.show(); 
      break;
  }
}

// ------------------------------------------------------------------------------------------------

void followline() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);

  while (true) {
    position = trs.readLine(sensorValues);
    
    int proportional = (int)position - 2000;
    int derivative = proportional - last_proportional;
    integral += proportional;
    last_proportional = proportional;

    int power_difference = proportional / 7 + derivative * 5;
    
    const int maximum = 60;

    if (power_difference > maximum)
      power_difference = maximum;
    if (power_difference < - maximum)
      power_difference = - maximum;
      Serial.println(power_difference);
      
    if (power_difference < 0)
    {
       analogWrite(PWMA,maximum + power_difference);
       analogWrite(PWMB,maximum);
     }
     else
     {
        analogWrite(PWMA,maximum);
        analogWrite(PWMB,maximum - power_difference);
     } 

    if (sensorValues[0] > 850 && sensorValues[1] > 850 && sensorValues[2] > 850 &&
        sensorValues[3] > 850 && sensorValues[4] > 850) {
      forward(); delay(130); stop(); delay(1000); break;
    }
  }
}

// ------------------------------------------------------------------------------------------------

void forward() {
  analogWrite(PWMA, 60); analogWrite(PWMB, 60);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
}

void right() {
  analogWrite(PWMA, 50); analogWrite(PWMB, 50);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
}

void left() {
  analogWrite(PWMA, 50); analogWrite(PWMB, 50);
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
}

void stop() {
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
  RGB.setPixelColor(0, 0 ); 
  RGB.setPixelColor(1, 0 );
  RGB.setPixelColor(2, 0 );
  RGB.setPixelColor(3, 0 );
  RGB.show();  
}

// ------------------------------------------------------------------------------------------------

void linecalibration() {
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,HIGH);
  digitalWrite(BIN1,HIGH); 
  digitalWrite(BIN2,LOW);
  
   
  RGB.begin();
  RGB.setPixelColor(0,0x00FF00 ); //green
  RGB.setPixelColor(1,0x00FF00 );
  RGB.setPixelColor(2,0x00FF00 );
  RGB.setPixelColor(3,0x00FF00 );
  RGB.show();
  delay(500);
  
  analogWrite(PWMA, 80); analogWrite(PWMB, 80);

  for (int i = 0; i < 100; i++) {
    if (i < 25 || i >= 75) {
      digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
    } else {
      digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
      digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    }
    trs.calibrate();
  }

  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);

  //maybe replace this with a beep
  RGB.setPixelColor(0,0x0000FF ); //blue
  RGB.setPixelColor(1,0x0000FF );
  RGB.setPixelColor(2,0x0000FF );
  RGB.setPixelColor(3,0x0000FF );
  RGB.show();

  position = trs.readLine(sensorValues);

  last_proportional = 0;
  integral = 0;

  delay(500);
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,HIGH);
  digitalWrite(BIN1,LOW); 
  digitalWrite(BIN2,HIGH); 
}
