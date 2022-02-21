#include <WiFi.h>
#include <ESP32Servo.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <Arduino.h>
#include <analogWrite.h>

int stateTracker = 0;

// Constants
const char *ssid = "Solar_Car";
const char *password = "11111111";
const char *msg_toggle_led = "toggleLED";
const char *msg_get_led = "getLEDState";
const int dns_port = 53;
const int http_port = 80;
const int ws_port = 1337;
const int led_pin = 2;
const int buz = 0;
//L293 Connection
const int LS = 22; // left sensor
const int RS = 23; // right sensor
const int motorA1      = 16;
const int motorA2      = 17;
const int motorAspeed  = 4;
const int motorB1      = 18;
const int motorB2      = 19;
const int motorBspeed  = 5;
int vSpeed = 190;
int turn_speed = 170; // 0 - 255  max
// 180 horizontal MAX
Servo horizontal; // horizontal servo
int servohp = 25;
int servoh = 0;   // 90;     // stand horizontal servo

int servohLimitHigh = 180;
int servohLimitLow = 65;

// 65 degrees MAX
Servo vertical;   // vertical servo
int servovp = 26;
int servov = 0;    //   90;     // stand vertical servo

int servovLimitHigh = 120;
int servovLimitLow = 15;


// LDR pin connections
//  name  = analogpin;
int ldrrt = 35; //LDR top rigt - BOTTOM RIGHT
int ldrlt = 34; //LDR top left - BOTTOM LEFT    <--- BDG
int ldrrd = 33; //ldr down rigt - TOP RIGHT
int ldrld = 32; //LDR down left - TOP LEFT

// Globals
AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(1337);
char msg_buf[10];
int led_state = 0;


//Functions
// Callback: receiving any WebSocket message
void onWebSocketEvent(uint8_t client_num,
                      WStype_t type,
                      uint8_t * payload,
                      size_t length) {

  // Figure out the type of WebSocket event
  switch (type) {

    // Client has disconnected
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", client_num);
      break;

    // New client has connected
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(client_num);
        Serial.printf("[%u] Connection from ", client_num);
        Serial.println(ip.toString());
      }
      break;

    // Handle text messages from client
    case WStype_TEXT:

      // Print out raw message
      Serial.printf("[%u] Received text: %s\n", client_num, payload);

      // Toggle LED
      if ( strcmp((char *)payload, "toggleLED") == 0 ) {
        led_state = led_state ? 0 : 1;
        Serial.printf("Toggling LED to %u\n", led_state);
        digitalWrite(led_pin, led_state);

        // Report the state of the LED
      } else if ( strcmp((char *)payload, "getLEDState") == 0 ) {
        sprintf(msg_buf, "%d", led_state);
        Serial.printf("");
        webSocket.sendTXT(client_num, msg_buf);
      }
      else if ( strcmp((char *)payload, "truck") == 0 ) {
        truck();
        Serial.println("truck");
      }
      else if ( strcmp((char *)payload, "stop") == 0 ) {
        stop();
        Serial.println("stop");
      }

      else if ( strcmp((char *)payload, "right") == 0 ) {
        right();
        analogWrite (motorAspeed, 250);
        analogWrite (motorBspeed, 250);
        delay(500);
        stop();
        Serial.println("right");
      }
      else if ( strcmp((char *)payload, "left") == 0 ) {
        left();
        analogWrite (motorAspeed, 250);
        analogWrite (motorBspeed, 250);
        delay(500);
        stop();
        Serial.println("left");
      }
      else if ( strcmp((char *)payload, "up") == 0 ) {
        forward();
        analogWrite (motorAspeed, 250);
        analogWrite (motorBspeed, 250);
        delay(500);
        stop();
        Serial.println("up");
      }
      else if ( strcmp((char *)payload, "down") == 0 ) {
        backward();
        analogWrite (motorAspeed, 250);
        analogWrite (motorBspeed, 250);
        delay(500);
        stop();
        Serial.println("down");
      }
      else if ( strcmp((char *)payload, "buzzer") == 0 ) {
        digitalWrite(buz, HIGH);
        delay(500);
        digitalWrite(buz, LOW);
        Serial.println("buzzer");
      }
      else if ( strcmp((char *)payload, "trucker") == 0 ) {
        //function trucker
      //  trucker_state = trucker_state ? 0 : 1;
      //  trucker(trucker_state);
        //trucker();
        stateTracker =!stateTracker;
        Serial.println("trucker");
        
      }
      else {
        Serial.println("[%u] Message not recognized");
      }
      break;

    // For everything else: do nothing
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      break;
  }
}

// Callback: send homepage
void onIndexRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                 "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/index.html", "text/html");
}

// Callback: send 404 if requested file does not exist
void onPageNotFound(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                 "] HTTP GET request of " + request->url());
  request->send(404, "text/plain", "Not found");
}


void setup() {
  pinMode(led_pin, OUTPUT);
  pinMode(buz, OUTPUT);
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorAspeed, OUTPUT);
  pinMode(motorBspeed, OUTPUT);
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
  digitalWrite(led_pin, LOW);
  pinMode(LS, INPUT);
  pinMode(RS, INPUT);
  digitalWrite(led_pin, LOW);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  horizontal.setPeriodHertz(50);    // standard 50 hz servo
  horizontal.attach(servohp, 500, 2400); // attaches the servo on pin 18 to the servo object
  vertical.setPeriodHertz(50);    // standard 50 hz servo
  vertical.attach(servovp, 500, 2400); // attaches the servo on pin 18 to the servo object
  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep
  // move servos
  horizontal.write(90);
  vertical.write(45);
  //Start Serial port
  Serial.begin(115200);
  // Make sure we can read the file system
  if ( !SPIFFS.begin()) {
    Serial.println("Error mounting SPIFFS");
    while (1);
  }

  // Start access point
  WiFi.softAP(ssid, password);

  // Print our IP address
  Serial.println();
  Serial.println("AP running");
  Serial.print("My IP address: ");
  Serial.println(WiFi.softAPIP());

  // On HTTP request for root, provide index.html file
  server.on("/", HTTP_GET, onIndexRequest);

  // On HTTP request for style sheet, provide style.css
  //server.on("/style.css", HTTP_GET, onCSSRequest);

  // Handle requests for pages that do not exist
  server.onNotFound(onPageNotFound);

  // Start web server
  server.begin();

  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

}

// Tracker
  int lt;
  int rt;
  int ld;
  int rd;

  int dtime;
  int tol;
  
  int avt;
  int avd;
  int avl;
  int avr;

  int dvert;
  int dhoriz;

void loop() {

  // Look for and handle WebSocket data
  webSocket.loop();
  Serial.println("in main loop");
  Serial.println(stateTracker);
  if (stateTracker == 1){
  lt = analogRead(ldrlt); // top left
  rt = analogRead(ldrrt); // top right
  ld = analogRead(ldrld); // down left
  rd = analogRead(ldrrd); // down right

  
  // int dtime = analogRead(4)/20; // read potentiometers  
  // int tol = analogRead(5)/4;
  dtime = 10;
  tol = 50;
  
  avt = (lt + rt) / 2; // average value top
  avd = (ld + rd) / 2; // average value down
  avl = (lt + ld) / 2; // average value left
  avr = (rt + rd) / 2; // average value right

  dvert = avt - avd; // check the diffirence of up and down
  dhoriz = avl - avr;// check the diffirence og left and rigt
  
  
  Serial.print(avt);
  Serial.print(" ");
  Serial.print(avd);
  Serial.print(" ");
  Serial.print(avl);
  Serial.print(" ");
  Serial.print(avr);
  Serial.print("   ");
  Serial.print(dtime);
  Serial.print("   ");
  Serial.print(tol);
  Serial.println(" ");
  
    
  if (-1*tol > dvert || dvert > tol) // check if the diffirence is in the tolerance else change vertical angle
  {
  if (avt > avd)
  {
    servov = ++servov;
     if (servov > servovLimitHigh) 
     { 
      servov = servovLimitHigh;
     }
  }
  else if (avt < avd)
  {
    servov= --servov;
    if (servov < servovLimitLow)
  {
    servov = servovLimitLow;
  }
  }
  vertical.write(servov);
  }
  
  if (-1*tol > dhoriz || dhoriz > tol) // check if the diffirence is in the tolerance else change horizontal angle
  {
  if (avl > avr)
  {
    servoh = --servoh;
    if (servoh < servohLimitLow)
    {
    servoh = servohLimitLow;
    }
  }
  else if (avl < avr)
  {
    servoh = ++servoh;
     if (servoh > servohLimitHigh)
     {
     servoh = servohLimitHigh;
     }
  }
  else if (avl = avr)
  {
    // nothing
  }
  horizontal.write(servoh);
  }
   delay(dtime);

  }
 
}
//Car Motion
void left() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}
void right() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}
void stop() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
  analogWrite (motorAspeed, 0);
  analogWrite (motorBspeed, 0);
}
void backward() { //down
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  
}
void forward() { //up
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
  
}
void trucker() {
  if(stateTracker == 0)
      stateTracker = 1;
  else if (stateTracker == 1)
      stateTracker = 0;
}
void truck(){
  /*digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);     // send waves for 10 us
  delayMicroseconds(10);
  duration = pulseIn(echoPin, HIGH); // receive reflected waves
  distance = duration / 58.2;   // convert to distance
  delay(10);*/
  if (digitalRead(LS) && digitalRead(RS)) // Finish line, stop both the motors
  {
    Serial.println("1 1");
    stop();
    //delay(100);
  }
  if (digitalRead(LS) && !(digitalRead(RS))) // turn right by rotating right motors in forward and left ones in backward direction
  {
    Serial.println("LS");
    Serial.println("1 0\n right");
    analogWrite (motorAspeed, 0);
    analogWrite (motorBspeed, 0);
    delay(400);

    right();
    analogWrite (motorAspeed, 0);
    analogWrite (motorBspeed, turn_speed);
    //delay(100);
  }
  if (!(digitalRead(LS)) && digitalRead(RS)) // Turn left by rotationg left motors in forward and right ones in backward direction
  {
    Serial.println(LS);
    Serial.println("0 1");
    analogWrite (motorAspeed, 0);
    analogWrite (motorBspeed, 0);
    delay(400);
    left();
    analogWrite (motorAspeed, vSpeed);
    analogWrite (motorBspeed, 0);
    //delay(100);
  }

  if (!(digitalRead(LS)) && !(digitalRead(RS))) // Move Forward on line
  {
    Serial.println("0 0\n Forward");
    analogWrite (motorAspeed, 0);
    analogWrite (motorBspeed, 0);
forward();

    analogWrite (motorAspeed, vSpeed);
    analogWrite (motorBspeed, vSpeed);
    //delay(100);

  }
  if ((digitalRead(LS)) && (digitalRead(RS))) // Move backward on line
  {
    Serial.println("1 1 \n ");
    analogWrite (motorAspeed, 0);
    analogWrite (motorBspeed, 0);
    backward();
    analogWrite (motorAspeed, vSpeed);
    analogWrite (motorBspeed, vSpeed);
    //delay(100);
  }
}
