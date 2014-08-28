#include <FastSerial.h>
#include "../mavlink/include/mavlink.h"        // Mavlink interface

#define LED_RED 8
#define LED_GREEN 10
#define LED_BLUE 3

FastSerialPort0(Serial);

int oldMode = -1;
int mode = -1;
int uavType = 2;

long previousMillis = 0;
unsigned long currentMillis;

void setup() {
  Serial.begin(57600);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  
  startUpSequence();
}

void loop() {
  mavlink_message_t msg;
  mavlink_status_t status;
  
  if(Serial.available() > 0 ) {
    uint8_t c = Serial.read();
    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
          uavType = mavlink_msg_heartbeat_get_type(&msg);
          mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
          if(mode != oldMode){
            oldMode = mode;
            LEDs(LOW);
          }
          break;
      }
    }
  }
  
  switch(uavType){
    case 1: //plane
      switch(mode){
        case 0: //manual
          blinkLED(LED_GREEN, 250);
          break;
        case 1: //circle
          blink2LEDs(LED_RED, LED_GREEN, 250);
          break;
        case 2: //stabilize
          digitalWrite(LED_GREEN, HIGH);
          break;
        case 3: //training
          break;
        case 5: //FBWA
          digitalWrite(LED_RED, HIGH);
          blinkLED(LED_GREEN, 250);
          break;
        case 6: //FBWB
          digitalWrite(LED_RED, HIGH);
          blinkLED(LED_BLUE, 250);
          break;
        case 10: //auto
          digitalWrite(LED_RED, HIGH);
          break;
        case 11: //RTL
          blinkAllLEDs(150);
          break;
        case 12: //loiter
          digitalWrite(LED_BLUE, HIGH);
          break;
        case 15: //guided
          blinkLED(LED_RED, 250);
          break;
        default:
          blinkAllLEDs(750);
          break;
      }
      break; //end plane
      
    case 2: //quad
    case 13: //hex
    case 14: //octo
    case 15: //tri
      switch(mode){
        case 0: //stabilize
          digitalWrite(LED_GREEN, HIGH);
          break;
        case 1: //acro
          digitalWrite(LED_GREEN, HIGH);
          blinkLED(LED_BLUE, 250);
          break;
        case 2: //alt hold
          blinkLED(LED_BLUE, 250);
          break;
        case 3: //auto
          digitalWrite(LED_RED, HIGH);
          break;
        case 4: //guided
          blinkLED(LED_RED, 250);
          break;
        case 5: //loiter
          digitalWrite(LED_BLUE, HIGH);
          break;
        case 6: //RTL
          blinkAllLEDs(150);
          break;
        case 7: //circle
          blink2LEDs(LED_RED, LED_GREEN, 250);
          break;
        case 8: //pos hold
          blink2LEDs(LED_BLUE, LED_GREEN, 250);
          break;
        case 9: //land
          blink2LEDs(LED_RED, LED_BLUE, 250);
          break;
        case 11: //drift
          break;
        case 13: //sport
          break;
        case 15: //autotune
          break;
        case 16: //hybrid
          break;
        default:
          blinkAllLEDs(750);
          break;
      }
      break; //end copter
      
    default:
      blinkAllLEDs(750);
      break;
  } 
}

void startUpSequence(){
  for(int i = 0; i < 3; i++){
    LEDs(HIGH);
    delay(200);
    LEDs(LOW);
    delay(200);
  }
  
  LEDs(HIGH);
  delay(1000);
  LEDs(LOW);
}

void LEDs(int output){
  int ledLookup[3] = {LED_RED, LED_GREEN, LED_BLUE};
  for(int i = 0; i < 3; i++){
    digitalWrite(ledLookup[i], output);
  }
}

//blink led with the same on/off interval
void blinkLED(int ledPin, long interval){
  currentMillis = millis();
  
  if(currentMillis - previousMillis > interval){
    previousMillis = currentMillis;
    digitalWrite(ledPin, !digitalRead(ledPin));
  }
    
}

//blink 2 LEDs with the same on/off interval
void blink2LEDs(int ledPin1, int ledPin2, long interval){
  currentMillis = millis();
  
  if(currentMillis - previousMillis > interval*2){
    previousMillis = currentMillis;
  }else if(currentMillis - previousMillis > interval){
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, HIGH);
  }else if(currentMillis - previousMillis > 0){
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, LOW);
  }
}

void blinkAllLEDs(long interval){
  currentMillis = millis();
  
  if(currentMillis - previousMillis > interval*3){
    previousMillis = currentMillis;
  }else if(currentMillis - previousMillis > interval*2){
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, HIGH);
  }else if(currentMillis - previousMillis > interval){
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, LOW);
  }else if(currentMillis - previousMillis > 0){
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
  }
}


