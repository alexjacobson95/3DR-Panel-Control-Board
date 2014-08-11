#include <FastSerial.h>
#include "../mavlink/include/mavlink.h"        // Mavlink interface

#define LED_RED 3
#define LED_BLUE 9
#define LED_GREEN 10

FastSerialPort0(Serial);

int oldMode = -1;
int mode = -1;
int uavType = 2;

long previousMillisR = 0;
long previousMillisG = 0;
long previousMillisB = 0;

unsigned long currentMillis;

void setup() {
  Serial.begin(57600);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  
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
          // E.g. read GCS heartbeat and go into
          // comm lost mode if timer times out
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
          blinkLED(LED_RED, previousMillisR, 250);
          break;
        case 1: //circle
          break;
        case 2: //stabilize
          digitalWrite(LED_RED, HIGH);
          break;
        case 3: //?
        case 4: //?
          break;
        case 5: //FBWA
          break;
        case 6: //FBWB
          break;
        case 7: //?
        case 8: //?
        case 9: //?
          break;
        case 10: //auto
          blinkLED(LED_BLUE, previousMillisB, 250);
          break;
        case 11: //RTL
          blink2LEDs(LED_RED, LED_BLUE, previousMillisR, 250);
          break;
        case 12: //loiter
          break;
        case 13: //?
        case 14: //?
          break;
        case 15: //guided
        default:
          blinkLED(LED_RED, previousMillisR, 100);
          break;
      }
      break;
      
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
          blinkLED(LED_BLUE, previousMillisB, 250);
          break;
        case 2: //alt hold
          blinkLED(LED_BLUE, previousMillisB, 250);
          break;
        case 3: //auto
          digitalWrite(LED_RED, HIGH);
          break;
        case 4: //guided
          blinkLED(LED_RED, previousMillisR, 250);
          break;
        case 5: //loiter
          digitalWrite(LED_BLUE, HIGH);
          break;
        case 6: //RTL
          blinkAllLEDs(previousMillisR, 150);
          break;
        case 7: //circle
          blink2LEDs(LED_RED, LED_GREEN, previousMillisR, 250);
          break;
        case 8: //pos hold
          blink2LEDs(LED_BLUE, LED_GREEN, previousMillisG, 250);
          break;
        case 9: //land
          blink2LEDs(LED_RED, LED_BLUE, previousMillisR, 250);
          break;
        case 10: //of_loiter
          //none
          break;
        default:
          blinkLED(LED_RED, previousMillisR, 100);
          break;
      }
      break;
      
    default:
      for(int i = 0; i < uavType; i++){
        digitalWrite(LED_RED, HIGH);
        delay(500);
        digitalWrite(LED_RED, LOW);
        delay(500);
      }
      delay(10000);
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
  int ledLookup[3] = {LED_RED, LED_BLUE, LED_GREEN};
  for(int i = 0; i < 3; i++){
    digitalWrite(ledLookup[i], output);
  }
}

//blink LED with different on/off intervals
void blinkLED(int ledPin, long &preMil, long intervalOn, long intervalOff){
  currentMillis = millis();
  int ledState = digitalRead(ledPin);
  
  if(ledState == LOW){
    if(currentMillis - preMil > intervalOff){
      preMil = currentMillis;
      ledState == HIGH;
    }
  }else{
    if(currentMillis - preMil > intervalOn){
      preMil = currentMillis;
      ledState == LOW;
    }
  }
  
  digitalWrite(ledPin, ledState);
}

//blink led with the same on/off interval
void blinkLED(int ledPin, long &preMil, long interval){
  currentMillis = millis();
  
  if(currentMillis - preMil > interval){
    preMil = currentMillis;
    digitalWrite(ledPin, !digitalRead(ledPin));
  }
    
}

//blink 2 LEDs with the same on/off interval
void blink2LEDs(int ledPin1, int ledPin2, long &preMil, long interval){
  currentMillis = millis();
  
  if(currentMillis - preMil > interval*2){
    preMil = currentMillis;
  }else if(currentMillis - preMil > interval){
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, HIGH);
  }else if(currentMillis - preMil > 0){
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, LOW);
  }
}

void blinkAllLEDs(long &preMil, long interval){
  currentMillis = millis();
  
  if(currentMillis - preMil > interval*3){
    preMil = currentMillis;
  }else if(currentMillis - preMil > interval*2){
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_GREEN, HIGH);
  }else if(currentMillis - preMil > interval){
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_BLUE, HIGH);
    digitalWrite(LED_GREEN, LOW);
  }else if(currentMillis - preMil > 0){
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_GREEN, LOW);
  }
}


