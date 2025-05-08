#include "DHT.h"
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define ledG 4
#define ledR 5

#define DHTPIN 2
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

//motor variables
short int pwmPin = 3;
short int dirPin1 = 8;
short int dirPin2 = 9;
short int mtr_speed = 0;

// radio initialization
RF24 radio(7, 10); // CE, CSN
const byte address[6] = "00001"; // Address to send the data
short int receive_data[6]={90,0,0,0,0,0}; // joyX,joyY,pot,s1,s2,btn
short int ackTemp=0;

//light state machine states
enum{
  blinking,
  stable
}signal_state;

//led related variables
bool led_st=1;
unsigned long int led_temp;

//servo initialization
Servo rudder;

short int state_transition=0;

int prev_send=0;

void setup() {
  dht.begin();
  //servo setup
  rudder.attach(6);
  rudder.write(90);

  //radio setup
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.openReadingPipe(1, address);
  radio.enableAckPayload(); //in every instance of sending data receives a response
  radio.startListening();
  radio.writeAckPayload(1, &ackTemp, sizeof(ackTemp));

  Serial.begin(9600); // Default communication rate of the Bluetooth module

  //signalization setup
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  signal_state=blinking;
  led_temp=millis();

  //motor setup
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  digitalWrite(dirPin1, HIGH);
  digitalWrite(dirPin2, LOW);
  delay(2000);
}

int was_sent=0, to_be_sent=1;

void loop() {
  //send to second boat
  if(millis()-prev_send>700){
    String data = "<"+String(receive_data[0]) + "," + String(receive_data[1]) + "," + String(receive_data[2]) + "," + String(receive_data[3]) + "," + String(receive_data[4]) + "," + String(receive_data[5])+ "," + String(ackTemp)+"<";
    Serial.print(data);
    prev_send=millis();
    if(receive_data[5]==1){
      to_be_sent=0;
    }
  }
  
  if ( radio.available() ) {
      radio.read(receive_data, sizeof(receive_data) );
      ackTemp = dht.readTemperature();
      radio.writeAckPayload(1, &ackTemp, sizeof(ackTemp));
      if(receive_data[5]==1){
        to_be_sent=1;
      }
      if(receive_data[5]==0 && to_be_sent==1){
        receive_data[5]=1;
      }
  }

  mtr_speed = receive_data[1];
  
  if(mtr_speed<0){
    digitalWrite(dirPin2, LOW);
    digitalWrite(dirPin1, HIGH);
    mtr_speed*=-0.3; 
  }
  else{
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);  
  }

  if(Serial.available()>0){
    state_transition = Serial.read();
  }

  
  if(state_transition==0){
    rudder.write(receive_data[0]);  
  }else{
    rudder.write(90);
  }
  
  
  //light control
  switch(signal_state){
    case blinking:
      if(millis()-led_temp > 300){
        digitalWrite(ledG,led_st);
        digitalWrite(ledR,!led_st);
        led_st=!led_st;
        led_temp=millis();
      }
      if(receive_data[5]){
        signal_state=stable;
        digitalWrite(ledG,1);
        digitalWrite(ledR,1);
      }
    break;
    case stable:
      if(receive_data[5]){signal_state=blinking; led_temp=millis();}
    break;  
  }
  analogWrite(pwmPin, mtr_speed);
  
  delay(250);
}
