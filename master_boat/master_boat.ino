#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define ledG 4
#define ledR 5

//motor variables
short int directionPin = 12;
short int pwmPin = 3;
short int brakePin = 9;
short int mtr_speed = 0;

// radio initialization
RF24 radio(7, 10); // CE, CSN
const byte address[6] = "00001"; // Address to send the data
short int receive_data[6]={}; // joyX,joyY,pot,s1,s2,btn
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

void setup() {
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

  Serial.begin(38400); // Default communication rate of the Bluetooth module

  //signalization setup
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  signal_state=blinking;
  led_temp=millis();

  //motor setup
  pinMode(directionPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(brakePin, OUTPUT);
  digitalWrite(brakePin, LOW);
}

void loop() {
  if ( radio.available() ) {
      radio.read(receive_data, sizeof(receive_data) );
      //ackTemp = read temperature
      radio.writeAckPayload(1, &ackTemp, sizeof(ackTemp));
  }

  mtr_speed = receive_data[1];
  
  if(mtr_speed<0){
    digitalWrite(directionPin, HIGH);
    mtr_speed*=-1; 
  }
  else{
    digitalWrite(directionPin, LOW);  
  }

  analogWrite(pwmPin, mtr_speed);
  
  rudder.write(receive_data[0]);
  
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
  
  //send to second boat
  String data = String(receive_data[0]) + "," + String(receive_data[1]) + "," + String(receive_data[2]) + "," + String(receive_data[3]) + "," + String(receive_data[4]) + "," + String(receive_data[5])+ "," + String(ackTemp);
  Serial.write(data.c_str());
  delay(50);
}
