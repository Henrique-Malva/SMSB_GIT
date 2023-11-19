#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define ledG 4
#define ledR 5

// radio initialization
RF24 radio(7, 10); // CE, CSN
const byte address[6] = "00001"; // Address to send the data
short int receive_data[6]={}; // joyX,joyY,pot,s1,s2,btn
int ackTemp=0;

enum{
  blinking,
  stable
}signal_state;

bool led_st=1;
unsigned long int led_temp;

Servo rudder;

void setup() {
  rudder.attach(6);

  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.openReadingPipe(1, address);

  radio.enableAckPayload();
    
  radio.startListening();

  radio.writeAckPayload(1, &ackTemp, sizeof(ackTemp));

  Serial.begin(38400); // Default communication rate of the Bluetooth module

  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  signal_state=blinking;
  led_temp=millis();
}

void loop() {
  if ( radio.available() ) {
      radio.read(receive_data, sizeof(receive_data) );
      //ackTemp = read temperature
      radio.writeAckPayload(1, &ackTemp, sizeof(ackTemp));
  }
  //insert motor speed code
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
