#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// radio initialization
RF24 radio(7, 10); // CE, CSN
const byte address[6] = "00001"; // Address to send the data
short int receive_data[6]={}; // joyX,joyY,pot,s1,s2,btn
int ackTemp=0;


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
}

void loop() {
  if ( radio.available() ) {
      radio.read(receive_data, sizeof(receive_data) );
      //ackTemp = read temperature
      radio.writeAckPayload(1, &ackTemp, sizeof(ackTemp));
  }
  //insert motor speed code
  rudder.write(receive_data[0]);
  //insert light control
  
  //send to second boat
  String data = String(receive_data[0]) + "," + String(receive_data[1]) + "," + String(receive_data[2]) + "," + String(receive_data[3]) + "," + String(receive_data[4]) + "," + String(receive_data[5])+ "," + String(ackTemp);
  Serial.write(data.c_str());
  delay(50);
}
