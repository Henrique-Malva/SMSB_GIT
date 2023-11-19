#include <Servo.h>
#include <SPI.h>

#define trig 3
#define echo 2

#define ledG 4
#define ledR 5

enum{
  blinking,
  stable
}signal_state;

bool led_st=1;
unsigned long int led_temp;

short int state;
//state guide
  //0-follow_free,
  //1-follow_side,
  //2-follow_behind,
  //3-follow_circ

short int receive_data[6]={}; // joyX,joyY,pot,s1,s2,btn
short int temp;

Servo rudder, radar;

long measureDistance() {
  unsigned long int duration, distance;
  int sound_speed= 20.05 * sqrt(273.16 + temp);

    //Send pulse
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    //Wait for echo and measure time until it happens
    duration = pulseIn(echo,HIGH);

    //Compute distance (cm) // time (micro_s) * sound_speed(cm/s) / 2
    distance = (duration/2000000)*sound_speed;

    return distance;
}

void setup() {
  rudder.attach(6);
  radar.attach(7);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  
  Serial.begin(38400); // Default communication rate of the Bluetooth module

  state = 0; // Initialization of first formation state
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  signal_state=blinking;
  led_temp=millis();
}

void loop() {
  //read data sent from master via bluetooth
  if(Serial.available()>0){
    String values = Serial.readString();
    sscanf(values.c_str(), "%d,%d,%d,%d,%d,%d,%d",&receive_data[0],&receive_data[1],&receive_data[2],&receive_data[3],&receive_data[4],&receive_data[5],&temp);
  }
  if(receive_data[5]){ //only changes if button was pressed
    state = receive_data[3]*2 + receive_data[4]; }//s1(0or1)*2 + s2(0or1) - ns1 and ns2 = 0 (free) - ns1 and s2 = 1 (side) - s1 and ns2 = 2(behind) - s1 and s2 = 3 (circ)
  
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

  switch(state){
    case 0: //follow_free
      //insert motor speed code
      rudder.write(receive_data[0]);
    break;

    case 1: //follow_side

    
    break;

    case 2: // follow_behind

    
    break;

    case 3: // follow_circ

    
    break;

    default:
      state = 0;
      break;
    
    
  }

  delay(50);
}
