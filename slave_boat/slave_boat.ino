#include <Servo.h>
#include <SPI.h>

#define trig 2
#define echo 4

#define ledG 8
#define ledR 7


//motor variables
short int directionPin = 12;
short int pwmPin = 3;
short int brakePin = 9;
short int mtr_speed = 0;

enum{
  blinking,
  stable
}signal_state;

bool led_st=1;
unsigned long int led_time;

short int state, actual_state, last_state=1;
//state guide
  //0-follow_free,
  //1-follow_side,
  //2-follow_behind,
  //3-follow_circ

short int receive_data[6]={90,0,0,0,0,0}; // joyX,joyY,pot,s1,s2,btn
short int temp=0;

Servo rudder, sonar;
unsigned short int sonar_angle=90;
unsigned short int rudder_angle=90;
unsigned long int distance=40;
unsigned long int son_time=0;

long measureDistance() {
  unsigned long int duration;
  unsigned long int temp_dist;
  float sound_speed= 20.05 * sqrt(273.16 + temp);
  if(millis()-son_time > 60){
    //Send pulse
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    son_time = millis();

    //Wait for echo and measure time until it happens
    duration = pulseIn(echo,HIGH);

    //Compute distance (cm) // time (micro_s) * sound_speed(cm/s) / 2
    temp_dist = duration/58;
    //distance = (duration/2000000)*sound_speed;
  }else{temp_dist=distance;}
  return temp_dist;
}

void setup() {
  //servo setup
  rudder.attach(6);
  rudder.write(90);
  sonar.attach(5);
  sonar.write(sonar_angle);

  //ultra sound sensor setup
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  
  Serial.begin(9600); // Default communication rate of the Bluetooth module

  actual_state =0;
  state = 0; // Initialization of first formation state

  //signalization setup
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  signal_state=blinking;
  led_time=millis();

  son_time=millis();

  //motor setup
  pinMode(directionPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(brakePin, OUTPUT);
  digitalWrite(brakePin, LOW);
  delay(2000);
}

int first_check=0,first_doin=0,sec_check=0,sec_doin=0,tre_doin=0,tre_check=0,quad_doin=0;
int i=0;
char values[40];

bool st=1;
int first_change=1;

void loop() {
  //read data sent from master via bluetooth
  if (Serial.available() > 0) {
    delay(80);
    int len = Serial.available();
    Serial.readBytes(values, len);
    if(values[0]=='<' && values[len-1]=='<'){
      sscanf(values, "<%d,%d,%d,%d,%d,%d,%d<",&receive_data[0],&receive_data[1],&receive_data[2],&receive_data[3],&receive_data[4],&receive_data[5],&temp);
      //memset(values, '\0', 40 * sizeof(values[0]));
    }
  }
  if(receive_data[5] && receive_data[1]>=0 && actual_state!=(receive_data[3]*2 + receive_data[4]) ){ //only changes if button was pressed and if boats are going forward
    last_state = actual_state;
    actual_state = state = receive_data[3]*2 + receive_data[4];
  }//s1(0or1)*2 + s2(0or1) - ns1 and ns2 = 0 (free) - ns1 and s2 = 1 (side) - s1 and ns2 = 2(behind) - s1 and s2 = 3 (circ)

  Serial.print(state);
  
  //light control
  switch(signal_state){
    case blinking:
      if(millis()-led_time > 300){
        digitalWrite(ledG,led_st);
        digitalWrite(ledR,!led_st);
        led_st=!led_st;
        led_time=millis();
      }
      if(receive_data[5]){
        signal_state=stable;
        digitalWrite(ledG,1);
        digitalWrite(ledR,1);
      }
    break;
    case stable:
      if(receive_data[5]){signal_state=blinking; led_time=millis();}
    break;  
  }
  
  mtr_speed = receive_data[1];
  
  switch(state){ //the cases are used to make the transition between formation states. After the transition and respective movimentation are done the slave boat can follow along in follow_free, meaning it'll stay in the same relative position
    case 0: //follow_free
      if((distance=measureDistance()) < receive_data[2]*.9 && mtr_speed>=0){
        mtr_speed *= 0.7;
      }
      if((distance=measureDistance()) > receive_data[2]*1.1 && mtr_speed>=0){
        mtr_speed = constrain(mtr_speed*1.3,0,70);
      }

      if(mtr_speed<0){
        digitalWrite(directionPin, HIGH);
        mtr_speed*=-0.2; 
      }
      else{
        digitalWrite(directionPin, LOW);  
      }
      
      analogWrite(pwmPin, mtr_speed);
      rudder_angle = receive_data[0];
      rudder.write(receive_data[0]);
    break;

    case 1: //follow_side
      digitalWrite(directionPin, LOW);
      if(last_state==2 || first_change){
          first_change=0;
          sonar_angle=45;
          sonar.write(45);
          rudder_angle=30;
          rudder.write(30);
          analogWrite(pwmPin, mtr_speed);
          while((distance=measureDistance()) > 5){
            if(Serial.available()>0 ){
              Serial.readBytes(values, 20);
            }
            delay(50);
            if(rudder_angle < 90){  
              rudder_angle++;
              rudder.write(rudder_angle);
            }
          }
          delay(250);
          sonar_angle=0;
          sonar.write(0);
          rudder_angle=145;
          rudder.write(145);
          mtr_speed = constrain(mtr_speed*1.5,0,70);
          analogWrite(pwmPin, mtr_speed);
          while((distance=measureDistance()) > 5){
            delay(50);
            if(Serial.available()>0 ){
              Serial.readBytes(values, 20);
            }
            state=0;
            if(rudder_angle > 90){  
              rudder_angle--;
              rudder.write(rudder_angle);
            }
          }
          Serial.println(distance);
          delay(100);
      }else{
        first_change=0;
        rudder_angle=45;
        rudder.write(45);
        analogWrite(pwmPin, 70);
        while(cos(PI-(sonar_angle*PI/180))*(distance=measureDistance())<20){ //turns slightly to the left while the horizontal distance to the master boat is inferior to a certain threshold
          if(Serial.available()>0 ){
              Serial.readBytes(values, 20);
          }
          if(i==5){sonar_angle--; sonar.write(sonar_angle); i=0;} //sonar_angle increase (every iteration would be too fast)
          i++;
          delay(50);
          if(rudder_angle > 90){  
            rudder_angle++;
            rudder.write(rudder_angle);
          }
        }
        delay(250);
        i=0;
        sonar_angle=0;
        sonar.write(0);
        rudder_angle=135;
        rudder.write(135);
        mtr_speed = constrain(mtr_speed*1.5,0,70);
        analogWrite(pwmPin, mtr_speed);
        while((distance=measureDistance()) > 5){
          if(Serial.available()>0 ){
              Serial.readBytes(values, 20);
            }
          delay(50);
          if(rudder_angle > 90){  
            rudder_angle--;
            rudder.write(rudder_angle);
          }
        }
      }
      state=0;
      
      /*if(last_state==2 || first_change){ //transition between follow_behind and follow_side (known starting location)
        first_change=0;
        //goes to the left faster than the master boat to create horizontal distance and keep up with it
        //sonar makes a 45º with the horizontal axis, horizontal distance to be 7,07cm  when measured distance <10cm and the boats are in a 45º angle with one another
        //assuming that the slave boat is aproaching vertically the master boat
        if(first_doin==0){
          sonar_angle=45;
          sonar.write(45);
          rudder_angle=30;
          rudder.write(30);
          first_doin=1;
        }
        if((distance=measureDistance()) > receive_data[2] && first_check!=1){
          mtr_speed = constrain(mtr_speed*2,0,70);
          if(rudder_angle < 90){  
            rudder_angle++;
            rudder.write(rudder_angle);
          }
          analogWrite(pwmPin, mtr_speed);
        }else{
          //after that, starts going straight faster than the master boat until the measured distance is inferior to 10cm (sonar at 180º, to the side, to detect when side by side)
          if(sec_doin==0){
            first_check=1;
            sonar_angle=0;
            sonar.write(0);
            rudder_angle=135;
            rudder.write(135);
            sec_doin=1;
          }
          if((distance=measureDistance()) > receive_data[2]){
            mtr_speed = constrain(mtr_speed*1.5,0,70);
            analogWrite(pwmPin, mtr_speed);
            if(rudder_angle > 90){  
              rudder_angle--;
              rudder.write(rudder_angle);
            }
          }else{
            state=first_check=first_doin=sec_doin=0;
          }
        }
      } else{ // any other to follow_side (unknown starting location)   //possible combination between both transition methods?
        first_change=0;
        if(first_doin==0){
          rudder_angle=45;
          rudder.write(45);
          analogWrite(pwmPin, 70);
          first_doin=1;
        }
        if(first_check!=1 && cos(PI-(sonar_angle*PI/180))*(distance=measureDistance())<receive_data[2]*.6){ //turns slightly to the left while the horizontal distance to the master boat is inferior to a certain threshold
          if(i==15){sonar_angle--; sonar.write(sonar_angle); i=0;} //sonar_angle increase (every iteration would be too fast)
          i++;
          if(rudder_angle < 90){  
            rudder_angle++;
            rudder.write(rudder_angle);
          }
        }else{
        //after that, starts going straight faster than the master boat until the measured distance is inferior to 10cm (sonar at 180º, to the side, to detect when side by side)
          if(sec_doin==0){
            first_check=1;
            sonar_angle=0;
            sonar.write(0);
            rudder_angle=135;
            rudder.write(135);
            sec_doin=1;
          }
          if((distance=measureDistance()) > receive_data[2]){
            mtr_speed = constrain(mtr_speed*1.5,0,70);
            if(rudder_angle > 90){  
              rudder_angle--;
              rudder.write(rudder_angle);
            }
            analogWrite(pwmPin, mtr_speed);
          }else{
            state=first_check=first_doin=sec_doin=0;
            i=0;
          }
        }
      }*/
      
    break;

    case 2: // follow_behind
      digitalWrite(directionPin, LOW);
      if(last_state==1){
        sonar_angle=45;
        sonar.write(45);
        rudder_angle=90;
        rudder.write(90);
        analogWrite(pwmPin, mtr_speed*0.7);
        while((distance=measureDistance()) > 5){if(Serial.available()>0 ){
              Serial.readBytes(values, 20);
            }};
        delay(250);
        sonar_angle=115;
        sonar.write(115);
        rudder_angle=150;
        rudder.write(150);
        mtr_speed = constrain(mtr_speed*2,0,70);
        analogWrite(pwmPin, mtr_speed);
        while((distance=measureDistance()) > 5){
          if(Serial.available()>0 ){
              Serial.readBytes(values, 20);
          }
          delay(50);
          if(rudder_angle > 90){  
            rudder_angle--;
            rudder.write(rudder_angle);
          }
        }
        delay(250);
        sonar_angle=90;
        sonar.write(90);
        rudder_angle=30;
        rudder.write(30);
        mtr_speed = constrain(mtr_speed*1.5,0,70);
        analogWrite(pwmPin, mtr_speed);
        while((distance=measureDistance()) > 5){
          if(Serial.available()>0 ){
              Serial.readBytes(values, 20);
            }
          delay(50);
          if(rudder_angle < 90){  
            rudder_angle++;
            rudder.write(rudder_angle);
          }
        }
        rudder_angle=90;
        rudder.write(90);
      }else{
        rudder.write(90);
        rudder_angle=90;
        analogWrite(pwmPin, mtr_speed*0.6);
        while(sin((sonar_angle*PI/180))*(distance=measureDistance())<20 ){ //turns slightly to the left while the vertical distance to the master boat is inferior to a certain threshold
          if(Serial.available()>0 ){
              Serial.readBytes(values, 20);
          }
          if(i==15){sonar_angle++; sonar.write(sonar_angle); i=0;} //sonar_angle decrease (every iteration would be too fast)
          i++;
          delay(50);
        }
        delay(250);
        sonar_angle=115;
        sonar.write(115);
        rudder_angle=150;
        rudder.write(150);
        mtr_speed = constrain(mtr_speed*2,0,70);
        analogWrite(pwmPin, mtr_speed);
        while((distance=measureDistance()) > 5){
          if(Serial.available()>0 ){
              Serial.readBytes(values, 20);
            }
          delay(50);
          if(rudder_angle > 90){  
            rudder_angle--;
            rudder.write(rudder_angle);
          }
        }
        delay(250);
        sonar_angle=90;
        sonar.write(90);
        rudder_angle=30;
        rudder.write(30);
        mtr_speed = constrain(mtr_speed*1.5,0,70);
        analogWrite(pwmPin, mtr_speed);
        while((distance=measureDistance()) > 5){
          if(Serial.available()>0 ){
              Serial.readBytes(values, 20);
            }
          delay(50);
          if(rudder_angle < 90){  
            rudder_angle++;
            rudder.write(rudder_angle);
          }
        }
        rudder_angle=90;
        rudder.write(90);
      }
      state=0;
      i=0;
      /*
       *if(first_change){
          state=0;
        }
        else if(last_state==1){ //transition from side to behind
          //slows down until two boats make a 45º angle
          if(first_doin==0){
            first_doin=1;
            sonar_angle=45;
            sonar.write(45);
            rudder_angle=90;
            rudder.write(90);
          }
        if((distance=measureDistance()) > receive_data[2] && first_check!=1){
          analogWrite(pwmPin, mtr_speed*0.7);
        }else{
          //sonar at 65º to detect when slave boat should start straightening, slave boat going to the right with compensated speed to keep up with master
          if(sec_doin==0){
            sec_doin=1;
            first_check=1;
            sonar_angle=115;
            sonar.write(115);
            rudder_angle=150;
            rudder.write(150);
          }
          if((distance=measureDistance()) > receive_data[2]){
            mtr_speed = constrain(mtr_speed*2,0,70);
            analogWrite(pwmPin, mtr_speed);
            if(rudder_angle > 90){  
              rudder_angle--;
              rudder.write(rudder_angle);
            }
          }else{
            //then starts going straigt
            if(tre_doin==0){
              sonar_angle=90;
              sonar.write(90);
              rudder_angle=30;
              rudder.write(30);
              tre_doin=1;
            }
            if((distance=measureDistance()) > receive_data[2]){
              mtr_speed = constrain(mtr_speed*1.5,0,70);
              analogWrite(pwmPin, mtr_speed);
              if(rudder_angle < 90){  
                rudder_angle++;
                rudder.write(rudder_angle);
              }
            }else{
              rudder_angle=90;
              rudder.write(90);
              state=first_check=first_doin=sec_doin=0;}
          }
        }
      } else {
        if(first_doin==0){
          first_doin=1;
          rudder.write(90);
          rudder_angle=90;
          analogWrite(pwmPin, mtr_speed*0.6);
        }
        
        if(first_check!=1 && sin((sonar_angle*PI/180))*(distance=measureDistance())<receive_data[2]*.6 ){ //turns slightly to the left while the vertical distance to the master boat is inferior to a certain threshold
          if(i==25){sonar_angle++; sonar.write(sonar_angle); i=0;} //sonar_angle decrease (every iteration would be too fast)
          i++;
        }else{
          //after that sonar at 90º to detect when slave boat is behind master, slave boat going to the right with compensated speed to keep up with master
          if(sec_doin==0){
            sec_doin=1;
            first_check=1;
            sonar_angle=115;
            sonar.write(115);
            rudder_angle=150;
            rudder.write(150);
          }
          if((distance=measureDistance()) > receive_data[2]){
            mtr_speed = constrain(mtr_speed*2,0,70);
            analogWrite(pwmPin, mtr_speed);
            if(rudder_angle > 90){  
              rudder_angle--;
              rudder.write(rudder_angle);
            }
          }else{
            //then starts going straigt
            if(tre_doin==0){
              sonar_angle=90;
              sonar.write(90);
              rudder_angle=30;
              rudder.write(30);
              tre_doin=1;
            }
            if((distance=measureDistance()) > receive_data[2]){
              mtr_speed = constrain(mtr_speed*1.5,0,70);
              analogWrite(pwmPin, mtr_speed);
              if(rudder_angle < 90){  
                rudder_angle++;
                rudder.write(rudder_angle);
              }
            }else{
              rudder_angle=90;
              rudder.write(90);
              state=first_check=first_doin=sec_doin=0;}
          }
        }
      }*/
    break;

    case 3: // follow_circ
      digitalWrite(directionPin, LOW);
      rudder_angle=45;
      rudder.write(45);
      analogWrite(pwmPin, 70);
      while(cos(PI-(sonar_angle*PI/180))*(distance=measureDistance())<receive_data[2]*.6){ //turns slightly to the left while the horizontal distance to the master boat is inferior to a certain threshold
        if(Serial.available()>0 ){
              Serial.readBytes(values, 20);
        }
        if(i==5){sonar_angle--; sonar.write(sonar_angle); i=0;} //sonar_angle increase (every iteration would be too fast)
        i++;
        delay(50);
        if(rudder_angle < 90){  
          rudder_angle++;
          rudder.write(rudder_angle);
        }
      }
      i=0;
      sonar_angle=0;
      sonar.write(0);
      rudder_angle=135;
      rudder.write(135);
      mtr_speed = constrain(mtr_speed*1.5,0,70);
      analogWrite(pwmPin, mtr_speed);
      while((distance=measureDistance()) > 5){
        if(Serial.available()>0 ){
              Serial.readBytes(values, 20);
            }
        delay(50);
        if(rudder_angle > 90){  
          rudder_angle--;
          rudder.write(rudder_angle);
        }
      }
      delay(250);
      digitalWrite(directionPin, HIGH);
      analogWrite(pwmPin, 20);
      rudder_angle=135;
      rudder.write(135);
      sonar_angle=35;
      sonar.write(35);
      while((distance=measureDistance()) > 5){
        if(Serial.available()>0 ){
            Serial.readBytes(values, 20);
        }
        delay(50);
        if(rudder_angle > 90){  
          rudder_angle--;
          rudder.write(rudder_angle);
        }
      }
      delay(250);
      rudder_angle=35;
      rudder.write(35);
      sonar_angle=90;
      sonar.write(90);
      while((distance=measureDistance()) > 5){
        if(Serial.available()>0 ){
          Serial.readBytes(values, 20);
        }
        delay(50);
        if(rudder_angle < 90){  
          rudder_angle++;
          rudder.write(rudder_angle);
        }
      }
      i=0;
      /*if(first_doin==0){
        rudder_angle=45;
        rudder.write(45);
        analogWrite(pwmPin, 70);
        first_doin=1;
      }
      if(first_check!=1 && cos(PI-(sonar_angle*PI/180))*(distance=measureDistance())<receive_data[2]*.6){ //turns slightly to the left while the horizontal distance to the master boat is inferior to a certain threshold
        if(i==15){sonar_angle--; sonar.write(sonar_angle); i=0;} //sonar_angle increase (every iteration would be too fast)
        i++;
        if(rudder_angle < 90){  
          rudder_angle++;
          rudder.write(rudder_angle);
        }
      }else{
      //after that, starts going straight faster than the master boat until the measured distance is inferior to 10cm (sonar at 180º, to the side, to detect when side by side)
        if(sec_doin==0){
          first_check=1;
          sonar_angle=0;
          sonar.write(0);
          rudder_angle=135;
          rudder.write(135);
          sec_doin=1;
        }
        if((distance=measureDistance()) > receive_data[2] && sec_check!=1){
          mtr_speed = constrain(mtr_speed*1.5,0,70);
          if(rudder_angle > 90){  
            rudder_angle--;
            rudder.write(rudder_angle);
          }
          analogWrite(pwmPin, mtr_speed);
        }else{
          if(tre_doin==0){
            digitalWrite(directionPin, HIGH);
            tre_doin=1;
            sec_check=1;
            rudder_angle=135;
            rudder.write(135);
            sonar_angle=35;
            sonar.write(35);
          }
          if((distance=measureDistance()) > receive_data[2] && tre_check!=1){
            if(rudder_angle > 90){  
              rudder_angle--;
              rudder.write(rudder_angle);
            }
            analogWrite(pwmPin, 10);
          }else{
            if(quad_doin==0){
              quad_doin=1;
              tre_check=1;
              rudder_angle=35;
              rudder.write(35);
              sonar_angle=90;
              sonar.write(90);
            }
            if((distance=measureDistance()) > receive_data[2]){
              if(rudder_angle < 90){  
                rudder_angle++;
                rudder.write(rudder_angle);
              }
              analogWrite(pwmPin, 10);
            }else{
              first_check=first_doin=sec_doin=sec_check=tre_doin=tre_check=quad_doin=0;
              i=0;
              digitalWrite(directionPin, LOW);
            }
          }
        }
      }
    */
    break;

    default:
      state = 0;
      break; 
  }
  
  delay(250);
}



// side mtr_speed * x * cos(|90-rudder|)= mtr_speed <=> x = 1/cos(|90-rudder|)
// vertical speed of slave boat = vertical speed of master boat -> slave boat speed needs multiplier x
//if rudder 150 -> x=2
