#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>
#include <TFT.h>

#define potX A0
#define potY A1

// control pins
#define pot A3
int switch1 = 2;
int switch2 = 3;
int button = A5;

// screen variables
int x_Position=0, y_Position=0;
int l_t=0;

// screen initialization //cs, dc rst
TFT TFTscreen = TFT(5, 8, 9);

// radio initialization
RF24 radio(7, 10); // CE, CSN
const byte address[6] = "00001"; // Address to send the data

bool last_button_state = LOW; // state of the button on the last cycle
bool button_pressed = 0; // true if the button was pressed on this cycle
unsigned long int last_deb;
unsigned int deb_delay = 50;
int check=0;

int detect_press() {
  bool curr_state;
  if(analogRead(button)>512){curr_state=true;} else{curr_state=false;} 
  if(curr_state != last_button_state){ //mudou estado em relação ao anterior, muda o last_deb
    last_deb = millis();
  }
  if(millis() - last_deb > deb_delay){ //estado está estável
  if(curr_state == LOW){ //botao nao clicado
      button_pressed = curr_state;
      check = 0; //check a 0, possivel passar button_pressed a true
    }
    if(check ==1){ //se check a 1, significa que na iteração anterior button_pressed ja esteve a true, para evitar passar mais que 1 estado fica a false
      button_pressed = false;
    }
    if(curr_state == HIGH && check==0){ 
      button_pressed = curr_state;
      check = 1;
    }
      
  }
  last_button_state = curr_state;
  return button_pressed;
}

unsigned long prev_send=0, send_interval = 250;

void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  radio.setDataRate( RF24_250KBPS );
  radio.enableAckPayload();
  radio.setRetries(5,5); // delay, count
  
  for(int i=2; i<=3; i++){
    pinMode(i,INPUT); }

  TFTscreen.begin();
  //b,g,r
  TFTscreen.background(0,0,255);
  TFTscreen.fill(0,102,0);
  TFTscreen.rect(0,0,64,128);
  TFTscreen.fill(0,255,255);
  TFTscreen.circle(64,64,32);
  TFTscreen.stroke(0,0,255);
  TFTscreen.line(48,42,80,42);
  TFTscreen.line(48,42,48,71);
  TFTscreen.line(80,42,80,71);
  TFTscreen.line(48,71,64,86);
  TFTscreen.line(64,86,80,71);
  TFTscreen.stroke(255,0,0);
  TFTscreen.setTextSize(1);
  delay(2000);
}

short int data[6]={90,0,0,0,0,0};

void loop() {
  //TFTscreen.text("                   ", 80, 20);
  //TFTscreen.text("                   ", 80, 30);
  //TFTscreen.text("                   ", 80, 40);
  
  //sprintf(val,"X: %d Y: %d",data[0],data[1]);
  //TFTscreen.text(val, 80, 20);
  //sprintf(val,"Pot: %d",data[2]);
  //TFTscreen.text(val, 80, 30);
  //sprintf(val,"s1:%d s2:%d bt:%d",data[3],data[4],data[5]);
  //TFTscreen.text(val, 80, 40);
  //char val[20];
  
  bool check;

  // Read analog values
  data[0] = map(analogRead(potX),0,1023,30,150); // joystick X axis value - servo goes from 30º to 150º
  data[1] = map(analogRead(potY),0,1023,-150,150); // joystick Y axis value
  data[2] = analogRead(pot); // potenciometer value
  data[3] = digitalRead(switch1); // switch1 value (controls formation)
  data[4] = digitalRead(switch2); // switch2 value (controls sinalization)
  if(analogRead(button)>512 && last_button_state!=1){ // button value (controls formation)
    data[5]=1;
  }else{
    data[5]=last_button_state=0;
  }
  
  if(data[1]>-70 && data[1]<70){
    data[1]=0;  
  }
  
  if(millis() - prev_send >= send_interval){
    check = radio.write(data, sizeof(data));
    prev_send=millis();
    if(data[5]==1){
      last_button_state=1;  
    }
    if (check && radio.isAckPayloadAvailable()) {
        int temp;
        char temp_str[4];
        radio.read(&temp, sizeof(temp));
        if(l_t!=temp){
          TFTscreen.fill(0,255,255);
          TFTscreen.circle(64,64,32);
          TFTscreen.stroke(0,0,255);
          TFTscreen.line(48,42,80,42);
          TFTscreen.line(48,42,48,71);
          TFTscreen.line(80,42,80,71);
          TFTscreen.line(48,71,64,86);
          TFTscreen.line(64,86,80,71);
          TFTscreen.stroke(255,0,0);
          String(temp).toCharArray(temp_str,4);
          l_t=temp;
          TFTscreen.text(temp_str, 64, 64);
        }
    }
  }
}
