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
int button = 4;

// screen variables
int x_Position=0, y_Position=0;

// screen initialization
TFT TFTscreen = TFT(6, 8, 9);

// radio initialization
RF24 radio(7, 10); // CE, CSN
const byte address[6] = "00001"; // Address to send the data

bool last_button_state = LOW; // state of the button on the last cycle
bool button_pressed = 0; // true if the button was pressed on this cycle
unsigned long int last_deb;
unsigned int deb_delay = 50;
int check=0;

int detect_press() {
  bool curr_state = digitalRead(button);
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

unsigned long prev_send=0, send_interval = 50;

void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate( RF24_250KBPS );
  radio.enableAckPayload();
  radio.setRetries(5,5); // delay, count
  
  for(int i=2; i<=4; i++){
    pinMode(i,INPUT); }

  TFTscreen.begin();
  TFTscreen.background(176,5,51);
  TFTscreen.stroke(21,100,179);
  TFTscreen.setTextSize(2);
}

void loop() {
  if (radio.available()) {
    short int data[6];
    bool check;

    // Read analog values
    data[0] = map(analogRead(potX),0,1023,30,150); // joystick X axis value - servo goes from 30º to 150º
    data[1] = map(analogRead(potY),0,1023,-512,512); // joystick Y axis value
    data[2] = analogRead(pot); // potenciometer value
    data[3] = digitalRead(switch1); // switch1 value (controls formation)
    data[4] = digitalRead(switch2); // switch2 value (controls sinalization)
    data[5] = detect_press(); // button value (controls formation)

    if(millis() - prev_send >= send_interval){
      check = radio.write(data, sizeof(data));
      prev_send=millis();
      if (check && radio.isAckPayloadAvailable()) {
          int temp;
          char temp_str[4];
          radio.read(&temp, sizeof(temp));
          TFTscreen.text("      ", 20, 20);
          String(temp).toCharArray(temp_str,4);
          TFTscreen.text(temp_str, 20, 20);
          //print multiple temp values or reset screen aka only print one
      }
    }
  }
}
