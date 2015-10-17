#include <original.h>
#include <Servo.h> 
Servo myservo8;
Servo myservo9;
int tri,stalkon = 1,go_on,roton,di,pos = 0,meassurement[10],size,go_bak;
unsigned int back;
int left_on = 0,rit_on=0;
byte inByte;
boolean received = false; 
UDOO *regler=copypoint();

void setup() {
  regler->Init();
  init_NVIC_ADC();
  Init_Sense();
  Init_PIOC_Interrupt();
  myservo8.attach(A8);
  myservo9.attach(A9);
  Serial.begin(115200);
  delay(100);
}

void loop() {
  Serial.flush();
  byte msg[10];
  int count = 0;
  while (Serial.available() > 0) {
    
    byte inByte = (byte)Serial.read();
  
    msg[count] = inByte;
    count++;
    
    received = true;
  }
 
 if(received){
   //to go forward starts fromherew
  if(msg[0] == 'b'){//backward
    
    EVENT();
    ADC_IE();
    go_bak=1;
    regler->Set_back(msg[1]*10);
    regler->backdone=0;
    regler->posdone=0;
  }
  
  if(msg[0] == 'f'){//forward
    
    EVENT();
    ADC_IE();
    go_on=1;
    regler->Set_dist(msg[1]*10);
    regler->backdone=0;
    regler->posdone=0;
  }
  if (msg[0] == 'c' ) {// tell servo to go to position in variable 'pos'
     
      myservo8.write(msg[1]);                
      delay(15);
      
      
    }
    if (msg[0] == 't' ) {// tell servo to go to position in variable 'pos'
      tri=180-msg[1];
      myservo9.write(tri);                
      delay(15);
      
      
    }
 
    if ((msg[0]) ==  'l') {
      
      left_on = 1;
      regler->leftdone=0;
      regler->degr_left(msg[1]);//Set amount of degrees to rotate
      EVENT();
      ADC_IE();
      
    }
   if ((msg[0]) ==  'r') {
      
      rit_on = 1;
      regler->ritdone=0;
      regler->degr_right(msg[1]);//Set amount of degrees to rotate
      EVENT();
      ADC_IE();
      
    } 
 received=false;
 
 }
 
  if(regler->backdone==HIGH ){
     
     EVENT_off();
     ADC_ID();
     go_bak=0;
     regler->backdone=0;
    regler->posdone=0;
     Serial.print('k');
  }  
  if (go_bak==1){
    regler->Position_Control_Loop();
    regler->Speed_Control_Loop();
    regler->back_Planner();
  }
  
  
  //forward operation
  if(regler->posdone==HIGH ){
     
     EVENT_off();
     ADC_ID();
     go_on=0;
     regler->backdone=0;
    regler->posdone=0;
     Serial.print('h');
  }  
  if (go_on==1){
    regler->Position_Control_Loop();
    regler->Speed_Control_Loop();
    regler->Pos_Planner();
  }
  
  if(regler->leftdone==HIGH ){
     
     left_on=0;
     EVENT_off();
     ADC_ID();
     regler->leftdone=0;
     regler->ritdone=0;
     Serial.print('m');
     
  }  
  if(left_on==HIGH  ){
   
     regler->Position_Control_Loop();
     regler->Speed_Control_Loop();
     regler->rotate_Left();
     
  }
  
  if(regler->ritdone==HIGH ){
     
     rit_on=0;
     EVENT_off();
     ADC_ID();
     regler->leftdone=0;
     regler->ritdone=0;
     Serial.print('v');
     
  }  
  if(rit_on==HIGH  ){
   
     regler->Position_Control_Loop();
     regler->Speed_Control_Loop();
     regler->rotate_Right();
     
  }
  delay(10);
}

