#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;

Servo servo4;
Servo servo5;
Servo servo6;

Servo servo7;
Servo servo8;
Servo servo9;

Servo servo10;
Servo servo11;
Servo servo12;

Servo servo13;
Servo servo14;
Servo servo15;

Servo servo16;
Servo servo17;
Servo servo18;

float RFB;
float RFBa;
float RFBFiltered;
float RLR;
float RLRa;
float RLRFiltered;
float RT;
float RTa;
float RTFiltered;

float LFB;
float LFBa;
float LFBFiltered;
float LLR;
float LLRa;
float LLRFiltered;
float LT;
float LTa;
float LTFiltered;

float posa;
float posb;
float posc;
float posd;
float pose;
float posf;

float pos1;
float pos2;
float pos3;
float pos4;
float pos5;
float pos6;
float pos1Offset;
float pos2Offset;
float pos3Offset;
float pos4Offset;
float pos5Offset;
float pos6Offset;

float pos11;
float pos12;
float pos13;
float pos14;
float pos15;
float pos16;
float pos11Offset;
float pos12Offset;
float pos13Offset;
float pos14Offset;
float pos15Offset;
float pos16Offset;

float pos21;
float pos22;
float pos23;
float pos24;
float pos25;
float pos26;
float pos21Offset;
float pos22Offset;
float pos23Offset;
float pos24Offset;
float pos25Offset;
float pos26Offset;

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer

float transLateFactorF;
float transLateFactorR;

int sw1;
int sw2;

void setup() {

Serial.begin(115200);

pinMode(8, INPUT_PULLUP);
pinMode(9, INPUT_PULLUP);

servo1.attach(22);
servo2.attach(24); 
servo3.attach(26); 
servo4.attach(28);
servo5.attach(30); 
servo6.attach(32);

servo7.attach(34);
servo8.attach(36); 
servo9.attach(38);
servo10.attach(40);
servo11.attach(42); 
servo12.attach(44);

servo13.attach(46);
servo14.attach(48); 
servo15.attach(50);
servo16.attach(52);
servo17.attach(3); 
servo18.attach(4);

pos1Offset = -90;
pos2Offset = -90;
pos3Offset = 25;
pos4Offset = -30;
pos5Offset = -20;
pos6Offset = 80;

pos11Offset = -30;
pos12Offset = 30;
pos13Offset = -100;
pos14Offset = 0;
pos15Offset = 20;
pos16Offset = 20;

pos21Offset = -80;
pos22Offset = -100;
pos23Offset = -100;
pos24Offset = -100;
pos25Offset = 50;
pos26Offset = -80;
}

void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {  // start timed event    
      previousMillis = currentMillis;

      // read switches

      sw1 = digitalRead(9);
      sw2 = digitalRead(8);

      // read sticks

      RFB = analogRead(A3);
      RLR = analogRead(A2);
      RT = analogRead(A1);
      LT = analogRead(A0);
      LFB = analogRead(A4);
      LLR = analogRead(A5);

      // threshold sticks

      RFBa = thresholdStick(RFB);
      RFBa = RFBa*-1;
      RLRa = thresholdStick(RLR);
      RLRa = RLRa*-1;
      RTa = thresholdStick(RT);
      LFBa = thresholdStick(LFB);
      LLRa = thresholdStick(LLR);
      LTa = thresholdStick(LT); 

      // filter sticks

      RFBFiltered = filter(RFBa, RFBFiltered,50);
      RLRFiltered = filter(RLRa, RLRFiltered,50);
      RTFiltered = filter(RTa, RTFiltered,50);
      LFBFiltered = filter(LFBa, LFBFiltered,50);
      LLRFiltered = filter(LLRa, LLRFiltered,50);
      LTFiltered = filter(LTa, LTFiltered,50);

      // Hacky kinematics

      if (RFBFiltered < 0) {
        transLateFactorF = -0.33;
        transLateFactorR = 0;
      }
      else if (RFBFiltered > 0) {
        transLateFactorR = 0.45;
        transLateFactorF = 0;
      }
      else {
        transLateFactorR = 0;
        transLateFactorF = 0;
      }

      //      ROLL                    PITCH                   HEAVE             TRANSLATE-FB                          TRANSLATE-LR                      

      posa = (LLRFiltered * 0.66)   + LFBFiltered             + RTFiltered      + (RFBFiltered * transLateFactorF)    + (RLRFiltered * -0.66);
      posb = (LLRFiltered * -0.66)  + LFBFiltered             + RTFiltered      + (RFBFiltered * transLateFactorF)    + (RLRFiltered * 0.66);
      posc = (LLRFiltered * 1)      + (LFBFiltered * 0.33)    + RTFiltered      + RFBFiltered                         + (RLRFiltered * 0.33);  
      posd = (LLRFiltered * 0.33)   + (LFBFiltered * -1)      + RTFiltered      + (RFBFiltered * transLateFactorR)    + (RLRFiltered * 1);
      pose = (LLRFiltered * -0.33)  + (LFBFiltered * -1)      + RTFiltered      + (RFBFiltered * transLateFactorR)    + (RLRFiltered * -1);
      posf = (LLRFiltered * -1)     + (LFBFiltered * 0.33)    + RTFiltered      + RFBFiltered                         + (RLRFiltered * -0.33);

      // overall scaling

      posa = posa * 2;
      posb = posb * 2;
      posc = posc * 2;
      posd = posd * 2;
      pose = pose * 2;
      posf = posf * 2;

      // ****** STAGE ONE ****** 

      // apply offsets for fine tuning between servos

      pos1 = posa + 1500 + pos1Offset;
      pos2 = posb + 1500 + pos2Offset;
      pos3 = posc + 1500 + pos3Offset;
      pos4 = posd + 1500 + pos4Offset;
      pos5 = pose + 1500 + pos5Offset;
      pos6 = posf + 1500 + pos6Offset;

      // constrain servos

      pos1 = constrain(pos1,1000,2400);
      pos2 = constrain(pos2,1000,2400);
      pos3 = constrain(pos3,1000,2400);
      pos4 = constrain(pos4,1000,2400);
      pos5 = constrain(pos5,1000,2400);
      pos6 = constrain(pos6,1000,2400);

      // write to servos

      servo1.writeMicroseconds(pos1);
      servo2.writeMicroseconds(pos2);
      servo3.writeMicroseconds(pos3);
      servo4.writeMicroseconds(pos4);
      servo5.writeMicroseconds(pos5);
      servo6.writeMicroseconds(pos6);

      // ****** STAGE TWO ******

      if (sw1 == 1) {
      pos11 = posa;
      pos12 = posb;
      pos13 = posc;
      pos14 = posd;
      pos15 = pose;
      pos16 = posf;      
      }

      else if (sw1 == 0) {      // reverse stage 2
      pos11 = posa*-1;
      pos12 = posb*-1;
      pos13 = posc*-1;
      pos14 = posd*-1;
      pos15 = pose*-1;
      pos16 = posf*-1;
      }

      pos11 = pos11 + 1500 + pos11Offset;
      pos12 = pos12 + 1500 + pos12Offset;
      pos13 = pos13 + 1500 + pos13Offset;
      pos14 = pos14 + 1500 + pos14Offset;
      pos15 = pos15 + 1500 + pos15Offset;
      pos16 = pos16 + 1500 + pos16Offset;

      // constrain servos

      pos11 = constrain(pos11,1000,2400);
      pos12 = constrain(pos12,1000,2400);
      pos13 = constrain(pos13,1000,2400);
      pos14 = constrain(pos14,1000,2400);
      pos15 = constrain(pos15,1000,2400);
      pos16 = constrain(pos16,1000,2400);

      // write to servos

      servo7.writeMicroseconds(pos11);
      servo8.writeMicroseconds(pos12);
      servo9.writeMicroseconds(pos13);
      servo10.writeMicroseconds(pos14);
      servo11.writeMicroseconds(pos15);
      servo12.writeMicroseconds(pos16);

      // ****** STAGE THREE *****

      if (sw2 == 1) {
      pos21 = posa;
      pos22 = posb;
      pos23 = posc;
      pos24 = posd;
      pos25 = pose;
      pos26 = posf;      
      }

      else if (sw2 == 0) {      // reverse stage 2
      pos21 = posa*-1;
      pos22 = posb*-1;
      pos23 = posc*-1;
      pos24 = posd*-1;
      pos25 = pose*-1;
      pos26 = posf*-1;
      }

      pos21 = pos21 + 1500 + pos21Offset;
      pos22 = pos22 + 1500 + pos22Offset;
      pos23 = pos23 + 1500 + pos23Offset;
      pos24 = pos24 + 1500 + pos24Offset;
      pos25 = pos25 + 1500 + pos25Offset;
      pos26 = pos26 + 1500 + pos26Offset;

      // constrain servos

      pos21 = constrain(pos21,1000,2400);
      pos22 = constrain(pos22,1000,2400);
      pos23 = constrain(pos23,1000,2400);
      pos24 = constrain(pos24,1000,2400);
      pos25 = constrain(pos25,1000,2400);
      pos26 = constrain(pos26,1000,2400);

      // write to servos

      servo13.writeMicroseconds(pos21);
      servo14.writeMicroseconds(pos22);
      servo15.writeMicroseconds(pos23);
      servo16.writeMicroseconds(pos24);
      servo17.writeMicroseconds(pos25);
      servo18.writeMicroseconds(pos26);


  }   // end of timed loop

}


