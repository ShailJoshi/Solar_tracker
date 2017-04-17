
#include <Servo.h>
#include <Stepper.h>
#include <SoftwareSerial.h>
#include <avr/pgmspace.h>
#include "solar_data_az.h"
#include "solar_data_el.h"

Servo myservo;
int servopos = 0;

const int stepsPerRevolution = 200;
Stepper myStepper(stepsPerRevolution, 4, 5, 6, 7);
float azimth = 0;
float elev = 0;
float elev_corr = 0;  
int16_t steppersteps = 0;
SoftwareSerial BT(10, 11);
boolean mode;
char input;
char RTCstr[10];

int LDRel = 0;
int LDRaz = 0;
int thEl = 512;
int thAz = 512;
int diffEl = 0;
int diffAz = 0;
int diffElp = 0;
int diffAzp = 0;

uint8_t refAz=180;
uint8_t refEl=0;


void setup() {
  Serial.begin(9600);
  initializeHardware();
  Serial.println("Initialized");
Retry:
  BT.println("Select mode: a. Light sense b. LUT");
  while (BT.available() == 0) {};
  input = BT.read();
  if (input == 'a')
    mode = 0;
  else if (input == 'b')
    mode = 1;
  else {
    BT.println("Invalid selection. Please restart.");
    goto Retry;
  }
  BT.println("Input read:");
  BT.println(input);
  BT.println("mode:");
  BT.println(mode);
  Serial.println(mode);


  if (mode == 1) {
    while (BT.available() == 0) {};
    char refl=BT.read();
      uint8_t i = 0;
      int k = 0;
      uint8_t dashpos, pluspos, expos = 0;
      uint8_t  minu = 0;
      uint8_t  hr = 0;
      uint8_t  date = 0;
      for (i = 0; i < 9; i++) {
        while (BT.available() == 0) {};
        RTCstr[i] = BT.read();
        if (RTCstr[i] == '-')
          dashpos = i;
        if (RTCstr[i] == '+')
          pluspos = i;
        if (RTCstr[i] == '!')
        {
          expos = i;
          break;
        }
      }
      RTCstr[i + 1] = '\0';
      Serial.print("pluspos ");
      Serial.println(pluspos);
      Serial.print("ex ");
      Serial.println(expos);
      Serial.print("dash ");
      Serial.println(dashpos);
      Serial.println(RTCstr);

      for (k = expos - 1; k > pluspos; k--) {
        minu = minu + ((uint8_t)(RTCstr[k] - '0')) * (1 + 9 * (expos - 1 - k));

      }

      Serial.print(minu);
      Serial.println("  minutes");
      for (k = pluspos - 1; k > dashpos; k--) {
        hr = hr + ((uint8_t)(RTCstr[k] - '0')) * (1 + 9 * (pluspos - 1 - k));

      }
      Serial.print(hr);
      Serial.println("  hr");
      for (k = dashpos - 1; k >= 0; k--) {
        date = date + ((uint8_t)(RTCstr[k] - '0')) * (1 + 9 * (dashpos - 1 - k));

      }
      Serial.print(date);
      Serial.println("  date ");

      if (minu > 45) {
        minu = 0;
        hr = hr + 1;
      }
      else if (minu < 15)
        minu = 0;
      else if (minu > 15 && minu < 45)
        minu = 1;

      if (hr < 7 || hr > 19)
        Serial.print("Sleep tight");
      else {
        k = (date - 1) * 26 + (hr - 6) * 2 + minu;
        Serial.print("k  ");
        Serial.print(k);
        elev = pgm_read_byte_near(EleAngle + k);
        azimth = pgm_read_byte_near(azimthAngle + k);
        Serial.print("  elev read:  ");
        Serial.print(elev);
        Serial.print("   azimth read: ");
        Serial.println(azimth);
        
        elev = elev * Elemax / 256;
        
        azimth = azimth * Azmax / 256;
        
        
        if(refl=='a'){
         
          double r=sin((double)elev*3.14159/180)+sin((double)refEl*3.14159/180);
          double p=cos((double)refEl*3.14159/180)*sin((double)refAz*3.14159/180) + sin((double)azimth*3.14159/180)*cos((double)elev*3.14159/180);
          double q=cos((double)refAz*3.14159/180)*cos((double)refEl*3.14159/180) + cos((double)elev*3.14159/180)*cos((double)azimth*3.14159/180);
          azimth=atan(p/q);
          azimth=azimth*180/3.14159 + 180*(q<0);
          elev=asin(r/sqrt(r*r+q*q+p*p));
          elev=elev*180/3.14159;
        }
        elev_corr = elev * 0.89;
        steppersteps = azimth * 200 / 360;
        steppersteps = steppersteps - 50;
        Serial.print("elev: ");
        Serial.print((uint8_t)elev);
        Serial.print("  azimth: ");
        Serial.print(azimth);
        Serial.print("   steps: ");
        Serial.println(steppersteps);
        
      }
  }
}


void loop() {
  if (mode == 1) {
    myservo.write((uint16_t)elev_corr);
    delay(50);
    myStepper.step(-steppersteps);
    steppersteps = 0;
  }

  else {

    LDRel = analogRead(A0);

    LDRaz = analogRead(A2);

    diffEl = LDRel - thEl;
    diffAz = LDRaz - thAz;
    Serial.print(LDRel);
    if (diffEl > 25 || diffEl < -25 ) {
      elev = elev - (diffEl) / 75;
      myservo.write((int16_t)elev);
      Serial.print(elev);
      Serial.println("  Elevation");
      BT.print("Elev: ");
      BT.print(elev);
      delay(10);

    }
    if (diffAz > 5 || diffAz < -5) {

      azimth = azimth + (diffAz) / 75;

      myStepper.step((int16_t)diffAz / 75);
      Serial.print(azimth + 90);
      Serial.println("  Azimuth");
    }

  }

}


void initializeHardware()
{
  myservo.attach(9);
  myservo.write(0);
  delay(15);
  myStepper.setSpeed(15);
  Serial.println("Motors initialized");
  while (azimth > 0.2)
  {
    myStepper.step(1);
    //read azimth from mag
  }

  BT.begin(9600);  // Tx on D10 Rx on D11
  BT.println("Ready to Track");
  BT.println(" ");
} 




