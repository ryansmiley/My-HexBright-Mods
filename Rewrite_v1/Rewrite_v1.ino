/* 
  Factory firmware for HexBright FLEX with Modifications by Ryan Smiley
  My Version v.1  Jan 5, 2014
  Factory Version v2.4  Dec 6, 2012
*/

#include <math.h>
#include <Wire.h>
#include <EEPROM.h> 

#define HOLD_TIME 250 // milliseconds before going to strobe
#define OFF_TIME 650 // milliseconds before going off on the next normal button press

// Settings
#define OVERTEMP                340
// Constants
#define ACC_ADDRESS             0x4C
#define ACC_REG_XOUT            0
#define ACC_REG_YOUT            1
#define ACC_REG_ZOUT            2
#define ACC_REG_TILT            3
#define ACC_REG_INTS            6
#define ACC_REG_MODE            7
// Pin assignments
#define DPIN_RLED_SW            2
#define DPIN_GLED               5
#define DPIN_PGOOD              7
#define DPIN_PWR                8
#define DPIN_DRV_MODE           9
#define DPIN_DRV_EN             10
#define DPIN_ACC_INT            3
#define APIN_TEMP               0
#define APIN_CHARGE             3
// Interrupts
#define INT_SW                  0
#define INT_ACC                 1
// Modes
#define MODE_OFF                0
#define MODE_LOW                1
#define MODE_MED                2
#define MODE_HIGH               3
#define MODE_STROBE             4
#define MODE_MORSE              5


// State
byte mode = MODE_OFF;
byte pastMode = 0;
byte lightOn = 0;
byte strobeOn = 0;
byte sosOn = 0;
unsigned long btnTime = 0;
boolean btnDown = false;
unsigned long lastButtonPress = millis();
unsigned long lastChangeMode = millis();
long previousMillis = 0;
unsigned long currentMillis = 0;

byte StoredEEPROM = EEPROM.read(1);
byte StoredEEuse = EEPROM.read(3);

//Morse Code
char message[] = "SOS  ";
int millisPerBeat = 125;
byte endMorse = false;

// Low byte  = morse code, LSB first, 0=dot 1=dash
word morse[] = {
  0x0202, // A .-
  0x0401, // B -...
  0x0405, // C -.-.
  0x0301, // D -..
  0x0100, // E .
  0x0404, // F ..-.
  0x0303, // G --.
  0x0400, // H ....
  0x0200, // I ..
  0x040E, // J .---
  0x0305, // K -.-
  0x0402, // L .-..
  0x0203, // M --
  0x0201, // N -.
  0x0307, // O ---
  0x0406, // P .--.
  0x040B, // Q --.-
  0x0302, // R .-.
  0x0300, // S ...
  0x0101, // T -
  0x0304, // U ..-
  0x0408, // V ...-
  0x0306, // W .--
  0x0409, // X -..-
  0x040D, // Y -.--
  0x0403, // Z --..
  0x051F, // 0 -----
  0x051E, // 1 .----
  0x051C, // 2 ..---
  0x0518, // 3 ...--
  0x0510, // 4 ....-
  0x0500, // 5 .....
  0x0501, // 6 -....
  0x0503, // 7 --...
  0x0507, // 8 ---..
  0x050F, // 9 ----.
};

void setup()
{
  static unsigned long lastTime, lastTempTime, lastAccTime;
  static byte blink;
  unsigned long time = millis();
  
  // We just powered on!  That means either we got plugged 
  // into USB, or the user is pressing the power button.
  pinMode(DPIN_PWR,      INPUT);
  digitalWrite(DPIN_PWR, LOW);

  // Initialize GPIO
  pinMode(DPIN_RLED_SW,  INPUT);
  pinMode(DPIN_GLED,     OUTPUT);
  pinMode(DPIN_DRV_MODE, OUTPUT);
  pinMode(DPIN_DRV_EN,   OUTPUT);
  pinMode(DPIN_ACC_INT,  INPUT);
  digitalWrite(DPIN_DRV_MODE, LOW);
  digitalWrite(DPIN_DRV_EN,   LOW);
  digitalWrite(DPIN_ACC_INT,  HIGH);
  
  // Initialize serial busses
  Serial.begin(9600);
  Wire.begin();

  // Configure accelerometer
  byte config[] = {
    ACC_REG_INTS,  // First register (see next line)
    0xE4,  // Interrupts: shakes, taps
    0x00,  // Mode: not enabled yet
    0x00,  // Sample rate: 120 Hz
    0x0F,  // Tap threshold
    0x10   // Tap debounce samples
  };
  Wire.beginTransmission(ACC_ADDRESS);
  Wire.write(config, sizeof(config));
  Wire.endTransmission();
  
  // Enable accelerometer
  byte enable[] = {ACC_REG_MODE, 0x01};  // Mode: active!
  Wire.beginTransmission(ACC_ADDRESS);
  Wire.write(enable, sizeof(enable));
  Wire.endTransmission();
  
  btnTime = millis();
  btnDown = digitalRead(DPIN_RLED_SW);
  
//  if (StoredEEPROM > 5){
//    Serial.println("Writing to EEPROM 1");
//    EEPROM.write(1,0);
//    StoredEEPROM = 0;
//  }
//  if (StoredEEuse > 3){
//    Serial.println("Writing to EEPROM 3");
//    EEPROM.write(3,0);
//    StoredEEuse = 0;
//  } 
//  usePastMode = StoredEEuse;
//  if (usePastMode == 0)
//    pastMode = 0;
//  else  
//    pastMode = StoredEEPROM;
//  usePastMode = 0;
  
  Serial.println("Powered up!");
}

void loop()
{
  static unsigned long lastTime, lastTempTime, lastAccTime;
  unsigned long time = millis();
  static unsigned long flash_time = millis();
  byte pastModeReset = 1;
  
  // Check the state of the charge controller
  int chargeState = analogRead(APIN_CHARGE);
  if (chargeState < 128)  // Low - charging
  {
    digitalWrite(DPIN_GLED, (time&0x0100)?LOW:HIGH);
  }
  else if (chargeState > 768) // High - charged
  {
    digitalWrite(DPIN_GLED, HIGH);
  }
  else // Hi-Z - shutdown
  {
    digitalWrite(DPIN_GLED, LOW);    
  }
  
  // Check the temperature sensor
  if (time-lastTempTime > 1000)
  {
    lastTempTime = time;
    int temperature = analogRead(APIN_TEMP);
//    Serial.print("Temp: ");
//    Serial.println(temperature);
    if (temperature > OVERTEMP && mode != MODE_OFF)
    {
      Serial.println("Overheating!");

      for (int i = 0; i < 6; i++)
      {
        digitalWrite(DPIN_DRV_MODE, LOW);
        delay(100);
        digitalWrite(DPIN_DRV_MODE, HIGH);
        delay(100);
      }
      digitalWrite(DPIN_DRV_MODE, LOW);

      mode = MODE_LOW;
    }
  }
  
  // Check if the accelerometer wants to interrupt
  byte tapped = 0, shaked = 0;
  if (!digitalRead(DPIN_ACC_INT))
  {
    Wire.beginTransmission(ACC_ADDRESS);
    Wire.write(ACC_REG_TILT);
    Wire.endTransmission(false);       // End, but do not stop!
    Wire.requestFrom(ACC_ADDRESS, 1);  // This one stops.
    byte tilt = Wire.read();
    
    if (time-lastAccTime > 500)
    {
      lastAccTime = time;
  
      tapped = !!(tilt & 0x20);
      shaked = !!(tilt & 0x80);
  
      if (tapped) Serial.println("Tap!");
      if (shaked) Serial.println("Shake!");
    }
  }
  
  // Periodically pull down the button's pin, since
  // in certain hardware revisions it can float.
  pinMode(DPIN_RLED_SW, OUTPUT);
  pinMode(DPIN_RLED_SW, INPUT);

  // Check for mode changes
  byte newBtnDown = digitalRead(DPIN_RLED_SW);
  byte btnPressTime = (time-btnTime);


  if (btnDown && !newBtnDown){
    Serial.print("Button Press Time: ");
    Serial.println((time-btnTime));
    if ((time-btnTime) < 400){
      Serial.println("In Less than 400 if statement");
      Serial.print("Time since last light change: ");
      Serial.println((millis() - lastChangeMode));
      if ((millis() - lastChangeMode) < 900 || (lightOn == 0)){
        Serial.println("In Less than 900 if statement");
        if (mode==MODE_OFF){
          Serial.println("mode=low");
          // Light Settings Low
          pinMode(DPIN_PWR, OUTPUT);
          digitalWrite(DPIN_PWR, HIGH);
          digitalWrite(DPIN_DRV_MODE, LOW);
          analogWrite(DPIN_DRV_EN, 64);
          // End Light
          pastMode = MODE_OFF;
          mode=MODE_LOW;
        } else if (mode==MODE_LOW){
          Serial.println("mode=med");
          // Light Settings Medium
          pinMode(DPIN_PWR, OUTPUT);
          digitalWrite(DPIN_PWR, HIGH);
          digitalWrite(DPIN_DRV_MODE, LOW);
          analogWrite(DPIN_DRV_EN, 255);
          // End Light
          pastMode = MODE_LOW;
          mode=MODE_MED;
        } else if (mode==MODE_MED){
          if (pastMode == MODE_LOW){
            Serial.println("mode=high");
            // Light Settings High
            pinMode(DPIN_PWR, OUTPUT);
            digitalWrite(DPIN_PWR, HIGH);
            digitalWrite(DPIN_DRV_MODE, HIGH);
            analogWrite(DPIN_DRV_EN, 255);
            // End Light
            mode=MODE_HIGH;
          } else {
            Serial.println("mode=low");
            // Light Settings Low
            pinMode(DPIN_PWR, OUTPUT);
            digitalWrite(DPIN_PWR, HIGH);
            digitalWrite(DPIN_DRV_MODE, LOW);
            analogWrite(DPIN_DRV_EN, 64);
            // End Light
            mode=MODE_LOW;
          }
          pastMode = MODE_MED;
        } else if (mode==MODE_HIGH){
          Serial.println("mode=medium");
          // Light Settings Medium
          pinMode(DPIN_PWR, OUTPUT);
          digitalWrite(DPIN_PWR, HIGH);
          digitalWrite(DPIN_DRV_MODE, LOW);
          analogWrite(DPIN_DRV_EN, 255);
          // End Light
          pastMode = MODE_HIGH;
          mode=MODE_MED;
        } 
        lastChangeMode = millis();
        lightOn = 1;
      } else {
        Serial.println("mode=off");
        // Light Settings Off
        pinMode(DPIN_PWR, OUTPUT);
        digitalWrite(DPIN_PWR, LOW);
        digitalWrite(DPIN_DRV_MODE, LOW);
        digitalWrite(DPIN_DRV_EN, LOW);
        // End Light
        lastChangeMode = 0;
        lightOn = 0;
        mode=MODE_OFF;
        pastMode = mode;
      }
      sosOn = 0;
      strobeOn = 0;
    } else if ((time-btnTime) >= 400  && (time-btnTime) < 800 ){
      if (mode != MODE_OFF){
        Serial.println("long button press while on");
        Serial.println("mode=off");
        pastMode = mode;
        mode=MODE_OFF;
      } else {
        Serial.println("long button press while off");
      }
      sosOn = 0;
      strobeOn = 0;
    } else if (btnDown && (time-btnTime)>=800){
      Serial.println("mode=Strobe");
//      // Light Settings Dazzle
//      pinMode(DPIN_PWR, OUTPUT);
//      digitalWrite(DPIN_PWR, HIGH);
//      digitalWrite(DPIN_DRV_MODE, HIGH);
//      // End Light
//      if (time-lastTime < 10)
//        lastTime = time;
//      digitalWrite(DPIN_DRV_EN, random(4)<1);
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);
      digitalWrite(DPIN_DRV_MODE, LOW);
      analogWrite(DPIN_DRV_EN, 255);
      strobeOn = 1;
    } 
  }
  if (tapped && strobeOn == 1){
    Serial.println("mode=SOS");
    strobeOn = 0;
    sosOn = 1;
  }
  
      
  // Remember last button state
  if (newBtnDown != btnDown)
  {
    btnTime = time;
    btnDown = newBtnDown;
    if (mode==MODE_STROBE)
      mode = pastMode;
    Serial.print("btnTime = ");
    Serial.println(btnTime);
    Serial.print("btnDown = ");
    Serial.println(btnDown);
    Serial.print("mode = ");
    Serial.println(mode);
    //delay(50);
  }

}


void readAccel(char *acc)
{
  while (1)
  {
    Wire.beginTransmission(ACC_ADDRESS);
    Wire.write(ACC_REG_XOUT);
    Wire.endTransmission(false);       // End, but do not stop!
    Wire.requestFrom(ACC_ADDRESS, 3);  // This one stops.

    for (int i = 0; i < 3; i++)
    {
      if (!Wire.available())
        continue;
      acc[i] = Wire.read();
      if (acc[i] & 0x40)  // Indicates failed read; redo!
        continue;
      if (acc[i] & 0x20)  // Sign-extend
        acc[i] |= 0xC0;
    }
    break;
  }
}

float readAccelAngleXZ()
{
  char acc[3];
  readAccel(acc);
  return atan2(acc[0], acc[2]);
}
