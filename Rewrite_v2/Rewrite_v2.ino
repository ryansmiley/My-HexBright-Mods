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
#define MODE_FIXOFF             6

// State
byte mode = MODE_OFF;
byte pastMode = 0;
byte beforeStrobe = 0;
byte useBeforeStrobe = 0;
byte lightOn = 0;
byte strobeOn = 0;
byte sosOn = 0;
unsigned long btnTime = 0;
boolean btnDown = false;
unsigned long lastButtonPress = millis();
unsigned long lastChangeMode = millis();
long previousMillis = 0;
unsigned long currentMillis = 0;
byte longPress = 0;
byte usePastMode;
byte startPastMode = 0;
//byte remembered from previous go around
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
  
  if (StoredEEPROM > 5){
    Serial.println("Writing to EEPROM 1:0");
    EEPROM.write(1,0);
    StoredEEPROM = 0;
  }
  if (StoredEEuse > 3){
    Serial.println("Writing to EEPROM 3:0");
    EEPROM.write(3,0);
    StoredEEuse = 0;
  }
  Serial.print("StoredEEPROM:");
  Serial.println(StoredEEPROM);
  Serial.print("StoredEEuse:");
  Serial.println(StoredEEuse);
  usePastMode = StoredEEuse;
  if (usePastMode == 1) {
    startPastMode = 1;
    Serial.println("Writing to EEPROM 3:0");
    EEPROM.write(3,0);
  }
  usePastMode = 0;
  
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
  byte newMode = mode;
  if (useBeforeStrobe==1){
    newMode = beforeStrobe;
    if (newMode==0)
      mode = 1;
    else
      mode = 0;
    beforeStrobe = 0;
    useBeforeStrobe = 0;
  }
  if (endMorse == true){
    newMode = 0;
    mode = 1;
    endMorse = false;
    sosOn = 0;
  }
  if (longPress == 1){
    newMode = 0;
    mode = 1;
    longPress = 0;
  }
  byte newBtnDown = digitalRead(DPIN_RLED_SW);
  byte btnPressTime = (time-btnTime);

  if (btnDown && !newBtnDown){
    if ((time-btnTime) < 400){ 
      if ((millis() - lastChangeMode) < 900 || (lightOn == 0)){
        if (mode==MODE_OFF){
          // Mode Low
          pastMode = MODE_OFF;
          newMode=MODE_LOW;
        } else if (mode==MODE_LOW){
          // Mode Medium
          pastMode = MODE_LOW;
          newMode=MODE_MED;
        } else if (mode==MODE_MED){
          if (pastMode == MODE_LOW){
            // Mode High
            newMode=MODE_HIGH;
          } else {
            // Mode Low
            newMode=MODE_LOW;
          }
          pastMode = MODE_MED;
        } else if (mode==MODE_HIGH){
          // Mode Medium
          pastMode = MODE_HIGH;
          newMode=MODE_MED;
        } 
        lastChangeMode = millis();
        lightOn = 1;
      } else {
        // Mode Off       
        lastChangeMode = 0;
        lightOn = 0;
        newMode=MODE_OFF;
        pastMode = mode;
        
      }
      sosOn = 0;
      strobeOn = 0;
    } else if ((time-btnTime) >= 400  && (time-btnTime) < 800 ){
      if (mode != MODE_OFF){
        Serial.print("long press: ");
        Serial.println(mode);
        // Record for later use
        if ((StoredEEPROM != mode) && (mode != 1)){
          Serial.println("Saving to EEPROMs");
          EEPROM.write(1,mode);
        } 
        EEPROM.write(3,1);  
        newMode = MODE_OFF;
      } else {
        //Serial.println("long button press while off");
      }
      sosOn = 0;
      strobeOn = 0;
      lightOn = 0;
      longPress = 1;
    }
    if (startPastMode==1){
      Serial.print("On PastMode:");
      Serial.println(StoredEEPROM);
      newMode=StoredEEPROM;
      mode = 0;
      lastChangeMode = millis();
      lightOn = 1;
      startPastMode=0;
    }
  } else if (btnDown && (time-btnTime)>=800) {
      if (time-lastTime > 20){
        lastTime = time;
        digitalWrite(DPIN_DRV_EN, random(4)<1);
      }
      Serial.print("pastMode: ");
      Serial.println(pastMode);
      Serial.print("Mode: ");
      Serial.println(mode);
      if (mode != MODE_STROBE)
        beforeStrobe = mode;
      newMode=MODE_STROBE;
      strobeOn = 1;
  }
  
  if (strobeOn == 1 && !btnDown){
    // Turn off Strobe on or go to last Mode
    newMode=MODE_OFF;
    strobeOn = 0;
    sosOn = 0;
  }
  
  if ((tapped && strobeOn == 1) || (sosOn == 1)){
    // Morse Code
    endMorse = false;
    byte buttonStillPressed = true;
    for (int i = 0; i < sizeof(message); i++)
    {
      if (digitalRead(DPIN_RLED_SW) && buttonStillPressed==false){
        Serial.println("Button Pressed in Morse");
        endMorse = true; 
        break; 
      } else if(!digitalRead(DPIN_RLED_SW)){
        buttonStillPressed=false;
      }
      
      char ch = message[i];
      if (ch == ' ')
      {
        // 7 beats between words, but 3 have already passed
        // at the end of the last character
        Serial.println("Writing SPACES");
        currentMillis = millis();
        previousMillis = 0;
        while (previousMillis < millisPerBeat * 4 ){
           previousMillis = millis() - currentMillis;
           if (digitalRead(DPIN_RLED_SW) && buttonStillPressed==false){
             Serial.println("Button Pressed in Morse");
             endMorse = true; 
             break;
           } else if(!digitalRead(DPIN_RLED_SW)){
             buttonStillPressed=false;
           }
        }
        //delay(millisPerBeat * 4);
      }
      else
      {
        // Remap ASCII to the morse table
        if      (ch >= 'A' && ch <= 'Z') ch -= 'A';
        else if (ch >= 'a' && ch <= 'z') ch -= 'a';
        else if (ch >= '0' && ch <= '9') ch -= '0' - 26;
        else continue;
        
        // Extract the symbols and length
        byte curChar  = morse[ch] & 0x00FF;
        byte nSymbols = morse[ch] >> 8;
        
        // Play each symbol
        for (int j = 0; j < nSymbols; j++)
        {
          
          if (digitalRead(DPIN_RLED_SW) && buttonStillPressed==false){
             Serial.println("Button Pressed in Morse");
             endMorse = true; 
             break;
          } else if(!digitalRead(DPIN_RLED_SW)){
            buttonStillPressed=false;
          }
          Serial.println("Writing SYMBOL");
          digitalWrite(DPIN_DRV_EN, HIGH);
          if (curChar & 1)  // Dash - 3 beats
          {  
            currentMillis = millis();
            previousMillis = 0;
            while (previousMillis < millisPerBeat * 3 ){
              previousMillis = millis() - currentMillis;
               if (digitalRead(DPIN_RLED_SW) && buttonStillPressed==false){
                 Serial.println("Button Pressed in Morse");
                 endMorse = true; 
                 break;
               } else if(!digitalRead(DPIN_RLED_SW)){
                 buttonStillPressed=false;
               }
            }
            //delay(millisPerBeat * 3);
          }
          else              // Dot - 1 beat
          {
            currentMillis = millis();
            previousMillis = 0;
            while (previousMillis < millisPerBeat){
               previousMillis = millis() - currentMillis;
               if (digitalRead(DPIN_RLED_SW) && buttonStillPressed==false){
                 Serial.println("Button Pressed in Morse");
                 endMorse = true; 
                 break;
               } else if(!digitalRead(DPIN_RLED_SW)){
                 buttonStillPressed=false;
               } 
            }
            //delay(millisPerBeat);
          }
          digitalWrite(DPIN_DRV_EN, LOW);
          // One beat between symbols
          currentMillis = millis();
          previousMillis = 0;
          while (previousMillis < millisPerBeat){
             previousMillis = millis() - currentMillis;
             if (digitalRead(DPIN_RLED_SW) && buttonStillPressed==false){
               Serial.println("Button Pressed in Morse");
               endMorse = true; 
               break;
             } else if(!digitalRead(DPIN_RLED_SW)){
               buttonStillPressed=false;
             } 
          }      
          //delay(millisPerBeat);
          curChar >>= 1;
        }
        // 3 beats between characters, but one already
        // passed after the last symbol.
        currentMillis = millis();
        previousMillis = 0;
        while (previousMillis < millisPerBeat * 2 ){
           previousMillis = millis() - currentMillis;
           if (digitalRead(DPIN_RLED_SW) && buttonStillPressed==false){
             Serial.println("Button Pressed in Morse");
             endMorse = true; 
             break;
           } else if(!digitalRead(DPIN_RLED_SW)){
             buttonStillPressed=false;
           }
        }
        //delay(millisPerBeat * 2);
      } 
    }
    if (endMorse == true){
      Serial.println("Leaving Morse");
      pastMode = MODE_OFF;
      mode = MODE_OFF;
      //strobeOn = 0;
      //sosOn = 0;
    } else {
      newMode=MODE_MORSE;
      strobeOn = 0;
      sosOn = 1;
    }

  }

  if (newMode != mode)
  {
    switch (newMode)
    {
    case MODE_OFF:
      Serial.println("Mode = off");
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, LOW);
      digitalWrite(DPIN_DRV_MODE, LOW);
      digitalWrite(DPIN_DRV_EN, LOW);
      break;
    case MODE_LOW:
      Serial.println("Mode = low");
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);
      digitalWrite(DPIN_DRV_MODE, LOW);
      analogWrite(DPIN_DRV_EN, 64);
      break;
    case MODE_MED:
      Serial.println("Mode = medium");
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);
      digitalWrite(DPIN_DRV_MODE, LOW);
      analogWrite(DPIN_DRV_EN, 255);
      break;
    case MODE_HIGH:
      Serial.println("Mode = high");
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);
      digitalWrite(DPIN_DRV_MODE, HIGH);
      analogWrite(DPIN_DRV_EN, 255);
      break;
    case MODE_STROBE:
      Serial.println("Mode = Strobe");
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);
      digitalWrite(DPIN_DRV_MODE, HIGH);
      break;
    case MODE_MORSE:
      Serial.println("Mode = Morse");
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);
      digitalWrite(DPIN_DRV_MODE, HIGH);
      analogWrite(DPIN_DRV_EN, 255);
      break;
    }

    mode = newMode;
  }
      
  // Remember last button state
  if (newBtnDown != btnDown)
  {
    btnTime = time;
    btnDown = newBtnDown;
    if (mode==MODE_STROBE){
      mode = beforeStrobe;
      useBeforeStrobe = 1;
      strobeOn = 0;
    }
    if (mode==MODE_MORSE){
      mode = MODE_OFF;
    }
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
