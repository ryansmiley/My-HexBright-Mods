/* 
  Factory firmware for HexBright FLEX with Modifications by Ryan Smiley
  My Version v.1  Jan 4, 2014
  Factory Version v2.4  Dec 6, 2012
*/

#include <math.h>
#include <Wire.h>

//#define BUILD_HACK
//#include <hexbright.h>

//hexbright hb;

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
#define MODE_STROBE_PREVIEW     5
//#define MODE_BLINKING           4
//#define MODE_BLINKING_PREVIEW   5


// State
byte mode = 0;
unsigned long btnTime = 0;
boolean btnDown = false;
unsigned long lastButtonPress = millis();
unsigned long lastChangeMode = millis();
byte pastMode = 1;
//Morse Code
char message[] = "SOS SOS SOS SOS";
int millisPerBeat = 100;
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
  digitalWrite(DPIN_DRV_MODE, LOW);
  digitalWrite(DPIN_DRV_EN,   LOW);
  
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
  mode = MODE_OFF;

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
    Serial.print("Temp: ");
    Serial.println(temperature);
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

  // Do whatever this mode does
  switch (mode)
  {
  case MODE_STROBE:
  case MODE_STROBE_PREVIEW:
//    for (int i = 0; i < sizeof(message); i++)
//    {
//      char ch = message[i];
//      if (ch == ' ')
//      {
//        // 7 beats between words, but 3 have already passed
//        // at the end of the last character
//        delay(millisPerBeat * 4);
//      }
//      else
//      {
//        // Remap ASCII to the morse table
//        if      (ch >= 'A' && ch <= 'Z') ch -= 'A';
//        else if (ch >= 'a' && ch <= 'z') ch -= 'a';
//        else if (ch >= '0' && ch <= '9') ch -= '0' - 26;
//        else continue;
//        
//        // Extract the symbols and length
//        byte curChar  = morse[ch] & 0x00FF;
//        byte nSymbols = morse[ch] >> 8;
//        
//        // Play each symbol
//        for (int j = 0; j < nSymbols; j++)
//        {
//          digitalWrite(DPIN_DRV_EN, HIGH);
//          if (curChar & 1)  // Dash - 3 beats
//            delay(millisPerBeat * 3);
//          else              // Dot - 1 beat
//            delay(millisPerBeat);
//          digitalWrite(DPIN_DRV_EN, LOW);
//          // One beat between symbols
//          delay(millisPerBeat);
//          curChar >>= 1;
//        }
//        // 3 beats between characters, but one already
//        // passed after the last symbol.
//        delay(millisPerBeat * 2);
//      } 
//    }

    //Dazzle Strobe
    if (time-lastTime < 10) break;
      lastTime = time;
    digitalWrite(DPIN_DRV_EN, random(4)<1);
    pastModeReset = 0;
    //Constant Strobe
    //digitalWrite(DPIN_DRV_EN, (time%225)<25);
    break;
  }
  
  // Periodically pull down the button's pin, since
  // in certain hardware revisions it can float.
  pinMode(DPIN_RLED_SW, OUTPUT);
  pinMode(DPIN_RLED_SW, INPUT);
  
  // Check for mode changes
  byte newMode = mode;
  byte newBtnDown = digitalRead(DPIN_RLED_SW);
  if (newBtnDown){
    //Serial.println("The Button was pressed");
    lastButtonPress = millis();
  }
  switch (mode)
  {
  case MODE_OFF:
    if (btnDown && !newBtnDown && (time-btnTime)>20)
      newMode = MODE_LOW;
    if (btnDown && newBtnDown && (time-btnTime)>500){
      newMode = MODE_STROBE_PREVIEW;
      pastMode = MODE_OFF;
    }
    break;
  case MODE_LOW:
    if (btnDown && !newBtnDown && (time-btnTime)>50){ 
      if (millis()-lastChangeMode < 900)
        newMode = MODE_MED;
      else
        newMode = MODE_OFF;
    }
    if (btnDown && newBtnDown && (time-btnTime)>500){
      newMode = MODE_STROBE_PREVIEW;
      pastMode = MODE_LOW;
    }
    break;
  case MODE_MED:
    if (btnDown && !newBtnDown && (time-btnTime)>50){
      if (millis()-lastChangeMode < 900)
        newMode = MODE_HIGH;
      else
        newMode = MODE_OFF;
    }
    if (btnDown && newBtnDown && (time-btnTime)>500){
      newMode = MODE_STROBE_PREVIEW;
      pastMode = MODE_MED;
    }
    break;
  case MODE_HIGH:
    if (btnDown && !newBtnDown && (time-btnTime)>50){
      if (millis()-lastChangeMode < 900)
        newMode = MODE_LOW;
      else
        newMode = MODE_OFF;
    }
    if (btnDown && newBtnDown && (time-btnTime)>500){
      newMode = MODE_STROBE_PREVIEW;
      pastMode = MODE_HIGH;
    }
    break;
  case MODE_STROBE_PREVIEW:
    // This mode exists just to ignore this button release.
    if ((time-btnTime)< 250)
      newMode = MODE_STROBE;
    break;
  case MODE_STROBE:
      //newMode = MODE_OFF;
      newMode = pastMode;
      pastModeReset = 0;
    break;
  }

  // Do the mode transitions
  if (newMode != mode)
  {

    if (pastModeReset == 1)
      lastChangeMode = millis();
    else
      lastChangeMode = 0;
      
    switch (newMode)
    {
    case MODE_OFF:
      Serial.println("Mode = off");
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, LOW);
      digitalWrite(DPIN_DRV_MODE, LOW);
      digitalWrite(DPIN_DRV_EN, LOW);
      pastModeReset = 1;
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
    case MODE_STROBE_PREVIEW:
      Serial.println("Mode = strobe");
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);
      digitalWrite(DPIN_DRV_MODE, HIGH);
      break;
    }

    mode = newMode;
  }

  // Remember button state so we can detect transitions
  if (newBtnDown != btnDown)
  {
    btnTime = time;
    btnDown = newBtnDown;
    delay(50);
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
