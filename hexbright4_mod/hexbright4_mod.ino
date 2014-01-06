/*             Test firmware for HexBright

Notes:
  Requires Arduino 1.0.1!

*/

#include <math.h>
#include <Wire.h>

// Settings
#define OVERTEMP                315
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
#define MODE_POWERUP            0
#define MODE_OFF                1
#define MODE_LOW                2
#define MODE_MED                3
#define MODE_HIGH               4
#define MODE_KNOBBING           5
#define MODE_KNOBBED            6
#define MODE_BLINKING           7
#define MODE_BLINKING_PREVIEW   8
#define MODE_DAZZLING           9
#define MODE_DAZZLING_PREVIEW   10

// State
byte mode = 0;
unsigned long btnTime = 0;
boolean btnDown = false;
unsigned long lastChangeMode = millis();

//Morse Code
char message[] = " SOS ";
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
  // We just powered on!  That means either we got plugged 
  // into USB, or (more likely) the user is pressing the 
  // power button.  We need to pull up the enable pin of 
  // the regulator very soon so we don't lose power.
  pinMode(DPIN_PWR,      INPUT);
  digitalWrite(DPIN_PWR, LOW);
//  pinMode(DPIN_PWR,      OUTPUT);
//  digitalWrite(DPIN_PWR, LOW);

  // Initialize GPIO
  pinMode(DPIN_RLED_SW,  INPUT);
  pinMode(DPIN_GLED,     OUTPUT);
  pinMode(DPIN_DRV_MODE, OUTPUT);
  pinMode(DPIN_DRV_EN,   OUTPUT);
  pinMode(DPIN_ACC_INT,  INPUT);
  pinMode(DPIN_PGOOD,    INPUT);
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
  mode = MODE_OFF;

  Serial.println("Powered up!");
}

void loop()
{
  static unsigned long lastTime, lastTempTime, lastAccTime;
  static float lastKnobAngle, knob;
  static byte blink;
  unsigned long time = millis();
  
  // Blink the indicator LED now and then
  digitalWrite(DPIN_GLED, (time&0x03FF)?LOW:HIGH);

  // Check the serial port
  if (Serial.available())
  {
    char c = Serial.read();
    switch (c)
    {
    case 's':
      {
        int temperature = analogRead(APIN_TEMP);
        Serial.print("Temperature = ");
        Serial.println(temperature);

        char accel[3];
        readAccel(accel);
        Serial.print("Acceleration = ");
        Serial.print(accel[0], DEC);
        Serial.print(", ");
        Serial.print(accel[1], DEC);
        Serial.print(", ");
        Serial.println(accel[2], DEC);
      
        byte pgood = digitalRead(DPIN_PGOOD);
        Serial.print("LED driver power good = ");
        Serial.println(pgood?"Yes":"No");
      }
      break;
    }
  }

  // Check the temperature sensor
  if (time-lastTempTime > 1000)
  {
    lastTempTime = time;
    int temperature = analogRead(APIN_TEMP);
    Serial.print("Temperature = ");
    Serial.println(temperature);
    if (temperature > OVERTEMP)
    {
      Serial.println("Overheat shutdown!");
      mode = MODE_OFF;
      digitalWrite(DPIN_DRV_MODE, LOW);
      digitalWrite(DPIN_DRV_EN, LOW);
      digitalWrite(DPIN_PWR, LOW);
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

  // Do whatever this mode does
  switch (mode)
  {
  case MODE_KNOBBING:
    {
      if (time-lastTime < 100) break;
      lastTime = time;
      
      float angle = readAccelAngleXZ();
      float change = angle - lastKnobAngle;
      lastKnobAngle = angle;
      if (change >  PI) change -= 2.0*PI;
      if (change < -PI) change += 2.0*PI;
      knob += -change * 40.0;
      if (knob < 0)   knob = 0;
      if (knob > 255) knob = 255;

      // Make apparent brightness changes linear by squaring the
      // value and dividing back down into range.  This gives us
      // a gamma correction of 2.0, which is close enough.
      byte bright = (long)(knob * knob) >> 8;
      // Avoid ever appearing off in this mode!
      if (bright < 8) bright = 8;
      analogWrite(DPIN_DRV_EN, bright);
  
      Serial.print("Ang = ");
      Serial.print(angle);
      Serial.print("\tChange = ");
      Serial.print(change);
      Serial.print("\tKnob = ");
      Serial.print(knob);
      Serial.print("\tBright = ");
      Serial.println(bright);
    }
    break;
  case MODE_BLINKING:
  case MODE_BLINKING_PREVIEW:
    //Morse Code
    for (int i = 0; i < sizeof(message); i++)
    {  
      char ch = message[i];
      Serial.print("Morse Letter = ");
      Serial.println(ch);
      if (ch == ' ')
      {
        // 7 beats between words, but 3 have already passed
        // at the end of the last character
        delay(millisPerBeat * 4);
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
          digitalWrite(DPIN_DRV_EN, HIGH);
          if (curChar & 1)  // Dash - 3 beats
            delay(millisPerBeat * 3);
          else              // Dot - 1 beat
            delay(millisPerBeat);
          digitalWrite(DPIN_DRV_EN, LOW);
          // One beat between symbols
          delay(millisPerBeat);
          curChar >>= 1;
        }
        // 3 beats between characters, but one already
        // passed after the last symbol.
        delay(millisPerBeat * 2);
      } 
    }
    
    // Flash the green LED to indicate we're done
    for (int k = 0; k < 3; k++)
    {
      digitalWrite(DPIN_GLED, HIGH);
      delay(50);
      digitalWrite(DPIN_GLED, LOW);
      delay(50);    
    }
    digitalWrite(DPIN_PWR, LOW);
    mode = MODE_LOW;
  //  if (time-lastTime < 250) break;
  //  lastTime = time;

  //  blink = !blink;
  //  digitalWrite(DPIN_DRV_EN, blink);
    break;
  case MODE_DAZZLING:
  case MODE_DAZZLING_PREVIEW:
    if (time-lastTime < 10) break;
    lastTime = time;
    
    digitalWrite(DPIN_DRV_EN, random(4)<1);
    break;
  }
  
  // Check for mode changes
  byte newMode = mode;
  byte newBtnDown = digitalRead(DPIN_RLED_SW);
  switch (mode)
  {
  case MODE_OFF:
    if (btnDown && !newBtnDown)  // Button released
      newMode = MODE_LOW;
    if (btnDown && newBtnDown && (time-btnTime)>500)  // Held
      newMode = MODE_KNOBBING;
    break;
  case MODE_LOW:
    if (btnDown && !newBtnDown)  // Button released
    { 
      if (millis()-lastChangeMode < 700)
        newMode = MODE_MED;
      else
        newMode = MODE_OFF;
    }
    if (btnDown && newBtnDown && (time-btnTime)>500)  // Held
      newMode = MODE_KNOBBING;
    break;
  case MODE_MED:
    if (btnDown && !newBtnDown)  // Button released
    { 
      if (millis()-lastChangeMode < 700)
        newMode = MODE_HIGH;
      else
        newMode = MODE_OFF;
    }
    if (btnDown && newBtnDown && (time-btnTime)>500)  // Held
      newMode = MODE_KNOBBING;
    break;
  case MODE_HIGH:
    if (btnDown && !newBtnDown)  // Button released
    { 
      if (millis()-lastChangeMode < 700)
        newMode = MODE_LOW;
      else
        newMode = MODE_OFF;
    }
    if (btnDown && newBtnDown && (time-btnTime)>500)  // Held
      newMode = MODE_KNOBBING;
    break;
  case MODE_KNOBBING:
    if (btnDown && !newBtnDown)  // Button released
      newMode = MODE_KNOBBED;
    if (btnDown && newBtnDown && tapped)
      newMode = MODE_BLINKING_PREVIEW;
    break;
  case MODE_KNOBBED:
    if (btnDown && !newBtnDown)  // Button released
      newMode = MODE_OFF;
    if (btnDown && newBtnDown && (time-btnTime)>500)  // Held
      newMode = MODE_KNOBBING;
    break;
  case MODE_BLINKING:
    if (btnDown && !newBtnDown)  // Button released
      newMode = MODE_OFF;
    if (btnDown && newBtnDown && (time-btnTime)>500)  // Held
      newMode = MODE_BLINKING_PREVIEW;
    break;
  case MODE_BLINKING_PREVIEW:
    if (btnDown && !newBtnDown)  // Button released
      newMode = MODE_BLINKING;
    if (btnDown && newBtnDown && tapped)
      newMode = MODE_DAZZLING_PREVIEW;
    break;
  case MODE_DAZZLING:
    if (btnDown && !newBtnDown)  // Button released
      newMode = MODE_OFF;
    if (btnDown && newBtnDown && (time-btnTime)>500)  // Held
      newMode = MODE_DAZZLING_PREVIEW;
    break;
  case MODE_DAZZLING_PREVIEW:
    if (btnDown && !newBtnDown)  // Button released
      newMode = MODE_DAZZLING;
    if (btnDown && newBtnDown && tapped)
      newMode = MODE_BLINKING_PREVIEW;
    break;
  }

  // Do the mode transitions
  if (newMode != mode)
  {
    lastChangeMode = millis();
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
    case MODE_KNOBBING:
      Serial.println("Mode = knobbing");
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);
      lastKnobAngle = readAccelAngleXZ();
      knob = (mode==MODE_OFF) ? 0 : 255;
      break;
    case MODE_KNOBBED:
      Serial.println("Mode = knobbed");
      break;
    case MODE_BLINKING:
    case MODE_BLINKING_PREVIEW:
      Serial.println("Mode = blinking");
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);
      digitalWrite(DPIN_DRV_MODE, LOW);
      break;
    case MODE_DAZZLING:
    case MODE_DAZZLING_PREVIEW:
      Serial.println("Mode = dazzling");
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

