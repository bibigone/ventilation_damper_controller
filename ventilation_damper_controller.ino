#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "microDS3231.h"      // https://github.com/GyverLibs/microDS3231 
#include "TM1637Display.h"    // https://github.com/avishorp/TM1637
#include "RCSwitch.h"         // https://github.com/sui77/rc-switch/

//// DESTINATION CONFIGURANTION
//#define BT_SERIAL               // Use Bluetooth serial?

//// HARDWARE CONFIGURANTION
// main output pin
const int MAIN_OUTPUT_PIN = 11;        // the Arduino pin that forms signal voltage for throttle motor (must support PWM)
                                       // NB! For Speaker tone() function is called and it can conflict with WMI via writeAnalogue
                                       //     Use of the tone() function will interfere with PWM output on pins 3 and 11 (on boards other than the Mega).
// relay that powers on/off motor of a throttle
const int MOTOR_RELAY_PIN = 4;         // the Arduino pin, connected to the IN pin of a relay
// speaker
const int SPEAKER_PIN = 7;             // NB! Do NOT use tone() function for speaker beacuse it affects PWN on MAIN_OUTPUT_PIN and listening interrupt on RF_RECEIVER_INT_NUMBER
// display
const int DISP_CLK_PIN = 5;
const int DISP_DIO_PIN = 6;
const uint8_t DISP_BRIGHTNESS = 3;     // 0 - minimum, 2 - default, 7 - maximum
// bluetooth
const int BT_TX_PIN = 0;
const int BT_RX_PIN = 1;
// 433MHZ RF Receiver
const int RF_RECEIVER_INT_NUMBER = 1;   // 1 means PIN3 for Uno, 0 means PIN3 for Leonardo
// Interrupt number to digital pin table (for different boards):
//                  INT 0  INT 1  INT 2  INT 3  INT 4  INT  5
// Nano, UNO, Mini  D2     D3     -      -      -      -
// Leonardo, Micro  D3     D2     D0     D1     D7     -
// Mega             D2     D3     D21    D20    D19    D18

/// PARAMETERS
const int ACTIVE_PERIOD_MS = 30 * 1000;           // 30 sec
const uint8_t ANGLE_STEP = 2;                     // degrees
const uint8_t ANGLE_STEP_SMALL = 1;               // degrees
const uint8_t ANGLE_SMALL_STEP_THRESHOLD = 36;    // degrees
const uint8_t ANGLE_MIN_OPENED = 5;               // degrees (minimum angle that is treated as "opened throttle")
const int AUTO_DISPLAY_OFF_DELAY_MS = 3 * 1000;   // 3 sec
const int DISPLAY_ANGLE_DELAY_MS = 4 * 1000;      // 4 sec
const int RC_ANTI_BOUNCE_DELAY_MS = 250;

//// GLOBAL CONTEXT
// real-time clock DS3231
MicroDS3231 clock;
// RC receiver 433 MHz
RCSwitch mySwitch;
// display
TM1637Display disp(DISP_CLK_PIN, DISP_DIO_PIN);
// throttle angle
uint8_t throttleAngleDegrees;         // 0 - totally closed, 90 - totally opened
bool forcedTotalClose;                // true - throttle is forced to be closed (overrides throttleAngleDegrees value)
// RC 433MHz codes for increment, decrement and other commands
unsigned long rcCodeIncrementCommand;
unsigned long rcCodeDecrementCommand;
unsigned long rcCodeOpenCommand;
unsigned long rcCodeCloseCommand;
unsigned long rcCodeBeepCommand;
// serial
#ifdef BT_SERIAL
SoftwareSerial mySerial(BT_TX_PIN, BT_RX_PIN);
#else
#define mySerial Serial
#endif

// state
enum { STATE_NONE, STATE_IDLE, STATE_ACTIVE } state;
unsigned long lastChangeMs;
unsigned long lastDispOnMs;
unsigned long lastRcCommandReceivedMs;
bool isClockAvailable;
String serialInput;                   // accumulator for characters from serial port (to read line)

//// SETUP
void setup() {
  // configure pins
  pinMode(MAIN_OUTPUT_PIN, OUTPUT);
  pinMode(MOTOR_RELAY_PIN, OUTPUT);
  pinMode(SPEAKER_PIN, OUTPUT);
  
  // init display
  disp.clear();
  disp.setBrightness(DISP_BRIGHTNESS, true);

  // 433MHz receiver
  mySwitch.setReceiveTolerance(60);
  mySwitch.enableReceive(RF_RECEIVER_INT_NUMBER);

  // restore context from EEPROM
  restoreFromEEPROM();
  setOutputLevel();
 
  // initial state
  state = STATE_NONE;
  lastChangeMs = 0;
  lastDispOnMs = 0;
  lastRcCommandReceivedMs = millis();
  powerMotorOff();
  serialInput = "";

  // start serial port
  mySerial.begin(9600);

  // clock
  isClockAvailable = clock.begin();
  if (!isClockAvailable) {
    mySerial.println(F("DS3231 real-time clock not found"));
  }  
}


//// MAIN LOOP
void loop() {
  if (lastDispOnMs != 0 && elapsedMillis(lastDispOnMs) >= AUTO_DISPLAY_OFF_DELAY_MS) {
    dispTurnOff();
  }

  // Process commands from Remote Control
  if (mySwitch.available()) {
    unsigned long rcCode = mySwitch.getReceivedValue();
    mySwitch.resetAvailable();

    if (rcCode == rcCodeIncrementCommand) {      
      if (state == STATE_IDLE || abs(elapsedMillis(lastRcCommandReceivedMs)) >= RC_ANTI_BOUNCE_DELAY_MS) {
        lastRcCommandReceivedMs = millis();
        incrementAngle();
        mySerial.print(F("Increment command received from RC. Result throttle angle = "));
        mySerial.println(throttleAngleDegrees);
      }
    } else if (rcCode == rcCodeDecrementCommand) {
      if (state == STATE_IDLE || abs(elapsedMillis(lastRcCommandReceivedMs)) >= RC_ANTI_BOUNCE_DELAY_MS) {
        lastRcCommandReceivedMs = millis();
        decrementAngle();
        mySerial.print(F("Decrement command received from RC. Result throttle angle = "));
        mySerial.println(throttleAngleDegrees);
      }
    } else if (rcCode == rcCodeOpenCommand) {
      if (state == STATE_IDLE || abs(elapsedMillis(lastRcCommandReceivedMs)) >= RC_ANTI_BOUNCE_DELAY_MS) {
        lastRcCommandReceivedMs = millis();
        openThrottle();
      }      
    } else if (rcCode == rcCodeCloseCommand) {
      if (state == STATE_IDLE || abs(elapsedMillis(lastRcCommandReceivedMs)) >= RC_ANTI_BOUNCE_DELAY_MS) {
        lastRcCommandReceivedMs = millis();
        closeThrottle();
      }      
    } else if (rcCode == rcCodeBeepCommand) {
      if (abs(elapsedMillis(lastRcCommandReceivedMs)) >= RC_ANTI_BOUNCE_DELAY_MS) {
        lastRcCommandReceivedMs = millis();
        beep(440, 30);
      }      
    } else {
      mySerial.print("Unknown RC command received: ");
      mySerial.println(rcCode);
    }
  }

  // change state if needed
  if (state == STATE_NONE) {
    state = STATE_IDLE;
    displayAngle();
  } else if (state == STATE_ACTIVE) {
    int elapsedMs = elapsedMillis(lastChangeMs);
    if (elapsedMs > ACTIVE_PERIOD_MS) {
      deactivate();
    } else if (elapsedMs >= DISPLAY_ANGLE_DELAY_MS && elapsedMs < DISPLAY_ANGLE_DELAY_MS + AUTO_DISPLAY_OFF_DELAY_MS && lastDispOnMs == 0) {
      displayAngle();
    }
  }
  
  // Process command from Serial port if ready
  if (readInputLine()) {
    // what command?
    serialInput.trim();
    if (serialInput == "++" || serialInput == "--") {
      // increment/decrement command
      bool isIncrement = serialInput[0] == '+';
      if (isIncrement) incrementAngle(); else decrementAngle();
      mySerial.print(isIncrement ? F("Increment") : F("Decrement"));
      mySerial.print(F(" command received. Result throttle angle = "));
      mySerial.println(throttleAngleDegrees);
    } else if (serialInput == "close") {
      closeThrottle();
    } else if (serialInput == "open") {
      openThrottle();
    } else if (serialInput.startsWith("=")) {
      // set-value command
      int v = serialInput.substring(1).toInt();
      if (v < 0 || v > 90) {
         mySerial.print(F("ERROR! Invalid angle value "));
         mySerial.println(v);
         mySerial.println(F("Throttle angle must be between 0 and 90, where 0 means totally closed, and 90 means totally opened."));
      } else {
        setAngle(v);
        mySerial.print(F("Set-value command received. Result throttle angle = "));
        mySerial.println(throttleAngleDegrees);
      }
    } else if (serialInput == "?") {
      // get value command
      mySerial.print(F("Current throttle angle = "));
      mySerial.println(throttleAngleDegrees);
      mySerial.print(F("Open/close status = "));
      mySerial.println(forcedTotalClose ? "closed" : "opened");
    } else if (serialInput == "RC++?" || serialInput == "RC--?"
            || serialInput == "RC^^?" || serialInput == "RCvv?"
            || serialInput == "RC##?") {
      // get RC 433 MHz button code for angle increment/decrement/beep command
      mySerial.print(F("RC 433 MHz button code for "));
      if (serialInput[2] == '+') {
        mySerial.print(F("increment: "));
        mySerial.println(rcCodeIncrementCommand);
      } else if (serialInput[2] == '-') {
        mySerial.print(F("decrement: "));
        mySerial.println(rcCodeDecrementCommand);
      } else if (serialInput[2] == '^') {
        mySerial.print(F("open: "));
        mySerial.println(rcCodeOpenCommand);
      } else if (serialInput[2] == 'v') {
        mySerial.print(F("close: "));
        mySerial.println(rcCodeCloseCommand);
      } else {
        mySerial.print(F("beep: "));
        mySerial.println(rcCodeBeepCommand);
      }
    } else if (serialInput.startsWith("RC++=") || serialInput.startsWith("RC--=")
            || serialInput.startsWith("RC^^=") || serialInput.startsWith("RCvv=")
            || serialInput.startsWith("RC##=")) {
      // set RC 433 MHz button code for angle increment/decrement/open/close/beep command
      String codeStr = serialInput.substring(5);
      unsigned long code = codeStr.toInt();
      if (code == 0) {
         mySerial.print(F("ERROR! Invalid RC button code value "));
         mySerial.println(codeStr);
         mySerial.println(F("RC button code must be 4-bytes integer."));
      } else {
        mySerial.print(F("RC 433 MHz button code for "));
        if (serialInput[2] == '+') {
          rcCodeIncrementCommand = code;
          mySerial.print(F("increment"));
        } else if (serialInput[2] == '-') {
          rcCodeDecrementCommand = code;
          mySerial.print(F("decrement"));
        } else if (serialInput[2] == '^') {
          rcCodeOpenCommand = code;
          mySerial.print(F("open"));
        } else if (serialInput[2] == 'v') {
          rcCodeCloseCommand = code;
          mySerial.print(F("close"));
        } else {
          rcCodeBeepCommand = code;
          mySerial.print(F("beep"));
        }
        storeToEEPROM();
        mySerial.print(F(" command has been changed to: "));
        mySerial.println(code);
      }
    } else if (serialInput.startsWith("date") && serialInput.length() >= 5 && (serialInput[4] == '?' || serialInput[4] == '=')) {
      if (checkThatClockIsAvailable()) {
        if (serialInput[4] == '?') {
          mySerial.print(F("Current date: "));
          mySerial.println(clock.getDateString());
        } else {
          DateTime dt = clock.getTime();
          String dateStr = serialInput.substring(5);
          if (tryParseDateValue(dateStr, dt)) {
            clock.setTime(dt);
            mySerial.print(F("Corrected date: "));
            mySerial.println(clock.getDateString());
          } else {
            mySerial.print(F("ERROR! Invalid date value "));
            mySerial.println(dateStr);
            mySerial.println(F("Expected value in DD.MM.YYYY format, where DD=00..31, MM=01..12, YYYY=2000..."));
          }
        }
      }
    } else if (serialInput.startsWith("time") && serialInput.length() >= 5 && (serialInput[4] == '?' || serialInput[4] == '=')) {
      if (checkThatClockIsAvailable()) {
        if (serialInput[4] == '?') {
          mySerial.print(F("Current time: "));
          mySerial.println(clock.getTimeString());
        } else {
          DateTime dt = clock.getTime();
          String timeStr = serialInput.substring(5);
          if (tryParseTimeValue(timeStr, dt)) {
            clock.setTime(dt);
            mySerial.print(F("Corrected time: "));
            mySerial.println(clock.getTimeString());
          } else {
            mySerial.print(F("ERROR! Invalid time value "));
            mySerial.println(timeStr);
            mySerial.println(F("Expected value in hh:mi:ss format, where hh=00..23, mi=00..59, ss=00..59"));
          }
        }
      }
    } else {
      // something unknown
      // inform about supported commands
      printHowToUse();
    }

    // reset input buffer
    serialInput = "";
  }
}

void printHowToUse() {
  mySerial.print(F("Unknown command received: "));
  mySerial.println(serialInput);
  mySerial.println(F("Supported commands:"));
  mySerial.println(F("? - get current throttle angle"));
  mySerial.println(F("=n - set throttle angle to n degrees (from 0 to 90)"));
  mySerial.println(F("++ - increment throttle angle"));
  mySerial.println(F("-- - decrement throttle angle"));
  mySerial.println(F("open - restore throttle angle after forced close"));
  mySerial.println(F("close - force full close of throttle"));
  mySerial.println(F("RC++? - get RC 433 MHz button code for angle increment command"));
  mySerial.println(F("RC++=<code> - set RC 433 MHz button code for angle increment command"));
  mySerial.println(F("RC--? - get RC 433 MHz button code for angle decrement command"));
  mySerial.println(F("RC--=<code> - set RC 433 MHz button code for angle decrement command"));
  mySerial.println(F("RC^^? - get RC 433 MHz button code for open command"));
  mySerial.println(F("RC^^=<code> - set RC 433 MHz button code for open command"));
  mySerial.println(F("RCvv? - get RC 433 MHz button code for close command"));
  mySerial.println(F("RCvv=<code> - set RC 433 MHz button code for close command"));
  mySerial.println(F("RC##? - get RC 433 MHz button code for beep command"));
  mySerial.println(F("RC##=<code> - set RC 433 MHz button code for beep command"));
  mySerial.println(F("date? - print current date as DD.MM.YYYY"));
  mySerial.println(F("date=<DD.MM.YYYY> - set current date to a specified value"));
  mySerial.println(F("time? - print current time as hh:mi:ss"));
  mySerial.println(F("time=<hh:mi:ss> - set current time to a specified value"));
}

bool checkThatClockIsAvailable() {
  if (!isClockAvailable) {
    mySerial.println(F("ERROR! Real-time clock device is not available."));
  }
  return isClockAvailable;
}

bool isDigit(char c) {
  return c >= '0' && c <= '9';
}

bool twoDigitsToInt(const String& str, int pos, uint8_t& res) {
  if (str.length() < pos + 2) return false;
  if (!isDigit(str[pos]) || !isDigit(str[pos + 1])) return false;
  res = (str[pos] - '0') * 10 + (str[pos + 1] - '0');
  return true;
}

bool tryParseDateValue(const String& str, DateTime& dt) {
  // general checks
  if (str.length() != 10) return false;
  if (str[2] != '.' || str[5] != '.') return false;
  // DD
  if (!twoDigitsToInt(str, 0, dt.date)) return false;
  if (dt.date > 31 || dt.date < 1) return false;
  // MM
  if (!twoDigitsToInt(str, 3, dt.month)) return false;
  if (dt.month > 12 || dt.month < 1) return false;
  // YYYY
  if (str[6] != '2' || str[7] != '0') return false;
  uint8_t y;
  if (!twoDigitsToInt(str, 8, y)) return false;
  dt.year = 2000 + y;
  // OK
  return true;
}

bool tryParseTimeValue(const String& str, DateTime& dt) {
  // general checks
  if (str.length() != 8) return false;
  if (str[2] != ':' || str[5] != ':') return false;
  // hh
  if (!twoDigitsToInt(str, 0, dt.hour)) return false;
  if (dt.hour > 23) return false;
  // mi
  if (!twoDigitsToInt(str, 3, dt.minute)) return false;
  if (dt.minute > 59) return false;
  // ss
  if (!twoDigitsToInt(str, 6, dt.second)) return false;
  if (dt.second > 59) return false;
  // OK
  return true;
}

bool readInputLine() {
  // listen for serail port input
  int newCharsCount = mySerial.available();
  for (int i = 0; i < newCharsCount; i++) {
    // reading new characted from serial port
    int r = mySerial.read();
    if (r <= 0) break;
    char newChar = (char)r;

    // in Windows new line mark consists of two characters: "\r\n"
    // take into account only '\n' as an end of line
    // and simply ignore '\r'
    if (newChar == '\n') {
      return true;
    } else if (newChar != '\r') {
      // simply collect symbols
      serialInput += newChar;
    }
  }

  return false;
}

void setAngle(uint8_t v) {
  v = constrain(v, 0, 90);
  lastChangeMs = millis();
  throttleAngleDegrees = v;
  forcedTotalClose = false;
  setOutputLevel();
  beepAngle();
  activate();
  if (lastDispOnMs != 0) {
    displayAngle();
  }
}

void incrementAngle() {
  uint8_t angleStep = throttleAngleDegrees >= ANGLE_SMALL_STEP_THRESHOLD ? ANGLE_STEP : ANGLE_STEP_SMALL;
  setAngle(throttleAngleDegrees + angleStep);
}

void decrementAngle() {
  uint8_t angleStep = throttleAngleDegrees > ANGLE_SMALL_STEP_THRESHOLD ? ANGLE_STEP : ANGLE_STEP_SMALL;
  uint8_t v = throttleAngleDegrees <= angleStep ? 0 : throttleAngleDegrees - angleStep;
  setAngle(v);
}

void closeThrottle() {
  mySerial.println(F("Force full close command received."));
  if (forcedTotalClose) {
    mySerial.println(F("Already closed"));
  } else {
    forcedTotalClose = true;
    lastChangeMs = millis();
    setOutputLevel();
    activate();
    if (lastDispOnMs != 0) {
      displayAngle();
    }
    mySerial.println(F("Throttle has been totally closed."));
  }
}

void openThrottle() {
  mySerial.println(F("Restore throttle angle command received."));
  if (!forcedTotalClose && throttleAngleDegrees >= ANGLE_MIN_OPENED) {
    mySerial.println(F("Already opened"));
  } else if (throttleAngleDegrees < ANGLE_MIN_OPENED) {
    setAngle(ANGLE_MIN_OPENED);
    mySerial.print(F("Throttle angle has been set to a minimum opened value "));
    mySerial.println(ANGLE_MIN_OPENED);
  } else {
    forcedTotalClose = false;
    lastChangeMs = millis();
    setOutputLevel();
    activate();
    if (lastDispOnMs != 0) {
      displayAngle();
    }
    mySerial.print(F("Throttle angle has been restored to "));
    mySerial.println(throttleAngleDegrees);
  }
}

void deactivate() {
  if (state == STATE_ACTIVE) {
    state = STATE_IDLE;
    powerMotorOff();
    storeToEEPROM();
  }
}

void activate() {
  if (state == STATE_IDLE) {
    setOutputLevel();
    delay(50);
    powerMotorOn();
  }
  state = STATE_ACTIVE;
  lastChangeMs = millis();
}

void setOutputLevel() {
  int v = forcedTotalClose ? 0 : constrain((throttleAngleDegrees * 255) / 90, 0, 255);
  analogWrite(MAIN_OUTPUT_PIN, v);
}

void powerMotorOn() {
  digitalWrite(MOTOR_RELAY_PIN, LOW);
}

void powerMotorOff() {
  digitalWrite(MOTOR_RELAY_PIN, HIGH);
}

void beepAngle() {
  int freqHz = 199 + 17 * throttleAngleDegrees;
  int durationMs = throttleAngleDegrees <= 0 || throttleAngleDegrees >= 90 ? 200 : 50;
  beep(freqHz, durationMs);
}

void beep(int freqHz, int durationMs) {
  // NB! Do NOT use tone() function for speaker beacuse it affects PWN on MAIN_OUTPUT_PIN and listening interrupt on RF_RECEIVER_INT_NUMBER

  long periodMicro = 1000000L / freqHz;
  long lenMicro = durationMs * 1000L;
  int iterations = (int)(lenMicro / periodMicro);
  long halfPeriodMicro = periodMicro / 2 - 10;
  int waitMs = (int)(halfPeriodMicro / 10000) * 10;
  int waitMicro = (int)(halfPeriodMicro % 10000);

  for (int i = 0; i < iterations; i++)
  {
    digitalWrite(SPEAKER_PIN, HIGH);
    delay(waitMs);
    delayMicroseconds(waitMicro);    
    digitalWrite(SPEAKER_PIN, LOW);
    delay(waitMs);
    delayMicroseconds(waitMicro);    
  }
}

void displayAngle() {
  uint8_t b[4];
  if (forcedTotalClose) {
    b[0] = SEG_A | SEG_D | SEG_E | SEG_F;  // C
    b[1] = SEG_D | SEG_E | SEG_F;  // L
    b[2] = SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F;  // O
    b[3] = SEG_A | SEG_F | SEG_G | SEG_C | SEG_D; // S
  } else {
    b[0] = 0;
    int v1 = throttleAngleDegrees / 10;
    b[1] = v1 > 0 && v1 <= 9 ? disp.encodeDigit((uint8_t)v1) : 0;
    int v2 = throttleAngleDegrees % 10;
    b[2] = v2 >= 0 && v2 <= 9 ? disp.encodeDigit((uint8_t)v2) : 0;
    b[3] = 0x63/*degree*/;
  }
  disp.setSegments(b);
  lastDispOnMs = millis();
}

void dispTurnOff() {
  if (lastDispOnMs != 0) {
    disp.clear();
    lastDispOnMs = 0;
  }
}

//// CONTEXT PERSISTENCY
// reading global context from EEPROM
void restoreFromEEPROM() {
  EEPROM.get(0, throttleAngleDegrees);
  throttleAngleDegrees = constrain(throttleAngleDegrees, 0, 90);
  rcCodeIncrementCommand = 0;
  EEPROM.get(4, rcCodeIncrementCommand);
  rcCodeDecrementCommand = 0;
  EEPROM.get(8, rcCodeDecrementCommand);
  rcCodeBeepCommand = 0;
  EEPROM.get(12, rcCodeBeepCommand);
  forcedTotalClose = false;
  EEPROM.get(16, forcedTotalClose);
  rcCodeOpenCommand = 0;
  EEPROM.get(20, rcCodeOpenCommand);
  rcCodeCloseCommand = 0;
  EEPROM.get(24, rcCodeCloseCommand);
}
// writing global context to EEPROM
void storeToEEPROM() {
  EEPROM.put(0, throttleAngleDegrees);
  EEPROM.put(4, rcCodeIncrementCommand);
  EEPROM.put(8, rcCodeDecrementCommand);
  EEPROM.put(12, rcCodeBeepCommand);
  EEPROM.put(16, forcedTotalClose);
  EEPROM.put(20, rcCodeOpenCommand);
  EEPROM.put(24, rcCodeCloseCommand);
}

//// HELPERS
// difference of milliseconds taking into account overflow of unsigned long
int elapsedMillis(unsigned long startMs) {
  unsigned long nowMs = millis();
  if (nowMs >= startMs) return (int)(nowMs - startMs);
  return (int)(nowMs + (0xFFFFFFFFul - startMs));
}