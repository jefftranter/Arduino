/*
  The heart of the code that manages the AD9850 was written by
  Richard Visokey AD7C - www.ad7c.com

  Modifications were made by Jack Purdum and Dennis Kidder:
   Rev 4.00:  Feb. 2, 2015
   Rev 5.00:  Jul. 8, 2015, Jack Purdum
   Rev 6.00:  Aug. 1, 2015, Jack Purdum

  This version has additional changes by Hank Ellis K5HDE:
  V1.0 14 June 2016 Implementation of the basic voltmeter
  V1.1 23 June 2016 Fixed bug that blanks the voltmeter when the frequency step is changed via the encoder pushbutton
  V2.0 16 July 2016 Added voltage low and high cautions and warnings

  This version has additional changes by Jeff Tranter VE3ICH:
  - Finish implementing support for RIT.
  - Indicate transmit on Arduino LED.
  - Indicate transmit on LCD.
  - Modified startup screen.
  - Make options configurable at compile time.
  - Small code formatting changes and cleanup (e.g. fix compile warnings)
  - Idle timeout for LCD backlight.

  Tested with Arduino IDE V-1.6.9
*/

// Uncomment next line if you want the Voltmeter display option enabled.
#define VOLTMETER
// Uncomment next line if you want the Arduino LED to indicate transmit.
#define TXLED
// Uncomment next line if you want the LCD display to indicate transmit.
#define TXLCD
// Uncomment next line if you want the RIT offset feature.
#define RIT
// Uncomment next line if you want the backlight screen blanking feature.
#define BLANKING

#include <Rotary.h>   // From Brian Low: https://github.com/brianlow/Rotary
#include <EEPROM.h>   // Shipped with IDE
#include <Wire.h>     //         "

// Get the LCD I2C Library here:
// https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
// Move all *_I2C files into a new folder named LiquidCrystal_I2C
// in the Arduino library folder
#include <LiquidCrystal_I2C.h>

#define MYTUNINGCONSTANT     34.35977500000000    // Replace with your calculated TUNING CONSTANT. See article.

#define SPACES      "       "          // Altered in V1.1 of Voltmeter implementation
#define SPACES16    "                " // A full line of spaces
#define HERTZ       "Hz "
#define KILOHERTZ   "kHz "
#define MEGAHERTZ   "MHz "

#define GENERAL     2
#define TECH        1
#define EXTRA       0

#define ELEMENTCOUNT(x) (sizeof(x) / sizeof(x[0]))  // A macro that calculates the number
// of elements in an array.
#define pulseHigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW); } // Write macro

// Set up some VFO band edges
#define VFOUPPERFREQUENCYLIMIT  7300000L            // Upper band edge
#define VFOLOWERFREQUENCYLIMIT  7000000L            // Lower band edge
#define VFOLOWALLBUTEXTRA       7025000L            // Frequency of Extra licensees only
#define VFOHIGHTECH             7125000L            // Hi Tech cutoff
#define VFOLOWTECH              7100000L            // Low Tech cutoff
#define VFOGENERALLOWGAP        7175000L            // General class edge

#define W_CLK             8               // Pin  8 - connect to AD9850 module word load clock pin (CLK)
#define FQ_UD             9               // Pin  9 - connect to freq update pin (FQ)
#define DATA             10               // Pin 10 - connect to serial data load pin (DATA)
#define RESET            11               // Pin 11 - connect to reset pin (RST)

#define LCDCOLS          16               // LCD stuff
#define LCDROWS           2
#define SPLASHDELAY    2000               // Hold splash screen for 2 seconds

#define ROTARYSWITCHPIN   4               // Used by switch for rotary encoder
#define RITPIN            7               // Used by pushbutton switch to toggle RIT
#define RXTXPIN          12               // When HIGH, the xcvr is in TX mode
#define RITOFFSETSTART  700L              // Offset for RIT

#define FREQINBAND        0               // Used with range checking
#define FREQOUTOFBAND     1

// ===================================== EEPROM Offsets and data ==============================================
#define READEEPROMFREQ        0       // The EEPROM record address of the last written frequency
#define READEEPROMINCRE       1       // The EEPROM record address of last frequency increment
#define READEEPROMRIT         2       // The EEPROM record address of last RIT offset

#define DELTAFREQOFFSET      25       // Must move frequency more than this to call it a frequency change
#define DELTATIMEOFFSET   60000       // If not change in frequency within 1 minute, update the frequency

#ifdef BLANKING
#define BACKLIGHTTIMEOUT 180000       // If no knob/button activity within this time (in milliseconds), dim the backlight
#endif

unsigned long markFrequency;          // The frequency just written to EEPROM
long eepromStartTime;                 // Set when powered up and while tuning
long idleTime;                        // Time since system was last active (i.e. button pushed or knob turned)

// ============================ ISR variables: ======================================
volatile int_fast32_t currentFrequency;     // Starting frequency of VFO
volatile long currentFrequencyIncrement;
volatile long ritOffset;

// ============================ General Global Variables ============================
bool ritState;                    // Receiver incremental tuning state HIGH, LOW
bool oldRitState;

char temp[17];

int ritDisplaySwitch;             // Variable to index into increment arrays (see below)
int incrementIndex = 0;

long oldRitOffset;
int_fast32_t oldFrequency = 1;    // Variable to hold the updated frequency

#ifdef VOLTMETER
static const char *bandWarnings[] = { "Ext", "Tec", "Gen" }; // Altered in V1.0 of voltmeter implementation
#else
static const char *bandWarnings[] = { "Extra  ", "Tech   ", "General" };
#endif // VOLTMETER

static int whichLicense;
static const char *incrementStrings[] = {"10", "20", "100", ".5", "1", "2.5", "5", "10", "100" }; // These two align
static long incrementTable[]          = { 10,   20,   100,  500, 1000, 2500, 5000, 10000, 100000 };
static long memory[]                  = { VFOLOWERFREQUENCYLIMIT, VFOUPPERFREQUENCYLIMIT };

Rotary r = Rotary(2, 3);       // Create encoder object and set the pins the rotary encoder uses.  Must be interrupt pins.
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Create LCD object and set the LCD I2C address


void setup() {

  // ===================== Set up from EEPROM memory ======================================

  currentFrequency = readEEPROMRecord(READEEPROMFREQ);            // Last frequency read while tuning
  if (currentFrequency < 7000000L || currentFrequency > 7300000L) // New EEPROM usually initialized with 0xFF
    currentFrequency = 7030000L;                                  // Default QRP freq if no EEPROM recorded yet
  markFrequency = currentFrequency;                               // Save EEPROM freq.

  incrementIndex = (int) readEEPROMRecord(READEEPROMINCRE);       // Saved increment as written to EEPROM
  if (incrementIndex < 0 || incrementIndex > 9)                   // If none stored in EEPROM yet...
    incrementIndex = 0;                                           // ...set to 10Hz

  ritOffset = readEEPROMRecord(READEEPROMRIT);                    // Last RIT offset
  if (ritOffset < -100000L || ritOffset > 100000L)                // New EEPROM usually initialized with 0xFF
    ritOffset = RITOFFSETSTART;                                   // Default RIT offset if no EEPROM recorded yet

  currentFrequencyIncrement = incrementTable[incrementIndex];     // Store working freq variables
  eepromStartTime = millis();                                     // Need to keep track of EEPROM update time

  pinMode(ROTARYSWITCHPIN, INPUT_PULLUP);
  pinMode(RITPIN, INPUT_PULLUP);
#ifdef TXLED
  pinMode(13, OUTPUT);                // Controls on-board LED
#endif // TXLED
  oldRitState = ritState = LOW;       // Receiver incremental tuning state HIGH, LOW
  ritDisplaySwitch = 0;

  lcd.init();
  lcd.backlight();
  Splash();                           // Tell 'em we're here...

  idleTime = millis();                // Initialize idle time.

  PCICR |= (1 << PCIE2);              // Interrupt service code
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();
  pinMode(FQ_UD, OUTPUT);             // Tied to AD9850 board
  pinMode(W_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(RESET, OUTPUT);

  pulseHigh(RESET);
  pulseHigh(W_CLK);
  pulseHigh(FQ_UD);  // This pulse enables serial mode on the AD9850 - Datasheet page 12.

  ProcessSteps();
  DoRangeCheck();
  sendFrequency(currentFrequency);
}

void loop() {
  static int state = 1;      // 1 because of pull-ups on encoder switch
  static int oldState = 1;
  int flag;

  // Handle actions for transmit
  if (!digitalRead(RXTXPIN)) {
#ifdef RIT
    sendFrequency(currentFrequency); // Transmit, set VFO for no RIT offset.
#endif // RIT
#ifdef TXLED
    digitalWrite(13, HIGH); // Set Arduino on board LED to status of T/R pin to indicate transmit.
#endif // TXLED
#ifdef TXLCD
    lcd.setCursor(0, 0);  // Indicate transmit on LCD with *.
    lcd.print("*");
#endif // TXLCD
  } else {
#ifdef RIT
    sendFrequency(currentFrequency + ritOffset); // Receive, set VFO for RIT offset.
#endif // RIT
#ifdef TXLED
    digitalWrite(13, LOW);
#endif // TXLED
#ifdef TXLCD
    lcd.setCursor(0, 0);
    lcd.print(" ");
#endif // TXLCD
  }

  state = digitalRead(ROTARYSWITCHPIN);    // See if they pressed encoder switch
  ritState = !digitalRead(RITPIN);         // Also check RIT button

  if (state != oldState) {                 // Only if it's been changed...
    if (state == 1) {                      // Only adjust increment when HIGH
      if (incrementIndex < ELEMENTCOUNT(incrementTable) - 1) {    // Adjust the increment size
        incrementIndex++;
      } else {
        incrementIndex = 0;                // Wrap around to zero
      }
      currentFrequencyIncrement = incrementTable[incrementIndex];

      ProcessSteps();
    }
    oldState = state;
  }

  if (currentFrequency != oldFrequency) { // Are we still looking at the same frequency?
    NewShowFreq(0, 0);                    // Nope, so update display.
#ifdef BLANKING
    idleTime = millis();                  // Reset idle time
#endif
    flag = DoRangeCheck();
    if (flag == FREQOUTOFBAND) {          // Tell user if out of band; should not happen
      lcd.setCursor(0, 0);
      lcd.print("* Out of Band *");
    }
    sendFrequency(currentFrequency);      // Send frequency to chip
    oldFrequency = currentFrequency;
  }

  // Only update EEPROM if necessary... both time and currently stored freq.
  if (millis() - eepromStartTime > DELTATIMEOFFSET && markFrequency != currentFrequency) {
    writeEEPROMRecord(currentFrequency, READEEPROMFREQ);                  // Update freq
    writeEEPROMRecord((unsigned long) incrementIndex, READEEPROMINCRE);   // Update increment
    writeEEPROMRecord(ritOffset, READEEPROMRIT);                          // Update RIT offset
    eepromStartTime = millis();
    markFrequency = currentFrequency;                                     // Update EEPROM freq.
  }

#ifdef BLANKING
  if (ritState != oldRitState) {
    idleTime = millis(); // Reset idle time
  }

  // See if it is time to dim the backlight because system is idle.
  if (millis() - idleTime > BACKLIGHTTIMEOUT) {
    lcd.noBacklight(); // Turn off backlight
  } else {
    lcd.backlight(); // Turn on backlight
  }
#endif

  if (ritState == HIGH) {      // Change RIT?
#ifdef RIT
    DoRitDisplay();
#endif // RIT
    ritDisplaySwitch = 1;
  }
  if (oldRitState != ritState) {
    ProcessSteps();
    oldRitState = ritState;
    ritDisplaySwitch = 0;
  }
#ifdef VOLTMETER
  if (ritState != HIGH) {      // No room on display when showing RIT
    Voltmeter();
  }
#endif // VOLTMETER
}

void DoRitDisplay()
{
  char tempOffset[8];

  if (oldRitOffset == ritOffset && ritDisplaySwitch == 1)
    return;

  DisplayLCDLine(SPACES16, 1, 0);
  ltoa(ritOffset, tempOffset, 10);

  strcpy(temp, "Offset: ");
  strcat(temp, tempOffset);
  strcat(temp, " Hz");
  DisplayLCDLine(temp, 1, 0);

  oldRitOffset = ritOffset;
  ritDisplaySwitch = 1;
}


// Original interrupt service routine, as modified by Jack
ISR(PCINT2_vect) {
  unsigned char result = r.process();

  switch (result) {
    case 0:                                          // Nothing done...
      return;

    case DIR_CW:                                     // Turning Clockwise, going to higher frequencies
      if (ritState == LOW) {
        currentFrequency += currentFrequencyIncrement;
      } else {
        ritOffset += currentFrequencyIncrement / 10; // Use a slower tuning rate for RIT
      }
      break;

    case DIR_CCW:                                    // Turning Counter-Clockwise, going to lower frequencies
      if (ritState == LOW) {
        currentFrequency -= currentFrequencyIncrement;
      } else {
        ritOffset -= currentFrequencyIncrement / 10; // Use a slower tuning rate for RIT
      }
      break;

    default:                                          // Should never be here
      break;
  }
  if (currentFrequency >= VFOUPPERFREQUENCYLIMIT) {   // Upper band edge?
    currentFrequency = oldFrequency;
  }
  if (currentFrequency <= VFOLOWERFREQUENCYLIMIT) {   // Lower band edge?
    currentFrequency = oldFrequency;
  }
}


void sendFrequency(int32_t frequency) {
  /*
    Formula: int32_t adjustedFreq = frequency * 4294967295/125000000;

    Note the 125 MHz clock on 9850.  You can make 'slight' tuning
    variations here by adjusting the clock frequency. The constants
    factor to 34.359
  */
  int32_t freq = (int32_t) (((float) frequency * MYTUNINGCONSTANT));  // Redefine your constant if needed

  for (int b = 0; b < 4; b++, freq >>= 8) {
    tfr_byte(freq & 0xFF);
  }
  tfr_byte(0x000);   // Final control byte, all 0 for 9850 chip
  pulseHigh(FQ_UD);  // Done!  Should see output
}

// transfers a byte, a bit at a time, LSB first to the 9850 via serial DATA line
void tfr_byte(byte data)
{
  for (int i = 0; i < 8; i++, data >>= 1) {
    digitalWrite(DATA, data & 0x01);
    pulseHigh(W_CLK);   // After each bit sent, CLK is pulsed high
  }
}

/*****
  This method is used to format a frequency on the LCD display. The currentFrequency variable holds the display
  frequency.

  Return value:
    void
*****/
void DisplayLCDLine(const char *message, int row, int col)
{
  lcd.setCursor(col, row);
  lcd.print(message);
}

/*****
  This method is used to read a record from EEPROM. Each record is 4 bytes (sizeof(unsigned long)) and
  is used to calculate where to read from EEPROM.

  Argument list:
    int record                the record to be read. While tuning, it is record 0

  Return value:
    unsigned long            the value of the record,

*****/
unsigned long readEEPROMRecord(int record)
{
  int offset;
  union {
    byte array[4];
    unsigned long val;
  } myUnion;

  offset = record * sizeof(unsigned long);

  myUnion.array[0] = EEPROM.read(offset);
  myUnion.array[1] = EEPROM.read(offset + 1);
  myUnion.array[2] = EEPROM.read(offset + 2);
  myUnion.array[3] = EEPROM.read(offset + 3);

  return myUnion.val;
}

/*****
  This method is used to write the latest data to EEPROM. This routine
  is called every DELTATIMEOFFSET (default = 60 seconds). This is done
  because EEPROM can only be written/erased 100K times before it gets
  flaky.

  Argument list:
    unsigned long freq        the current frequency of VFO
    int record                the record to be written. While tuning, it is record 0

  Return value:
    void
*****/
void writeEEPROMRecord(unsigned long freq, int record)
{
  int offset;
  union {
    byte array[4];
    unsigned long val;
  } myUnion;

  myUnion.val = freq;
  offset = record * sizeof(unsigned long);

  EEPROM.write(offset, myUnion.array[0]);
  EEPROM.write(offset + 1, myUnion.array[1]);
  EEPROM.write(offset + 2, myUnion.array[2]);
  EEPROM.write(offset + 3, myUnion.array[3]);
}

/*****
  This method is used to format a frrequency on the LCD display. The currentFrequency variable holds the display
  frequency. This is kinda clunky...

  Return value:
    void
*****/
void NewShowFreq(int row, int col) {
  char part[10];
  dtostrf((float)currentFrequency, 7, 0, temp);

  strcpy(part, &temp[1]);
  strcpy(temp, "7.");
  strcat(temp, part);
  strcpy(part, &temp[5]);
  temp[5] = '.';
  strcpy(&temp[6], part);
  strcat(temp, " ");
  strcat(temp, MEGAHERTZ);
  lcd.setCursor(col, row);
  lcd.print(SPACES);
  lcd.setCursor(col + 2, row);
  lcd.print(temp);
}

/*****
  This method is used to see if the current frequency displayed on line 1 is within a ham band.
  The code does not allow you to use the band edge.

  Argument list:
    void

  Return value:
    int            0 (FREQINBAND) if within a ham band, 1 (FREQOUTOFBAND) if not
*****/
int DoRangeCheck()
{

  if (currentFrequency <= VFOLOWERFREQUENCYLIMIT || currentFrequency >= VFOUPPERFREQUENCYLIMIT) {
    return FREQOUTOFBAND;
  }
  // Set up some VFO band edges
  if (currentFrequency <= VFOLOWALLBUTEXTRA) {
    whichLicense = EXTRA;
  }
  if (currentFrequency > VFOLOWTECH && currentFrequency < VFOHIGHTECH) {
    whichLicense = TECH;
  }

  if ((currentFrequency >= VFOLOWALLBUTEXTRA && currentFrequency < VFOHIGHTECH) ||
      (currentFrequency > VFOGENERALLOWGAP && currentFrequency < VFOUPPERFREQUENCYLIMIT) ) {
    whichLicense = GENERAL;
  }

  if (ritState != HIGH) {      // No room on display when showing RIT
    ShowMarker(bandWarnings[whichLicense]);
  }
  return FREQINBAND;
}

/*****
  This method is used to display the current license type

  Argument list:
    char *c       // Pointer to the type of license as held in bandWarnings[]

  Return value:
    void
*****/
void ShowMarker(const char *c)
{
  lcd.setCursor(13, 1); // Altered in V1.0 of Voltmeter implementation
  lcd.print(c);
}

/*****
  This method is used to change the number of Hertz associated with one click of the rotary encoder. Ten step
  levels are provided and increasing or decreasing beyonds the array limits rolls around to the start/end
  of the array.

  Argument list:
    void

  Return value:
    void
*****/
void ProcessSteps()
{
  DisplayLCDLine(SPACES, 1, 0);                    // This clears the line

  strcpy(temp, incrementStrings[incrementIndex]);

  if (incrementIndex < 3) {
    strcat(temp, HERTZ);
  } else {
    strcat(temp, KILOHERTZ);
  }

  DisplayLCDLine(temp, 1, 0);
  if (ritState != HIGH) {      // No room on display when showing RIT
    ShowMarker(bandWarnings[whichLicense]);
  }
}

/*****
  This method simply displays a sign-on message

  Argument list:
    void

  Return value:
    void
*****/
void Splash()
{
  lcd.setCursor(0, 0);
  lcd.print("Arduino Forty9er");
  lcd.setCursor(0, 1);
  lcd.print(__DATE__);
  delay(SPLASHDELAY);
}

/*****
   This method is used to read the value from the R7, R8, C7 voltage
   divider circuit going to pin 26/A7 of the Arduino, process the
   requiredcalcul ations, then display the voltage on the LCD.
   by Hank Ellis K5HDE.
   V1.0 14 June 2016 Implementation of the basic voltmeter
   V1.1 23 June 2016 Fixed bug that blanks the voltmeter when the frequency step is changed via the encoder pushbutton
   V2.0 16 July 2016 Added voltage low and high cautions and warnings
*****/
#ifdef VOLTMETER
void Voltmeter()
{
  static boolean state = false; // Sets the state of the display
  static float input_voltage = 0.0; // The end result we want to display
  static unsigned long volt_last_update = millis();  // Record when the last voltage display was updated

#define TIME_LOOP 250          // Time between voltage display updates
#define VOLT_LOW_WARNING 10.0  // Voltage triggers for cautions and warnings, adjust as desired.
#define VOLT_LOW_CAUTION 11.0
#define VOLT_HIGH_CAUTION 13.8
#define VOLT_HIGH_WARNING 14.0

  if ((millis() - volt_last_update) > TIME_LOOP) // If the time since last update has elapsed
  {
    if (input_voltage <= VOLT_LOW_WARNING || input_voltage >= VOLT_HIGH_WARNING) // Is the voltage within the warning triggers?
    {
      if (state == false) // Blank the display
      {
        blank_volt_display();
      }
      else
      {
        input_voltage = volt_display(); // Turn it back on
      }
      state = !state; // Toggle the display state
      volt_last_update = millis(); // and update the last voltage update time
    }
    if ((millis() - volt_last_update) > (TIME_LOOP * 3)) // Since we didn't have a voltage warning a longer refresh time is acceptable
    {
      if (input_voltage <= VOLT_LOW_CAUTION || input_voltage >= VOLT_HIGH_CAUTION) // is the voltage within the caution triggers?
      {
        if (state == false) // Blank the display
        {
          blank_volt_display();
        }
        else // or
        {
          input_voltage = volt_display(); // Turn it back on
        }
        state = !state; // Toggle the display state
        volt_last_update = millis(); // and update the last voltage update time
      }
      else // If the voltage is within the normal range
      {
        input_voltage = volt_display(); // Display the voltage
        volt_last_update = millis(); // And update the last voltage update time
      }
    }
  }
}

float volt_display()
{
  float analog_value = 0.0; // Value from Arduino pin 26/A7
  float temp = 0.0; // Temporary number used in calculations
  float r7 = 20000.0; // Value of R7
  float r8 = 8000.0; // Arbitrary value assigned to R8. Intent is to provide a 3.5 voltage divider.
  float disp_voltage = 0.0;

  analog_value = analogRead(A7); // Read the value from the voltage divider on pin A7
  temp = (analog_value * 5.0) / 1023.0; // Convert it to a usable number
  disp_voltage = temp / (r8 / (r7 + r8)); // Formula of a voltage divider circuit

  lcd.setCursor(7, 1); // Display the result
  lcd.print(disp_voltage, 1);
  lcd.print("V ");

  return disp_voltage;
}

void blank_volt_display()
{
  lcd.setCursor(7, 1); // Set the cursor and print spaces
  lcd.print("      ");
}
#endif // VOLTMETER
