/*
  The heart of the code that manages the AD9850 was written by
  Richard Visokey AD7C - www.ad7c.com

 Modifications were made by Jack Purdum and Dennis Kidder:
   Rev 4.00:  Feb. 2, 2015
   Rev 5.00:  Jul. 8, 2015, Jack Purdum
   Rev 6.00:  Aug. 1, 2015, Jack Purdum

   Tested with Arduino IDE V-1.6.5
*/

#include <Rotary.h>   // From Brian Low: https://github.com/brianlow/Rotary
#include <EEPROM.h>   // Shipped with IDE
#include <Wire.h>     //         "

// Get the LCD I2C Library here:
// https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
// Move all *_I2C files into a new folder named LiquidCrystal_I2C
// in the Arduino library folder
#include <LiquidCrystal_I2C.h>

#define MYTUNINGCONSTANT     34.35977500000000    // Replace with your calculated TUNING CONSTANT. See article

#define SPACES      "                "
#define HERTZ       "Hz"
#define KILOHERTZ   "kHz"
#define MEGAHERTZ   "MHz"

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
#define SPLASHDELAY    4000               // Hold splash screen for 4 seconds

#define ROTARYSWITCHPIN   4               // Used by switch for rotary encoder
#define RITPIN            8               // Used by push button switch to toggle RIT
#define RXTXPIN          12               // When HIGH, the xcvr is in TX mode
#define RITOFFSETSTART  700L              // Offset for RIT

#define FREQINBAND        0               // Used with range checking
#define FREQOUTOFBAND     1

// ===================================== EEPROM Offsets and data ==============================================
#define READEEPROMFREQ        0       // The EEPROM record address of the last written frequency
#define READEEPROMINCRE       1       // The EEPROM record address of last frequency increment

#define DELTAFREQOFFSET      25       // Must move frequency more than this to call it a frequency change
#define DELTATIMEOFFSET   60000       // If not change in frequency within 1 minute, update the frequency

unsigned long markFrequency;          // The frequency just written to EEPROM
long eepromStartTime;                 // Set when powered up and while tuning
long eepromCurrentTime;               // The current time reading

// ============================ ISR variables: ======================================
volatile int_fast32_t currentFrequency;     // Starting frequency of VFO
volatile long currentFrequencyIncrement;
volatile long ritOffset;

// ============================ General Global Variables ============================
bool ritState;                    // Receiver incremental tuning state HIGH, LOW
bool oldRitState;

char temp[17];

int ritDisplaySwitch;             // variable to index into increment arrays (see below)
int incrementIndex = 0;

long oldRitOffset;
int_fast32_t oldFrequency = 1;    // variable to hold the updated frequency

//static const char *bandWarnings[] = {"Extra  ", "Tech   ", "General"};
static const char *bandWarnings[]   = {"Ext", "Tec", "Gen"};
static int whichLicense;
static const char *incrementStrings[] = {"10", "20", "100", ".5", "1", "2.5", "5", "10", "100" }; // These two align
static long incrementTable[]   = { 10, 20, 100, 500, 1000, 2500, 5000, 10000, 100000 };
static long memory[]           = { VFOLOWERFREQUENCYLIMIT, VFOUPPERFREQUENCYLIMIT };

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
  currentFrequencyIncrement = incrementTable[incrementIndex];     // Store working freq variables
  markFrequency = currentFrequency;
  eepromStartTime = millis();                                     // Need to keep track of EEPROM update time

  pinMode(ROTARYSWITCHPIN, INPUT_PULLUP);
  pinMode(RITPIN, INPUT_PULLUP);
  pinMode(RXTXPIN, INPUT_PULLUP);     // Start in RX mode

  oldRitState = ritState = LOW;       // Receiver incremental tuning state HIGH, LOW
  ritOffset = RITOFFSETSTART;         // Default RIT offset
  ritDisplaySwitch = 0;

  lcd.init();
  lcd.backlight();
  Splash();                           // Tell 'em were here...

  PCICR |= (1 << PCIE2);              // Interrupt service code
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();
  pinMode(FQ_UD, OUTPUT);             // Tied to AD9850 board
  pinMode(W_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(RESET, OUTPUT);

  pulseHigh(RESET);
  pulseHigh(W_CLK);
  pulseHigh(FQ_UD);  // this pulse enables serial mode on the AD9850 - Datasheet page 12.`

  ProcessSteps();
  DoRangeCheck();
  ShowMarker(bandWarnings[whichLicense]);
  sendFrequency(currentFrequency);
}

void loop() {
  static int state = 1;      // 1 because of pull-ups on encoder switch
  static int oldState = 1;

  int flag;

  state = digitalRead(ROTARYSWITCHPIN);    // See if they pressed encoder switch
  ritState = digitalRead(RITPIN);          // Also check RIT button

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

    flag = DoRangeCheck();
    if (flag == FREQOUTOFBAND) {          // Tell user if out of band; should not happen
      lcd.setCursor(0, 0);
      lcd.print("* Out of Band *");
    }
    sendFrequency(currentFrequency);      // Send frequency to chip
    oldFrequency = currentFrequency;
  }

  eepromCurrentTime = millis();
  // Only update EEPROM if necessary...both time and currently stored freq.
  if (eepromCurrentTime - eepromStartTime > DELTATIMEOFFSET && markFrequency != currentFrequency) {
    writeEEPROMRecord(currentFrequency, READEEPROMFREQ);                  // Update freq
    writeEEPROMRecord((unsigned long) incrementIndex, READEEPROMINCRE);   // Update increment
    eepromStartTime = millis();
    markFrequency = currentFrequency;                                     // Update EEPROM freq.
  }

  if (ritState == HIGH) {      // Change RIT?
    //DoRitDisplay();
    ritDisplaySwitch = 1;
  }
  if (oldRitState != ritState) {
    ProcessSteps();
    oldRitState = ritState;
    ritDisplaySwitch = 0;
  }
  Voltmeter();
}

void DoRitDisplay()
{
  char tempOffset[8];

  if (oldRitOffset == ritOffset && ritDisplaySwitch == 1)
    return;
  DisplayLCDLine(SPACES, 1, 0);
  ltoa(ritOffset, tempOffset, 10);

  strcpy(temp, "Offset: ");
  strcat(temp, tempOffset);

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
        ritOffset += RITOFFSETSTART;
      }
      break;

    case DIR_CCW:                                    // Turning Counter-Clockwise, going to lower frequencies
      if (ritState == LOW) {
        currentFrequency -= currentFrequencyIncrement;
      } else {
        ritOffset -= RITOFFSETSTART;
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
  This method is used to format a frequency on the lcd display. The currentFrequency variable holds the display
  frequency.

  Argument list:
    void

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

  CAUTION:  Record 0 is the current frequency while tuning, etc. Record 1 is the number of stored
            frequencies the user has set. Therefore, the stored frequencies list starts with record 23.
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
  This method is used to test and perhaps write the latest frequency to EEPROM. This routine is called
  every DELTATIMEOFFSET (default = 10 seconds). The method tests to see if the current frequency is the
  same as the last frequency (markFrequency). If they are the same, +/- DELTAFREQOFFSET (drift?), no
  write to EEPROM takes place. If the change was larger than DELTAFREQOFFSET, the new frequency is
  written to EEPROM. This is done because EEPROM can only be written/erased 100K times before it gets
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

  if (abs(markFrequency - freq) < DELTAFREQOFFSET) {  // Is the new frequency more or less the same as the one last written?
    return;                                           // the same as the one last written? If so, go home.
  }
  myUnion.val = freq;
  offset = record * sizeof(unsigned long);

  EEPROM.write(offset, myUnion.array[0]);
  EEPROM.write(offset + 1, myUnion.array[1]);
  EEPROM.write(offset + 2, myUnion.array[2]);
  EEPROM.write(offset + 3, myUnion.array[3]);
  markFrequency = freq;                               // Save the value just written
}

/*****
  This method is used to format a frrequency on the lcd display. The currentFrequqncy variable holds the display
  frequency. This is kinda clunky...

  Argument list:
    void

  Return value:
    void
*****/
void NewShowFreq(int row, int col) {
  char part[10];
  dtostrf( (float) currentFrequency, 7,0, temp);

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
  ShowMarker(bandWarnings[whichLicense]);

  return FREQINBAND;
}

/*****
  This method is used to display the current license type

  Argument list:
    char *c       // pointer to the type of license as held in bandWarnings[]

  Return value:
    void
*****/
void ShowMarker(const char *c)
{
  lcd.setCursor(13, 1);
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
  ShowMarker(bandWarnings[whichLicense]);
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
This method is used to read the value from the R7, R8, C7 voltage divider circuit going to pin 26/A7 of the Arduino, process the required calculations, then display the voltage on the LCD.
14 June 2016 by Hank Ellis K5HDE.
*****/

void Voltmeter()
{
  float input_voltage = 0.0; // The end result we want to display
  float temp = 0.0;          // Temporary number used in calculations
  float r7 = 20000.0;        // Value of R7
  float r8 = 8000.0;         // Arbitrary value assigned to R8. Intent is to provide a 3.5 voltage divider.

  static unsigned long volt_last_update = millis();  // Record when the last voltage display was updated
  static unsigned long DELTA_TIME_LOOP = 1000;       // Time between voltage display updates

  //#define VOLT_LOW_WARNING  = 11.3 // For future expansion
  //#define VOLT_LOW_CAUTION  = 11.5
  //#define VOLT_HIGH_CAUTION = 12.5
  //#define VOLT_HIGH_WARNING = 12.8
 
  if ((millis() - volt_last_update) > DELTA_TIME_LOOP) // If the time since last update has elapsed
  {
    int analog_value = analogRead(A7);       // Read the value from the voltage divider on pin A7
    temp = (analog_value * 5.0) / 1023.0;    // Convert it to a usable number
    input_voltage = temp / (r8 / (r7 + r8)); // Formula of a voltage divider circuit

    lcd.setCursor(7, 1); // Display the result
    lcd.print(input_voltage, 1);
    lcd.print("V");

    volt_last_update = millis(); // Set the update time
  }
}
