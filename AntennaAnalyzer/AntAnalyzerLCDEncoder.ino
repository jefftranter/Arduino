/*
 * Arduino and AD8950-based Antenna Analyzer.
 *
 * Original by J M Harvey. See https://github.com/jmharvey1/DDS_AD9850_AntennaAnalyzer
 * Modified by Jeff Tranter <tranter@pobox.com>
 *
 */

#include <SoftwareSerial.h>
#include <Encoder.h>

#define W_CLK 8             // Pin 8  - connect to AD9850 module word load clock pin (CLK)
#define FQ_UD 9             // Pin 9  - connect to freq update pin (FQ)
#define DATA 10             // Pin 10 - connect to serial data load pin (DATA)
#define RESET 11            // Pin 11 - connect to reset pin (RST)
#define BUTTON 12           // Pin 12 - connected to encoder push switch

#define pulseHigh(pin) { digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }

const int readings = 50;    // Number of readings to average over
double frequency = 7000000; // Current frequency in Hertz
long step = 1000000;        // Current step in Hertz
int position;               // Encoder position
int lastPosition;           // Previous encoder position
double lastVSWR = 0;        // Previous SWR value
double lastEffOhms = 0;     // Previous Ohms value
bool changed;               // Set when results changed and display needs to be updated

SoftwareSerial lcd(6, 7);   // RX, TX pins for serial LCD
Encoder encoder(2, 3);      // Rotary encoder

// Transfers a byte, a bit at a time, LSB first to the 9850 via serial DATA line.
void tfr_byte(byte data)
{
    for (int i = 0; i < 8; i++, data >>= 1) {
        digitalWrite(DATA, data & 0x01);
        pulseHigh(W_CLK); // After each bit is sent, CLK is pulsed high
    }
}

// Frequency calculation from datasheet page 8 = <sys clock> * <frequency tuning word> / 2^32
void sendFrequency(double frequency)
{
    int32_t freq = frequency * 0xffffffff / 125000000;  // Note 125 MHz clock on 9850
    for (int b = 0; b < 4; b++, freq >>= 8) {
        tfr_byte(freq & 0xFF);
    }
    tfr_byte(0x000);   // Final control byte, all 0 for 9850 chip
    pulseHigh(FQ_UD);  // Done! Should see output
}

void setup()
{
    // Configure Arduino data pins for output
    pinMode(FQ_UD, OUTPUT);
    pinMode(W_CLK, OUTPUT);
    pinMode(DATA, OUTPUT);
    pinMode(RESET, OUTPUT);

    pulseHigh(RESET);
    pulseHigh(W_CLK);
    pulseHigh(FQ_UD);     // This pulse enables serial mode - datasheet page 12 figure 10

    lcd.begin(19200);    // Serial LCD
    //Serial.begin(9600);  // Serial port to computer

    pinMode(BUTTON, INPUT_PULLUP); // Encoder switch, enable pullup

    position = encoder.read(); // Get initial encoder position
    lastPosition = position;

    lcd.print("\fAntenna Analyzer"); // Startup message
    lcd.print(__DATE__);
    delay(1000);
 }

void loop()
{
    changed = false;

    // See if encoder button pressed. If so, change frequency step.
    if (digitalRead(BUTTON) == 0) {
        switch (step) {
        case 1000:
            step = 10000;
            break;
        case 10000:
            step = 100000;
            break;
        case 100000:
            step = 1000000;
            break;
        case 1000000:
            step = 1000;
            break;
        default:
            step = 1000;
            break;
        }
        // Display new step
        lcd.print("\fStep: ");
        lcd.print(step/1000);
        lcd.print(" kHz");
        changed = true;
        delay(1000);
    }

    // See if knob was turned. If so, adjust frequency up or down by a step.
    // Look for change of at least two to make it less sensitive.
    position = encoder.read();
    if (abs(position - lastPosition) >= 2) {
        if (position > lastPosition) {
            frequency += step;
        } else {
            frequency -= step;
        }
        lastPosition = position;
        changed = true;
     }

    // Ensure frequency is in range 0 Hz to 40 MHz
    frequency = constrain(frequency, 0, 40000000);

    // Calculate and display SWR
    PrintSWR(frequency);
}

void PrintSWR(double frequency)
{
    double FWD = 0.0;
    double REV = 0.0;
    double VSWR;
    double EffOhms;

    sendFrequency(frequency);
    delay(100);
    // Read the forward and reverse voltages. Average over Readings readings.
    for (int i = 0; i < readings; i++) {
        REV += analogRead(A0);
        FWD += analogRead(A1);
    }

    REV = REV / (double)readings; // Compensate for reading multiple times.
    FWD = FWD / (double)readings;
    REV = sqrt(REV);
    FWD = sqrt(FWD);

    if (REV >= FWD) {
        // To avoid a divide by zero or negative VSWR then set to max 999
        VSWR = 999;
    } else {
        // Calculate VSWR
        VSWR = (FWD + REV) / (FWD - REV);
    }

    // Only refresh display if reading changed.
    if (labs(VSWR - lastVSWR) > 0.01) {
        changed = true;
        lastVSWR = VSWR;
    }

    if (FWD > 24) // Experimentally found this was point of lowest/matched SWR
        EffOhms = VSWR * 50.0;
    else
        EffOhms = 50.0 / VSWR;

    // Only refresh display if reading changed.
    if (labs(EffOhms - lastEffOhms) > 0.01) {
        changed = true;
        lastEffOhms = EffOhms;
    }
    // Send current line back to PC over serial bus.
    // e.g.
    // Freq: 7160 kHz Fwd: 24.21 Rev: 5.92 SWR: 1.65 Ohms: 82.39
    //Serial.print("Freq: ");
    //Serial.print(int(frequency / 1000));
    //Serial.print(" kHz Fwd: ");
    //Serial.print(FWD);
    //Serial.print(" Rev: ");
    //Serial.print(REV);
    //Serial.print(" SWR: ");
    //Serial.print(VSWR);
    //Serial.print(" Ohms: ");
    //Serial.print(EffOhms);
    //Serial.println("");

    // Use these lines instead of above if you want CSV output of frequency, FWD, REV, SWR, ohms.
    // e.g. 7.160,24.21,5.92,1.65,82.39
    //Serial.print(current_freq / 1000000);
    //Serial.print(",");
    //Serial.print(FWD);
    //Serial.print(",");
    //Serial.print(REV);
    //Serial.print(",");
    //Serial.print(VSWR);
    //Serial.print(",");
    //Serial.println(EffOhms);

    // Display on 2x16 serial LCD
    // Freq 7050 kHz
    // SWR 1.6 82.3Ω
    if (changed) {
        lcd.print("\fFreq ");
        lcd.print(long(frequency / 1000));
        lcd.print(" kHz\nSWR ");
        lcd.print(VSWR, 1);
        lcd.print(" ");
        lcd.print(EffOhms,1);
        lcd.print("\xF4"); // Ohms symbol
    }
}
