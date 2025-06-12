/*
 * Arduino and AD8950-based Antenna Analyzer.
 *
 * Original by J M Harvey. See https://github.com/jmharvey1/DDS_AD9850_AntennaAnalyzer
 * Modified by Jeff Tranter <tranter@pobox.com>
 *
 */

/*

To reset serial LCD to defaults:
- jumper GPIO pins 3 and 4 (if settings messed up)
- baud rate 19200: 254, 57, 51
- backlight 255: 254 66 0
- contrast 128: 254 145 128

serial.write(254); serial.write(57); serial.write(51);
serial.write(254); serial.write(66); serial.write(0);
serial.write(254); serial.write(145); serial.write(128);

*/

#include <SoftwareSerial.h>

#define W_CLK 8       // Pin 8  - connect to AD9850 module word load clock pin (CLK)
#define FQ_UD 9       // Pin 9  - connect to freq update pin (FQ)
#define DATA 10       // Pin 10 - connect to serial data load pin (DATA)
#define RESET 11      // Pin 11 - connect to reset pin (RST)

#define pulseHigh(pin) { digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }

double Start_Freq =  7000000; // Start frequency in Hz
double End_Freq   =  7300000; // End frequency in Hz
double Step_Freq  =    10000; // Step frequency in Hz
double current_freq;          // Current frequency
int Readings = 50;            // Number of readings to average over.

SoftwareSerial mySerial(2, 3); // RX, TX

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
    pulseHigh(FQ_UD);  // This pulse enables serial mode - datasheet page 12 figure 10
    current_freq = Start_Freq;

    mySerial.begin(19200);
    Serial.begin(9600);
 }

void loop()
{
    bool done = false;
    Serial.println("Start Scan");
    delay(1000);
    while (!done) {
         done = PrintNextPoint();
    }
}

bool PrintNextPoint()
{
    double FWD = 0.0;
    double REV = 0.0;
    double VSWR;
    double EffOhms;
    if (current_freq > End_Freq) {
        current_freq = Start_Freq;
        return true;
    }
    sendFrequency(current_freq);  // Frequency
    delay(100);
    // Read the forward and reverse voltages. Average over Readings readings.
    for (int i = 0; i < Readings; i++) {
         REV += analogRead(A0);
         FWD += analogRead(A1);
    }

    REV = REV / (double)Readings; // Compensate for reading multiple times.
    FWD = FWD / (double)Readings;
    //Serial.print("FWD="); Serial.print(FWD);
    //Serial.print(" REV="); Serial.println(REV);
    REV = sqrt(REV);
    FWD = sqrt(FWD);

    if (REV >= FWD) {
        // To avoid a divide by zero or negative VSWR then set to max 999
        VSWR = 999;
    } else {
        // Calculate VSWR
        VSWR = (FWD + REV) / (FWD - REV);
    }
    if (FWD > 24) // Experimentally found this was point of lowest/matched SWR
        EffOhms = VSWR * 50.0;
    else
        EffOhms = 50.0 / VSWR;

    // Send current line back to PC over serial bus.
    // e.g.
    // Freq: 7160 kHz Fwd: 24.21 Rev: 5.92 SWR: 1.65 Ohms: 82.39
    Serial.print("Freq: ");
    Serial.print(int(current_freq / 1000));
    Serial.print(" kHz Fwd: ");
    Serial.print(FWD);
    Serial.print(" Rev: ");
    Serial.print(REV);
    Serial.print(" SWR: ");
    Serial.print(VSWR);
    Serial.print(" Ohms: ");
    Serial.print(EffOhms);
    Serial.println("");

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

    mySerial.print("\nFreq ");
    mySerial.print(int(current_freq / 1000));
    mySerial.print(" kHz\nSWR ");
    mySerial.print(VSWR, 1);
    mySerial.print(" ");
    mySerial.print(EffOhms,1);
    mySerial.print("\xF4");

    current_freq += Step_Freq;
    return false;
}
