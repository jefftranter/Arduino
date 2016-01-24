#include <SoftwareSerial.h>

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

/* 
 * Arduino and AD8950-based Antenna Analyzer.
 *
 * Original by  J M Harvey. See https://github.com/jmharvey1/DDS_AD9850_AntennaAnalyzer
 * Modified by Jeff Tranter <tranter@pobox.com>
 */
 
#define W_CLK 8       // Pin 8  - connect to AD9850 module word load clock pin (CLK)
#define FQ_UD 9       // Pin 9  - connect to freq update pin (FQ)
#define DATA 10       // Pin 10 - connect to serial data load pin (DATA)
#define RESET 11      // Pin 11 - connect to reset pin (RST)
 
#define pulseHigh(pin) { digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }

double Start_Freq =  2.0e6;  // Start frequency in Hz
double End_Freq   =  23.0e6; // End frequency in Hz
double Step_Freq  =  25000;  // Step frequency in Hz
double current_freq;
int FwdOffSet;
int RevOffSet;
int FwdSCVal = 30;     // Set to forward reading found when antenna leg of bridge is shorted; Diode sensitivity compensation.
int RevSCVal = 40;     // Set to reverse reading found when antenna leg of bridge is shorted; Diode sensitivity compensation.
int FwdOpAmpGain = 92; // Set to forward reading found when cathodes of D1 and D2 are shorted together; (Op Amp Gain loop compensation).
int RevOpAmpGain = 88; // Set to reverse reading found when cathodes of D1 and D2 are shorted together; (Op Amp Gain loop compensation).

SoftwareSerial mySerial(2, 3); // RX, TX
    
// Transfers a byte, a bit at a time, LSB first to the 9850 via serial DATA line
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
    pulseHigh(FQ_UD);  // Done!  Should see output
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
    pulseHigh(FQ_UD);  // This pulse enables serial mode - Datasheet page 12 figure 10
    current_freq = Start_Freq;
    mySerial.begin(19200);
 }

void loop()
{
    int incomingByte = 0;   // For incoming serial data
    bool RunCurve = true;
  
    if (mySerial.available() > 0) {
        // Read the incoming byte:
        while (mySerial.available()) {
            incomingByte = mySerial.read();
            //mySerial.println(incomingByte);
        }
        RunCurve = !RunCurve;
    } else {
        sendFrequency(1.0);  // Set AD9850 output to 1 Hz
        delay(200);
        RevOffSet = analogRead(A0);
        FwdOffSet = analogRead(A1);
        //delay(2000);
    }

    while (RunCurve) {
         RunCurve = PrintNextPoint(RunCurve);
    }
}

bool PrintNextPoint(bool RunCurve)
{
    double FWD = 0.0;
    double REV = 0.0;
    double VSWR;
    double EffOhms;
    if (current_freq > End_Freq) {
        current_freq = Start_Freq;
        RunCurve = !RunCurve;
        mySerial.println("Scan Complete");
        return RunCurve; 
    }
    sendFrequency(current_freq);  // Frequency
    delay(100);
    // Read the forawrd and reverse voltages
    for (int i = 0; i < 70; i++) {
         REV += (analogRead(A0) - RevOffSet);
         FWD += (analogRead(A1) - FwdOffSet);
    }
    REV = REV / 70.0;
    FWD = FWD / 70.0;    
    REV = (FwdOpAmpGain * REV) / RevOpAmpGain; // Apply Op Amp Gain loop compensation
    REV = CorrectReading(REV); // Now using table apply small signal correction value
    
    FWD = (RevSCVal * FWD) / FwdSCVal; // Apply "short circuit" offset
    FWD = CorrectReading(FWD); // Now using table apply small signal correction value
    if (REV >= FWD) {
        // To avoid a divide by zero or negative VSWR then set to max 999
        VSWR = 999;
    } else {
        // Calculate VSWR
        VSWR = ((FWD + REV) / (FWD - REV));
    }
    if (FWD >= 115)
        EffOhms = VSWR * 50.0; // FWD >= 94
    else
        EffOhms = 50.0 / VSWR;

    // Display on 2x16 serial LCD
    //   7050 KHz 5.65
    // XXXXXXXXXXXXXXXX
    // Freq 7050 kHz
    // SWR 5.65 R 2000

    mySerial.print("\nFreq ");
    mySerial.print(int(current_freq / 1000));
    mySerial.print(" kHz\nSWR ");
    mySerial.print(VSWR, 1);
    mySerial.print(" R ");
    mySerial.print(EffOhms,1);
    //delay(100);

    current_freq += Step_Freq;
    return RunCurve;
}

double CorrectReading(float ReadVal)
{
    if (ReadVal > 70)
        return 0.8 * ReadVal + 57;

    if (ReadVal < 13)
        return 3.6 * (ReadVal * ReadVal) / 15;

    float CalcVal = 1.1 * (8 + (2.1 * ReadVal) - ((ReadVal * ReadVal) * 10.7 / 1000));
    return CalcVal;
}
