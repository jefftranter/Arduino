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

double Start_Freq =  2.0e6;  // Start frequency in MHz
double End_Freq   =  23.0e6; // End frequency in MHz
double Step_Freq  =  25000;  // Step frequency in Hz
double current_freq;
int FwdOffSet;
int RevOffSet;
int FwdSCVal = 30;     // Initially set to 1; then set to reading found when antenna leg of bridge is shorted; Diode sensitivity compensation
int RevSCVal = 40;     // Initially set to 1; then set to reading found when antenna leg of bridge is shorted; Diode sensitivity compensation
int FwdOpAmpGain = 92; // Initially set to 1; then set to FWD reading found when cathodes of D1 and D2 are shorted together; (Op Amp Gain loop compensation)
int RevOpAmpGain = 88; // Initially set to 1; then set to REV reading found when cathodes of D1 and D2 are shorted together; (Op Amp Gain loop compensation)

// Small signal correction table [array] 
int CrtdVal[] = {
  0, 0, 2, 4, 7, 9, 12, 14, 16, 19, 22, 25, 28, 31, 33, 35,36, 37, 39, 40, 41, 42, 43, 44, 44, 46, 47, 49, 50, 52, 53, 54, 55, 57, 58, 
  59, 60, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 79, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97,
  98, 99, 100, 101, 102, 103, 104, 105, 105, 106, 107, 108, 109, 110, 111, 111, 112, 113, 114, 114, 115, 116, 117, 117, 118, 119, 120, 120, 121, 122, 122,
  123, 124, 124, 125, 126, 126, 127, 128, 129, 130, 131, 132, 133, 133, 134, 135, 136, 137, 138, 139, 139, 140, 141, 142, 143, 143, 144, 145, 146, 147, 147,
  148, 149, 150, 151, 151, 152, 153, 154, 154, 155, 156, 157, 157, 158, 159, 160, 160, 161, 162, 162, 163, 164, 164, 165, 166, 166, 167, 168, 168, 169, 170,
  171, 172, 172, 173, 174, 175, 176, 176, 177, 178, 179, 179, 180, 181, 182, 182, 183, 184, 185, 185, 186, 187, 187, 188, 189, 190, 190, 191, 192, 193, 193,
  194, 195, 196, 196, 197, 198, 198, 199, 200, 200, 201, 202, 203, 203, 204, 205, 205, 206, 207, 207, 208, 209, 209, 210, 211, 211, 212, 213, 213, 214, 215,
  215, 216, 216, 217, 218, 218, 219, 220, 220, 221, 221, 222, 223, 223, 224, 225, 225, 226, 226, 227, 228, 228, 229, 229, 230, 230, 231, 232, 232, 233, 233,
  234, 234, 235, 235
  };

SoftwareSerial mySerial(2, 3); // RX, TX
    
 // Transfers a byte, a bit at a time, LSB first to the 9850 via serial DATA line
void tfr_byte(byte data)
{
    for (int i = 0; i < 8; i++, data >>= 1) {
        digitalWrite(DATA, data & 0x01);
        pulseHigh(W_CLK); // After each bit sent, CLK is pulsed high
    }
}

 // Frequency calculation from datasheet page 8 = <sys clock> * <frequency tuning word> / 2^32
void sendFrequency(double frequency) {
    int32_t freq = frequency * 4294967295 / 125000000;  // Note 125 MHz clock on 9850
    for (int b = 0; b < 4; b++, freq >>= 8) {
        tfr_byte(freq & 0xFF);
    }
    tfr_byte(0x000);   // Final control byte, all 0 for 9850 chip
    pulseHigh(FQ_UD);  // Done!  Should see output
}

void setup() {
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

void loop() {
    int incomingByte = 0;   // For incoming serial data
    bool RunCurve = true;
  
    if (mySerial.available() > 0) {
        // Read the incoming byte:
        while (mySerial.available()) {
            incomingByte = mySerial.read();
            mySerial.println(incomingByte);
        }
        RunCurve = !RunCurve;
    } else {
        //mySerial.println("Hit 'enter' Key to Start Scan");
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

//*******************************************************//
bool PrintNextPoint(bool RunCurve){
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
    REV = CorrectReading(REV); // Now using table apply Small Signal correction value
    
    FWD = (RevSCVal * FWD) / FwdSCVal; // Apply "short circuit" offset
    FWD = CorrectReading(FWD); // Now using table apply Small Signal correction value
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

double CorrectReading(float ReadVal) {
    if (ReadVal > 70)
        return 0.8 * ReadVal + 57;

    if (ReadVal < 13)
        return 3.6 * (ReadVal * ReadVal) / 15;

    float CalcVal = 1.1 * (8 + (2.1 * ReadVal) - ((ReadVal * ReadVal) * 10.7 / 1000));
    return CalcVal;
}
