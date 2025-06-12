/*

  Example Arduino sketch to control a PE4302 or PE4304 attenuator
  chip. The code has not been tested.

  Jeff Tranter <tranter@pobox.com> 2 Mar 2017

 */

// Pins
int LE = 2;
int CLOCK = 3;
int DATA = 4;

// The setup routine runs once when you press reset.
void setup() {

  //Serial.begin(9600);

  // Initialize pin as outputs.
  pinMode(LE, OUTPUT);
  pinMode(CLOCK, OUTPUT);
  pinMode(DATA, OUTPUT);
}

// The loop routine runs over and over again forever.
void loop()
{
  int value;   // Attenuation value, 0 through 63.
  int ms = 1000; // How log to pause before next value (in msec)

  // Step though each attenuation level pausing for 1 second at each
  // value.
  for (value = 0; value <= 63; value++) {
    //Serial.print(value); Serial.println(" dB");
    setAttenuator(value);
    delay(ms); // wait for a second before repeating
  }

  for (value = 63; value >= 0; value--) {
    //Serial.print(value); Serial.println(" dB");
    setAttenuator(value);
    delay(ms); // wait for a second before repeating
  }
}

/*

  Set attenuation level of PE4302 or PE4304 Digital Step Attenuator in
  serial mode. Level is in range 0 to 63 dB. Do not call this at a
  rate faster than 25 KHz (every 0.05 msec). Should work with other
  PE430x series chips with minor changes. The code runs slowly enough
  that we do not need to add any delays.

*/

void setAttenuator(int value)
{
  int level; // Holds level of DATA line when shifting

  // Check for value range of input.
  if (value < 0 || value > 63)
    return;

  // Initial state
  digitalWrite(LE, LOW);
  digitalWrite(CLOCK,LOW);

  for (int bit = 5; bit >= 0; bit--) {
    level = ((value << 1) >> bit) & 0x01; // Level is value of bit

    digitalWrite(DATA, level); // Write data value
    digitalWrite(CLOCK, HIGH); // Toggle clock high and then low
    digitalWrite(CLOCK, LOW);
  }

  digitalWrite(LE, HIGH); // Toggle LE high to enable latch
  digitalWrite(LE, LOW);  // and then low again to hold it.
}
