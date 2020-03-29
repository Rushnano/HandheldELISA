/*
  February 14, 2017 (Started as NC1194_Potentiostat.ino)

  Updated March 23, 2017- largely to address hardware design changes on new design iteration NC1194 Potentiostat V3. These include use of i2c octal switch ADG715 for 
  potentiostat signal switching (A0 and A1 on ADG715 pulled to ground, so i2c address is 0x48). Network analyzer AD5933 address is 0x0D. Other changes are use of 
  low dropout 3.3V regulator AP7365 for wireless devices (high side switching transistor of TPS 62172was not able to deliver required current from single cell Lithium battery),
  corrected i2c pinout for ESP8266 (GPIO5 for SCL, GPIO4 for SDA), and reconfigured potentiostat network (error made in earlier designs).

  Updated dates up to about April 24, 2017- include code to read and communicate temperature, and impedance register values from AD5933 for given frequency, as well as
  current and voltage applied through transimpedance amplifier...

  Updated (to version 04_Exp) on April 30, 2017, for testing various functionality necessary for development of more fully developed Android GUI.

  Changed name to ABE-Stat_1_0_03 (previous versions in summer 2017) on August 20, 2017, mostly to run different basic measurement features of NC1194 Potentiostat. Version 1_0_03 is the
  first to include specific analytical methods (i.e. starting with cyclic voltammetry and electrochemical impedance spectroscopy); ABE-Stat_1_0_04 first version to implement EIS; also polishing
  up CV...

  ABE-Stat 1_0_5 started September 8, 2017- building on fully functional ABE-Stat1_0_04 which has calibration routines including for battery, EIS functions using AD5933 (from 5 kHz to 100 kHz),
  CV functions up to 0.2 scan rate. Adding EIS functionality from 0.1Hz to about 100 Hz using signals generated entirely from DAC5061, and DFT of TIA samples from ADS1220 (in this case running 
  in turbo mode at highest data rate 2000 sps...

  ABE-Stat 1_0_06 started September 14, 2017; admittance vs frequency calibrations change with different AD5933 settings (i.e. PGAx5GainSetting, excitationCode, and TIAGainCode;
  so update calibration routines to calibrate admittance magnitude (scaled by load resistor from calibration
  Version 06_worked on through September 26 2017 to include open circuit configuration and open circuit voltage measurement (i.e. pH electrode); upgraded this version through
  September 26 to include calibration for every setting of AD5933, and all basic interfaces with Android ABE-Stat app through Cyclic Voltammetry and Electrochemical
  Impedance spectroscopy, including for low frequencies using ADC (ADS1220) for customized single frequency DFT...

  ABE-Stat_1_0_07 started September 27, 2017; simple check for TIA saturation not always effective (at saturation amplifier inputs violate ideality(?) and the observed TIA output voltage 
  is not always close to positive analog rail value (i.e. in some instances is lower than 1 V)- so updated resistance measurement, current measurements, and EIS measurements
  to confirm linearity of a result at a different setting before accepting a value... Also, added methods to conduct Differential Pulse Voltammetry.
  Also included a firmware number to report back to Android... Also tried to check all calculated values for divide by 0 to prevent "nan" or "inf" being written to Android when it expects
  a parsable number string...

  ABE-Stat_1_0_08 October 26; add code to shut off power after 1 minute idle with no bluetooth connection.
  ABE-Stat_1_0_09 November 3, 2017; blink LED a few times and turn off if battery is discharged; corrected EEPROM_Int_Write so that 16 bit value gets written to two 8 bit blocks (previously LSB overwrites MSB)

  ABE-Stat_1_0_10 December 15, 2017, turn off WiFi at startup, and keep off to suppress noise; also increase update rates for CV to try to smooth applied potential

  ABE-Stat_1_01_00 started January 19, 2018, update functions to use external clock 250kHz clock with AD5933 for lower frequency EIS; update requires updated board design ABE-Stat_1.0.01
  (previous versions worked with NC-1194_Potentiostat). Need to add additional EEPROM and calibration assignments to calibrate device operating over new frequencies and clock,
  and ABE-Stat_DigitalConfigurations (i.e. !O12 controls power enable to AD5933 and external clock- leave off for other voltammetric applications to limit noise).
  Also update DigitalConfigurations to use ADG715 S02 to make biasing amplifier a voltage follower for open circuit configuration (don't allow unpredictable leakage from drifting
  output). (used for preliminary testing of EIS with MCLK external clock, and also some troubleshooting to improve LED modulation for indicating charge state)

  ABE-Stat_1_0_01 finished preliminary function updates to enable rough characterization of AD5933 with slower clock (changed frequency argument to double data type,
  fixed bug where device gets stuck in infinite loop in TestImpedance(), and also checked for repeating admittance magnitudes (sign that AD5933 no longer reading commands correctly;
  situation is addressed by cycling power to AD5933 and repeating measurement.
  
  ABE-Stat_1_01_02 started March 13, 2018, used to test EIS characteristics for different passive component networks (mostly update EIS Analysis for new 

  ABE-Stat_1_01_02_700HzTest March 14, to compare AD5933 Vout / synthesizer signal to signal applied at reference pin to determine source of signal distortions near 700 Hz...
  To facilitate oscilloscope measurement, long delays are embedded in TestImpedance function to ensure signals are generated for long periods of time...

  ABE-Stat_1_01_03_500HzTest started May 9, 2018; to work with hardware version ABE-Stat 1_0_02, (separate power controller on O13 for AD5933 external clock); calibration
  indicates ~500Hz harmonic distortion in signal...
  
  Shift register outputs (2x serialized 74HC595, O00 - O15):
      O00 74LVG1G53 switch1 signal (!CS to !CS1 (ADS1220/ "0") or !CS2 (AD5061/ "1")
      O01 74LVG1G53 switch1 !enable (!CS1 && !CS2 pulled high if O01 high)
      O02 74LVG1G53 switch2 signal (RN42 BT_Connected "0" or ADS1220 !DRDY "1" to GPIO00)
      O03 74LVG1G53 switch2 !enable (GPIO00 floats when O03 high)
      O04 ADG779 switch1 signal (TIA feedback to LMP7721 Vout "0" or AD59933 RFB "1")
      O05 ADG779 switch2 signal (TIA input to LMP7721 V- "0" or AD5933 Vin "1")
      O06 Input to illuminate red segment of 3-color LED
      O07 PMOS gate driver for multicolor LED (active low)
      O08 / O09 A0 and A1 of ADG704 switch (set TIA gain, 00 = 100M; 01 = 10k; 10 = 1M; 11 = 100
      O10 TIA gain setting enable (otherwise feedback is open circuit)
      O11 Software shutoff ("0" removes power from circuit, but only when no USB supply is provided)
      O12 !Enable for AD5933 and external clock power (leave off / high for voltammetric applications to remove potential source of noise

      O12 - O15 general purpose outputs broken out to board periphery (careful using O12 as on ABE-Stat hardware implementation; can 
        conflict with enable / power circuit for AD5933

  ADG715 Switch Connections (S0 - S7; states set by state of corresponding bit written to i2c address 0x48):
      S0 Connect working electrode to TIA
      S1 Not used (contacts held at ground)
      S2 voltage follower on biasing amplifier (use to stabilize output in open circuit configuration)
      S3 Feedback reference electrode to amplifier setting counter electrode potential
      S4 Connect Counter electrode amplifier output to reference electrode (connect counter and reference electrodes)
      S5 Connect counter electrode to it's amplifier output
      S6 Sum AD5933 output/ signal source into potentiostat network
      S7 Sum AD5061 DAC voltage into potentiostat network
   
by Daniel M. Jenkins
*/

#include <SPI.h>
#include <Wire.h> // reserved for when we start communicating with AD5933
#include <EEPROM.h>

#include "ESP8266WiFi.h"  // necessary for disabling WiFi- try to suppress extra noise

// IO for SPI communication  (CS shared with shift register clock )
#define MISO             12
#define MOSI             13
#define SCK              14
#define CS_RCK           15 // shared with shift register clock

//inputs to shift register (configure as outputs from ESP8266)
#define SER              16 // data into shift registers
#define CLOCK            2  // clock data in from SER to data registers (note that this is separate from the register clock)

// i2c and SPI default pins based on definitions using the pinout shown at https://github.com/esp8266/Arduino/blob/master/doc/reference.md 

// GPIO00 assigned as input (listen for either !DRDY from ADS1220, or BluetoothConnected Signal)
#define GPIO00            0

// map to connect ESP8266 !CS to corresponding !CS pins on ADS1220 or AD5061 (only allow one to connect at a time, other is always pulled high- make sure that both
// devices are not writing to MISO simultaneously (even though AD5061 doesn't have a data out pin)
#define ADS1220_ADC          0  // SPI chip select designation for ADS1220 (value on shift register output 00 to connect !CS to ADS1220)
#define AD5061_DAC           1  // SPI chip select designation for AD5061 (value on shift register output 00 to connect !CS to AD5061)

#define BAUD             115200 // baud rate for serial communication

// i2c addresses
#define ADG715OctalSwitch      0x48
#define AD5933NetworkAnalyzer  0x0D

// Control / data register assignments for network analyzer
#define AD5933_Control            0x80  // 2 byte control register
#define AD5933_StartFrequency     0x82  // 3 byte control register
#define AD5933_FrequencyIncr      0x85  // 3 byte control register
#define AD5933_IncrNumber         0x88  // 2 byte control register
#define AD5933_SettleCycleNumber  0x8A // 2 byte control register
#define AD5933_Status             0x8F   // 1 byte status register
#define AD5933_TemperatureData    0x92  // 2 byte data register
#define AD5933_RealData           0x94  // 2 byte data register
#define AD5933_ImaginaryData      0x96  // 2 byte data register


// Addresses in EEPROM for calibration constants/ coefficients
const int EEPROMBlockSize = sizeof(double); // so maximum double precision values to store in EEPROM is 512 (4096 / 8)

#define DAC_SLOPE_EEPROM_ADDRESS      0
#define DAC_OFFSET_EEPROM_ADDRESS     EEPROMBlockSize
#define DAC_LOW_LIMIT_EEPROM_ADDRESS   2 * EEPROMBlockSize
#define DAC_HIGH_LIMIT_EEPROM_ADDRESS  3 * EEPROMBlockSize

#define GAIN_0_EEPROM_ADDRESS         4 * EEPROMBlockSize
#define GAIN_1_EEPROM_ADDRESS         5 * EEPROMBlockSize
#define GAIN_2_EEPROM_ADDRESS         6 * EEPROMBlockSize
#define GAIN_3_EEPROM_ADDRESS         7 * EEPROMBlockSize

#define PHASE_SLOPE_EEPROM_ADDRESS    8 * EEPROMBlockSize // these are now the base addresses for these values- now that we have different calibration data for each of 24 settings on AD5933
#define PHASE_OFFSET_EEPROM_ADDRESS   9 * EEPROMBlockSize
#define Z_SLOPE_EEPROM_ADDRESS        10 * EEPROMBlockSize
#define Z_OFFSET_EEPROM_ADDRESS       11 * EEPROMBlockSize

  // after inclusion of calibration with external clock, memory start address for G_PER_V is 200 = (48 * 4) blocks of double values for 48 different settings, + 8 double values for DAC and TIA Gain calibration values
#define AD5933_G_PER_V_EEPROM_ADDRESS 200 * EEPROMBlockSize // prior to calibrations for external clock, starting block address 104 = (24 * 4) blocks of double values for impedance / phase calibration coefficients for 24 different settings, + 8 double values for DAC and TIA Gain calibration values
#define AD5933_BIAS_uV_EEPROM_ADDRESS 201 * EEPROMBlockSize

#define V_BATT_FULL_EEPROM_ADDRESS    202 * EEPROMBlockSize  // note that this is simply a 16 bit number- 10 bit value summed 64 times (saved as 16 bit unsigned integer in a 2 byte block)

// Other system constants
#define fCLK                          16776000  // 16.776 MHz internal clock on AD5933
#define fMCLK                         250000    // 250 kHz external clock to MCLK of AD5933

boolean MCLKext = false;  // boolean to keep track of whether AD5933 is using external clock...

// calibration constants
double vslope = 1.00;
double voffset = 0.00;
double vlowlimit = -1.6;
double vhighlimit = 1.6;

double Gain0 = 1000.0;
double Gain1 = 10000.0;
double Gain2 = 100000.0;
double Gain3 = 1000000.0;

double AD5933_GperVoltOut = 10000.0;
double AD5933_BiasuV = 200.0;

double phaseSlope[48];
double phaseOffset[48];
double ZSlope[48];
double ZOffset[48];

unsigned int ref100Charge = 64000;  // uncalibrated 100% battery value corresponding to 10 bit digital ADC read 3.3V range, through voltage divider 33k and 10k resistors- measured 64 times

const int LED_modulationPeriod = 1000;
int LED_perMilDutyCycle;
long LEDcycleTime;  // start of (on) period for modulating LED
long LEDoffTime;    // start of "off" portion of LED modulation...

const long timeoutInterval = 60000; // period in ms (1 minute) before device shuts itself off if idle (no communication received from Android
long shutoffTime;

unsigned int ShiftRegisterValue = 0x3000; // starting ABE-Stat_1_0_11, leave bit 12 high (default no power to AD5933 and its external clock)
//unsigned int ShiftRegisterValue = 0x0000; // first prototype i2c stopped working- SDA getting clipped- short circuit on AD5933 when latter is not powered?
byte ADG715SwitchStates = 0x00;

SPISettings ADS1220Settings(4000000, MSBFIRST, SPI_MODE1);  // SPI configuration for ADS1220 ADC
SPISettings AD5061Settings(30000000, MSBFIRST, SPI_MODE1);  // SPI configuration for AD5061; since no data is returned, can also use MODE2

union { // Union makes conversion from 4 bytes to an unsigned 32-bit int easy; mostly used for ADS1220 24 bit data reads
    uint8_t bytes[4];
    uint32_t word32;
  } data;

union { // here make a union of double with individual constituent bytes
  uint8_t doubleBytes[sizeof(double)];
  double floatingValue;
} doubleValue;

int16_t realZ, imaginaryZ; // real and imaginary parts of impedance- these are 16 bit signed (two's complement) integers...
double battCharge;

byte TIAGainCode, excitationCode;  // TIA Gain code 0 - 3 => 100 10k 1M 100M; AD5933 Excitation code 0 - 3 => 200mV 400mV 1V 2V
boolean PGAx5Setting; // PGA setting on AD5933 (x5 when true; x1 when false)
double TIAGain = 10000;

uint32_t completedCalibrationsCode = 0x0000;  // used to keep track of which settings of AD5933 have been calibrated in most recent invocation of calibration routine
boolean calibratedAD5933biasAndGperVolt = false;  // used to keep track of whether AD5933 bias and admittance values per output voltage have been calibrated on this cycle...

byte command;
double frequency, frequencyInternal, frequencyExternal;

void setup()
{
  battCharge = 100.0;
  LED_perMilDutyCycle = 1000; // initialize battery to full charge (don't let device shut down if initialized to uncharged state)
  
  WiFi.forceSleepBegin();                  // turn off ESP8266 RF
  delay(1);                                // give RF section time to shutdown
  
  // output pins controlling shift register
  pinMode(SER, OUTPUT); // outputs for driving shift registers/ expanded outputs
  pinMode(CS_RCK, OUTPUT); // also used as CS for SPI
  pinMode(CLOCK, OUTPUT);

  pinMode(0, INPUT);  // GPIO 00 is input (used to measure BT connected or ADS1220 !DRDY signals

  // SPI pins
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, OUTPUT);

  pinMode(GPIO00, INPUT);

  devicePower(true);  // activate digital pin to supply power from battery

  SPI.begin();
  int allocatedEEPROM  = (204 * EEPROMBlockSize); // we're storing 202 double values of EEPROMBlockSize, + 1 16 bit integer for battery charge value...
              // Note for ABE-Stat_1_01_01 and beyond- assuming double requires 8 bytes of memory, we now need 1618 bytes of EEPROM
  EEPROM.begin(allocatedEEPROM); // initialize EEPROM with 1632 bytes (can assign up to 4096)

  // retrieve calibration constants...
  vslope = EEPROM_Double_Read(DAC_SLOPE_EEPROM_ADDRESS);
  voffset = EEPROM_Double_Read(DAC_OFFSET_EEPROM_ADDRESS);
  vlowlimit = EEPROM_Double_Read(DAC_LOW_LIMIT_EEPROM_ADDRESS);
  vhighlimit = EEPROM_Double_Read(DAC_HIGH_LIMIT_EEPROM_ADDRESS);
  
  Gain0 = EEPROM_Double_Read(GAIN_0_EEPROM_ADDRESS);
  Gain1 = EEPROM_Double_Read(GAIN_1_EEPROM_ADDRESS);
  Gain2 = EEPROM_Double_Read(GAIN_2_EEPROM_ADDRESS);
  Gain3 = EEPROM_Double_Read(GAIN_3_EEPROM_ADDRESS);

  AD5933_GperVoltOut = EEPROM_Double_Read(AD5933_G_PER_V_EEPROM_ADDRESS);
  AD5933_BiasuV = EEPROM_Double_Read(AD5933_BIAS_uV_EEPROM_ADDRESS);

  for (int zCoefficient = 0; zCoefficient < 48; zCoefficient++) { // 48 AD5933 settings (4 excitation settings x 3 gain settings x 2 PGA Settings x 2 clock settings)
      int blockOffset = (4 * EEPROMBlockSize) * zCoefficient;  // offset from given base address for given index
      phaseSlope[zCoefficient] = EEPROM_Double_Read(PHASE_SLOPE_EEPROM_ADDRESS + blockOffset);
      phaseOffset[zCoefficient] = EEPROM_Double_Read(PHASE_OFFSET_EEPROM_ADDRESS + blockOffset);
      ZSlope[zCoefficient] = EEPROM_Double_Read(Z_SLOPE_EEPROM_ADDRESS + blockOffset);
      ZOffset[zCoefficient] = EEPROM_Double_Read(Z_OFFSET_EEPROM_ADDRESS + blockOffset);
  }

  ref100Charge = EEPROM_Int_Read(V_BATT_FULL_EEPROM_ADDRESS);

  Wire.begin(); //Note on ESP8266 Wire.pins(int sda, int scl) from http://www.devacron.com/arduino-ide-for-esp8266/
  //Wire.setClock(400000L); // upper limit on speed- AD5933 and ADG715 max clock rate for SCL is 400000Hz// to be safe comment this out...
  Wire.setClock(100000L); // apparently rising edge time constant on SDA is ~1us on some devices, so slow clock down a little bit for clear communication...

  Serial.begin(BAUD);
  
  Configure_ADS1220();  // configure default settings on ADS1220 Analog to Digital Converter

  // initialize variables to obviously contrived values, to facilitate troubleshooting (if functions don't update values with real data

  TIAGainCode = 0x01; // initialize TIA gain code to 0x01 (for AD5933 test, only use 0x01 and 0x02- 10k and 1M)
  excitationCode = 0x00; // initialize excitation code to 0x00 (200 mVp-p, so weakest excitation signal)
  PGAx5Setting = false; // initially set AD5933 PGA gain to 1

  // configure potentiostat network
  setTIAGain(TIAGainCode);  // enable feedback on transimpedance amplifier- gain doesn't matter for test (otherwise working potential will float) argument of 1 uses 10k feedback resistor
  //threeElectrodeConfig(); // use 3-electrode configuration
  twoElectrodeConfig(); // two electrode configuration- counter and reference connected- ensures feedback to set ref potential correctly when testing, even in absence of electrolytic 
                          // or other connection between Counter and Reference
                          // in these preliminary tests we'll use two electrode configuration to test currents / impedance of simple passive component Resistor or Capacitor)

  //openCircuitConfig();  // here we are testing open circuit configuration to measure open circuit voltage; must be < 2.048 V...
                          
  Release_GPIO00(); // don't connect GPIO00 to !DRDY or BT_Connected (allow user a time to reset ESP8266 into programming mode)

  illuminateLED();  // make sure power is supplied to tri-color LED
  redLED(true); // then turn on red LED of tri-color initially just to make sure it works

  delay(500);  // use a software delay function here to enable program to be reset to reprogram mode
      //- now we're changing the code to generate a sin wave as fast as possible, but still only measure the thermocouple voltage periodically (so removing delays from main loop- won't get another opportunity to keep GPIO00 released)
  
  
  redLED(false);  // then turn LED back off (should go back to green, unless bluetooth is connected which is highly unlikely at startup)
  
  DAC_AD5061_Connect(true); // connect AD5061 Digital to Analog Converter from network
  AD5933_Connect(false);  // disconnect network analyzer input by default
  AD5933_PowerOn(false);  // and remove power from it (will just add noise to voltammetric applications if not used)
  setTIAGain(0x01);  // make sure selected feedback resistor connected for TIA
  //TIA_LMP7721();
  shutoffTime = timeoutInterval + millis();
  batteryChargePercent();
  long startyMcStartyPants = millis();
  LEDcycleTime = startyMcStartyPants + LED_modulationPeriod;
  LEDoffTime = startyMcStartyPants + LED_perMilDutyCycle;  // set times for LED modulation...

  MCLKext = true; // here we're only testing harmonics occuring on slower external clock for AD5933
  twoElectrodeConfig();
  AD5933_Connect(true);
  DAC_AD5061_Connect(true);
  DAC_AD5061_SetCalibratedVoltage(0.0);
  TIA_AD5933();  // set TIA to return to AD5933 network analyzer (so we can record admittance estimated by AD5933)
  frequencyExternal = fMCLK / (1024.0 * 16.0);
  frequency = frequencyExternal;
  frequencyInternal = fCLK / (1024.0 * 16.0);
}

void loop() {
    //Configure_ADS1220();  // configure default settings (normal mode, 50/60Hz recursive filters, data ready signal only on !DRDYpin) here; may have been changed during analyses...
    while(Serial.available()) {
      command = Serial.read();
      //ADS1220_ReportTemperature();  // always send the temperature data no matter what electrical measurements are requested
      //batteryReport();  // also report battery charge every time data is requested
      //double computedValue0, computedValue1;
      switch(command) { // in this example, data is only communicated from Android device connected through bluetooth; ESP8266 WiFi only receives data passively to share to internet
        /*case 'a': // code that an analytical method is selected- first decode what analysis...
          startAnalyticalMethod();
          break;*/
        case 's':// code from Android that it has just connected- requests information about how to populate the Android Interface; for testing with ABE-VIEW
          populateAndroidInterface();
          break;
        case 'b':// code from Android that one of the buttons on the Android interface was pushed
          parseButton();
          break;
        case 'r':// code from Android requesting cell resistance
          parseRadioButtons();
          break;
          /*
           * section commented out below are codes for executing different functions in ABE-Stat software, that are not used in this system evaluation with ABE-VIEW app...
           */
        /*case 'c':// code from Android that one of the control input values has been updated
          Calibrate();
          break;
        case 'f': // code from Android to set the cell configuration ('2' is 2 cell, '3' is 3 cell, '0' or any other is open circuit)
          setElectrodeConfig();
          break;
        case 'i': // code from Android requesting current
          computedValue0 = cellCurrent();
          computedValue1 = cellPotential();
          Serial.print('i');
          Serial.print(computedValue0, 3); // cellCurrent is in nA, so returned resolution is pA
          Serial.print('\t');
          Serial.print(computedValue1, 1);
          Serial.print('\t');
          break;
        case 'n':
          Serial.print("vabestat1.01.00\t"); // response to request for firmware version
          break;
        case 'o': // Android requests open circuit potential
          openCircuitConfig();
          DAC_AD5061_Connect(false);  // important to disconnect source of voltage from amplifier
          AD5933_Connect(false);
          TIA_LMP7721();
          computedValue0 = -ADS1220_Diff_Voltage(0x04, 0x00) / 1000.0;// value returned is in mV
          Serial.print('o');
          Serial.print(computedValue0, 4);  // in mV, resolution uV
          Serial.print('\t');
          break;
        case 'r':// code from Android requesting cell resistance
          computedValue0 = cellResistance();
          Serial.print('r');
          Serial.print(computedValue0, 1);
          Serial.print('\t');
          break;
        case 'v': // code to set bias voltage on cell (for constant bias current measurement)
          DAC_AD5061_SetCalibratedVoltage(Serial.parseFloat());
          break; 
        case 'z': // code from Android requesting cell capacitance (also send measured phase)
          computedValue0 = cellCapacitance();
          computedValue1 = correctPhase();
          Serial.print('z');
          Serial.print(computedValue0, 0);
          Serial.print('\t');
          Serial.print(computedValue1, 1);
          Serial.print('\t');
          break;*/
        default:
          break;
      }
      devicePower(true);  // any time we receive communication restore battery power (will prevent shutoff if remove USB cable), and reset timeout clock
      shutoffTime = millis() + timeoutInterval;
    }
    if (millis() < (shutoffTime - timeoutInterval)) shutoffTime = millis() + timeoutInterval; // timer overrun- reset shutoffTime...
    if (millis() > shutoffTime) {
        devicePower(false); // remove battery power from device if timed out...
    }
    if (battCharge < 1.0) blinkOff();
    if (millis() > LEDcycleTime) {
        illuminateLED();  // turn on LED...
        batteryChargePercent(); // measure battery state
        LEDoffTime = LED_perMilDutyCycle + LEDcycleTime;  // value is updated in batteryChargePercent() function- is battery charge in % * 10...
        LEDcycleTime += LED_modulationPeriod;
    }
    if (millis() > LEDoffTime) {
        if (LED_perMilDutyCycle < 1000) noLED();  // turn off LED for rest of cycle (if not 100% duty cycle), and tentatively set next value to turn off...
        LEDoffTime += LEDcycleTime;
    }
}

void setElectrodeConfig() {
    while(!Serial.available()); // wait for character to arrive
    char bcmd = Serial.read();
    if (bcmd == '2') twoElectrodeConfig();
    else if (bcmd == '3') threeElectrodeConfig();
    else openCircuitConfig();
}

void blinkOff() {
    for (int i = 0; i < 3; i++) {
        noLED();
        delay(500);
        illuminateLED();
        delay(500);
    }
    devicePower(false);
}

//invoke the desired interface elemnts so they are displayed as desired on the Android device- this is just an example to show the syntax expected by the Android application
// note that if any of the commands addressing individual interface elements are not issued, the corresponding element will not appear on the interface
void populateAndroidInterface()
{
  Serial.print("b0\tApply Signal\t");  // activates button "0" on Android interface, with a label "LED Toggle"
  Serial.print("b1\tIncrement Frequency\t");  // here we label the second available button with "Load Toggle"
  Serial.print("b2\tDecrement Frequency\t");  // just filling in all available buttons in example to debug the Android interface...

  Serial.print("h0\t1\tFrequency (Hz): \t");  // frequency of applied signal
  Serial.print("h1\t0\tExcitation Code: \t");  // excitation code (0 is 10 mVp, 1 is 20 mVp, 2 is 50 mVp, 3 is 100 mVp
  Serial.print("h2\t0\tGain (Ohm): \t");  // transimpedance amplifier gain; 0 is ~1000, 1 is ~10000, 2 is ~100000
  Serial.print("h3\t0\tClock Source: \t");  // 1 is internal 16.776 MHz, 0 is external 250kHz
  Serial.print("h4\t1\tRaw Admittance value: \t");  // observed system phase in degrees
  Serial.print("h5\t1\tSystem phase (deg): \t");  // observed system phase in degrees

  Serial.print("r0\tGain\t1000\t10000\t100000\t\t");  // radio buttons to select transimpedance amplifier gain
  Serial.print("r1\tSignal Amplitude\t10mV\t20mV\t50mV\t100mV\t");  // radio buttons to select AC signal amplitude (peak voltage value)
  Serial.print("r2\tClock Source\tExt 250kHz\tInt 16.776MHz\t\t\t");  // select which clock source to use

  Serial.print("g1\t"); // enable graph (we've enabled frequency and system phase for display on chart)
}

void parseButton()
{
  int button_id = Serial.parseInt();
  switch (button_id) {
    case 0:
      applySignal();
      break;
    case 1:
      if (MCLKext) {
        if (frequencyExternal < 7800.0) {
          frequencyExternal += fMCLK / (16.0 * 1024.0); // Maximum frequency from DDS is fMCLK (250kHz) / 32
          frequency = frequencyExternal;
        }
      }
      else {
        if (frequencyInternal < 100000.0) {
          frequencyInternal += fCLK / (16.0 * 1024.0);
          frequency = frequencyInternal;
        }
      }
      Serial.print("d0\t");  // code that data is coming- send cell potential first, then current using tab delimitation
      Serial.print(frequency, 2);
      Serial.print('\t');
      break;
    case 2:
      if (MCLKext) {
          if (frequencyExternal > 30.0) {
            frequencyExternal -= fMCLK / (16.0 * 1024.0); // Maximum frequency from DDS is fMCLK (250kHz) / 32
            frequency = frequencyExternal;
          }
      }
      else {
        if (frequencyInternal > 2000.0) {
          frequencyInternal -= fCLK / (16.0 * 1024.0);
          frequency = frequencyInternal;
        }
      }
      Serial.print("d0\t");  // code that data is coming- send cell potential first, then current using tab delimitation
      Serial.print(frequency, 2);
      Serial.print('\t');
      break;
    case 3:
      // code when button "3" is clicked...
      break;
    default:
      break;
  }
}

void parseRadioButtons() {
  int rb_id = Serial.parseInt();
  switch (rb_id) {
    case 11:  // first radio group sets gain of TIA amplifier
      setTIAGain(0);
      break;
    case 12:
      setTIAGain(1);
      break;
    case 13:
      setTIAGain(2);
      break;
    /*case 14:
      setTIAGain(3);
      break;*/
    case 21:  // The second radio group configures the setting of ADC
      excitationCode = 0;
      break;
    case 22: 
      excitationCode = 1;
      break;
    case 23:
      excitationCode = 2;
      break;
    case 24:
      excitationCode = 3;
      break;
    case 31:
      MCLKext = true;
      frequency = frequencyExternal;
      Serial.print("d0\t");  // if we changed the clock source, selected signal frequency must also change...
      Serial.print(frequency, 2);
      Serial.print('\t');
      Serial.print("d3\tExt 250KHz\t");  // and also record the clock source for record...
      break;
    case 32:
      MCLKext = false;
      frequency = frequencyInternal;
      Serial.print("d0\t");  // if we changed the clock source, selected signal frequency must also change...
      Serial.print(frequency, 2);
      Serial.print('\t');
      Serial.print("d3\tInt 16.776MHz\t");  // and also record the clock source for record...
      break;
    default:
      break;
  }
  Serial.print("d1\t");  // now update the excitation code on the GUI / data display
  Serial.print(excitationCode);
  Serial.print('\t');
  Serial.print("d2\t");  // (as well as the transimpedance amplifier gain, in case that's what was changed by user)
  Serial.print(TIAGain, 1); // this value is the actual calibrated value of gain (not the nominal value shown on radio buttons)
  Serial.print('\t');
}

/*
   * Apply the desired test signal, andif we get a valid impedance measurement 
   * (note that in this example, a large delay is put in TestImpedance function so we can easily probe the design operating under the desired conditions)
   * report the observed raw admittance and system phase under the conditions and append data to ArrayLists so they can be plotted and/or saved...
   */
void applySignal() {
    twoElectrodeConfig();
    AD5933_Connect(true);
    DAC_AD5061_Connect(true);
    DAC_AD5061_SetCalibratedVoltage(0.0);
    TIA_AD5933();  // set TIA to return to AD5933 network analyzer (so we can record admittance estimated by AD5933)
  
    if(TestImpedance(frequency, excitationCode, false)) {   // test impedance with given signal amplitude and frequency, don't enable internal x5 amplifier     
      
        Serial.print("d0\t"); // go ahead and send frequency value again...
        Serial.print(frequency, 2);
        Serial.print('\t');

        Serial.print("d4\t"); // data element 4 is the raw admittance (so report the data to that index)
        Serial.print(rawAdmittance(), 1);
        Serial.print('\t');

        Serial.print("d5\t"); // data element 5 is the System phase (so report the data to that index)
        Serial.print(rawPhase(), 1);
        Serial.print('\t');

        Serial.print("u");  // concatenate most recent data values into Arraylists (so they can be plotted and/or saved...
    }
    
}



