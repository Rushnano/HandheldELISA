/*
 * Creator: Daniel M. Jenkins
 * Date: February 14, 2017
 * Description: Higher level functions to set and read analog voltages in potentiostat circuit.
 * 
 * Partitioned from other generic library functions 7/30/2017 to facilitate management
 */

/*
 * Apply small potential, measure resulting current, and return the estimated resistance value...
 * make sure value / linearity is confirmed by measurement at a lower gain / input voltage setting...
 */
double cellResistance() {
    AD5933_PowerOn(false);  // remove power from AD5933 network- not being used and will just add noise...
    //twoElectrodeConfig();
    DAC_AD5061_Connect(true);
    AD5933_Connect(false);
    TIA_LMP7721();
    setTIAGain(0x03); // set highest feedback resistor initially (then work way down if offscale)
    boolean quit = false;
    double volts = 0.5;
    double vin = 0.0;
    double vout = 0.1;
    double resistance[] = {1.0, 1.0};
    boolean voltageAdjusting = false;
    while (!quit) {
        if (!voltageAdjusting) {
          DAC_AD5061_SetCalibratedVoltage(volts);
          delay(10);
          vin = -ADS1220_Diff_Voltage(0x04, 0x00) / 1000000.0;
          vout = ADS1220_Diff_Voltage(0x02, 0x00) / 1000000.0;
        }
        if (vout == 0.0) resistance[0] = 1e9;  // if otherwise would divide by zero, just set resistance to high value (1 GOhm, pretty much upper limit or beyond what we can measure on this device)
        else resistance[0] = (vin * TIAGain / vout);
        if (TIAGainCode > 0x00) {
          TIAGainCode--;
          setTIAGain(TIAGainCode);
          delay(10);
          vout = ADS1220_Diff_Voltage(0x02, 0x00) / 1000000.0;
        }
        else if (volts > (abs(voffset))) { // should reach lower allowable limit for applied voltage within 4 cycles...{
          volts /= 8.0;
          DAC_AD5061_SetCalibratedVoltage(volts);
          delay(10);
          vin = -ADS1220_Diff_Voltage(0x04, 0x00) / 1000000.0;
          vout = ADS1220_Diff_Voltage(0x02, 0x00) / 1000000.0;
          voltageAdjusting = true; // just use these values for estimating resistance[0] next iteration...
        }
        if (vout == 0.0) resistance[1] = 1e9;  // if otherwise would divide by zero, just set resistance to high value (1 GOhm, pretty much upper limit or beyond what we can measure on this device)
        else resistance[1] = (vin * TIAGain / vout);
        double deltaResistance = abs(resistance[0] - resistance[1]);
        if (deltaResistance < (resistance[0] / 10.0)) quit = true;  // if answer at different settings is within 10% accept the answer...
          // note that quit will be forced to be true if we are already at the lowest possible gain / applied voltage, as new value of resistance
            // will be based on the same values of input voltage and output voltage used for first resistance measurement...
    }
    return resistance[0]; // return the value at higher input voltage / gain (should be more accurate if approximately confirmed by lower gain setting)
}

/*void printGain() {
    Serial.print("mGain:");
    Serial.print(TIAGain,1);
    Serial.print('\t');
}

void printGainCode() {
    Serial.print("mGainCode:");
    Serial.print(TIAGainCode);
    Serial.print('\t');
}*/

/*
 * returns the cell current (in nA)... (make sure to confirm linearity at lower gain setting before accepting answer)
 * Calling this function will also set the TIAGain at the highest value where linearity / no saturation is ensured
 */
double cellCurrent() {
    AD5933_PowerOn(false);  // remove power from AD5933 network- not being used and will just add noise...
    //twoElectrodeConfig();
    Serial.print("mMeasuring current...\t");
    DAC_AD5061_Connect(true);
    boolean quit = false;
    boolean lowestTIA = false;
    double vamp = 0.0;
    double current[] = {1.0, 1.0};
    AD5933_Connect(false);
    TIA_LMP7721();
    setTIAGain(0x03); // set highest feedback resistor initially (then work way down if offscale)
    while (!quit) {
        vamp = ADS1220_Diff_Voltage(0x02, 0x00);// returns microVolts
        current[0] = (vamp * 1e3 / TIAGain);  // multiply by 1000 to convert to nV, then divide by TIA gain to get current in nA
        Serial.print("mstarting measurement ");
        Serial.print(current[0], 0);
        Serial.print("\t");
        if (TIAGainCode > 0x00) {
          TIAGainCode--;
          setTIAGain(TIAGainCode);
          vamp = ADS1220_Diff_Voltage(0x02, 0x00);
          Serial.print("mmade new setting measurement\t");
          delay(1); // strategic delay to prevent MCU timeout...
        }
        else lowestTIA = true;  // if TIAGain is already at 0x00, we know we didn't get a good measurement at higher TIAGains
        current[1] = (vamp * 1e3 / TIAGain);
        Serial.print("mconfirmation measurement ");
        Serial.print(current[1], 0);
        Serial.print("\t");
        double deltaCurrent = abs(current[0] - current[1]);
        if ((deltaCurrent < (abs(current[0]) / 10.0)) || (abs(current[0]) < 100.0)) quit = true;  // if measurements are reproducible, or small especially less than value to saturate at high gain...
    }
    if (!lowestTIA) { // if we got a valid measurement at a TIAGain other than 0x00, then return to that setting...
      TIAGainCode++;
      setTIAGain(TIAGainCode);
    }
    Serial.print("mCompleted current measurement...\t");
    return current[0];
}

/*
 * Report battery % charge remaining  (single cell lithium ion- assuming linear charge characteristic between 3.7 and 4.2 V...)
 */
 void batteryReport() {
    int battChargePercent = (int) battCharge;
    if (battChargePercent > 100) battChargePercent = 100;
    if (battChargePercent < 0) battChargePercent = 0;
    Serial.print("h");  // code "b" already reserved for counting down equilibration time, so use "h" for battery charge
    Serial.print(battChargePercent);
    Serial.print("\t");
 }

 /*
  * Estimate battery charge remaining, as a % value (assume full charge is 4.2 V, "0" charge is 3.7, then calculate % charge ratiometrically based on full charge conversion in EEPROM
  */
  void batteryChargePercent() {
    unsigned int battChargeIntegrator = 0;
    for (int n = 0; n < 64; n++) {
      battChargeIntegrator += analogRead(0);
    }
    unsigned int zeroBattCharge = ref100Charge * 0.8809;  // 3.7 V at 0% charge, 4.2 V at 100%
    battCharge = 100.0 * (battChargeIntegrator - zeroBattCharge) / (ref100Charge - zeroBattCharge);
    LED_perMilDutyCycle = (int) (battCharge * 10);
    if (battCharge < 0) LED_perMilDutyCycle = 0;
    else if (battCharge >= 100.0) LED_perMilDutyCycle = 1000;
      //as of version 1.0.11 battCharge is a global variable...
    /*Serial.print("mFullCharge ");
    Serial.print(ref100Charge);
    Serial.print(" ZeroCharge ");
    Serial.print(zeroBattCharge);
    Serial.print(" battCharge ");
    Serial.print(battCharge);
    Serial.print(" raw reading ");
    Serial.print(battChargeIntegrator);
    Serial.print("\t");*/
    //return battCharger;
  }

/*
 * returns the cell potential in mV
 */
 double cellPotential() {
    AD5933_PowerOn(false);  // remove power from AD5933 network- not being used and will just add noise...
    AD5933_Connect(false);
    TIA_LMP7721();
    double vCell = -ADS1220_Diff_Voltage(0x04, 0x00) / 1000.0; // convert from uV to mV
    return vCell;
 }

/*
 * set voltage (after applying calibration equation to ensure accurate voltage output)
 */
void DAC_AD5061_SetCalibratedVoltage(double voltage) {
  double calibratedVoltage = (voltage * vslope) + voffset;
  DAC_AD5061_SetVoltage(calibratedVoltage);
}

/*
 * Write 24 bit instruction (0x00****) to AD5061 to set voltage; assumes that reference is 3.3 V, with virtual ground (at 1.65V)...
 * but does not take into account any deviations/ offsets.
 */
void DAC_AD5061_SetVoltage(double voltage) {
  if (voltage < -1.65) voltage = -1.65;
  else if (voltage > 1.65) voltage = 1.65;  // apply constraints from the voltage reference of DAC
  voltage += 1.65;  // (virtual "ground" is at 1.65V from system ground and negative voltage reference...
  long DAC_value = ((voltage / 3.3) * 0xFFFF);
  if (DAC_value > 0xFFFF) DAC_value = 0xFFFF; // apply upper limit on value written to DAC; otherwise voltage will overrun 16 bit DAC (wrap back to negative voltages)
                                               // limit is necessary because actual upper limit of voltage range may be lower than the presumed 1.65V value assumed here
                                               // so after application of calibration equation transformed DAC_value may be out of range
  else if (DAC_value < 0) DAC_value = 0; // similarly don't let values beyond lower limit wrap around to upper end of voltage range...
  byte Byte01 = (DAC_value & 0xFF00) >> 8;
  byte Byte00 = (DAC_value & 0x00FF);
  SPI_Chip_Select(AD5061_DAC);
  digitalWrite(CS_RCK, LOW);
  SPI.beginTransaction(AD5061Settings);
  byte SPI_Junk_Received = SPI.transfer(0x00); // first byte is control- last two bits can be used to set into power down mode; here we leave in normal mode
  SPI_Junk_Received = SPI.transfer(Byte01);
  SPI_Junk_Received = SPI.transfer(Byte00);
  SPI.endTransaction();
  digitalWrite(CS_RCK, HIGH); // CS/ SYNC must go back high before another instruction can be sent...
  Release_Chip_Select();
}

void ADS1220_Settings(byte reg00, byte reg01, byte reg02) {
  SPI_Chip_Select(ADS1220_ADC);
  digitalWrite(CS_RCK, LOW);  // pull corresponding chip select low
  SPI.beginTransaction(ADS1220Settings); // slower than 150 ns minimum SPI clock period; MSBFirst; only SPI_MODE 1 supported
  byte SPI_Junk_Received = SPI.transfer(0x42); // WREG command- write 3 bytes to/starting at configuration register 00. WREG command is b0100 rrnn where nn is (number of bytes - 1) to write starting at register rr
  SPI_Junk_Received = SPI.transfer(reg00); // byte written to register 00(binary)- default input setting (differential input; AIN0 + relative to AIN1 -), gain 4, PGA (programmable gain amplifier disabled)
  SPI_Junk_Received = SPI.transfer(reg01); // Operate in single shot mode (have to send "Start/sync" command to start a result conversion cycle_
                        // also controls Internal temperature sensor (here off, turn on with 0x06); register also controls data rate and mode (default is normal mode, 20 samples per second)
  SPI_Junk_Received = SPI.transfer(reg02); // Set voltage reference to internal 2.048V, and turn on 60 Hz digital recursive filter
  SPI.endTransaction();
  digitalWrite(CS_RCK, HIGH);  // release chip select after each change of settings
  Release_Chip_Select();
}

/*
 * Read the three data registers on the ADS1220 (they get written to a global 4 byte variable so function does not return a value)
 * Function assumes that ADS1220 is already in communication with microcontroller- i.e. has CS asserted low, for example by preceding
 * data read with ADS1220_Settings above.
 */
void ADS1220_Read_Data() {
  SPI_Chip_Select(ADS1220_ADC);
  digitalWrite(CS_RCK, LOW);  // pull corresponding chip select low
  SPI.beginTransaction(ADS1220Settings); // slower than 150 ns minimum SPI clock period; MSBFirst; only SPI_MODE 1 supported
  byte SPI_Junk_Received = SPI.transfer(0x08); // Start/Sync command (required to start a single shot conversion)
  waitADS1220DataReady();  // wait until data is ready from ADS1220, then read it- in normal mode should be complete in 50 ms; will be a lot faster in Turbo mode / higer data rates...
  //delay(51);  // here we'll have software delay of 51 ms (greater than conversion time at 20 SPS standard mode)- if DRDY pin is not working correctly previous command can hang up program
  // only 3 bytes/ 24 bits are transferred from device, so just put them in least significant 3 bytes of variable "data"
  data.bytes[2] = SPI.transfer(0x00);
  data.bytes[1] = SPI.transfer(0x00);
  data.bytes[0] = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(CS_RCK, HIGH);  // pull corresponding chip select high (release device as soon as data is read completely)
  Release_Chip_Select();

  data.bytes[3] = 0x00; // most significant byte of 4-byte word is empty for 24 bit conversions
}
 
/*
 * Read differential voltage on ADS1220 input pair. 
 * Function writes values into control register 0 (ADS1220 datasheet page 40) before starting read operation
 * Gain code is b000x - b111x 1, 2, 4, 8, 16, 32, 64, 128 (x is PGA bypass bit- forces gain = 1 when on)
 * MUX code is 0x0x to 0xfx. Specific codes for measuring:
 * AIN0 vs AIN3 (transimpedance amplifier vs analog reference- proportional to working current)- 0x2x
 * AIN1 vs AIN3 (VREF vs analog reference; negative of working potential vs reference electrode potential) - 0x4x
 * AIN2 vs AIN3 (Thermocouple + vs analog reference connected to Thermocouple- ) 0x5x
 */
double ADS1220_Diff_Voltage(byte mux, byte gain) {
  if (mux > 0x0E) mux = 0x0E;// 1111 (0xf) for mux bits is reserved
  else if (mux < 0x00) mux = 0x00;
  if (gain > 0x07) gain = 0x07;
  if (gain < 0) gain = 0;
  byte muxbits = mux << 4;
  byte gainbits = 0x00;
  if (gain == 0x00) gainbits = 0x01; // if gain is set to zero- just bypass PGA (don't want amplifier voltage offset)
  else gainbits = gain << 1; // otherwise, shift the "gain" setting one bit (corresponding to the gain bits of configuration register 0) 
  byte reg00 = muxbits | gainbits;  // compose the contents of configuration register 00...

  ADS1220_Settings(reg00, 0x00, 0x10);
  
  //ADS1220_Read_Data();// read data has been folded into ADS_1220_microVolts
  return ADS1220_microVolts(gain);   // value returned is in microvolts
}

/*
 * return the value of the last read of ADS1220 in microvolts (assuming internal 2.048 V reference)
 */
double ADS1220_microVolts(byte gainSetting) {
    int32_t value = 0;  // signed 32 bit integer (two's complement- sign indicated by MostSignificant_bit)
    double voltage = 0.0;
    int ADCCount = 0;
    ADS1220_Read_Data();
    if (data.word32 & 0x00800000) { // Sign extend negative numbers- conversion is a two's complement 24 bit value
        value = 0xFF800000 | ((data.word32) & 0x007FFFFF);  // write signed 24 bit value into 32 bit twos complement
    }
    else {
        value = data.word32; // not negative number (MSb or bit 23 != 1) then the existing value is the correctly signed as a 32 bit number
    }
    while (((value == 0) || (value == -1) || (value == 1)) && (ADCCount < 3)) {
        ADCCount++;
        ADS1220_Read_Data();
        if (data.word32 & 0x00800000) { // Sign extend negative numbers- conversion is a two's complement 24 bit value
          value = 0xFF800000 | ((data.word32) & 0x007FFFFF);  // write signed 24 bit value into 32 bit twos complement
        }
        else {
          value = data.word32; // not negative number (MSb or bit 23 != 1) then the existing value is the correctly signed as a 32 bit number
        }
    }
    int PGA = (0x0001 << gainSetting); // estimate gain- gain setting corresponds to 2^ PGA bits in control register, or 0x01 << by PGA bits
    voltage = (value * 0.244141) / PGA;  // signed 24 bit conversion with voltage reference span +/-2.048V, so range 4.096e6 uV divided by 16777216 discrete 2^24 intervals
    /*Serial.print("mV(uV):");
    Serial.print(voltage, 3);
    Serial.print("\t");*/
    return voltage;
}

/*
 * Measures reference temperature, and observed thermocouple potential and reports to Android (rely on Android to estimate board temperature
 * using polynomial relationships to convert ref temperature to equivalent thermoelectric voltage, sum to observed thermoelectric voltage, and
 * then calculate temperature of resulting "virtual" thermoelectric voltage relative to ice point / 0C.
 * 
 * This approach also allows user to toggle thermocouple type in Android interface, if different thermocouple is used...
 */
void ADS1220_ReportTemperature() {
    double refTemp = ADS1220_RefTemperature();
    double tcE = ADS1220_Diff_Voltage(0x05, 0x00); // Thermoelectric potential in uV; use http://www.omega.com/techref/pdf/z198-201.pdf for conversion using E in uV
      // conversion to thermocouple temperature performed in Android application (presumed to have better floating point accuracy for high order polynomial calculations)
    Serial.print("t");
    Serial.print(refTemp, 2);
    Serial.print("\t");
    Serial.print(tcE, 1); // 1 decimal uV resolution equivalent to about 0.0025C...
    Serial.print("\t");
}

/*
 * Read internal reference temperature on ADS1220 
 */
double ADS1220_RefTemperature() {
  ADS1220_Settings(0x00, 0x02, 0x10); // settings to measure device temperature in single shot mode- 50 and 60 Hz rejection FIR filters active
  ADS1220_Read_Data();
  
  int16_t value;  // signed 16 bit integer (two's complement- sign indicated by MostSignificant_bit)
  double ref_temp = 0;

  value = data.word32 >> 10;  // 14 bit conversion value is sent left justified in 24 bit word (so shift right 10 bits to get number in right frame
  
  if (value & 0x00002000) { // Sign extend negative numbers- conversion is a two's complement 14 bit value
    value = 0xC000 | ((value) & 0x3FFF);  // write into 16 bit twos complement
    ref_temp = value * 0.03125;
    }
  else {
    value = value; // not negative number (MSb or bit 24 !=1) then the existing value is the correctly signed as a 32 bit number
    ref_temp = value * 0.03125; // resolution is different for positive (0.03125 C/ LSB) and negative numbers (0.25 C/LSB)
    }
  return ref_temp;
}

/*
 * Configure the ADS1220
*/
void Configure_ADS1220() {
  SPI_Chip_Select(ADS1220_ADC); // connect ADS1220 chip select to the chip select driver
  digitalWrite(CS_RCK, LOW);  // pull corresponding chip select low
  SPI.beginTransaction(ADS1220Settings); // slower than 150 ns minimum SPI clock period; MSBFirst; only SPI_MODE 1 supported
  byte SPI_Junk_Received = SPI.transfer(0x06); // reset ADS1220
  SPI_Junk_Received = SPI.transfer(0x43); // WREG command- write 4 bytes to/starting at configuration register 0. WREG command is b0100 rrnn where nn is (number of bytes - 1) to write starting at register rr
  SPI_Junk_Received = SPI.transfer(0x01); // byte written to register 00(binary)- default input setting (differential input; AIN0 + relative to AIN1 -), gain 1, PGA (programmable gain amplifier disabled)
  SPI_Junk_Received = SPI.transfer(0x00); // Operate in single shot mode (have to send "Start/sync" command to start a result conversion cycle_
                        // also controls Internal temperature sensor (here off, turn on with 0x06); register also controls data rate and mode (default is normal mode, 20 samples per second)
  SPI_Junk_Received = SPI.transfer(0x10); // Set voltage reference to internal 2.048V, and turn on 50 and 60 Hz digital recursive filter
  SPI_Junk_Received = SPI.transfer(0x00); // register 0x03; IDACs disabled, DRDY flagged only on !DRDY pin (not SPI data out / MISO)
  SPI.endTransaction();
  digitalWrite(CS_RCK, HIGH);  // pull corresponding chip select high (release device)
  Release_Chip_Select();
  //SPI_Chip_Select(EXT_SPI); // just leave chip select high- not active
}

/*
 * Write a single control register for ADS1220- for when speed is critical (i.e. just toggle MUX bits to measure current and voltage in voltammetric scans
 */
void ADS1220_SetSingleCtrlRegister(byte address, byte setting) {
    if (address > 0x03) address = 0x03;
    else if (address < 0x00) address = 0x00;
    address = address << 2;   // left shift register address bits by 2 (so command can be assembled to write to correct starting address)
    byte ADS1220Instruction = (0x40 | address);
    SPI_Chip_Select(ADS1220_ADC); // connect ADS1220 chip select to the chip select driver
    digitalWrite(CS_RCK, LOW);  // pull corresponding chip select low
    SPI.beginTransaction(ADS1220Settings); // slower than 150 ns minimum SPI clock period; MSBFirst; only SPI_MODE 1 supported
    byte SPI_Junk_Received = SPI.transfer(ADS1220Instruction); // WREG command- write 1 byte starting at designated register address
    SPI_Junk_Received = SPI.transfer(setting); // byte written to register 00(binary)- default input setting (differential input; AIN0 + relative to AIN1 -), gain 1, PGA (programmable gain amplifier disabled)
    SPI.endTransaction();
    digitalWrite(CS_RCK, HIGH);  // pull corresponding chip select high (release device)Release_Chip_Select();
}

