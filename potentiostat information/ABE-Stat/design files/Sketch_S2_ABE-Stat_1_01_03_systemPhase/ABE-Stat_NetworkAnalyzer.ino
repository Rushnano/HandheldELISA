/*
 * Creator: Daniel M. Jenkins
 * Date: February 14, 2017
 * Description: Basic calls to AD5933 network analyzer
 * 
 * partitioned into separate library for ABE-Stat (7/30/2017) to facilitate function management
 */

/*
 * Measure cell capacitance (value returned is in pF) (in version 1.0.07 included check to make sure does not result in divide by 0
 */
double cellCapacitance() {
      //twoElectrodeConfig();
      //DAC_AD5061_Connect(false); // ... applying zero bias
      AD5933_PowerOn(true); // have to turn on AD5933 to use it!
      DAC_AD5061_Connect(true); // connect DAC from AD5061 to network...  // calibration appears to be different with and without AD5061, so go ahead and connect,
          // then for calibrations set bias to 0- so make all measurements connected to this network (and apply "0" bias / output voltage)
      DAC_AD5061_SetCalibratedVoltage(0.0); // here we'll apply unbiased voltage
      AD5933_Connect(true); // connect cell input to network analyzer
      TIA_AD5933(); // connect cell output to network analyzer...
      setTIAGain(0x02); // use 2nd most sensitive gain- 1000000 ohm resistor results in poor amplifier performance on network analyzer
      boolean q = false;
      long f = 30000;
      while (!TestImpedance(f, 0x03, false));  // first try to evaluate impedance at 30k, highest sensitivity settings, but without PGA which can distort signal...
      double graw = rawAdmittance();
      if (graw > (0.9 * AD5933_GperVoltOut)) f = 6000; // if admittance is too high / TIA output > 0.9V, test at lower frequency (admittance will decrease)
      else if (graw < (0.2 * AD5933_GperVoltOut)) f = 100000; // if admittance is too low, test at higher frequency (admittance will increase)
      else q = true;  // if in range we have a good impedance measurement so don't need another measurement...
      if (!q) autoRangeTestImpedance(f);
      double Zmagi = impedanceMagnitude();
      double Capi = 10000000; // if low impedance, just assume we have a large capacitor, here 10 microF (1e7 pF)
      if (Zmagi != 0.0) Capi = (1000000000000 / (2 * PI * f * impedanceMagnitude()));
      return Capi;
}

/* 
 *  Estimate the raw phase from realZ and imaginaryZ values; 
 *  The vector magnitudes are actually proportional to admittance, but the phase corresponds to the impedance phase (phase relationship of voltage to current)
 *  in version 1.0.07 modified to make sure real number (phasor 90 or -90 degree) returned if real component is zero- prevent divide by zero / nan result
 */
double rawPhase() {
    double phi = 0.0;
    double phasor = 0.0;
    if (realZ == 0) {
      if (imaginaryZ > 0) phasor = 90.0;
      else if (imaginaryZ < 0) phasor = -90.0;
    }
    else {
      double ratio = imaginaryZ * 1.0 / (realZ * 1.0);
      phi = 180.0 * atan(ratio) / PI;
      if (realZ > 0) phasor = phi;  // in 1st and 4th quadrants arctan gives direct phase angle
      else if (imaginaryZ > 0) phasor = 180.0 + phi;  // in second quadrant, arctan returns negative of angle, with magnitude equivalent to deviation from 180 deg (so just add the negative number)
      else phasor = -180.0 + phi; // in third quadrant, arctan returns positive angle; so just add the angle to -180...
    }
    return phasor;
}

/*
 * return the impedance phase from realZ and imaginaryZ values, corrected for calibrated system phase
 */
 double correctPhase() {
    int AD5933settingIndex = excitationCode + (TIAGainCode * 4);
    if (PGAx5Setting) AD5933settingIndex += 12;
    if (MCLKext) AD5933settingIndex += 24;
    /*double sysfase = (phaseOffset[AD5933settingIndex] + phaseSlope[AD5933settingIndex] * frequency);
    Serial.print("mZslope ");
    Serial.print(phaseSlope[AD5933settingIndex], 4);
    Serial.print(" ZOffset ");
    Serial.print(phaseOffset[AD5933settingIndex]);
    Serial.print(" systemPhase ");
    Serial.print(sysfase);
    Serial.print(" settingIndex ");
    Serial.print(AD5933settingIndex);
    Serial.print("\t");*/
    double frase = rawPhase() - (phaseOffset[AD5933settingIndex] + phaseSlope[AD5933settingIndex] * frequency);
    while (frase > 180.0) frase -= 360.0;
    while (frase < -180.0) frase += 360.0;  // add or subtract 360 degrees until result falls in range -180 to +180...
    return frase;
    
 }

/*
 * Return the raw admittance value
 */
 double rawAdmittance() {
    double realzsq = 1.0 * pow(realZ, 2);
    double imagzsq = 1.0 * pow(imaginaryZ, 2);
    double Gval = sqrt(realzsq + imagzsq);
    /*Serial.print("mRaw G value ");
    Serial.print(Gval, 2);
    Serial.print("\t");*/
    return Gval;
 }

/*
 * Return estimated impedance magnitude (corrected based on TIA gain setting, PGAx5, and excitation...
 * 
 * impedance is inversely proportional to rawAdmittance value, which itself is directly proportional to 
 * excitation voltage, feedback / TIAGainSetting, and PGAx5 setting (so these latter must be factored into the actual impedance estimate
 */
 double impedanceMagnitude() {
    int AD5933settingIndex = excitationCode + (TIAGainCode * 4);
    if (PGAx5Setting) AD5933settingIndex += 12;
    if (MCLKext) AD5933settingIndex += 24;
    /*double Zfcoefficient = (ZSlope[AD5933settingIndex] * frequency * 1.0) + ZOffset[AD5933settingIndex];
    Serial.print("mZslope ");
    Serial.print(ZSlope[AD5933settingIndex]);
    Serial.print(" ZOffset ");
    Serial.print(ZOffset[AD5933settingIndex]);
    Serial.print(" Zcoeff ");
    Serial.print(Zfcoefficient);
    Serial.print(" settingIndex ");
    Serial.print(AD5933settingIndex);
    Serial.print("\t");*/
    double Impedance = 1e9; // if raw admittance is zero, just use this value for impedance magnitude (pretty much upper limit or beyond range on device...
    double rawG = rawAdmittance();
    if (rawG != 0.0) Impedance = (((ZSlope[AD5933settingIndex] * frequency * 1.0) + ZOffset[AD5933settingIndex]) / (rawG * 1.0));
    /*Serial.print("mZabs ");
    Serial.print(Impedance, 0);
    Serial.print(" exc_code ");
    Serial.print(excitationCode);
    Serial.print(" TIAGain ");
    Serial.print(TIAGain, 0);
    if (PGAx5Setting) Serial.print("PGAx5");
    Serial.print("\t");*/
    return Impedance;
 }

/*
 * allow system to autotune settings (TIAGain, Excitation voltage, and PGAGain at given frequency setting), starting at high excitation, to find valid admittances
 */
 void autoRangeTestImpedance(long frequ) {
      PGAx5Setting = true;  // start with AD5933 PGAx5 gain on (want largest signal possible without going to close to margin)
      excitationCode = 0x03; // start with 2Vpp / 10 excitation (largest, for largest signal)
      setTIAGain(0x02);  // start calibration with 3rd feedback resistor (100k)
      //DAC_AD5061_Connect(false);
      DAC_AD5061_Connect(true); // connect DAC from AD5061 to network...  // calibration appears to be different with and without AD5061, so go ahead and connect,
          // then for calibrations set bias to 0- so make all measurements connected to this network (and apply "0" bias / output voltage)
      DAC_AD5061_SetCalibratedVoltage(0.0); // here we'll apply unbiased voltage
      AD5933_Connect(true);  // AND connect network analyzer voltage source to network
      TIA_AD5933();  // connect network back to TIA on network analyzer AD5933
      autoRangeZFast(frequ, false, 0.0); // here we allow device to dial down excitation amplitude as necessary for good reading, and assume 0 bias signal
        // always send 0 bias argument from here, as 
        // autoRangeTestImpedance function is only invoked to measure capacitance (when DAC is disconnected, so 0 bias), or in calibration under similar circumstances
 }

/*
 * core of AutoRangeTestImpedance function- dials down gain and excitation settings until valid measurement is achieved; unlike AutoRangeTestImpedance
 * excitation and gain values are not initiated at highest level, so this function can save time for subsequent measurements of same network by starting
 * at settings that worked previously (i.e. used in ElectrochemicalImpedanceSpectroscopy() under ABE-Stat_AnalyticalMethods
 * boolean fixed Excitation used for scans with a designated signal amplitude- don't let it change
 * For trying to determine if TIA of AD5933 is saturating, assume max output is 1.6 V; value returned for 100kOhm resistor on 100kOhm TIAGain, 100mV peak excitation (0 bias)
 * x5PGA on is about 5000 from calibration. This would result in expected output peak value of 500 mV; so at 1.6V would be about 16000 or so... For safety margin just make cutoff as 10000,
 * but again this assumes no bias- value actually should be set based on narrower margin of 1.6 - bias. In other words let 1.6 - biasVoltage be the margin, TIA amplifier
 * results in 2000 for 1Vpeak, so actual threshold in reading should be (1.6 - bias) * 2000. For some margin of safety, let the value be (1.6 - Bias) * 1500.
 * Starting version 1_0_07, noticed that above check is not sufficient, because TIA amplifier does not behave "ideally" at saturation (so output is not necessarily close to rail voltage
 * relative to analog ground / inputs which may drift. So make sure to take a second measurement at a lower gain to ensure system is still in linear realm...
 */
 void autoRangeZFast(long f, boolean fixedExcitation, double biasV) {
      if (PGAx5Setting && (abs(biasV) > 0.3)) PGAx5Setting = false; // don't let PGAx5 turn on if it would put bias signal anywhere near saturation at 1.6V...
      while (!TestImpedance(f, excitationCode, PGAx5Setting)); // keep trying until we get a valid non-repeated measurement
      double originalGangster = rawAdmittance();
      double admittanceMargin = (1.6 - abs(biasV)) * AD5933_GperVoltOut / 1.1;  // margin between applied signal bias and rail of amplifier; 1.1 is safety margin- make cuttoff 10% lower than predicted cutoff
      if (PGAx5Setting) admittanceMargin = (1.6 - (5 * abs(biasV))) * AD5933_GperVoltOut / 1.1;
      
      while ((originalGangster > admittanceMargin) && (TIAGainCode != 0x00)) {  // if observed raw admittance value is too large, TIA voltage is likely getting clipped, lower excitation and or gain and try again
          if (PGAx5Setting) {
            PGAx5Setting = false;  // first try lowering the PGA gain
            //Serial.print("mJust turned off PGAx5\t");
          }
          else if ((excitationCode != 0x00) && !fixedExcitation) {  // 
            excitationCode--; // then try decrementing the excitation voltage setting...
            /*Serial.print("mExcitation Code ");
            Serial.print(excitationCode);
            Serial.print(" not successful!\t");*/
          }
          else if (TIAGainCode != 0x00) {
            TIAGainCode--; // then start decrementing the first stage feedback resistance
            setTIAGain(TIAGainCode);
            /*Serial.print("mDecrementing to Gain Code ");
            Serial.print(TIAGainCode);
            Serial.print("\t");*/
          }
          admittanceMargin = (1.6 - abs(biasV)) * AD5933_GperVoltOut / 1.1;  // margin between applied signal bias and rail of amplifier  // if we have to take second measurement, know PGAx5 is off...
          while (!TestImpedance(f, excitationCode, PGAx5Setting));  // then take measurement again at dialed down settings..
          originalGangster = rawAdmittance(); // and estimate the new raw admittance value...
      }
 }

/*
 * 
 */
 double AD5933_RefTemperature() {
    AD5933_PowerOn(true);
    i2c_Write(AD5933NetworkAnalyzer, AD5933_Control, 0x90); // this part of register has bits we set to ask to start temperature measurement...
    //i2c_Write(AD5933NetworkAnalyzer, AD5933_Control + 1, 0x00); //this is the rest of bits for control, but most are reserved / 0 so just leave them. Only exceptions are reset bit (D4) 
              // and clock source which we generally want to leave as 0 (no reset; internal clock source)
    delayMicroseconds(810); // from documentation conversion requires 800 microseconds..., other option is to poll status register d0 but this is actually probably faster...
    int TData = (i2c_Read(AD5933NetworkAnalyzer, AD5933_TemperatureData) << 8) | i2c_Read(AD5933NetworkAnalyzer, AD5933_TemperatureData + 1);  // just read 16 bits of 4 byte data block returned into local variable- only reading two bytes
    if ((TData & 0x2000) != 0) { // if 14 bit twos complement conversion indicates negative number- sign extend to ...
        TData = 0xE000 | ((TData) & 0x1FFF);
    }
    double deviceTemp = TData * 0.03125;
    return deviceTemp;
 }
 
/*
 * Crude function to go through simple frequency "sweep" cycle of only a single frequency increment
 * Note for programming frequency registers, internal oscillator is 16.776 MHz
 * Start Frequency code = (4 * desired frequency / (clock freq)) * 2^27 
 * (same formula for frequency interval code)
 * Using an the internal oscillator as clock, code for start frequency of 30kHz is 960070  (0x0EA646)
 * frequency increment code for 10 Hz is 320 (0x000140)
 * latest version with freq and excitation arguments; excitation is code for 
 * 
 * returns boolean indicating if valid result occurs (i.e. if repeated measurements are identical, something is wrong so power is cycled, user can ask for new measurement
 */
 boolean TestImpedance(double freq, byte excitation, boolean PGAx5) {
    AD5933_PowerOn(true);
    excitationCode = excitation;  // save excitation setting to global variable (so adjusted admittance / impedances can be estimated elsewhere without passing settings)- do it now before convoluting code to AD5933 bit order
    boolean endOfSweep = false;
    int16_t cycleCount = 0; // keep track of # measurements executed- force exit if reproducibility still poor after 5 measurements
    int16_t realZ0 = 0;
    int16_t imaginaryZ0 = 0;  // initialize real and imaginary Z0 values- these are used to force validation of measurement if measurement values are in a normal range (prevent large random errors)

    int16_t realZs[5] = {0,0,0,0,0};
    int16_t imaginaryZs[5] = {0,0,0,0,0}; // arrays of 5 admittance values (stored locally- if 5 successive readings don't settle, we'll pick the median value...
    double admittances[5] = {0.0,0.0,0.0,0.0,0.0};  // going to sort realZs and imaginaryZs according to order of admittance magnitude values
        
    if (freq < 15) freq = 15; // minimum signal frequency is ~15 Hz (assuming we are using external clock)
    else if (freq > 100000) freq = 100000;  // maximum signal frequency is 100 kHz

    if (freq > 8000) MCLKext = false; // don't use external clock if requested frequency is above 8000
    else if (freq < 1025) MCLKext = true; // for frequencies below 1025 override any instruction to use internal clock...
    
    if (excitation > 0x03) excitation = 0x03;
    else if (excitation < 0x00) excitation = 0x00;  // set boundaries on excitation code, from datasheet: b00 is 2.0Vp-p, b01 is 200mV, b10 is 400mV, b11 is 1V
    excitationCode = excitation;  // record argument sent to this function in external variable (before it's transformed to the excitation map of AD5933 control register)
    excitation = (excitation + 0x01) & 0x03;// after adding one and "AND"ing off all but bits 0 & 1, map of input argument to actual datasheet excitation codes: b00 => b01(200mV);
                                //b01 => b10(400mV); b10 => b11(1.0V); b11 => b00 (2.0V)

    long fAD5933CLK = fCLK; // by default use frequency of internal clock
    // check the desired setting for external clock, and write appropriate setting to AD5933 control register
    if (MCLKext) {
        //i2c_Write(AD5933NetworkAnalyzer, AD5933_Control + 1, 0x08); // no reset, use external clock 250kHz
        fAD5933CLK = fMCLK; // only overwrite default clock frequency if using external clock...
        MCLK_Enable(true);  // also make sure to power the external clock if we need to use it!
    }
    //else i2c_Write(AD5933NetworkAnalyzer, AD5933_Control + 1, 0x00); // no reset, use internal system clock- 16.776MHz  // don't write to use external clock until ready to give start sweep command

    long DDSphaseAccumulatorFactor = pow(2, 29);  // max step size for DDS Phase Accumulator for output frequency (2^27), * fclk divisor to Phase Accumulator (4)
    
    byte lowCtrlNybble = (excitation << 1); // left shift excitation code into corresponding bits in low nybble of control register, append 1 to LSbit (PGA gain = 1)
    if (!PGAx5) lowCtrlNybble |= 0x01;  // set lowest bit on if PGA gain is only 1 (not 5)
    
    long freq_code = (long) (DDSphaseAccumulatorFactor * (freq / fAD5933CLK));  // using system clock 16.776 MHz and formula freq_code = 4 * f * 2^27/ fclock
    byte regContent = (byte) ((freq_code & 0x00ff0000) >> 16);
    i2c_Write(AD5933NetworkAnalyzer, AD5933_StartFrequency, regContent);
    regContent = (byte) ((freq_code & 0x0000ff00) >> 8);
    i2c_Write(AD5933NetworkAnalyzer, AD5933_StartFrequency + 1, regContent);
    regContent = (byte) (freq_code & 0x000000ff);
    i2c_Write(AD5933NetworkAnalyzer, AD5933_StartFrequency + 2, regContent);  // start frequency is now programmed...
    
    i2c_Write(AD5933NetworkAnalyzer, AD5933_FrequencyIncr, 0x00);
    i2c_Write(AD5933NetworkAnalyzer, AD5933_FrequencyIncr + 1, 0x01);
    i2c_Write(AD5933NetworkAnalyzer, AD5933_FrequencyIncr + 2, 0x40); // frequency increment is now 10 Hz (if set to internal clock), even though we're not going to use it...
    
    i2c_Write(AD5933NetworkAnalyzer, AD5933_IncrNumber, 0x00);
    i2c_Write(AD5933NetworkAnalyzer, AD5933_IncrNumber + 1, 0x01);  // just program a single frequency interval / measurement
    
    i2c_Write(AD5933NetworkAnalyzer, AD5933_SettleCycleNumber, 0x00);
    if (freq < 100) i2c_Write(AD5933NetworkAnalyzer, AD5933_SettleCycleNumber + 1, 0x10);  //allow 16 full cycles of new frequency to be applied for low frequencies
    else if (freq < 1000) i2c_Write(AD5933NetworkAnalyzer, AD5933_SettleCycleNumber + 1, 0x80); // 128 full cycles equilibrarion for freq < 1000 Hz
    else i2c_Write(AD5933NetworkAnalyzer, AD5933_SettleCycleNumber + 1, 0xff); // 255 full cycles equilibration for freq > 1000 Hz
    
    regContent = (0xb0 | lowCtrlNybble);
    i2c_Write(AD5933NetworkAnalyzer, AD5933_Control, regContent);   // put device on standby...

    if (MCLKext) {  // once all other settings are finished, set clock source before starting sweep...
        i2c_Write(AD5933NetworkAnalyzer, AD5933_Control + 1, 0x08); // no reset, use external clock 250kHz
        //fAD5933CLK = fMCLK; // only overwrite default clock frequency if using external clock...
    }
    else i2c_Write(AD5933NetworkAnalyzer, AD5933_Control + 1, 0x00);
    
    regContent = (0x10 | lowCtrlNybble);
    i2c_Write(AD5933NetworkAnalyzer, AD5933_Control, regContent);   // issue "initialize" command (start Vout from digital synthesizer)!
    delay(10000); // delay 10 seconds (easier to find and record signal on oscilloscope)
    
    //delay(2);  // wait 50 ms; additional  crude way of ensuring transients in system settle- unlikely system time constants will be this large
    regContent = (0x20 | lowCtrlNybble);
    i2c_Write(AD5933NetworkAnalyzer, AD5933_Control, regContent); // now issue frequency sweep command (now device will start measuring impedance after equilibration cycles

    regContent = (0x30 | lowCtrlNybble);  // next instruction would be to increment frequency if sweep not complete (but this shouldn't be necessary for 1 increment sweep)

    //Serial.print("mEvaluating first measurement!\t");
    /*byte networkAnalyzerStatus = 0x00;
    while ((networkAnalyzerStatus & 0x07) == 0x00) {
        i2c_Write(AD5933NetworkAnalyzer, AD5933_Control, (0x40 | lowCtrlNybble)); // repeat frequency measurement (?)
        networkAnalyzerStatus = i2c_Read(AD5933NetworkAnalyzer, AD5933_Status);
        Serial.print("mStatus byte: ");
        Serial.print(networkAnalyzerStatus, BIN);
        Serial.print("\t");
        delay(10);
    }*/
    while((i2c_Read(AD5933NetworkAnalyzer, AD5933_Status) & 0x02) == 0x00) {
        delay(1); // wait for status register to confirm that valid conversion/ impedance data measured; don't let MCU timeout...
    }
    
    //Serial.print("mFirst measurement complete!\t");
    
    realZ = (i2c_Read(AD5933NetworkAnalyzer, AD5933_RealData) << 8) | i2c_Read(AD5933NetworkAnalyzer, AD5933_RealData + 1); // first read initial data to get baseline
    imaginaryZ = (i2c_Read(AD5933NetworkAnalyzer, AD5933_ImaginaryData) << 8) | i2c_Read(AD5933NetworkAnalyzer, AD5933_ImaginaryData + 1);

    double readingAdmittance = rawAdmittance(); // estimate raw admittance values from returned real and imaginary components
    double readingPhase = rawPhase(); // estimate raw phase values from returned real and imaginary components
    
    /*int16_t realZabs = abs(realZ);  // find absolute values of admittance components for comparison (only accept stable readings...
    int16_t imaginaryZabs = abs(imaginaryZ);
    realZabs /= 10; // scale absolute magnitude down by factor of 20 (10% threshold)
    if (realZabs < 5) realZabs = 5;
    imaginaryZabs /= 10;  // scale absolute magnitude down by 20 (10% threshold)
    if (imaginaryZabs < 5) imaginaryZabs = 5; // don't let comparison threshold be too small...
    int16_t realZdeltaplus = realZ0 + realZabs;
    int16_t realZdeltaminus = realZ0 - realZabs;
    int16_t imaginaryZdeltaplus = imaginaryZ0 + imaginaryZabs;
    int16_t imaginaryZdeltaminus = imaginaryZ0 - imaginaryZabs;*/

    double admittanceThreshold = readingAdmittance / 20.0; // set allowable tolerance for successive observations as 5%
    double phaseThreshold = 5.0;  // set allowable tolerance for phase on successive observations as 5 degrees
    
    while (!endOfSweep) {
      if ((i2c_Read(AD5933NetworkAnalyzer, AD5933_Status) & 0x04) == 0x00) {  // check status register- as long as sweep not complete...
        if ((readingAdmittance > 20) && (readingAdmittance < AD5933_GperVoltOut)) { // don't bother repeating measurements for extremely small or extremely large admittances
          if ((cycleCount == 0) || (abs(readingAdmittance - rawAdmittance()) > admittanceThreshold) || (abs(readingPhase - rawPhase()) > phaseThreshold)) { // if any values are unacceptably out of range repeat measurement (+/- 5% admittance / 5degree phase)
            if (cycleCount < 5) { // if haven't tried 5 measurements to get stable reading, read frequency again...
              readingAdmittance = rawAdmittance(); // update raw admittance values for next comparison
              readingPhase = rawPhase();  // ...
              
              realZs[cycleCount] = realZ; // save latest realZ observation in local array...
              imaginaryZs[cycleCount] = imaginaryZ; // save latest imaginaryZ observation in local array...
              admittances[cycleCount] = readingAdmittance;

              admittanceThreshold = abs(readingAdmittance / 20.0); // set (new) allowable tolerance for successive observations as 5% of last reading
                // update threshold before taking next reading...
              //phaseThreshold = 5.0;  // set allowable tolerance for phase on successive observations as 5 degrees 
              
              i2c_Write(AD5933NetworkAnalyzer, AD5933_Control, (0x40 | lowCtrlNybble)); // instruction to repeat measurement at same frequency
              //Serial.print("mZ out of range, requesting new value\t");
              cycleCount++;
              
              /*realZabs = abs(realZ);  // find absolute values of admittance components for comparison (only accept stable readings...
              imaginaryZabs = abs(imaginaryZ);
              realZabs /= 10; // scale absolute magnitude down by factor of 20 (10% threshold)
              if (realZabs < 5) realZabs = 5;
              imaginaryZabs /= 10;  // scale absolute magnitude down by 20 (10% threshold)
              if (imaginaryZabs < 5) imaginaryZabs = 5; // don't let comparison threshold be too small...
              realZdeltaplus = realZ0 + realZabs;
              realZdeltaminus = realZ0 - realZabs;
              imaginaryZdeltaplus = imaginaryZ0 + imaginaryZabs;
              imaginaryZdeltaminus = imaginaryZ0 - imaginaryZabs;*/
              
              while((i2c_Read(AD5933NetworkAnalyzer, AD5933_Status) & 0x02) == 0x00) {
                if (freq < 300) delay(1); // wait for status register to confirm that valid conversion/ impedance of new data measured (at low frequencies include delay so no timeout
              }
              
              realZ = (i2c_Read(AD5933NetworkAnalyzer, AD5933_RealData) << 8) | i2c_Read(AD5933NetworkAnalyzer, AD5933_RealData + 1); // read new data to compare to old
              imaginaryZ = (i2c_Read(AD5933NetworkAnalyzer, AD5933_ImaginaryData) << 8) | i2c_Read(AD5933NetworkAnalyzer, AD5933_ImaginaryData + 1);

              if (readingAdmittance == rawAdmittance())  { // if we get the exact same number on second measurement (and it's not really small or really large), seems like device is stuck
                Serial.print("mJust observed consecutive identical measurements- cycle power on AD5933\t");
                AD5933_PowerOn(false);
                delay(10);
                AD5933_PowerOn(true);
                delay(100);
                endOfSweep = true;  // hard code to force this routine to exit...
                return false;
              }
              else { // otherwise keep making measurements until they are close...
                Serial.print("mRequesting another measurement! cycleCount: ");
                Serial.print(cycleCount);
                Serial.print(", Frequency: ");
                Serial.print(freq);
                Serial.print("\t");
              }
            }
            else {  // otherwise just give up - made 5 non-repeatable measurements; just take the median observed value and go on to the next frequency
              cycleCount = 0;
              i2c_Write(AD5933NetworkAnalyzer, AD5933_Control, regContent); // instruction to increment frequency (will result in end of sweep since we are only evaluating 1 frequency)

              int admittanceIndeces[5] = {0,1,2,3,4}; // array of indeces- sort these simultaneously with admittance values to indeces for median values of realZ and imaginaryZ
              for (int sorter = 0; sorter < 5; sorter++) {  // evaluate array of admittances to identify index of median value...
                  for (int compareIndex = sorter + 1; compareIndex < 5; compareIndex++) {
                      if (admittances[sorter] > admittances[compareIndex]) {
                          int storageIndex = admittanceIndeces[sorter];
                          admittanceIndeces[sorter] = admittanceIndeces[compareIndex];
                          admittanceIndeces[compareIndex] = storageIndex;
                          
                          double storageValue = admittances[sorter];
                          admittances[sorter] = admittances[compareIndex];
                          admittances[compareIndex] = storageValue;
                      }
                  }
              }
              // now admittance values are sorted- pick the starting index of the middle / median value to identify indeces of realZ and imaginaryZ to use...
              int medianIndex = admittanceIndeces[2];
              realZ = realZs[medianIndex];
              imaginaryZ = imaginaryZs[medianIndex];
              Serial.print("mMeasurements not repeatable; returning median of 5 values!\t");
            }
          }
          else { // otherwise we made a repeatable measurement; just use the most recent values of realZ and imaginaryZ go to next frequency...
            cycleCount = 0; // we were able to get a repeatable measurement at given conditions- move to next frequency
            i2c_Write(AD5933NetworkAnalyzer, AD5933_Control, regContent); // otherwise consecutive measurements are acceptably close, so keep latest values
            Serial.print("mWe made a good measurement!\t");
          }
        }
        else { // otherwise the measurement is out of range (not going to get good value at current settings), so just go to next frequency...
          cycleCount = 0;
          i2c_Write(AD5933NetworkAnalyzer, AD5933_Control, regContent); // otherwise measurement settings don't seem very good for good resolution, so just leave it/ go to next frequency...
          Serial.print("mSettings give out of range readings!\t");
        }
      }
      else {
        Serial.print("mEnd of sweep!\t");
        endOfSweep = true;
      }
      Serial.print("mMeasurement Completed!\t"); // we managed to get through reading before device registers sweep complete (?)
      endOfSweep = true;
    }
    regContent = (0xa0 | lowCtrlNybble);  // control register content to put device in power down mode
    i2c_Write(AD5933NetworkAnalyzer, AD5933_Control, regContent);   // put device in power down mode...
    frequency = freq;
    //excitationCode = excitation;
    if (readingAdmittance == -1.0) {
        
    }
    MCLK_Enable(false); // make sure to turn off clock at end of measurement...
    PGAx5Setting = PGAx5; // save settings for successful measurement (so returned values can be used to estimate impedance and phase values...
    return true;
 }

