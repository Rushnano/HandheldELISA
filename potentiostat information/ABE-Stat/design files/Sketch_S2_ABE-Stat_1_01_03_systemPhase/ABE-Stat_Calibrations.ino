/*
 * Creator: Daniel M. Jenkins
 * Date: July 30, 2017
 * Description: Basic calibration routines for ABE-Stat (i.e. calibrate DAC, transimpedance gains, 
 * system phase and admittance / impedance coefficients...
 * 
 * Partitioned into separate libraries, including for higher level Analog Circuit reading / writing, and 
 * for configuring and reading network analyzer results (7/30/2017)
 */
 
/*
 * After startup, call into Calibration function to manage calibrations with 'c'
 */
 void Calibrate() {

    delay(1500); // hard code delay just so that information doesn't get garbled with incoming data...
    Serial.print("mCalibration Data:");
    Serial.print(vslope, 4);
    Serial.print(',');
    Serial.print(voffset, 4);
    Serial.print(',');
    Serial.print(vlowlimit, 3);
    Serial.print(',');
    Serial.print(vhighlimit, 3);
    Serial.print(',');
    Serial.print(Gain0, 1);
    Serial.print(',');
    Serial.print(Gain1, 1);
    Serial.print(',');
    Serial.print(Gain2, 1);
    Serial.print(',');
    Serial.print(Gain3, 1);
    Serial.print(',');
    Serial.print(AD5933_GperVoltOut,1);
    Serial.print(',');
    Serial.print(AD5933_BiasuV);
    Serial.print(',');
    for (int AD5933Settings = 0; AD5933Settings < 48; AD5933Settings++) {
      Serial.print(phaseSlope[AD5933Settings], 6);
      Serial.print(',');
      Serial.print(phaseOffset[AD5933Settings], 1);
      Serial.print(',');
      Serial.print(ZSlope[AD5933Settings], 6);
      Serial.print(',');
      Serial.print(ZOffset[AD5933Settings], 1);
      Serial.print(',');
    }
    Serial.print('\t');

    completedCalibrationsCode = 0x0000;
    calibratedAD5933biasAndGperVolt = false;
    boolean ex = false;
    while (!ex) {
      while (!Serial.available());
      command = Serial.read();
      switch(command) {
        case 'b':
          calibrateBattery();
          break;
        case 'd':
          vslope = Serial.parseFloat();
          voffset = Serial.parseFloat();
          EEPROM_Double_Write(DAC_SLOPE_EEPROM_ADDRESS, vslope);
          EEPROM_Double_Write(DAC_OFFSET_EEPROM_ADDRESS, voffset);
          EEPROM.commit();
          break;
        case 'e':
          writePhaseCoefficients();
          break;
        /*case 'f':
          calibratePhase();
          break;*/
        case 'g':
          calibrateGain();
          break;
        case 'v':// Android says set voltage...
          calibrateDAC();
          break;
        case 'x':// exit calibration routine
          ex = true;
          break;
        case 'y':
          writeAdmittanceCoefficients();
          break;
        case 'z':
          calibrateZ();
          break;
        default:
          break;
      }
    }
 }

 /*
  * read the returning coefficients of system phase calibration and write into EEPROM
  */
  void writePhaseCoefficients() {
          int settingsIndex = Serial.parseInt();
          phaseSlope[settingsIndex] = Serial.parseFloat();
          phaseOffset[settingsIndex] = Serial.parseFloat();
          int EEPROMOffsetAddress = (4 * EEPROMBlockSize) * settingsIndex;  // offset from given base address for given index
          EEPROM_Double_Write(PHASE_SLOPE_EEPROM_ADDRESS + EEPROMOffsetAddress, phaseSlope[settingsIndex]);
          EEPROM_Double_Write(PHASE_OFFSET_EEPROM_ADDRESS + EEPROMOffsetAddress, phaseOffset[settingsIndex]);
          EEPROM.commit();
  }

  /*
  * read the returning coefficients of admittance* load resistance calibration and write into EEPROM
  */
  void writeAdmittanceCoefficients() {
          int settingsIndex = Serial.parseInt();
          ZSlope[settingsIndex] = Serial.parseFloat();
          ZOffset[settingsIndex] = Serial.parseFloat();
          int EEPROMOffsetAddress = (4 * EEPROMBlockSize) * settingsIndex;  // offset from given base address for given index
          EEPROM_Double_Write(Z_SLOPE_EEPROM_ADDRESS + EEPROMOffsetAddress, ZSlope[settingsIndex]);
          EEPROM_Double_Write(Z_OFFSET_EEPROM_ADDRESS + EEPROMOffsetAddress, ZOffset[settingsIndex]);
          EEPROM.commit();
  }

 /*
  * Calibrate the full charge value of battery (assumes that battery is fully charged)
  */
 void calibrateBattery() {
          unsigned int fullCharge = 0;
          for (int refV = 0; refV < 64; refV++) {
            fullCharge += analogRead(0);
          }
          EEPROM_Int_Write(V_BATT_FULL_EEPROM_ADDRESS, fullCharge);
          EEPROM.commit();
 }

/*
 * Set desired nominal voltage, and report this with actual voltage so Android can determine calibration equation
 * Then find the actual voltage for lowest and highest voltage sets...
 */
 void calibrateDAC() {
      AD5933_PowerOn(false);  // remove power from AD5933 for calibrating DAC
      DAC_AD5061_Connect(true); // connect AD5061 Digital to Analog Converter from network
      AD5933_Connect(false);  // AND connect network analyzer input (here we are testing the superposition of the two signal sources)
      setTIAGain(0x01);  // make sure selected feedback resistor connected for TIA (otherwise non-ideal op-amp performance can bias the "reference"
      TIA_LMP7721();  // connect to TIA amplifier LMP7721 to ensure virtual ground reference...
      twoElectrodeConfig(); // to allow calibration with single resistor between Working and reference contacts
      double nominalVoltage = -1.50;
      while (nominalVoltage <= 1.51) {
        setAndReportVoltage(nominalVoltage);
        delay(40);
        nominalVoltage += 0.05;
      }
      DAC_AD5061_SetVoltage(-1.65);
      delay(10);  // let transients settle...
      vlowlimit = -ADS1220_Diff_Voltage(0x04, 0x00) / 1000000.0;
      DAC_AD5061_SetVoltage(1.65);
      delay(10);  // let transients settle...
      vhighlimit = -ADS1220_Diff_Voltage(0x04, 0x00) / 1000000.0;
      EEPROM_Double_Write(DAC_LOW_LIMIT_EEPROM_ADDRESS, vlowlimit);
      EEPROM_Double_Write(DAC_HIGH_LIMIT_EEPROM_ADDRESS, vhighlimit);
      EEPROM.commit();
      Serial.print('l');  // flag that voltage range limits coming...
      Serial.print(vlowlimit, 4);
      Serial.print('\t');
      Serial.print(vhighlimit, 4);
      Serial.print('\t');
      Serial.print('c');  // flag that the linear calibration range has been completed.
 }

/*
 * Set voltage to desired level, then report back observed voltage
 */
void setAndReportVoltage(double volts) {
      DAC_AD5061_SetVoltage(volts);
      delay(10);  // let transients settle...
      double actualVoltage = -ADS1220_Diff_Voltage(0x04, 0x00) / 1000000.0;
      Serial.print('v');
      Serial.print(volts, 4);
      Serial.print('\t');
      Serial.print(actualVoltage, 4);
      Serial.print('\t');
 }

/*
 * Calibrate system phase (folded into calibrateZ function as of ABE-Stat1_0_06
 */
/*void calibratePhase() {
    //DAC_AD5061_Connect(false); // remove DAC_AD5061 signal...
    DAC_AD5061_Connect(true); // connect DAC from AD5061 to network...  // calibration appears to be different with and without AD5061, so go ahead and connect,
          // then for calibrations set bias to 0- so make all measurements connected to this network (and apply "0" bias / output voltage)
    DAC_AD5061_SetCalibratedVoltage(0.0); // here we'll apply unbiased voltage
    AD5933_Connect(true);  // AND connect network analyzer voltage source to network
    TIA_AD5933();  // connect network back to TIA on network analyzer AD5933
    twoElectrodeConfig(); // to allow calibration with single resistor between Working and reference contacts
    long frequent = 6000; // start at frequency of 6000 (at lower frequencies, finite samples at given clock rate do not cover enough cycles for accurate FFT
    autoRangeTestImpedance(frequent); // use autoranging function at first to find settings to get reliable admittance values at frequency
    while (frequent < 100500) {
       TestImpedance(frequent, excitationCode, PGAx5Setting);
       Serial.print('f');
       Serial.print(frequent);
       Serial.print('\t');
       Serial.print(rawPhase(), 2); // send the raw phase value (during calibration, with resistor, raw phase is the system phase...
       Serial.print('\t');
       frequent += 1000;  // increment frequency and take / report new calibration measurement
    }
    Serial.print('c');  // flag that the linear calibration range has been completed (Android can now Execute_LSR() with compiled data)
}*/

/*
 * Calibrate impedances
 */
void calibrateZ() {
    double networkR = cellResistance(); // As of ABE-Stat1_0_06, evaluate the load resistance for AD5933 impedance calibrations- don't just read Android input from user Serial.parseFloat();
    AD5933_PowerOn(true); // now turn on the network analyzer- this is what we're actually calibrating!
    delay(100);  // small delay to let AD5933 power up and get itself sorted out...
    Serial.print('r');
    Serial.print(networkR, 1);
    Serial.print('\t'); // reports the load resistance (impedance)
    boolean completedAvailableSettings = false;
    double admitCoefficient = 1.0;
    DAC_AD5061_Connect(true); // connect DAC from AD5061 to network...  // calibration appears to be different with and without AD5061, so go ahead and connect,
          // then for calibrations set bias to 0- so make all measurements connected to this network (and apply "0" bias / output voltage)
    DAC_AD5061_SetCalibratedVoltage(0.0); // here we'll apply unbiased voltage
    AD5933_Connect(true);  // AND connect network analyzer voltage source to network
    TIA_AD5933();  // connect network back to TIA on network analyzer AD5933
    twoElectrodeConfig(); // to allow calibration with single resistor between Working and reference contacts
    PGAx5Setting = true;
    TIAGainCode = 0x02;
    setTIAGain(TIAGainCode);
    excitationCode = 0x03;
    double Vexcitation = 0.1;
    while (!completedAvailableSettings) {
        int AD5933settingIndex = excitationCode + (TIAGainCode * 4);
        if (PGAx5Setting) AD5933settingIndex += 12;
        switch (excitationCode) {
            case 0x00:
              Vexcitation  = 0.01;
              break;
            case 0x01:
              Vexcitation  = 0.02;
              break;
            case 0x02:
              Vexcitation  = 0.05;
              break;
            case 0x03:
              Vexcitation  = 0.1;
              break;
            default:
              break;
        }
        double Voutexpected = (TIAGain / networkR) * Vexcitation; // estimate expected TIA output voltage based on settings and load impedance
        if (PGAx5Setting) Voutexpected *= 5.0;
        MCLKext = false;  // make first test measurement with internal clock / higher frequency
        if ((Voutexpected > 0.2) && (Voutexpected < 1.0)) {  // we haven't calibrated the admittance coeffient corresponding to 1V, or bias, and conditions appear to be good for it...
            if (TestImpedance(20000, excitationCode, PGAx5Setting)) { // test impedance at given conditions- at 20kHz (make sure we're well above bottom of range so we get good number of cycles sampled.
                  AD5933_GperVoltOut = rawAdmittance() / Voutexpected;
            }
            else AD5933_GperVoltOut = 10000;
            //openCircuitConfig();  // put in open circuit configuration to test the bias voltage (if resistor connects working to reference, the ref voltage will be working voltage bias
              // from AD5933 relative to circuit board analog reference; AD5933 internal reference and board reference are both supposed to be Vcc / 2, but not actually physically connected
            /*
            Evidently AD5933 Vin is connected to system ground (not virtual ground / split rail voltage) until configured to generate signal on Vout; so cannot measure system bias of AD5933
            virtual ground relative to analog reference unless configured to generate AD5933 signal; will defer this part of software for later...
            So AD5933_BiasuV returned here is useless; go ahead and leave it in, but don't use it anywhere- i.e. just assume AD5933 bias is 0 for now...
            */
            delay(400);  // let transient occur after removing Vout signal of AD5933 following impedance measurement
            AD5933_BiasuV = ADS1220_Diff_Voltage(0x04, 0x00); // when working and reference electrodes connected in open circuit config (and working connected to AD5933),
            //
              // +ve value returned is actually the bias on working electrode relative to analog reference- which is the bias we want to correct for (subtract out) later for AD5933 measurements
            //twoElectrodeConfig(); // put it back into two electrode configuration after measuring AD5933 bias voltage (so we can actually measure impedance with applied signal)
            //delay(20); // let transients die again after changing  back electrode configuration...
            Serial.print("h");
            Serial.print(AD5933_GperVoltOut, 1);
            Serial.print("\t");
            Serial.print(AD5933_BiasuV, 1);
            Serial.print("\t");
            if (!calibratedAD5933biasAndGperVolt) {
                  EEPROM_Double_Write(AD5933_G_PER_V_EEPROM_ADDRESS, AD5933_GperVoltOut);
                  EEPROM_Double_Write(AD5933_BIAS_uV_EEPROM_ADDRESS, AD5933_BiasuV);
                  EEPROM.commit();
                  calibratedAD5933biasAndGperVolt = true; // we've calibrated values, so don't overwrite again this invocation of calibration app...
                  }
        }
        long bitTest = (0x0001 << AD5933settingIndex);
        boolean conditionTested = (completedCalibrationsCode & bitTest) >> AD5933settingIndex; // determine whether this condition has been calibrated yet...
        if (!conditionTested && (Voutexpected > 0.09) && (Voutexpected < 1.0)) {  // if we haven't calibrated current settings, and predicted output looks like a suitable range calibrate this setting!
            
            MCLKext = false;  // for higher frequencies use the internal AD5933 clock
            double freeIncrement = fCLK / (16.0 * 1024.0);
            double frequent = freeIncrement; // start at frequency of 15000 (at lower frequencies, finite samples at given clock rate do not cover enough cycles for accurate FFT
            while (frequent < 100500) {
              if (TestImpedance(frequent, excitationCode, PGAx5Setting)) {
                  admitCoefficient = networkR * rawAdmittance();
                  Serial.print('z');
                  Serial.print(AD5933settingIndex);
                  Serial.print('\t');
                  Serial.print(frequent);
                  Serial.print('\t');
                  Serial.print(admitCoefficient, 2);
                  Serial.print('\t');
                  Serial.print(rawPhase(), 2);
                  Serial.print('\t');
                  frequent += freeIncrement;  // increment frequency and take / report new calibration measurement
                  delay(1); // don't let MCU timeout...
              } // if not valid result, make network analyzer repeat measurement at same frequency (after cycling power in TestImpedance())
            }
            Serial.print('c');  // flag that the linear calibration range has been completed (Android can now Execute_LSR() with compiled data)
            // completed calibration of performance at given settings over frequencies with internal AD5933 clock

            AD5933_PowerOn(false);  // turn off power to let device rest between calibration settings
            delay(50);  // wait a little to make sure calibration data come back, and to prevent MCU from timing out...
            while (Serial.available()) {
              char commandant = Serial.read();
              if (commandant == 'y') writeAdmittanceCoefficients();
              else if (commandant == 'e') writePhaseCoefficients();
            }

            MCLKext = true;  // for lower frequencies use the external 250kHz clock for AD5933
            AD5933_PowerOn(true);  // restore power to analyze calibration settings with external clock...
            delay(100);  // turn chip back on, and let it settle a little bit...
            AD5933settingIndex += 24; // new range of setting indeces for calibrations with external clock (otherwise index order for AD5933 settings the same)
            freeIncrement = fMCLK / (16.0 * 1024.0); // for external MCLK frequency increment, to cover integer number of cycles with 1024 ADC samples at fclk / 16
            frequent = freeIncrement;
            double fmax = (fMCLK / 100.0) - 1.0;  // ~2.5kHz for 250kHz clock
            while (frequent < (fmax)) {
              Serial.print("mEvaluating ");
              Serial.print(frequent);
              Serial.print("Hz\t");
              if (TestImpedance(frequent, excitationCode, PGAx5Setting)) {
                    admitCoefficient = networkR * rawAdmittance();
                    Serial.print('z');
                    Serial.print(AD5933settingIndex);
                    Serial.print('\t');
                    Serial.print(frequent);
                    Serial.print('\t');
                    Serial.print(admitCoefficient, 2);
                    Serial.print('\t');
                    Serial.print(rawPhase(), 2);
                    Serial.print('\t');
                    frequent += freeIncrement;
                    //if (frequent < 200.0) frequent += freeIncrement;  // increment frequency and take / report new calibration measurement
                    //else frequent += (freeIncrement * 4.0); // increase frequency 
                    delay(1); // don't let MCU timeout...
              }// if result returned doesn't make sense, frequency is not incremented and same frequency value is requested again after cycling power on AD5933...
            }
            Serial.print('c');  // flag that the linear calibration range has been completed (Android can now Execute_LSR() with compiled data)

            AD5933_PowerOn(false);  // turn off power to let device rest between calibration settings
            delay(50);  // wait a little to make sure calibration data come back, and to prevent MCU from timing out...
            while (Serial.available()) {
              char commandant = Serial.read();
              if (commandant == 'y') writeAdmittanceCoefficients();
              else if (commandant == 'e') writePhaseCoefficients();
            }
            
            completedCalibrationsCode = (completedCalibrationsCode | bitTest);  // add current conditions to set of that calibration is complete
            Serial.print('i');  // now send Android the latest completed settings code to display (so user knows if more conditions need to be calibrated, with different networkR
            Serial.print(completedCalibrationsCode, BIN);  // send as binary number to facilitate identification of settings remaining to calibrate...
            Serial.print('\t');
        } // now we've either completed calibration at given setting, or determined that it has already been completed or is expected to result in sub-optimal results with given networkR

        if (excitationCode > 0x00) {  // so update settings and run through cycle again...
            excitationCode--;
        }
        else if (PGAx5Setting) {
            PGAx5Setting = false;
            excitationCode = 0x03;  // start back at excitation code = 0x03 at new PGA setting
        }
        else if (TIAGainCode > 0x00) {
            TIAGainCode--;
            setTIAGain(TIAGainCode);
            PGAx5Setting = true;
            excitationCode = 0x03;
        }
        else {
            completedAvailableSettings = true; // if excitation, gain, and PGA settings are *all* at lowest values at completion of calibration, we've completed every possible combination...
            Serial.print("y");  // flag for Android to know that we've calibrated as many settings as give good results with given networkR (so swap out resistor and start calibration again)
        }
    }
}

/*
 * Calibrate TIA gain
 */
void calibrateGain() {
    AD5933_PowerOn(false);  // remove power from AD5933- just a noise source for measuring feedback resistors / gains...
    int index = Serial.parseInt();
    if (index < 0) index = 0;
    else if (index > 0x03) index = 0x03;
    double networkR = Serial.parseFloat();
    DAC_AD5061_Connect(true); // connect AD5061 Digital to Analog Converter from network
    AD5933_Connect(false);  // AND connect network analyzer input (here we are testing the superposition of the two signal sources)
    setTIAGain(index);  // make sure selected feedback resistor connected for TIA (otherwise non-ideal op-amp performance can bias the "reference"\
    TIA_LMP7721();  // connect to TIA amplifier LMP7721 to ensure virtual ground reference...
    twoElectrodeConfig(); // to allow calibration with single resistor between Working and reference contacts
    double vamp = 2.1;
    double vsource = 1.5;
    double vworking = 0.0;
    while (vamp > (vhighlimit - 0.02)) {  // set threshold somewhat low- is it not trigering near the voltage reference value 2.048 (?)
      DAC_AD5061_SetVoltage(vsource);
      delay(10); // let transient settle
      vworking = -ADS1220_Diff_Voltage(0x04, 0x00) / 1000000.0;
      vamp = ADS1220_Diff_Voltage(0x02, 0x00) / 1000000.0;
      vsource /= 2.0; // if offscale divide source voltage by 2- keep repeating until get onto scale for valid measurement
    }
    double gain = networkR * vamp / vworking;
    switch (index) {
      case 0x00:
        Gain0 = gain;
        EEPROM_Double_Write(GAIN_0_EEPROM_ADDRESS, gain);
        break;
      case 0x01:
        Gain1 = gain;
        EEPROM_Double_Write(GAIN_1_EEPROM_ADDRESS, gain);
        break;
      case 0x02:
        Gain2 = gain;
        EEPROM_Double_Write(GAIN_2_EEPROM_ADDRESS, gain);
        break;
      case 0x03:
        Gain3 = gain;
        EEPROM_Double_Write(GAIN_3_EEPROM_ADDRESS, gain);
        break;
      default:
        break;
    }
    EEPROM.commit();
    Serial.print('g');
    Serial.print(index);
    Serial.print('\t');
    Serial.print(gain, 1);
    Serial.print('\t');
}

