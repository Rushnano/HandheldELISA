/*
 * Creator: Daniel M. Jenkins
 * Date: August 20, 2017
 * Description: First functions for parsing instructions to conduct an electrochemical / analytical method and reporting data back to ABE-Stat.apk...
 * 
 */
 
/*
 * Parse command for selected analytical method...
 * Note that electrode configuration is not set here- make sure it is set in the main "loop" function before getting sent here!
 */
 void startAnalyticalMethod() {
    boolean ex = false;
    while (!ex) {
      while (!Serial.available());
      command = Serial.read();
      switch(command) {
        case 'a':
          AnodicStrippingVoltammetry();
          ex = true; // always exit analytical method routine when selected method is complete...
          break;
        case 'c':
          CyclicVoltammetry();
          ex = true; // always exit analytical method routine when selected method is complete...
          break;
        case 'd':
          DifferentialPulseVoltammetry();
          ex = true; // always exit analytical method routine when selected method is complete...
          break;
        case 'e':
          ElectrochemicalImpedanceSpectroscopy();
          ex = true; // always exit analytical method routine when selected method is complete...
          break;
        case 'x':// exit calibration routine
          ex = true;
          break;
        default:
          break;
      }
    }
}

void AnodicStrippingVoltammetry() {
      
}

/*
 * Cyclic voltammetry; settings read as starting potential (in V), end potential (in V), scan rate (in V / s). Potential is updated every 
 * Note that electrode configuration needs to be set before going to this routine...
 */
void CyclicVoltammetry() {
      TIA_LMP7721();  // set TIA to return to LMP7721 and ADS1220 ADC...
      DAC_AD5061_Connect(true); // connect DAC from AD5061 to network...
      AD5933_Connect(false);
      setTIAGain(0x03); // initially start at most sensitive gain (will dial down if offscale later)
      int incrementTime = 1; // time in millisseconds at which potential is updated (note that voltage is incremented more often than measurements are made)...
      double measureCVIncrement = 0.005;  // applied potential increments (in V) at which data is recorded...
      double startE = Serial.parseFloat();  // start potential (in V)
      double endE = Serial.parseFloat();  // end potential (in V)
      double scanRate = Serial.parseFloat();  // scan rate in V / s (so at 5000 microsecond increment time, voltage increments are ~ 1 mV at highest scan rate of 0.2 V/s)
      double voltageIncrement = scanRate * incrementTime / 1000.0; // voltage change to DAC every incrementTime
      double currentVoltage = startE; // keep track of ongoing voltage setting...
      DAC_AD5061_SetCalibratedVoltage(currentVoltage);  // first set the potential at the starting value...

      boolean endOfCycle = false;
      boolean endOfAnalysis = equilibrationPeriod();  // request the duration of equilibration period; returns "true" if user exits analysis during equilibration...

      double currentGarbage = cellCurrent();  // just invoke cellCurrent() function to one more time to find TIAGain setting for measuring current before changing to fast voltammetry settings.
      
      long updateTime = millis() + incrementTime; // now we've completed setup and configuration reading- start scanning!
      double measureCVThreshold = startE + measureCVIncrement;
      voltammetryScanSettings();  // speed up data rate (1200 sps), turn off filters (not useful at higher data rate anyway), PGA off, leave in single shot mode (need to solicit data)

      boolean oxidationScan = true; // boolean to determine direction of sweep ("oxidation" is increasing potential scan) first scan of cycle is oxidation scan...
      ReportVoltammetryValues(0.0); // report observed values of cell potential and current- not a differential measurement so just send 0.0 for "preStepCurrent"
      
      while (!endOfAnalysis) {
          while (!endOfAnalysis && !endOfCycle) {
             while (millis() < updateTime) {
                if (millis() < (updateTime - incrementTime)) {
                  updateTime = millis() + incrementTime;  // check for and handle any overruns in microsecond timer...
                }
                if (Serial.available()) { // listen for instructions to exit
                  char instruction = Serial.read();
                  switch (instruction) {
                    case 'x':
                        endOfAnalysis = true; // instruction received to exit analysis...
                        break;
                    default:
                        break;
                  }
                }
             }
             delay(1); // put in a delay so ESP8266 has time to take care of any background WiFi or other tasks...
             updateTime += incrementTime;  // increment the update time for setting potential
             if (oxidationScan) {
                if (currentVoltage >= measureCVThreshold) {
                    measureCVThreshold += measureCVIncrement;
                    ReportVoltammetryValues(0.0);  // increment threshold for next data report, and then send data report for cell potential and current; not differential measurement so 0 preStepCurrent
                }
                currentVoltage += voltageIncrement;  // increment voltage to new setting
                if (currentVoltage > endE) {
                    oxidationScan = false;  // set scan direction to negative / reduction after oxidation sweep is complete
                    currentVoltage -= voltageIncrement;
                    measureCVThreshold -= measureCVIncrement;
                }
             }
             else {   // we are in reduction scan going from endE to startE...
                if (currentVoltage <= measureCVThreshold) {
                    measureCVThreshold -= measureCVIncrement;
                    ReportVoltammetryValues(0.0);
                }
                currentVoltage -= voltageIncrement;  // begin decrementing voltage for reduction scan...
                if (currentVoltage < startE) { // once we reach the start potential on reduction sweep, cycle has ended...
                    Serial.print('c');  // send back 'c' for "Cycle Complete" (android will respond with instruction to exit, or to run another cycle...
                    endOfCycle = true;
                    currentVoltage = startE;
                    oxidationScan = true; // set it back to oxidation sweep if another cycle is requested...
                    measureCVThreshold = startE + measureCVIncrement;
                }
             }
             DAC_AD5061_SetCalibratedVoltage(currentVoltage);
          } // end of one CV cycle...
          while (!Serial.available() && !endOfAnalysis) {
            delay(1);  // now await further instructions; Android will either ask for another cycle, or quit / exit...
          }
          if (!endOfAnalysis) {
            char nextInstruction = Serial.read();
            switch (nextInstruction) {
              case 'c': // instruction to repeat/ conduct another CV cycle...
                endOfCycle = false;
                break;
              case 'x': // instruction to end Analysis (this will cause us to completely exit analytical methods and return to main loop()
                endOfAnalysis = true;
                break;
              default:
                endOfAnalysis = true;
                break;
             } // end switch(nextInstruction)
          }
      } // end while(!endOfAnalysis)
}

void DifferentialPulseVoltammetry() {
      TIA_LMP7721();  // set TIA to return to LMP7721 and ADS1220 ADC...
      DAC_AD5061_Connect(true); // connect DAC from AD5061 to network...
      AD5933_Connect(false);
      setTIAGain(0x03); // initially start at most sensitive gain (will dial down if offscale later)
      double startE = Serial.parseFloat();  // start potential (in V)
      double endE = Serial.parseFloat();  // end potential (in V)
      int stepFrequency = Serial.parseInt();
      double stepE = Serial.parseInt() / 1000.0;  // convert from mV to V
      double pulseAmp = Serial.parseInt() / 1000.0; // convert from mV to V
      double currentVoltage = startE - pulseAmp; // keep track of ongoing voltage setting- start pulse amplitude short of start voltage (we can get a differential measurement at start Voltage)
      DAC_AD5061_SetCalibratedVoltage(currentVoltage);  // first set the potential at the starting value...
      
      boolean endOfCycle = false;
      boolean endOfAnalysis = equilibrationPeriod();  // request the duration of equilibration period; returns "true" if user exits analysis during equilibration...

      double currentGarbage = cellCurrent();  // just invoke cellCurrent() function to one more time to find TIAGain setting for measuring current before changing to fast voltammetry settings.
      
      voltammetryScanSettings();  // speed up data rate (1200 sps), turn off filters (not useful at higher data rate anyway), PGA off, leave in single shot mode (need to solicit data)

      long incrementTime = 500000 / stepFrequency;  // micros increments at which applied potential is changed (2 x the frequency, as potential is stepped up and down each cycle)
      long updateTime = micros() + incrementTime; // now we've completed setup and configuration reading- start scanning!

      boolean oxidationScan = true; // boolean to determine direction of sweep ("oxidation" is increasing potential scan) first scan of cycle is oxidation scan...
      boolean stepUp = false; // initially we're going to measure the preStep Current

      double preStepCurrent = 0.0;  // current in nA

      while (!endOfAnalysis) {
          while (!endOfAnalysis && !endOfCycle) {
             while (micros() < updateTime) {
                if (micros() < (updateTime - incrementTime)) {
                  updateTime = micros() + incrementTime;  // check for and handle any overruns in microsecond timer...
                }
                if (Serial.available()) { // listen for instructions to exit
                  char instruction = Serial.read();
                  switch (instruction) {
                    case 'x':
                        endOfAnalysis = true; // instruction received to exit analysis...
                        break;
                    default:
                        break;
                  }
                }
             }
             delay(1); // put in a delay so ESP8266 has time to take care of any background WiFi or other tasks...
             updateTime += incrementTime;  // increment the update time for setting potential
             if (oxidationScan) {
                if (!stepUp) {
                    ADS1220_SetSingleCtrlRegister(0x00, 0x21);  // configure to measure transimpedance amp voltage (Vtia vs. analog ref)
                    ADS1220_Read_Data();
                    preStepCurrent = (ADS1220_microVolts(0x00) * 1000.0 / TIAGain);  // current in nA
                    currentVoltage += pulseAmp;
                    stepUp = true;
                    if (currentVoltage > (endE + pulseAmp)) {
                      oxidationScan = false;  // now we need to sweep back on reduction scan
                      currentVoltage = endE + pulseAmp; // and set up to record pre step current...
                      stepUp = false;
                    }
                    DAC_AD5061_SetCalibratedVoltage(currentVoltage);  // now we've measured the pre-step current- apply the step and toggle flag so we measure post step next time
                }
                else {
                    ReportVoltammetryValues(preStepCurrent);
                    currentVoltage += (stepE - pulseAmp); // step back down to pre-step values
                    DAC_AD5061_SetCalibratedVoltage(currentVoltage);  // now we've measured the pre-step current- apply the step and toggle flag so we measure post step next time
                    stepUp = false;
                }
             }
             else {   // we are in reduction scan going from endE to startE...
                if (!stepUp) {
                    ADS1220_SetSingleCtrlRegister(0x00, 0x21);  // configure to measure transimpedance amp voltage (Vtia vs. analog ref)
                    ADS1220_Read_Data();
                    preStepCurrent = (ADS1220_microVolts(0x00) * 1000.0 / TIAGain);  // current in nA
                    currentVoltage -= pulseAmp; // going in reverse direction so pulse is in negative direction
                    stepUp = true;
                    if (currentVoltage < (startE - pulseAmp)) {
                      oxidationScan = true;  // now we need to sweep back on reduction scan
                      currentVoltage = startE - pulseAmp; // and set up to record pre step current...
                      stepUp = false;
                      endOfCycle = true;
                      Serial.print('c');  // send back 'c' for "Cycle Complete" (android will respond with instruction to exit, or to run another cycle...
                    }
                    DAC_AD5061_SetCalibratedVoltage(currentVoltage);  // now we've measured the pre-step current- apply the step and toggle flag so we measure post step next time
                }
                else {
                    ReportVoltammetryValues(preStepCurrent);
                    currentVoltage += (pulseAmp - stepE); // step back up to next pre-step value
                    DAC_AD5061_SetCalibratedVoltage(currentVoltage);  // now we've measured the pre-step current- apply the step and toggle flag so we measure post step next time
                    stepUp = false;
                }
             }
          } // end of one DPV cycle...
          while (!Serial.available() && !endOfAnalysis) {
            delay(1);  // now await further instructions; Android will either ask for another cycle, or quit / exit...
          }
          if (!endOfAnalysis) {
            char nextInstruction = Serial.read();
            switch (nextInstruction) {
              case 'c': // instruction to repeat/ conduct another CV cycle...
                endOfCycle = false;
                break;
              case 'x': // instruction to end Analysis (this will cause us to completely exit analytical methods and return to main loop()
                endOfAnalysis = true;
                break;
              default:
                endOfAnalysis = true;
                break;
             } // end switch(nextInstruction)
          }
      } // end while(!endOfAnalysis)
      
}

void ElectrochemicalImpedanceSpectroscopy() {
      TIA_LMP7721();  // initially if analyzing low frequencies, connect TIA output through LMP7721 to ADC (ADS1220)...
      DAC_AD5061_Connect(true); // connect DAC from AD5061 to network...
      AD5933_Connect(false); // for arbitrary bias need to connect voltage sources from DAC (AD5061) and network analyzer (AD5933)
      setTIAGain(0x03); // initially start at most sensitive gain that for small frequencies analyzed with ADS1220 high gain should be OK at first...
      
      double analysisFrequency = Serial.parseFloat();  // start frequency (in Hz)
      double endFrequency = Serial.parseFloat();  // end frequency (in Hz)
      long deltaFrequency = Serial.parseInt();  // interval frequency (in Hz)
      double biasVoltage = Serial.parseFloat(); //- (AD5933_BiasuV / 1000000.0); // applied DC bias on signal...// compensate for existing bias on AD5933 Vout...
      DAC_AD5061_SetCalibratedVoltage(biasVoltage);  // first set the potential at the starting value...
      excitationCode = Serial.parseInt(); // read the excitation code- 0x03 for 200mVp-p, 0x02 for 100mVp-p, 0x01 for 40mVp-p and 0x00 for 20mVp-p
      
      boolean endOfCycle = false;
      boolean endOfAnalysis = equilibrationPeriod();  // request and wait the duration of equilibration period; returns "true" if user exits analysis during equilibration...

      //double lowFrequency[] = {0.1, 0.2, 0.4, 1, 2, 4, 10, 20, 40, 100, 200, 400};  // create an array of low frequency values to conduct EIS analysis on (using ADC)
      //double lowFrequency = 0.1;
      //int frequencyArraySize = 12;//sizeof(lowFrequency);
      //int frequencyIndex = 0;
      while (!endOfAnalysis && (analysisFrequency < 100) && (analysisFrequency <= endFrequency)) {
          endOfAnalysis = measureReportLowfImpedance(analysisFrequency, biasVoltage); // allow subroutine to exit if user sends 'x' character
          analysisFrequency *= sqrt(2.0);
          delay(1);
      }// end of new section for ABE-Stat_1_0_05 (first implementation- just measure and report low frequency data without being solicited...
      
      // For higher frequencies- change hardware switches to direct source and TIA current to AD5933 Network analyzer
      TIA_AD5933();  // set TIA to return to AD5933 network analyzer...
      DAC_AD5061_Connect(true); // connect DAC from AD5061 to network (all AD5933 calibrations are conducted with this switch connected/ closed, so connect it even if 0 bias)
      //biasVoltage -= (AD5933_BiasuV / 1000000.0); // now that we've switched to AD5933 connection, correct for AD5933 signal bias...
      DAC_AD5061_SetCalibratedVoltage(biasVoltage);
      AD5933_Connect(true); // for arbitrary bias need to connect voltage sources from DAC (AD5061) and network analyzer (AD5933)
      setTIAGain(0x02); // initially start at most sensitive gain that AD5933 can handle (setting 0x03 causes too much signal distortion)
      PGAx5Setting = true;
      delay(20); // short delay to allow transients to die from new settings...
      // now measure new bias potential with "0.0 V" applied from DAC5061- to determine offset of "Vin" terminal of AD5933 from analog reference signal- need to correct this out
      

      double freq_increment = 2047.85;
      int waveForms = 1;
      long freq_lowAD5933 = round((waveForms * freq_increment) + 0.5);
      double ZmagGainOptimizer[] = {1.0, 1.0};
      boolean endOfGainOptimization = false;
      boolean lowestGain = false;
      if (freq_increment <= endFrequency) { // first make sure PGA and gain settings ensure linear response / no TIA saturation if using the 
          while (!endOfGainOptimization) {
            while(!TestImpedance(freq_lowAD5933, excitationCode, PGAx5Setting));
            ZmagGainOptimizer[0] = impedanceMagnitude();
            if (PGAx5Setting) {
              PGAx5Setting = false;
              while(!TestImpedance(freq_lowAD5933, excitationCode, PGAx5Setting));
            }
            else if (TIAGainCode > 0x00) {
              TIAGainCode--;
              setTIAGain(TIAGainCode);
              while(!TestImpedance(freq_lowAD5933, excitationCode, PGAx5Setting));
            }
            else lowestGain = true; // if PGAx5 off and TIAGainCode already 0x00, then we're already at lowest gain...
            ZmagGainOptimizer[1] = impedanceMagnitude();
            double deltaZmag = abs(ZmagGainOptimizer[0] - ZmagGainOptimizer[1]);
            if (deltaZmag < (ZmagGainOptimizer[0] / 10.0))  endOfGainOptimization = true; // if system is behaving linearly at tested settings, we have appropriate settings...
          }
          if (!lowestGain) {  // if linearity was observed above lowest gain value we need to recover this setting...
            if (TIAGainCode < 0x02) { // if TIAGainCode is not already highest; note 0x02 is the highest value used for for AD5933, dial the code back up
              TIAGainCode++;
              setTIAGain(TIAGainCode);
            }
            else PGAx5Setting = true; // otherwise enable Programmable Gain Amplifier on AD5933        
          }
      }
      while (!endOfAnalysis && (freq_lowAD5933 < 15000) && (freq_lowAD5933 <= endFrequency)) {
          measureReportImpedance(freq_lowAD5933, biasVoltage);
          delay(1); // just in case MCU needs to do something- don't let controller time out...
          waveForms++;
          freq_lowAD5933 = round((waveForms * freq_increment) + 0.5);
          if (Serial.available()) { // listen for instructions to exit
                  char instruction = Serial.read();
                  switch (instruction) {
                    case 'x':
                        endOfAnalysis = true; // instruction received to exit analysis...
                        break;
                    default:
                        break;
                  }
          }
      }
      analysisFrequency = 15000;
      while (!endOfAnalysis && (analysisFrequency < (endFrequency + 0.5))) {
          measureReportImpedance(analysisFrequency, biasVoltage);  // this function will take the impedance measurement- dialing down excitation and gain as appropriate to get result in valid range...
          
          delay(1);// give ESP8266 break to check WiFi and other background stuff...
          if (Serial.available()) { // listen for instructions to exit
                  char instruction = Serial.read();
                  switch (instruction) {
                    case 'x':
                        endOfAnalysis = true; // instruction received to exit analysis...
                        break;
                    default:
                        break;
                  }
          }

          analysisFrequency += deltaFrequency;  // increment frequency
          if (analysisFrequency >= endFrequency) measureReportImpedance(endFrequency, biasVoltage); // take one last measurement at end frequency
      }
      Serial.print('c');  // send back 'c' for "Cycle Complete" so Android knows we're done!
}

/*
 * Function to read the "equilibration time" and then wait corresponding number of seconds under pre-programmed conditions
 * for "equilibration". Returns "true" if user asks to exit routing during the equilibration time, "false" if entire 
 * equilibration period completes without user exiting...
 */
 boolean equilibrationPeriod() {
      boolean endAnalysis = false;
      int equilibrationCountDown = Serial.parseInt(); // in seconds
      long count = millis();
      Serial.print('b');
      Serial.print(equilibrationCountDown);
      Serial.print('\t');
      batteryReport();
      ADS1220_ReportTemperature();

      while (equilibrationCountDown > 0 && !endAnalysis) {  // wait for equilibration time unless user decides to exit...
          if ((millis() - count) > 1000) {
              count += 1000;
              equilibrationCountDown--;
              Serial.print('b');
              Serial.print(equilibrationCountDown);
              Serial.print('\t');
              batteryReport();
              ADS1220_ReportTemperature();
          }
          else if (millis() < count) count = millis();
          if (Serial.available()) {
            if (Serial.read() == 'x') endAnalysis = true;  // check for instruction to exit...
          }
          delay(1); // put in a delay so ESP8266 has time to take care of any background WiFi or other tasks...
      }
      return endAnalysis;
 }

/*
 * measure and report impedance values (used in (ElectrochemicalImpedanceSpectroscopy()")
 */
 void measureReportImpedance(long f, double bias) {
          autoRangeZFast(f, true, bias);  // this function will take the impedance measurement- for EIS scan amplitude is prescribed; but allow PGA and TIA gains to be adjusted to get good reading...
            // function also needs to know the bias so it can determine the margin for "rawadmittance" value before saturation occurs...
          double ZMagnitude = impedanceMagnitude();
          double ZPhase = correctPhase();

          Serial.print('f');  // code that data is coming- send cell potential first, then current using tab delimitation
          Serial.print(f);
          Serial.print('\t');
          Serial.print(ZMagnitude, 1);
          Serial.print('\t');
          Serial.print(ZPhase, 2);
          Serial.print('\t');
 }

/*
 * measure and report impedance values at low frequencies (below cutoff of where AD5933 internal clock speed does not sample enough cycles for accurate DFT) by setting potential signal
 * with DAC5061, and measuring transimpedance signal with ADS1220. Very upper limit of frequency is 1000 Hz with 2000 sps; probably best to keep to under 500 given latency in voltage 
 * programming and digital device reads / mathematical manipulation.
 * accepts parameters F (frequency) and B (bias potential)
 */
 boolean measureReportLowfImpedance(double F, double B) {
    DAC_AD5061_SetCalibratedVoltage(B);
    boolean leaveAnalysis = false;
    double A = 0.01; // amplitude of AC signal applied for EIS analysis- needs to match values used in AD5933 at given excitation code setting, after dividing Vin of AD5933 by 10 in hardware.
    switch (excitationCode) {
        case 0x03:
          A = 0.1;
          break;
        case 0x02:
          A = 0.05;
          break;
        case 0x01:
          A = 0.02;
          break;
        case 0x00:
          A = 0.01;
          break;
        default:
          break;
    }
    int N = 24; // number of discrete observations to make for Discrete Fourier Transform
    int K = 1;  // index of fourier transform for desired frequency (K cycles in N samples)
    if (F > 2) K = 8; // measure over multiple periods at higher frequencies for better accuracy without having to wait too long...
    int unmeasuredCycles = 1;
    if (F < 1) unmeasuredCycles = 1;
    else {
      unmeasuredCycles = round((F / K) + 0.5);  // make sure that we let an integer multiple of K waves go by, or we won't start summing fourier transform components at start of sin wave...
      // these are unmeasured cycles of N samples each, covering 5 sinusoidal cycles at given frequency; this allows us to let transients die down...
    }
    int totalSamples = (unmeasuredCycles + 1) * N; // total number of samples before we complete the transform- only last set of N measurements is used for Fourier Transform evaluation
    int fourierSampleStart = unmeasuredCycles * N; // let the first transient "unmeasuredCycles" go by without starting the fourier transform...
    double cosCoefficient[N];
    double sinCoefficient[N];
    double XoneReal = 0;
    double XoneImag = 0;  // Real and Imaginary Fourier coefficients with argument "1" (amplitudes of cos and sin terms at frequency F)
    double omegaScaled = 2.0 * PI * F / 1000000.0;  // scaled frequency normalized to radians per microsecond... (multiply by time in micros to get argument of sinusoidal contribution of input voltage
    double appliedE = B;
    // now estimate coefficients of fourier transform at k = 1 (i.e., representing frequency of applied signal, assuming we sample N times over one cycle)
    // so we don't waste time having to estimate these during actual real time analysis- time will be precious then!
    for (int coeffs = 0; coeffs < N; coeffs++) {  
        cosCoefficient[coeffs] = 2.0 * cos(2 * PI * K *coeffs / N) / N;
        sinCoefficient[coeffs] = -2.0 * sin(2 * PI * K *coeffs / N) / N;
        /*Serial.print("mrealCoefficient ");
        Serial.print(coeffs);
        Serial.print(": ");
        Serial.print(cosCoefficient[coeffs]);
        Serial.print("\t");*/
    }
    setTIAGain(0x03); // initially use the most sensitive TIA Gain
    if (abs(B) < 0.1) DAC_AD5061_SetCalibratedVoltage(0.1); // ensure initial test has non-zero bias to ensure we set Gain to where system is linear (not saturated)
    double garbageCurrent = cellCurrent();  // call cellCurrent function just to ensure Gain is selected to a value where system is still linear (not saturated); recall that bias voltage is being applied...
    DAC_AD5061_SetCalibratedVoltage(B); // then apply the pre-specified bias (just in case it had been changed to ensure appropriate system analysis for linearity).
    
    long samplePeriod = (1000000 * K / (F * N));  // intervals (in microseconds) that we need to sample to get N samples over one cycle at F frequency
    boolean ADCBusy = false;   // make sure we don't request a sample while still waiting for previous conversion to be completed and read
    int sampleNumber = 0;
    EISLowFrequencySettings();
    data.bytes[3] = 0x00; // set high byte of data to 0x00 (here we're only populating "data" with 24 bit conversions of TIA amplifier read by ADS1220- high byte always 00
    long startTime = micros();
    long sampleTime = 0;  // take first sample instantaneously (at time 0 of applied signal)
    long currentTime = micros() - startTime;
    /*Serial.print("mstarting EIS at f ");
    Serial.print(F);
    Serial.print("\t");*/
    while (!leaveAnalysis && (sampleNumber < totalSamples)) {
        appliedE = B + A * sin(omegaScaled * currentTime);
        DAC_AD5061_SetCalibratedVoltage(appliedE);
        /*Serial.print("mapplying E ");
        Serial.print(appliedE);
        Serial.print("\t");*/
        if ((currentTime > sampleTime) && !ADCBusy) { // don't request new conversion if already busy doing a conversion
            sampleTime += samplePeriod; // update next sample time...
            SPI_Chip_Select(ADS1220_ADC);
            digitalWrite(CS_RCK, LOW);  // pull corresponding chip select low
            SPI.beginTransaction(ADS1220Settings); // slower than 150 ns minimum SPI clock period; MSBFirst; only SPI_MODE 1 supported
            byte SPI_Junk_Received = SPI.transfer(0x08); // Start/Sync command (required to start a single shot conversion)
            SPI.endTransaction();
            digitalWrite(CS_RCK, HIGH);  // pull corresponding chip select high (release device as soon as data is read completely)
            Release_Chip_Select();  // make sure we release ADS1220 so we can still update DAC value while waiting for conversion
            ADCBusy = true; // we just requested a new conversion, and set the flag that the ADC is busy doing the conversion
            //Serial.print("mrequested Conversion!\t");
        }
        if (ADCBusy && ADS1220DataReady()) {  // if we've requested a conversion, and see signal that conversion is complete...
            SPI_Chip_Select(ADS1220_ADC);
            digitalWrite(CS_RCK, LOW);  // pull corresponding chip select low
            SPI.beginTransaction(ADS1220Settings); // slower than 150 ns minimum SPI clock period; MSBFirst; only SPI_MODE 1 supported
            data.bytes[2] = SPI.transfer(0x00);
            data.bytes[1] = SPI.transfer(0x00);
            data.bytes[0] = SPI.transfer(0x00);
            SPI.endTransaction();
            digitalWrite(CS_RCK, HIGH);  // pull corresponding chip select high (release device as soon as data is read completely)
            Release_Chip_Select();
            int32_t conversion_value = 0;
            if (data.word32 & 0x00800000) { // Sign extend negative numbers- conversion is a two's complement 24 bit value
                conversion_value = 0xFF800000 | ((data.word32) & 0x007FFFFF);  // write signed 24 bit value into 32 bit twos complement
            }
            else {
                conversion_value = data.word32; // not negative number (MSb or bit 23 != 1) then the existing value is the correctly signed as a 32 bit number
            } // now convert digitized value to representative voltage
            double voltage = (conversion_value * 2.44141e-7);  // signed 24 bit conversion with voltage reference span +/-2.048V, so range 4.096 V divided by 16777216 discrete 2^24 intervals
            double current = voltage / TIAGain; // convert the observed voltage to current...
            if (((voltage > (vhighlimit - 0.05)) || (voltage < (vlowlimit + 0.05))) && (TIAGainCode > 0x00)) {
                TIAGainCode--;  // if amplifier is saturated, and can reduce the transimpedance gain, then lower it so next measurements might be accurate
                setTIAGain(TIAGainCode);
            }
            if (sampleNumber >= fourierSampleStart) {  // after the first N samples (we've already gone through one cycle at given frequency), start summing terms to determine Fourier coefficients
                int FourierIndex = sampleNumber % N;
                XoneReal += current * cosCoefficient[FourierIndex];
                XoneImag += current * sinCoefficient[FourierIndex];
            }
            ADCBusy = false;  // acknowledge now that ADC is free for next conversion
            /*Serial.print("mReading ");
            Serial.print(sampleNumber);
            Serial.print("th sample for Fourier\t");*/
            if (F < 3) delay(1); // at really low frequencies make sure we put in periodic delays so MCU doesn't time out... (best done *after* completing current measurement, so we don't distort signal
              // during measurement
            sampleNumber++; // when this increments to 2*N we are finished- completed all samples from 0 to N-1 (after sampleNumber%N)
        }
        if (Serial.available()) { // listen for instructions to exit
                  char instruction = Serial.read();
                  switch (instruction) {
                    case 'x':
                        leaveAnalysis = true; // instruction received to exit analysis...
                        break;
                    default:
                        break;
                  }
        }
        currentTime = micros() - startTime;
    }
    if (!leaveAnalysis) { // if we get a result without interruption from user report results...
        // divide applied amplitude by transimpedance signal (corrected by TIAGain to return current) to determine impedance magnitude, and phase
        double ADCphaseDelay = F * 360.0 * 3.3e-4;// phase delay in estimated current based on ADS1220 latency at given data rate (takes about 655 us to complete conversion, so actually value is representative
          // of current value approximately 330us into conversion
        double currentMagnitude = sqrt(pow(XoneReal, 2) + pow(XoneImag, 2));
        double ratio = XoneImag * 1.0 / (XoneReal * 1.0);
        double phily = 180.0 * atan(ratio) / PI;
        if (XoneReal > 0) phily = phily;  // in 1st and 4th quadrants arctan gives direct phase angle
        else if (XoneImag > 0) phily = 180.0 + phily;  // in second quadrant, arctan returns negative of angle, with magnitude equivalent to deviation from 180 deg (so just add the negative number)
        else phily = -180.0 + phily; // in third quadrant, arctan returns positive angle; so just add the angle to -180...
        double Zph = -(ADCphaseDelay + phily + 90.0); // source phase (sine) of voltage is  -90; subtract phily because it is phase of current (numerator in impedance ratio)
        while (Zph > 180.0) Zph -= 360.0;
        while (Zph < -180.0) Zph += 360.0;
        Serial.print("mX1Real ");
        Serial.print(XoneReal * 1000000);
        Serial.print("  X1Imag ");
        Serial.print(XoneImag * 1000000);
        Serial.print("\t");
        Serial.print('f');  // code that data is coming- send cell potential first, then current using tab delimitation
        Serial.print(F);
        Serial.print('\t');
        Serial.print((A / currentMagnitude), 0);
        Serial.print('\t');
        Serial.print(Zph, 2);
        Serial.print('\t');
    }
    return leaveAnalysis;
 }

/*
 * Function to measure and report back voltammetry data (instantaneous current and voltage measurements); measure potential first to allow
 * capacitive charging current from last potential increment to settle...
 * 
 */
void ReportVoltammetryValues(double preStepI) {
    ADS1220_SetSingleCtrlRegister(0x00, 0x41);  // configure to measure negative of cell voltage (Vref vs. analog ref- where working potential is analog ref); PGA off / gain = 1
    ADS1220_Read_Data();
    double cellPotential = -ADS1220_microVolts(0x00) * 1e-6; // argument of microvolts is gain bit setting- 0x00 corresponds to gain of 1
    ADS1220_SetSingleCtrlRegister(0x00, 0x21);  // configure to measure transimpedance amp voltage (Vtia vs. analog ref
    ADS1220_Read_Data();
    double TIAVoltage = ADS1220_microVolts(0x00) * 1e-6; // result is TIA voltage in volts...
    while (((TIAGainCode > 0x00) && ((TIAVoltage > (vhighlimit - 0.05)) || (TIAVoltage < (vlowlimit + 0.05)))) || TIAVoltage == 0.0) { // if TIA voltage out of range of ADC and not already at lowest gain, then lower gain and measure again...
      TIAGainCode--;
      setTIAGain(TIAGainCode);
      ADS1220_Read_Data();
      TIAVoltage = ADS1220_microVolts(0x00) * 1e-6;
    }
    double currentValue = (TIAVoltage * 1e9 / TIAGain) - preStepI; //  value in nA

    Serial.print('d');  // code that data is coming- send cell potential first, then current using tab delimitation
    Serial.print(cellPotential, 4);
    Serial.print('\t');
    Serial.print(currentValue, 3);
    Serial.print('\t');
    /*Serial.print("mE(V):");
    Serial.print(cellPotential, 4);
    Serial.print("I(nA):");
    Serial.print(currentValue, 3);
    Serial.print("\t");

    long starterTime = micros();
    
    starterTime = micros() - starterTime;
    Serial.print("mTask completed in ");
    Serial.print(starterTime);
    Serial.print("us\t");
    */
}

/*
 * Function to set higher data rate for voltammetric scanning analyses (also remove filtering- not useful for higher data rates anyway
 */
void voltammetryScanSettings() {
    ADS1220_Settings(0x01, 0xb0, 0x00); // register 0 set for default MUX and PGA off; register 01 set for data rate 1200 sps (in turbo mode); register 03 2.048 Vref, filters off
}

/*
 * Function to set highest data rate for EIS analysis at low frequencies using ADS1220 conversion values for DFT (need to run as fast as possible
 */
void EISLowFrequencySettings() {
    ADS1220_Settings(0x21, 0xd0, 0x00); // here we are only going to read TIA current (so leave mux / high nybble of reg00 as 0x2x; PGA disabled, so reg00 = 0x21; reg01 highest data rate 2000sps so 0xd0
}

