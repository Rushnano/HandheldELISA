/*
  (Note, everything between the /* and * / or after // is commentary intended for project documentation
  
  This Sketch reads color and ambient light data registers from APDS-9960 color/ gesture/ proximity sensor and reports over attached bluetooth
  to Android app ABE-VIEW (https://play.google.com/store/apps/details?id=com.uhmbe.DAQCTRL&hl=en_US) using serial to Bluetooth Modem (Roving Networks RN42)
  Setup configures the device to enable the "Ambient Light Sensor" (ALS) engine, for fastest sensor integration time
  
  Example assumes use of custom controller hardware "Coffee Color Cherry Sorter", a BE 420 project of Taylor Hori and Tiffany Ulep, Fall 2015

Hardware interface with Arduino:
  Analog Pins:
    A4 and A5 are used as the i2c interface to communicate with and control APDS-9960
    These are connected internally on the custom board through logic shifting transistors (3.3V for APDS-9960, 5V for ATMEGA328)
    
  Digital Pins:
    Serial communication through digital pins 00 (Rxd) and 01 (Txd)- these are connected to the UART of RN-42 bluetooth module
    D2 is connected (through logic level shifting transistor) to the INT pin of the local APDS-9960
    D3 drives an external load through a high current sinking/ n-channel MOSFET transistor (A03406; max 3.6 A- 
      load connected in parallel with reverse biased schottky diode for protection against inductive voltage spikes
    D4 toggles a switch for the SDA lines of different APDS-9960 (one on the board, one interfaced from off the board) to allow interfacing to multiple sensors with same
      i2c address (D4 low connects to local sensor on the board, so this example keeps D4 low)
    D5 enables AS1101 LED driver connected to two on board white LEDs for illumination
    D6 sets the regulation current on the AS1101 LED driver (D5 and D6 both need to be "high" to illuminate the LEDs)

  Note that hardware is a custom assembly built around ATMEGA328P (microcontroller used on Arduino Uno and Pro Mini). Project can be replicated by hardwiring
  evaluation boards with different sensor chips (i.e. APDS9960 evaluation board from Sparkfun, https://www.sparkfun.com/products/12787, RN42 bluetooth
  evaluation board https://www.sparkfun.com/products/12577), around off the shelf Arduino Uno or Pro Mini for example.

  For programming the custom assembly without the Arduino bootloader, you can use the binary / .hex file generated from Arduino (choose Pro Mini 5V 16 MHz board under "Tools"
  then Sketch >> Export compiled binary, and program the board through the ISP header using the ATMEL Studio IDE and programming tool. Alternatively you can program
  the device using Arduino as ISP (look under File >> Examples >> ArduinoISP)
 
by Daniel M. Jenkins (last edit June 13, 2018)
 
*/

#include <Wire.h>
#include <SparkFun_APDS9960.h>
/*
 * To install SparkFun_APDS9960 library directly from Arduino IDE, go to Sketch >> Include Library >> Manage Libraries..., 
 * then select / install "SparkFun APDS9960 RGB and Gesture Sensor"
*/

const long BAUD = 115200;  // baud rate for serial port

/*
 * Following addresses are used to directly access / read the color interrupt register of APDS-9960 through i2c;
 * Reading the register clears the interrupt on the IC and enables new measurements to be made.
 */
const byte APDS_9960 = 0x39;  // i2c address of APDS_9960 device
const byte CICLEAR = 0xe6;  // clear color interrupt register (write any value to clear register); "access" of register (without writing data) is sufficient to clear interrupt

const int EXTERNAL_LOAD = 3;  // digital pin for LED for signaling events
const int I2C_SELECT = 4;  // digital pin to control which color sensor is connected to ATMEGA328 i2c SDA line (external sensor when high)
const int AS1101_ENABLE = 5;  // enable pin on current driver ic (must be high to drive LEDs); hardware uses AMS AS1101 LED driver IC to regulate current
const int AS1101_I = 6;  // current regulation on current driver ic (also must be high to drive LEDs)

boolean LED_ENABLE = false;
boolean LOAD_ENABLE = false;

char command;  // character command (1 ASCII character, 8 bits) read from serial port (over bluetooth)

SparkFun_APDS9960 apds = SparkFun_APDS9960(); // declare object of class SparkFun_APDS9960 (this will handle all control / measurements from APDS9960 sensor)
uint16_t ambient_light = 0; // variables for storing data readings from APDS9960
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;

long measureTime; // a global variable to control the timing of measurements
int measureInterval = 1000;

/* 
 * i2c for reading one byte of data from device (we'll use this to clear the color interrupt register
 * of APDS-9960 after each reading, to enable new readings...
 */
byte i2c_Read(int i2cAddr, int regAddr)
{
  // This function reads one byte over IIC
  byte value;
  Wire.beginTransmission(i2cAddr);  // address of device to read data from
  Wire.write(regAddr); // Address of register to be read
  Wire.endTransmission(); 
  Wire.requestFrom(i2cAddr, 1);  // request 1 byte of data starting at regAddr
  while (Wire.available()) value = Wire.read();  // read and return the data returned by device
  return value;
}

/*
 * Setup() executes once when microcontroller is first powered on, or reset
 */
void setup()
{ 
  pinMode(EXTERNAL_LOAD, OUTPUT); // drive external load switch with output...
  pinMode(I2C_SELECT, OUTPUT); // Drive analog switch for i2c device select with output...
  pinMode(AS1101_ENABLE, OUTPUT);  // Drive enable pin of LED driver with output...
  pinMode(AS1101_I, OUTPUT);  // Drive the current regulation of LED driver with output...
  
  Wire.begin(); // initialize i2c port on ADC4 and ADC5 (to communicate with APDS9960)
  
  apds.init();  // initialize object of class APDS9960
  apds.enableLightSensor(false); // enable ambient light and color sensor (with no interrupts)
  
  Serial.begin(BAUD);  // initialize serial port to communicate to PC (this is connected to the RN42 Bluetooth Module communicating with ABE-VIEW)
  
  digitalWrite(AS1101_ENABLE, LOW);  // make sure LEDs start off
  digitalWrite(AS1101_I, LOW);
  digitalWrite(EXTERNAL_LOAD, LOW);  // make sure switch to external load starts off
  digitalWrite(I2C_SELECT, LOW);  // start with I2C_SELECT output low- only read the "local" APDS-9960 on the board

  byte dummy = i2c_Read(APDS_9960, CICLEAR);  // reads the color interrupt register of the APDS-9960 (this is required to enable a new reading)

  measureTime = millis() + measureInterval; // set up time to make next color measurements
}

/*
 * loop() executes repeatedly after execution of setup()
 */
void loop() {
  /*
   * Always check incoming serial traffic to see if user has interacted with GUI on ABE-VIEW
   * To use this code as a template for ABE-VIEW, it should be cut and paste into your project
   */
  while(Serial.available()) {
    command = Serial.read();
    switch(command) {
      case 's':// code from ABE-VIEW that Bluetooth has just connected- requests information about how to populate the Android Interface
        populateAndroidInterface();
        break;
      case 'b':// code from ABE-VIEW that one of the buttons on the Android interface was pushed
        parseButton();
        break;
      case 'c':// code from ABE_VIEW that one of the control input values has been updated
        parseControlInput();
        break;
      case 'r':// code from ABE-VIEW that one of the Radio buttons was selected
        parseRadioButtons();
        break;
      default: break;
    }
  }
  if (millis() >= measureTime) {  // if system time has reached time to make a new measurement...
      long start = micros();
      apds.readRedLight(red_light);
      apds.readGreenLight(green_light);
      apds.readBlueLight(blue_light);
      float Red_Green_Ratio = (float) (red_light * 1.0 / green_light);
      float lapse = (float) (micros() - start) / 1000;  // time (in milliseconds) required to read all of the sensor measurements

      /*
       * The following instruction reads the color interrupt register of the APDS-9960 (this is required to enable a new reading)
       */
      byte dummy = i2c_Read(APDS_9960, CICLEAR);  // read the color interrupt register of APDS-9960 to enable a new reading

      /*
       * The following series of print statements send ascii coded text back to ABE-VIEW to populate specific data arrays with new data,
       * then send instruction to concatenate these data into corresponding data arrays for plotting or data record, along with the time stamp
       */
      Serial.print("d0\t");  // code for ABE-VIEW that data for the 0th data element is forthcoming
      Serial.print(Red_Green_Ratio, 3); // report to ABE-VIEW the numerical value of ratio of red to green reading, with 3 decimal precision
      Serial.print('\t');  // tab termination to denote the end of the number representing the ratio of red to green values...
      
      Serial.print("d1\t");  // code for ABE-VIEW that data for the 1st data element is forthcoming
      Serial.print(red_light);  // report to ABE-VIEW the numerical value of the red sensor reading (this is an integer so use the default 0 decimal precision)
      Serial.print('\t'); // tab termination to denote the end of the number representing the red sensor reading...
      
      Serial.print("d2\t");  // code for ABE-VIEW that data for the 2nd data element (blue sensor reading) is forthcoming
      Serial.print(blue_light); //...
      Serial.print('\t');
      
      Serial.print("d3\t");  // code for ABE-VIEW that data for the 3rd data element (green sensor reading) is forthcoming
      Serial.print(green_light);  //...
      Serial.print('\t');
      
      Serial.print("d4\t");  // code for ABE-VIEW that data for the 4th data element (time in ms elapsed during sensor readings)
      Serial.print(lapse, 3);  // send time elapsed (in ms) during reading of color values, with 3 decimal point precision
      Serial.print('\t');  // tab termination to parse the lapsed value in data operations...
      
      Serial.print('u');  // instruction to tell the android to record the latest data values (and current system time in ms from start of BT connection) into memory/ graph...
        /* 
         *  note that the alternative for automating the concatanation of most recent data into data arrays from the Android / ABE-VIEW end is to use the "p" instruction
         *  see Serial("p10\t"); instruction commented out within the "populateAndroidInterface()" function.
         */

      measureTime += measureInterval;  // update the time for next measurements to occur...
  }
}

/*
 * Function called when device receives 's' instruction from ABE-VIEW, notifying that bluetooth connection was just made.
 * This is the flag to send back coded information about how the GUI should be configured...
 */
void populateAndroidInterface()
{
  // First label and populate available buttons (8 available)
  Serial.print("b0\tLED Toggle\t");  // activates button with index 0 on Android interface, with a label "LED Toggle"
  Serial.print("b1\tLoad Toggle\t");  // here we label the second available button (index 1) with "Load Toggle"
  Serial.print("b2\tHopper Gate\t");  // just filling in all available buttons in example to debug the Android interface...
  Serial.print("b3\tRelease EMI\t");  // just filling in all available buttons in example to debug the Android interface...
  Serial.print("b4\tDummy 1\t");  // just filling in all available buttons in example to debug the Android interface...
  Serial.print("b5\tDummy 2\t");  // just filling in all available buttons in example to debug the Android interface...
  Serial.print("b6\tDummy 3\t");  // just filling in all available buttons in example to debug the Android interface...
  Serial.print("b7\tDummy 4\t");  // just filling in all available buttons in example to debug the Android interface...

  // now populate / configure available numerical controls on interface (total of 8 are available)
  Serial.print("c0\t0\t0\t100\t255\t/Contrast\t");  // activate the numeric control inputs on the Android interface
      // this activates control with index 0, with resolution of 0 decimal digits, minimum value 0, starting/ default value 100, maximum allowable value of 255, and a Heading of "Contrast"
  Serial.print("c1\t1\t-1\t0\t1\tTint\t");  // similar for control at index 1, control resolution 1 decimal digit (0.1), minimum value -1, default 0, max 1, Heading "Tint"
  Serial.print("c2\t2\t-0.1\t0\t0.1\tHue\t");  // ... index 2, resolution of 2 decimal digits (0.01), min -0.1, default 0, max 0.1, Heading "Hue"
  Serial.print("c3\t3\t-0.01\t0\t0.01\tResolution\t");  // just filling in all interface elements to illustrate on the ABE-VIEW interface (we won't actually use any data returning from these controls...
  Serial.print("c4\t3\t-0.015\t0\t0.015\tExposure time\t");  // just filling in all interface elements to illustrate on the ABE-VIEW interface...
  Serial.print("c5\t2\t-0.15\t0\t0.15\tCompression\t");  // just filling in all interface elements to illustrate on the ABE-VIEW interface...
  Serial.print("c6\t1\t-1\t0\t1\tPlay Value 6\t");  // just filling in all interface elements to illustrate on the ABE-VIEW interface...
  Serial.print("c7\t0\t-10\t0\t0\tPlay Value 7\t");  // just filling in all interface elements to illustrate on the ABE-VIEW interface...

  // now populate available data headings (total of 16 available)...
  Serial.print("h0\t1\tRed / Green: \t");  // Create heading for the data display element "0", with associated checkbox (to display on chart) on ("1")- Heading is "Red / Green: "
  Serial.print("h1\t1\tRed: \t");  // Create heading for the data display element "1", with associated checkbox (to display on chart) on ("1")- Heading is "Red: "
  Serial.print("h2\t1\tBlue: \t");  // Create heading for the data display element "2", with associated checkbox (to display on chart) on ("1")- Heading is "Blue: "
  Serial.print("h3\t1\tGreen: \t");  // Create heading for the data display element "3", with associated checkbox (to display on chart) on ("1")- Heading is "Green: "
  Serial.print("h4\t0\tTime to Result (ms): \t");  // heading to display the time to result (read of red, green, and determine the ratio)- it will not have checkbox to enable it to be plotted on chart
  Serial.print("h5\t0\tCodeName1: \t");  // just for illustrating on ABE-VIEW interface, turn on the data element 5 (this will not be enabled to plot on chart)
  Serial.print("h6\t0\tCodeName2: \t");  // just for illustrating on ABE-VIEW interface, turn on the data element 6 (this will not be enabled to plot on chart)
  Serial.print("h7\t0\tCodeName3: \t");  // just for illustrating on ABE-VIEW interface, turn on the data element 7 (this will not be enabled to plot on chart)
  Serial.print("h8\t0\tCodeName4: \t");  // just for illustrating on ABE-VIEW interface, turn on the data element 8 (this will not be enabled to plot on chart)
  Serial.print("h9\t0\tCodeName5: \t");  // just for illustrating on ABE-VIEW interface, turn on the data element 9 (this will not be enabled to plot on chart)
  Serial.print("h10\t0\tCodeName6: \t");  // just for illustrating on ABE-VIEW interface, turn on the data element 10 (this will not be enabled to plot on chart)
  Serial.print("h11\t0\tCodeName7: \t");  // just for illustrating on ABE-VIEW interface, turn on the data element 11 (this will not be enabled to plot on chart)
  Serial.print("h12\t0\tCodeName8: \t");  // just for illustrating on ABE-VIEW interface, turn on the data element 12 (this will not be enabled to plot on chart)
  Serial.print("h13\t0\tCodeName9: \t");  // just for illustrating on ABE-VIEW interface, turn on the data element 13 (this will not be enabled to plot on chart)
  Serial.print("h14\t0\tCodeName10: \t");  // just for illustrating on ABE-VIEW interface, turn on the data element 14 (this will not be enabled to plot on chart)
  Serial.print("h15\t0\tCodeName11: \t");  // just for illustrating on ABE-VIEW interface, turn on the data element 15 (this will not be enabled to plot on chart)
  
  //Now lets prepopulate some of the data associated with headings with text (these are data fields that are not able to be plotted on chart, so just going to populate with text...
  Serial.print("d5\tCat in the Hat\t");
  Serial.print("d6\tThing 1\t");
  Serial.print("d7\tThing 2\t");
  Serial.print("d8\tStar Bellied Sneech\t");
  Serial.print("d9\tLorax\t");
  Serial.print("d10\tThomas\t");
  Serial.print("d11\tPercy\t");
  Serial.print("d12\tEmily\t");
  Serial.print("d13\tHenry\t");
  Serial.print("d14\tGordon\t");
  Serial.print("d15\tCBeasterton\t");

  // Now let's configure some of the available radio buttons / groups...
  Serial.print("r0\tSensor Select\tLocal\tRemote\t\t\t");  // Activate Radio group with index 0; String values are the group heading and 4 individual associated button labels
      // note that the last two strings are empty (no characters before the termination character '\t'- so last two available buttons do not show up on interface
  Serial.print("r1\tFruit\tbanana\tpapaya\tmango\tlilikoi\t");  // Activate Radio group index 1; String values are the group heading and 4 individual associated button labels
  Serial.print("r2\tOrigin\tKalimantan\tSulawesi\tJava\tSumatra\t");  // Activate Radio group index 2; String values are the group heading and 4 individual associated button labels
  Serial.print("r3\tTreatment\tA\tB\tC\tD\t");  // Activate Radio group index 3; String values are the group heading and 4 individual associated button labels

  Serial.print("g1\t");  // code to set up the chart- first numerical data ("1") enables the chart (anything else the chart won't show up)
  //Serial.print("p10\t");  
  /*
   * the previous instruction tells ABE-VIEW to record data (append most recent data values to corresponding data arrays) 
   * automatically with a period equal to the numeric argument in the string (ie. 10 seconds here)
   * In this example, we've commented this instruction out, because we'll just trigger ABE-VIEW 
   * directly from the "loop()" of this sketch using Serial.print("u"); every time we make sensor readings
   */

  // the instructions below allow some options for sending messages to ABE-VIEW for troubleshooting
  Serial.print("tQuick popup message on Android- now I'm connected!\t");  // syntax for sending a "toast"- brief message overlaying the ABE-VIEW / Android interface
  Serial.print("mThis is a debug message (need to run ABE-VIEW in debug mode on Eclipse or Android Studio to see it!\t");  // Android Bluetooth service displays this message in LogCat (during debug mode)
}

/*
 * One of the buttons on the ABE-VIEW GUI was pressed! Now determine which one it was and take appropriate action...
 */
void parseButton()
{
  int button_id = Serial.parseInt();
  switch (button_id) {
    case 0: // button with index 0 was pushed- user wants to toggle the state of LEDs!
      LED_ENABLE = !LED_ENABLE;  // toggle the state of LED_ENABLE
      if (LED_ENABLE) {
        digitalWrite(AS1101_ENABLE, HIGH);
        digitalWrite(AS1101_I, HIGH);
      }
      else {
        digitalWrite(AS1101_ENABLE, LOW);
        digitalWrite(AS1101_I, LOW);
      }
      break;
    case 1: // button with index 1 was pushed (user instruction to toggle current through the load / NMOS)
      LOAD_ENABLE = !LOAD_ENABLE;
      if (LOAD_ENABLE) digitalWrite(EXTERNAL_LOAD, HIGH);
      else digitalWrite(EXTERNAL_LOAD, LOW);
      break;
    case 2:
      // user puts code here for when button "2" is clicked...
      break;
    case 3:
      // user puts code here for when button "3" is clicked...
      break;
    default:
      break;
  }
}

/*
 * One of the numeric controls on the ABE-VIEW GUI was just updated- figure out which control was manipulated, what value was sent, 
 * and take appropriate action (here we're not actually doing anything but reading the data)
 */
void parseControlInput()
{
  float value;
  int control_id = Serial.parseInt();
  switch (control_id) {
    case 0:
      value = Serial.parseFloat();
      // now do something with the value returned from control index 0...
      break;
    case 1:
      value = Serial.parseFloat();
      // now do something with the value returned from control index 1...
      break;
      // now skipping cases 2 - 7 (since we're not doing anything with these values, just use default to populate value variable for all other user interactions
    default:
      value = Serial.parseFloat();
      break;
  }
}

/*
 * The user just pressed one of the radio buttons on the ABE-VIEW interface;
 * determine which button was pressed and take appropriate action.
 * In this case only one radio group does anything- changing the state of an analog switch to select whether 
 * the on board APDS9960 sensor is connected to i2c SDA, or whether an external sensor (with same i2c address) is connected
 * This hardware switch was removed from the most recent board design, so this code actually doesn't do anything in the hardware...
 */
void parseRadioButtons() {
  int rb_id = Serial.parseInt();
  switch (rb_id) {
    case 11:  // this is the first radio button in our first radio group (the one we labeled as "Local" sensor/ APDS-9960 select)
      digitalWrite(I2C_SELECT, LOW);  // local APDS-9960 is connected to ATMEGA328 SDA pin when I2C_SELECT/ D4 is low
      break;
    case 12: // this is the second radio button in our first radio group- we labeled it as "Remote" sensor
      digitalWrite(I2C_SELECT, HIGH);  // the remote sensor is connected when this line is driven high...
      break;
    default:
      break;  // here the default is to do nothing (other possible Radio button IDs that can be sent are 13 and 14 (3rd and 4th buttons in first group),
        // and 21, 22, 23, 24 (four buttons of second radio group), 31, 32, 33, 34 (third radio group), and 41, 42, 43, 44 (fourth radio group)
        // but here we don't bother handling those possibilities individually here
  }
}

