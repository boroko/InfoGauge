
/***************************************************
   InfoGauge5-57_T3.ino
   4/25/16, MAR. Using Teensy 3.1 and Adafruit_ILI9341.  Working version of 54_t3test
   PURPOSE: Display for vehicle monitoring. 5 bargraph + 8 numeric + hidden RPM reading for alarms
   STATUS: shifting readings, questionable 1-wire, except no acknowledge

   NEED:
   Add to Beeper driver routine:  2730Hz resonance of speaker or external 8ohm
   TC; need to display and alarm with 4 digits.
   ALARM Acknowledge routine
   Touchscreen getting multiple hits causing problems.  Slow down?
   new OneWire address addition routine integrated into menu to add.
   Decimal point on Volts readings
   Backlight PWM saved in EEPROM from menu, or dash light control, dim with headlights on: 2 settings
   RPM input figured out.
   "Dot" or "Bar" mode?
   Average conditions and place dot where normal should be.
   Alarms: maybe based on time, RPM, or Delta
   failed sensor check (CRC on OneWire), and pressure

   This Version:  Took out Color for alarm and its back.
   Not reading thermocouple 1wire. 4/9/16
   Changed Ain ports and using custom cable for remote 1.64. Will go to 1.66 eventually after proven.
 		Trying to make alarm Acknowledge routine.
		combining readings into an array
  New remote board: 1.64 for now: OneWire working with DS18B20s
   Oil Pressure,    Reading,       Bargraph                       ; 150# maybe 100#?
   Oil Temp,        Reading,       Bargraph                       ;   DS18B20
   Coolant Temp,    Reading,       Bargraph                       ;   DS18B20
   Trans Temp,      Reading,       Bargraph                       ;   DS18B20
   EGT,             Reading,       Bargraph                       ;   MAX31850
   Fuel pressure, Reading ; 100#       Fridge Temp,  Reading      ;   DS18B20
   Coolant pres,  Reading ; <15#       Air pressure  Reading      ; 150# maybe 100#
   Inside temp,   Reading ; DS18B20    Bat 1,        Reading
   Outside temp,  Reading ; DS18B20    Bat 2,        Reading
   NOT DISPLAYED: RPM (from Digital I/P) (maybe display on alternate screen)

   Possibles:
   A/C deltaT
   Bat 3 volts
   O2
   RPM
   Clock as default screen?
  	  Aux fuel
  	  Water level
  	  furnace run time
   manifold Vac (OBD)
   knock? (OBD)
  	  Modes: running/camping: Sleep when IGN off and wake up on touch?

   Menus:
   Add new sensor ID
   Backlight brightness: Lights ON/OFF
   Alarm Setpoints

    Digital Pins:								Analog Pins:may change
   0:	RX
   1:   TX                          A0: Bat 1
   2: OneWire 							        A1: Oil pressure sensor (Pot)
   3: PWM for Backlight             A2: Fuel Pressure
   4: MicroSD_CS                    A3: Coolant Pressure
   5: Alarm								          A4: Air Pressure
   6: Dash Light Dim                A5: Bat 2
   7: RPM
   8: STMP E_CS (Touch Screen)
   9: TFT_DC
   10: TFT_CS
   11: HW SPI
   12: HW SPI
   13: HW SPI

   Basic structure from "graphicstest", MAX31855 library, and GFX example  Written by Limor Fried/Ladyada for Adafruit Industries.  MIT license/ BSD license, check license.txt for more info
   Concept code from Matt McMillan to selectively refresh changing digits.  http://www.adafruit.com/products/1651
*/
/*-----( Import needed libraries )-----*/
#include "SPI.h"
#include "ILI9341_t3.h" // Adafruit ILI9341 2.8" GLCD optimized Teensy. DO NOT use _GFX.h with this.
#include "Wire.h"      // needed even though we aren't using it
#include "Adafruit_STMPE610.h" // The STMPE610 h/w SPI on shield, and #8
#include "OneWire.h"// 
#include "DallasTemperature.h"//
#include "EEPROM.h" // From Paul Stoffgren
/*-----( Declare Constants and Pin Numbers )-----*/
#define STMPE_CS 8 // Chip select for resistive touch screen
#define TFT_DC 9
#define TFT_CS 10
#define ONE_WIRE_BUS_PIN 2
#define backlight 3 // maybe the wrong pin?
#define Beep 5
#define TS_MINX 350// calibration data for raw touch data to the screen coordinates
#define TS_MINY 200
#define TS_MAXX 3800
#define TS_MAXY 3800
/*-----( Declare objects )-----*/
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);
Adafruit_STMPE610 ts = Adafruit_STMPE610(STMPE_CS); //
OneWire oneWire(ONE_WIRE_BUS_PIN);// Setup a oneWire instance to comm OneWire devices
DallasTemperature sensors(&oneWire);// Pass our oneWire reference to Dallas Temperature.
/*-----( Declare Variables )-----*/
// Assign the addresses of your 1-Wire temp sensors. each will have unique addresses
//***NEED to make these stored in EEPROM and changable
DeviceAddress Probe01 = {  0x28, 0x3D, 0x06, 0x77, 0x06, 0x00, 0x00, 0xDA };
DeviceAddress Probe02 = {  0x28, 0xA9, 0x0F, 0x96, 0x06, 0x00, 0x00, 0x82 };
DeviceAddress Probe03 = {  0x28, 0xF2, 0x83, 0x77, 0x06, 0x00, 0x00, 0x78 };
DeviceAddress Probe04 = {  0x28, 0xF8, 0x25, 0x99, 0x06, 0x00, 0x00, 0x44 };
DeviceAddress Probe05 = {  0x28, 0x4A, 0x7F, 0x7D, 0x06, 0x00, 0x00, 0x42 };
DeviceAddress Probe06 = {  0x3B, 0xFC, 0x6D, 0x18, 0x00, 0x00, 0x00, 0x74 };

int OilTempRead;       // SetPoint storage @ n=0, Condition storage @ c=20
int CoolTempRead;      // SetPoint storage @ n=1, Condition storage @ c=21
int TransTempRead;     // SetPoint storage @ n=2, Condition storage @ c=22
int EGTempRead;        // SetPoint storage @ n=3, Condition storage @ c=23
int InTempRead;        // SetPoint storage @ n=4, Condition storage @ c=24
int OutTempRead;       // SetPoint storage @ n=5, Condition storage @ c=25
int FridgeTempRead;    // SetPoint storage @ n=6, Condition storage @ c=26
int OilPresRead;       // SetPoint storage @ n=7, Condition storage @ c=27
int FuelPresRead;      // SetPoint storage @ n=8, Condition storage @ c=28
int CoolPresRead;      // SetPoint storage @ n=9, Condition storage @ c=29
int AirPresRead;       // SetPoint storage @ n=10, Condition storage @ c=30
int Bat1VoltRead;      // SetPoint storage @ n=11, Condition storage @ c=31
int Bat2VoltRead;      // SetPoint storage @ n=12, Condition storage @ c=32
int reading;		    // array for the readings
int prevCount[13];
int EGTval  =  1200;
int c; 				      // var for Condition of Alarm
int AlFlg = 0;      // Alarm flags
int ack = 0;        // Acknowledge flags
int i = 0;          // general var
int n = 0;			    // identifies parameter monitored
int v = 0;          // Value for each setpoint (n)
int sp[13];         // array for SetPoint (one each Parameter)
int cond[13];       // array for Condition of alarm setting
int blv;            // Backlight value. Stored at EEPROM #30. on pin 3
int backlightbox;
int barv;	        //Bar Value
int beep = 2730;		// alarm tone, resonant to element
const char* ParaName[] = {"Oil   F:", "Cool  F:", "Trans F:", "EGT   F:", "In    F:", "Out   F:", "FridgeF:", "Oil PSI:", "Fuel PSI:", "Cool PSI:", "Air PSI:", "Bat  1 :", "Bat  2 :"};
byte page = 0;
byte Row[] = {0, 28, 56, 84, 112, 140, 168, 196, 224, 232}; //Character Rows
word Col[] = {0, 106, 130, 145, 154, 280}; // X: Horiz spacing

void setup() {
  Serial.begin(9600);
  delay (500);
  Serial.println("InfoGauge 5-57_T3");
  Serial.println("4/25/16");
  //pinMode(backlight, OUTPUT); //no idea why this has to be REM'd to work!)
  sensors.begin();         // Initialize the Temperature measurement library
  sensors.setResolution(Probe01, 9);// set the resolution to 10 bit
  sensors.setResolution(Probe02, 9);//Can be 9-12 bits .. lower is faster)
  sensors.setResolution(Probe03, 9);
  sensors.setResolution(Probe04, 9);
  sensors.setResolution(Probe05, 9);
  tft.begin();
  if (!ts.begin()) {
    Serial.println("Couldn't start touchscreen controller");
    while (1);
  }
  //for(i = 0 ; i = 250; i++){
  /* for(i = 0 ; i <= blv; i+=1) {  // if blv set too low, stuck dark
     analogWrite(backlight, i);
     delay(2);
    }*/
  Serial.println("Touchscreen started");
  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(1); // Arrow indicates USB direction on Teensy: 0=^, 1=<, 2=v, 3=>
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(5, 0);
  tft.print("InfoGauge 5-57_T3:");
  tft.setCursor(5, 30);
  tft.print("4/25/16");
  //blv = EEPROM.read(33);    // Get the stored BlackLight value
  blv = 245;  //need to change to saved EEPROM eventually
  Serial.println("EEPROM read");
  Serial.println("Para, SP, cond, increm");
  delay(2000);
  tft.fillScreen(ILI9341_BLACK); //Set initial screen to black
  Screen0();
  //  page == 0;
  Serial.println("End Setup");
  tone(5, 440, 100); //  (pin, pitch, uS)
  delay(200);
  tone(5, beep, 30);
  delay(200);
} //END of void setup

void loop(void) {
  sensors.requestTemperatures();  // Command all devices on bus to read temperature
  if (page == 0) { //call subs
    OilTemperature();
    CoolTemperature();
    TransTemperature();
    EGTemp();
    InTemperature();
    OutTemperature();
    FridgeTemp();
    OilPressure();
    FuelPres();
    CoolPres();
    AirPres();
    Bat1Volt();
    Bat2Volt();
    Serial.println("");
  }
  if (ts.bufferEmpty()) {
    return;
  }
  TS_Point p = ts.getPoint();  //X & Y points range from 0-4095, Z = 0-255
  p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
  p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
  /* ***** TouchPositionSerial ******************
     Serial.print("    p.x: "); // Used to see response of touch screen
     Serial.print(p.x);
     Serial.print("  p.y: ");
     Serial.print(p.y);
     Serial.print("  blv: ");
     Serial.print(blv);
     Serial.print("  page: ");
     Serial.println(page);
  */
  while (! ts.bufferEmpty()) { // while not empty, keep calling buffer to empty.
    TS_Point p = ts.getPoint(); // to clear buffer and give single touches
  }
  if (page == 0) { // if page 0 (main)
    if (p.y > 0 && p.y < 320 && p.x > 0 && p.x < 226) { // And any part is pressed
      Serial.println(" TOUCHED");
      while (! ts.bufferEmpty()) { // while not empty, keep calling buffer to empty.
        TS_Point p = ts.getPoint(); // to clear buffer and give single touches
      }
      if (AlFlg > 0 && ack != AlFlg) { // any Alarm in and not acknowledged
        Serial.print(" Ack=");
        Serial.print(ack);
        Serial.print(", AlFlg=");
        Serial.println(AlFlg);
        Serial.print(" ALARM ACK");
        Serial.print(", n= ");
        Serial.println(n);
        // Turn off alarm beeper and stop screen flash.
        ack = AlFlg; // SET the acknowledge bits to match the alarms.
        Serial.print(" Ack=");
        Serial.print(ack);
        Serial.print(" AlFlg=");
        Serial.println(AlFlg);
        delay (200);
        return;
      }
      // set ACK bit for that parameter. alarm silence/flashing screen:clear in "Alarm Check"
      // Change color of reading if still in but ACK?
      else {
        page = 1;	// change the flag to 1
        Screen1(); // Change to the Setup Screen
        return;
      }
      alarmCheck();

      //      page = 1;	// change the flag to 1
      //      Screen1(); // Change to the Setup Screen
      //      return;
    }
    //    alarmCheck();
  }
  if (page != 0) { //If on any other than main page:
    //  Home button
    if (p.y > 280 && p.y < 340 && p.x > 0 && p.x < 48) { // if the home icon is pressed
      while (! ts.bufferEmpty()) { // while not empty, keep calling buffer to empty.
        TS_Point p = ts.getPoint(); // to clear buffer and give single touches
      }
      page = 0;	// change flag and
      Screen0(); // Change to the Main Screen
    }
  }
  // **** Menu Button Catch *****************
  if (page == 1) { //If on the Setup page
    if (p.y > 0 && p.y < 56 && p.x > 176 && p.x < 226) {// backlight setting buttons
      blightdown();
    }
    if (p.y > 260 && p.y < 320 && p.x > 180 && p.x < 230) {
      blightup();
    }
    if (p.y > 0 && p.y < 146 && p.x > 120 && p.x < 168) {     //Alarms button
      page = 2;
      n = 0;
      Screen2();
    }
    if (p.y > 0 && p.y < 146 && p.x > 54 && p.x < 104) {    // New TempID button
      page = 3;
      Screen3();
    }
  }
  // **** Alarm Button Catch *****************
  //Needs alarm, and EGT management. Combine button IF's
  if (page == 2) { // If on the Alarms screen:s
    if (p.y > 0 && p.y < 60) { // in the first row of buttons and ...
      //***** Parameter UP *****
      if (p.x > 180 && p.x < 230) { // Increment ParaName button pressed
        Serial.println(" INCREMENT");
        while (! ts.bufferEmpty()) { // while not empty, keep calling buffer to empty.
          TS_Point p = ts.getPoint(); // to clear buffer and give single touches
        }
        tft.fillRect(90, 35, 200, 25, ILI9341_BLACK);
        tft.setCursor(100, 40);
        n = (n + 1);			// Increment Parameter
        if (n > 12) n = 0;		// rollover if at end of list
        Serial.println(n);
        condButton(); 		//pull up the last stored parameter from EEPROM
        tft.fillRect(90, 162, 68, 45, ILI9341_BLACK);//clear the window
        tft.setCursor(100, 180);
        tft.print(v); // display the value assigned to a parameter pulled from EEPROM(n)
      }
      //***** Parameter Down *****
      if (p.x > 120 && p.x < 165) { //Decrement ParaName button pressed
        while (! ts.bufferEmpty()) { // while not empty, keep calling to empty.
          TS_Point p = ts.getPoint(); // to clear buffer and give single touches
        }	 tft.fillRect(90, 35, 200, 25, ILI9341_BLACK);
        tft.setCursor(100, 40);
        n = (n - 1);				  // decrement the Parameter [n=Parameter #]
        if (n < 0) n = 12;		// Rollover
        Serial.println(n);
        condButton();
        tft.fillRect(90, 162, 68, 45, ILI9341_BLACK);
        tft.setCursor(100, 180);
        tft.print(v);         // display the value
      }
      //***** Setpoint UP *****
      if (p.x > 55 && p.x < 100) { // Setpoint button pressed
        while (! ts.bufferEmpty()) { // while not empty, keep callingto empty.
          TS_Point p = ts.getPoint(); // to clear buffer and give single touches
        }
        tft.fillRect(90, 162, 68, 45, ILI9341_BLACK);
        tft.setCursor(100, 180);
        v = v + 1;			      // Increment the Setpoint [v=Value of setpoint]
        Serial.print(v);
        Serial.print(",");
        Serial.println(n);
        tft.print(v);
      }
      //***** Setpoint Down *****
      if (p.x > 0 && p.x < 45) { //Decrement ParaName
        while (! ts.bufferEmpty()) { // while not empty, keep calling buffer to empty.
          TS_Point p = ts.getPoint(); // to clear buffer and give single touches
        }
        tft.fillRect(90, 162, 68, 45, ILI9341_BLACK);
        tft.setCursor(100, 180);
        v = v - 1;            // Decrement the Setpoint [v=Value of setpoint]
        Serial.print(v);
        Serial.print(",");
        Serial.println(n);
        tft.print(v);         // display the value pulled from EEPROM(n)
      }
    }
    // Greater than
    if (p.y > 116 && p.y < 163 && p.x > 106 && p.x < 155) { // ">" button pressed
      while (! ts.bufferEmpty()) { // while not empty, keep calling buffer to empty.
        TS_Point p = ts.getPoint(); // to clear buffer and give single touches
      }
      GTbutON();			        // highlight the ">" button
      LTbutOFF();  			      // shade the "<" button
      c = (n + 20);			      // "cond[c]" is the memory space for storing the condition
      Serial.print(c);
      Serial.print(" > ");
      EEPROM.write(n + 20, 1);  // Save GreaterThan(1)for Para #n @ location (cond[i])
      delay(100);
      EEPROM.read (c);
      Serial.println(c);
    }
    // Less Than
    if (p.y > 199 && p.y < 257 && p.x > 107 && p.x < 156) {
      while (! ts.bufferEmpty()) { // while not empty, keep calling buffer to empty.
        TS_Point p = ts.getPoint(); // to clear buffer and give single touches
      }
      GTbutOFF();
      LTbutON();
      c = (n + 20);
      Serial.print(c);
      EEPROM.write(n + 20, 0); // Save LessThan(0)for Para #n @ location (cond[i])
      delay(100);
      Serial.print(" < ");
      EEPROM.read(c);
      Serial.println(c);
    }
    // Set
    if (p.y > 185 && p.y < 245 && p.x > 20 && p.x < 85) { // SET button pressed
      while (! ts.bufferEmpty()) { // while not empty, keep calling buffer to empty.
        TS_Point p = ts.getPoint(); // to clear buffer and give single touches
      }
      tft.fillRect(90, 162, 68, 45, ILI9341_BLACK);
      tft.setCursor(100, 180);;
      EEPROM.write(n, v);// in index[n], save setpoint value[v] in EEPROM
      Serial.print("Index:  ");
      Serial.print(n);
      Serial.print(", ");
      Serial.print("Value:  ");
      Serial.print(v);
      Serial.println(" Set");
      tft.print("SET");
      tft.fillRect(90, 162, 68, 45, ILI9341_BLACK);
      tft.setCursor(100, 180);
      //tft.print(sp[v]);
      tft.print(v); //maybe correct readout after SET?
      delay(4000);
    }
  }
  //******* New ID button catch *************
  if (page == 3) { // If on the New ID screen:
    tft.setTextSize(2);
    // Get Button
    if (p.y > 0 && p.y < 60 && p.x > 170 && p.x < 223) {
      discoverOneWireDevices();		// call discoverOneWireDevices
      i = 0;
    }
    //    // UP button:  MOVED TO discoverOneWireDevices
    //    if (p.y > 0 && p.y < 60 && p.x > 55 && p.x < 100) {
    //      tft.fillRect(85, 123, 210, 45, ILI9341_BLACK);
    //      tft.setCursor(90, 140);
    //      i = (i + 1);
    //      if (i > 6) i = 0;
    //      tft.print(ParaName[i]);
    //    }
    //    // Down button
    //    if (p.y > 0 && p.y < 60 && p.x > 0 && p.x < 45) { //Decrement ParaName
    //      tft.fillRect(85, 123, 210, 45, ILI9341_BLACK);
    //      tft.setCursor(90, 140);
    //      // tft.setTextSize(2);
    //      i = (i - 1);
    //      if (i < 0) i = 6;
    //      tft.print(ParaName[i]);
    //    }
    //    // ADD button     // assign captured number to selected parameter
    //    if (p.y > 145 && p.y < 205 && p.x > 0 && p.x < 50) {
    //      //  xxx();		// call "Assign" Hex address of OneWire to ParaName
    //      tft.fillRect(85, 123, 210, 45, ILI9341_BLACK);
    //      tft.setCursor(90, 140);
    //      tft.print("ADD Button");
    //    }
  }
  // }
}// END of void loop
//***** Condition Button change ***********
void condButton() { //
  tft.print(ParaName[n]);
  v = EEPROM.read(n);    //read the last stored setpoint for that parameter
  c = EEPROM.read(n + 20); // read condition for that parameter
  if (c > 0) {
    GTbutON();
    LTbutOFF();
  }
  else {
    GTbutOFF();
    LTbutON();
  }
}
//**** Alarm Check ****************************************
void alarmCheck() {


  Serial.print(" Alarm Check ");
  sp[n] = EEPROM.read(n);
  cond[n] = EEPROM.read(n + 20);
  Serial.print(ParaName[n]);      // Parameter name
  Serial.print("\t");
  Serial.print(sp[n]);            // Setpoint
  Serial.print("\t");
  Serial.print(cond[n]);          // "<" or ">" condition
  Serial.print("\t");
  Serial.print(v);                // The value read
  Serial.print("\t");
  Serial.println(n);              // sequence

  /*    if (AlFlg != ack) {
        if ((cond[n] = 1) && (v > sp[n])) {  // IF cond[n] is > and the value is > than the setpoint: alarm
          Serial.print("  > ALARM , ");
          alarmSound();
        }
        else   if ((cond[n] = 1) && (v < sp[n])) {
          bitClear(AlFlg, n);                    // Clear the alarm bit
          bitClear(ack, n);         // Reset the Ack bit
        }
        if ((cond[(n)] = 0) && (v < sp[n])) { // IF cond[n] is < and the value is < than the setpoint: alarm
          Serial.print("  < ALARM , ");
          alarmSound();
        }
        if ((cond[(n)] = 0) && (v > sp[n])) {
          bitClear(AlFlg, n);                     // Clear the alarm bit
          bitClear(ack, n);          // Reset the Ack bit
        }
      }*/
}
//***** Alarm Sound *****************************************
void alarmSound() {
  Serial.print(ParaName[n]);
  Serial.print(" , ");
  Serial.print(v);
  Serial.print("/");
  Serial.println(sp[n]);
  bitSet(AlFlg, n);           // SETs the bit in "a" if an alarm. "n" determines which bit/ correlates to parameter
  blv = 10;             // alarm: DIM
  analogWrite(backlight, blv);
  delay (200);
  blv = 250;            // alarm: BRIGHT
  analogWrite(backlight, blv);
  delay (500);
  blv = EEPROM.read(33);  // set backlight back to stored after alarm done.
  analogWrite(backlight, blv);
  // NEED to implement some sort of acknowledge for an alarm.
  // Set a flag and enable the touch screen to acknowledge until the next alarm.
  // Display the alarmed parameter when acknowledged or highlight the alarmed parameter.

}
//**** Oil Pressure  ***********************************
void OilPressure() {  //OIL PRESSURE read, scale, digit and bargraph display
  reading = analogRead(A0);  // reading from sensor: 0.5v-4.5v = 0-100 psi, 102.4 - 921.6
  reading = map(reading, 102, 820, 0, 100); // "map" works, but slows down / sucks memory..
  reading = constrain(reading, 0, 100);
  NumberParse (reading, 0, 0, 1);
  v = reading; //
  n = 7;          // pointer for alarm and condition values
  alarmCheck();    // check if this reading is in alarm
  //****SUB: Bargraph ** 23 elements, 150 pixels *******************************
  int bar = (reading * 1.4);
  tft.setTextSize(1);
  tft.fillRect(177, Row[0], bar, 8, ILI9341_RED);
  tft.fillRect(177 + bar, Row[0], (320 - 177 - bar), 8, ILI9341_BLACK);
}
//***** FuelPressure ******************************************************
void FuelPres() {
  reading = analogRead(A1);    //reading from sensor: 0.5v-4.5v = 0-100 psi, 102.4-921.6
  reading = map(reading, 102, 820, 0, 100); // "map" works, but really slows things down and sucks up memory.
  reading = constrain(reading, 0, 100);
  NumberParse (reading, 5, 5, 1);
  v = reading; //
  n = 8;          // pointer for alarm and condition values
  alarmCheck();    // check if this reading is in alarm
}
//***** CoolPressure ******************************************************
void CoolPres() {
  reading = analogRead(A2);    //reading from sensor: 0.5v-4.5v = 0-15 psi, 102.4-921.6
  reading = map(reading, 102, 820, 0, 15); // "map" works, but really slows things down.
  reading = constrain(reading, 0, 15);
  NumberParse (reading, 6, 6, 1);
  v = reading; //
  n = 9;          // pointer for alarm and condition values
  alarmCheck();    // check if this reading is in alarm
}
//***** Air Pressure ******************************************************
void AirPres() {
  reading = analogRead(A3); //sensor: 0.5v-4.5v = 0-150 psi, 102.4-921.6
  reading = map(reading, 102, 820, 0, 150);
  reading = constrain(reading, 0, 150);
  NumberParse (reading, 10, 6, 5);
  v = reading; //
  n = 10;          // pointer for alarm and condition values
  alarmCheck();    // check if this reading is in alarm
}
//***** Oil Temperature ********************************************************
void OilTemperature() {  // read the value from OIL TEMPERATURE sensor:
  OilTempRead = sensors.getTempF(Probe01);
  NumberParse (OilTempRead, 1, 1, 1);
  int bar = ((OilTempRead * 10) / 18); // 7 * 146 =1022
  tft.setTextSize(1);
  tft.fillRect(177, Row[1], bar, 8, ILI9341_RED);
  tft.fillRect(177 + bar, Row[1], (320 - 177 - bar), 8, ILI9341_BLACK);
  v = OilTempRead; //
  n = 0;          // pointer for alarm and condition values
  alarmCheck();    // check if this reading is in alarm
}
//***** Coolant Temperature ********************************************
void CoolTemperature() {
  CoolTempRead = sensors.getTempF(Probe02);
  NumberParse (CoolTempRead, 3, 3, 1);
  int bar = ((CoolTempRead * 10) / 18); // 7 * 146 =1022
  tft.setTextSize(1);
  tft.fillRect(177, Row[3], bar, 8, ILI9341_RED);
  tft.fillRect(177 + bar, Row[3], (320 - 177 - bar), 8, ILI9341_BLACK);
  v = CoolTempRead; //
  n = 1;          // pointer for alarm and condition values
  alarmCheck();    // check if this reading is in alarm
}
//**** Transmission Temperature***********************
void TransTemperature() {
  TransTempRead = sensors.getTempF(Probe03);
  NumberParse (TransTempRead, 2, 2, 1);
  int bar = ((TransTempRead * 10) / 18); // 7 * 146 =1022
  tft.setTextSize(1);
  tft.fillRect(177, Row[2], bar, 8, ILI9341_RED);
  tft.fillRect(177 + bar, Row[2], (320 - 177 - bar), 8, ILI9341_BLACK);
  v = reading;    // check if this reading is in alarm; //
  n = 2;          // pointer for alarm and condition values
  alarmCheck();    // check if this reading is in alarm
}
//***** EGT (Temperature) ******************************************************
void EGTemp() { // One-Wire comms to a MAX31850:
  EGTempRead = sensors.getTempF(Probe06);
  // Serial.println(EGTempRead);
  NumberParse (EGTempRead, 4, 4, 1);
  int bar = (EGTempRead / 9); //
  tft.setTextSize(1);
  tft.fillRect(177, Row[4], bar, 8, ILI9341_RED);
  tft.fillRect(177 + bar, Row[4], (320 - 177 - bar), 8, ILI9341_BLACK);
  v = EGTempRead;    // check if this reading is in alarm; //
  n = 3;          // pointer for alarm and condition values
  alarmCheck();    // check if this reading is in alarm

}  // Get stuff, correct reading for temp, parse it for position(4 digit),
//***** Inside Temperature ******************************************************
void InTemperature() {
  InTempRead = sensors.getTempF(Probe04);
  NumberParse (InTempRead, 7, 7, 1);
  v = InTempRead;    // check if this reading is in alarm; //
  n = 4;          // pointer for alarm and condition values
  alarmCheck();    // check if this reading is in alarm
}
//***** Outside Temperature ******************************************************
void OutTemperature() {  // these readings are in a different column
  OutTempRead = sensors.getTempF(Probe05);
  NumberParse (OutTempRead, 8, 8, 1);
  v = OutTempRead;    // check if this reading is in alarm; //
  n = 5;          // pointer for alarm and condition values
  alarmCheck();    // check if this reading is in alarm
}
//***** Fridge (Temperature) ******************************************************
void FridgeTemp() {
  FridgeTempRead = sensors.getTempF(Probe05);
  NumberParse (FridgeTempRead, 9, 5, 5);
  v = FridgeTempRead;    // check if this reading is in alarm; //
  n = 6;          // pointer for alarm and condition values
  alarmCheck();    // check if this reading is in alarm
}
//***** Bat1Volts ******************************************************
void Bat1Volt() {
  unsigned int raw = analogRead(A5);    // CHANGE TO A4. Sim from Bat1: 0.0v-5.0v = 0-15.36v, 0-1024
  unsigned int Bat1VoltRead = ((raw * .08) * 3);
  NumberParse (Bat1VoltRead, 11, 7, 5);
  v = Bat1VoltRead;    // check if this reading is in alarm; //
  n = 11;          // pointer for alarm and condition values
  alarmCheck();    // check if this reading is in alarm
}
//***** Bat2Volts ******************************************************
void Bat2Volt() {
  unsigned int raw = analogRead(A5);    // sim reading from Bat2: 0.0v-5.0v = 0-15.36v, 0-1024
  unsigned int Bat2VoltRead = ((raw * .08) * 3);
  NumberParse (Bat2VoltRead, 12, 8, 5);
  v = Bat2VoltRead;    // check if this reading is in alarm; //
  n = 12;          // pointer for alarm and condition values
  alarmCheck();    // check if this reading is in alarm
}
//******* Number Parse **********************************************************
unsigned int NumberParse(int Read, int LC, int R, int C) { //(Reading, # for LastCount, # for Row, # for Column)Maybe add DP
  int ValueDigits[] = {0, 0, 0}; // one for each digit, local
  int prevdigits[] = {0, 0, 0};
  ValueDigits[2] = Read % 10; // Grab the last digit:
  if (Read > 99) { // handling of middle digit depends on if 2 or 3
    ValueDigits[1] = (Read / 10) % 10;
  }
  else {
    // put blanking here
    ValueDigits[1] = Read / 10;
  }
  ValueDigits[0] = Read / 100; // Grab the first digit
  prevdigits[2] = prevCount[LC] % 10; // Split out digits of previous reading:
  if (prevCount[LC] > 99) {
    prevdigits[1] = (prevCount[LC] / 10) % 10;
  }
  else {
    prevdigits[1] = prevCount[LC] / 10;
  }
  prevdigits[0] = prevCount[LC] / 100;
  tft.setTextSize(2);
  if (Read != prevCount[LC]) { // Compare digit to previous and redraw if changed.
    if (ValueDigits[0] != prevdigits[0]) {
      tft.fillRect(Col[C], Row[R], 12, 18, ILI9341_BLACK);
      if ((Read > 99) and (ValueDigits[0] > 0 )) {
        tft.setCursor(Col[C], Row[R]);
        tft.setTextColor(ILI9341_WHITE);
        tft.print(ValueDigits[0]);
      }
    }
    if (ValueDigits[1] != prevdigits[1]) {
      tft.fillRect((Col[C] + 12), Row[R], 12, 18, ILI9341_BLACK);
    }
    if (Read >= 99) {
      tft.setCursor((Col[C] + 12), Row[R]);
      tft.setTextColor(ILI9341_WHITE);
      tft.print(ValueDigits[1]);
    }
    else if ((Read < 99) and (ValueDigits[1] > 0)) {
      tft.setCursor((Col[C] + 12), Row[R]);
      tft.setTextColor(ILI9341_WHITE);
      tft.print(ValueDigits[1]);
    }
    if (ValueDigits[2] != prevdigits[2]) {
      tft.fillRect((Col[C] + 24), Row[R], 12, 18, ILI9341_BLACK); // Pixel (X, Y, width, height, color)
      tft.setCursor((Col[C] + 24), Row[R]);
      tft.setTextColor(ILI9341_WHITE);
      tft.print(ValueDigits[2]);
    }
  }
  prevCount[LC] = Read; //
}



/* //unsigned int NumberParse(int Read, int LC, int R, int C) {
  void NumberParse(int Read, int LC, int R, int C) { //Reading, # for LastCount, # for Row, # for Column
  int ValueDigits[] = {0, 0, 0}; // one for each digit, local
  int prevdigits[] = {0, 0, 0};
  ValueDigits[2] = Read % 10; // Grab the last digit:
  if (Read > 99) { // handling of middle digit depends on if 2 or 3
    ValueDigits[1] = (Read / 10) % 10;
  }
  else {
    // put blanking here
    ValueDigits[1] = Read / 10;
  }
  ValueDigits[0] = Read / 100; // Grab the first digit
  prevdigits[2] = prevCount[LC] % 10; // Split out digits of previous reading:
  if (prevCount[LC] > 99) {
    prevdigits[1] = (prevCount[LC] / 10) % 10;
  }
  else {
    prevdigits[1] = prevCount[LC] / 10;
  }
  prevdigits[0] = prevCount[LC] / 100;
  tft.setTextSize(2);
  if (Read != prevCount[LC]) { // Compare digit to previous and redraw if changed.
    if (ValueDigits[0] != prevdigits[0]) {
      tft.fillRect(Col[C], Row[R], 12, 18, ILI9341_BLACK);
      if ((Read > 99) and (ValueDigits[0] > 0 )) {
        tft.setCursor(Col[C], Row[R]);
        if (AlFlg = 1) {
          tft.setTextColor(ILI9341_RED);
        }
        else {
          tft.setTextColor(ILI9341_WHITE);
          tft.print(ValueDigits[0]);
        }
      }
      if (ValueDigits[1] != prevdigits[1]) {
        tft.fillRect((Col[C] + 12), Row[R], 12, 18, ILI9341_BLACK);
      }
      if (Read >= 99) {
        tft.setCursor((Col[C] + 12), Row[R]);
        if (AlFlg = 1); {
          tft.setTextColor(ILI9341_RED);
        }
      }
      else {
        tft.setTextColor(ILI9341_WHITE);
        tft.print(ValueDigits[1]);
      }
    }
    else if ((Read < 99) and (ValueDigits[1] > 0)) {
      tft.setCursor((Col[C] + 12), Row[R]);
      if (AlFlg = 1) {
        tft.setTextColor(ILI9341_RED);
      }
      else {
        tft.setTextColor(ILI9341_WHITE);
        tft.print(ValueDigits[1]);
      }
    }
    if (ValueDigits[2] != prevdigits[2]) {
      tft.fillRect((Col[C] + 24), Row[R], 12, 18, ILI9341_BLACK); // Pixel (X, Y, width, height, color)
      tft.setCursor((Col[C] + 24), Row[R]);
      if (AlFlg = 1); {
        tft.setTextColor(ILI9341_RED);
      }
    }
    else {
      tft.setTextColor(ILI9341_WHITE);
      tft.print(ValueDigits[2]);
    }
  }
  prevCount[LC] = Read; //
  } */
//**** Bargraph *******************************************************
void BarGraph(int Color, int n) {
  tft.setTextColor(Color);
  tft.setCursor(170, n);
  tft.setTextSize(1);
  tft.print(" |......................|");
  tft.setTextSize(2);
}
// *** Draw Home Icon **********
void drawhomeicon() { // draws a white home icon
  tft.drawLine(280, 219, 299, 200, ILI9341_WHITE);
  tft.drawLine(300, 200, 304, 204, ILI9341_WHITE);
  tft.drawLine(304, 203, 304, 200, ILI9341_WHITE);
  tft.drawLine(305, 200, 307, 200, ILI9341_WHITE);
  tft.drawLine(308, 200, 308, 208, ILI9341_WHITE);
  tft.drawLine(309, 209, 319, 219, ILI9341_WHITE);
  tft.drawLine(281, 219, 283, 219, ILI9341_WHITE);
  tft.drawLine(316, 219, 318, 219, ILI9341_WHITE);
  tft.drawRect(284, 219, 32, 21, ILI9341_WHITE);
  tft.drawRect(295, 225, 10, 15, ILI9341_WHITE);
}
// **** Draw SetScreen ******************************************
void SetScreen() {
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  boxes();
  tft.setCursor(41, 97);
  tft.print("Alarms");  //Menu 3
  tft.setCursor(18, 157);
  tft.print("New TempID");      //Menu 5
}
// **** Draw Button Boxes ****************************************
void boxes() { // redraw the button outline boxes
  tft.drawRect(0, 80, 150, 50, ILI9341_CYAN);        // Menu 3
  tft.drawRect(0, 140, 150, 50, ILI9341_CYAN);       // Menu 5
}
// *** Draw Top +/- buttons **********************************
void Tbuttons() { // redraw the button outline boxes
  tft.fillRect(0, 0, 60, 50, ILI9341_GREEN);		//Top + button
  tft.drawRect(0, 0, 60, 50, ILI9341_WHITE);
  tft.drawRect(80, 30, 225, 35, ILI9341_CYAN);
  tft.fillRect(0, 60, 60, 50, ILI9341_RED);
  tft.drawRect(0, 60, 60, 50, ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setCursor(22, 16);
  tft.print("+");
  tft.setCursor(22, 75);
  tft.print("-");
  tft.setTextSize(2);
}
// *** Draw Bottom +/- buttons **********************************
void Bbuttons() { // redraw the button outline boxes
  tft.fillRect(0, 130, 60, 50, ILI9341_GREEN);  //Bottom +/- buttons
  tft.drawRect(0, 130, 60, 50, ILI9341_WHITE);
  tft.fillRect(0, 190, 60, 50, ILI9341_RED);
  tft.drawRect(0, 190, 60, 50, ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setCursor(22, 146);
  tft.print("+");
  tft.setCursor(22, 206);
  tft.print("-");
  tft.setTextSize(2);
}
//***** GreaterThan Button ON ********
void GTbutON() {
  tft.setTextSize(3);
  tft.drawRect(120, 80, 60, 50, ILI9341_WHITE);
  tft.fillRect(120, 80, 60, 50, ILI9341_GREEN);
  tft.setCursor(140, 95);
  tft.print(">");
  tft.setTextSize(2);
}
//***** GreaterThan Button OFF ********
void GTbutOFF() {
  tft.setTextSize(3);
  tft.drawRect(120, 80, 60, 50, ILI9341_WHITE);
  tft.fillRect(120, 80, 60, 50, ILI9341_LIGHTGREY);
  tft.setCursor(140, 95);
  tft.print(">");
  tft.setTextSize(2);
}
//***** LessThan Button ON ********
void LTbutON() {
  tft.setTextSize(3);
  tft.drawRect(200, 80, 60, 50, ILI9341_WHITE);
  tft.fillRect(200, 80, 60, 50, ILI9341_GREEN);
  tft.setCursor(220, 95);
  tft.print("<");
  tft.setTextSize(2);
}
//***** LessThan Button OFF ********
void LTbutOFF() {
  tft.setTextSize(3);
  tft.drawRect(200, 80, 60, 50, ILI9341_WHITE);
  tft.fillRect(200, 80, 60, 50, ILI9341_LIGHTGREY);
  tft.setCursor(220, 95);
  tft.print("<");
  tft.setTextSize(2);
}

//***** Backlight Screen ******************
void settingsscr() {    // backlight level
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.fillRect(0, 20, 60, 50, ILI9341_RED);
  tft.drawRect(0, 20, 60, 50, ILI9341_WHITE);
  tft.setCursor(22, 33);
  tft.print("-");
  tft.drawRect(80, 20, 160, 50, ILI9341_CYAN);
  tft.fillRect(260, 20, 60, 50, ILI9341_GREEN);
  tft.drawRect(260, 20, 60, 50, ILI9341_WHITE);
  tft.setCursor(282, 33);
  tft.print("+");
  tft.setTextSize(1);
  tft.setCursor(120, 31);
  tft.print("Backlight Level");
  tft.drawRect(110, 48, 100, 10, ILI9341_WHITE);
  blbar();
}
void blightup() { // increase the backlight brightness
  blv = blv + 5;
  if (blv >= 255) {
    blv = 255;
  }
  analogWrite(backlight, blv);
  EEPROM.write(33, blv);
  delay(100);
  blbar();
  Serial.print("BackLight Value: ");
  Serial.println(blv);
}
void blightdown() { // decrease the backlight brightness
  blv = blv - 5;
  if (blv <= 5) {
    blv = 5;
  }
  analogWrite(backlight, blv);
  EEPROM.write(33, blv);
  delay(100);
  blbar();
  Serial.print("BackLight Value: ");
  Serial.println(blv);
}
void blbar() { // this function fills the yellow bar in the backlight brightness adjustment
  if (blv < barv) {
    tft.fillRect(111, 49, 98, 8, ILI9341_BLACK);
  }
  backlightbox = map(blv, 1, 255, 0, 98);
  tft.fillRect(111, 49, backlightbox, 8, ILI9341_YELLOW);
  barv = blv;
  delay(25);
}
void clearmessage() {
  tft.fillRect(12, 213, 226, 16, ILI9341_BLACK); // black out the inside of the message box
}

//*** Discover OneWire Device *****************************************
//Instructions for connecting only one device for the test, marking it and installing it.
void discoverOneWireDevices(void) {
  byte i;   // for selecting the address
  byte para;   // for selecting the parameter
  byte present = 0;
  byte type_s; // for determining resolution and decode
  byte data[12];
  byte addr[8];
  tft.setCursor(10, 82);
  tft.setTextSize(1);
  //tft.print("Connect a single 1-Wire devices and press OK");
  // wait for "OK"
  tft.print("Looking for 1-Wire devices...");
  while (oneWire.search(addr)) {
    tft.fillRect(2, 78, 310, 22, ILI9341_BLACK);
    tft.setCursor(10, 85);
    //tft.print("Found \'1-Wire\' device with address:");
    for ( i = 0; i < 8; i++) {
      tft.print("0x");
      if (addr[i] < 16) {
        tft.print('0');
      }
      tft.print(addr[i], HEX);
      if (i < 7) {
        tft.print(", ");
      }
    }
    Serial.println("");
    Serial.print("ROM =");
    for ( i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr[i], HEX);
    }
    // the first ROM byte indicates which chip
    switch (addr[0]) {
        Serial.print(":");
      case 0x10:
        Serial.println("  Chip = DS18S20");  // or old DS1820
        type_s = 1;
        break;
      case 0x28:
        Serial.println("  Chip = DS18B20");
        type_s = 0;
        break;
      case 0x3B:
        Serial.println("  Chip = MAX31850");
        type_s = 0;
        break;
      default:
        Serial.println("Not a DS18x20 or MAX31855 device.");
        return;
    }

    if ( OneWire::crc8( addr, 7) != addr[7]) {
      tft.fillRect(2, 78, 310, 22, ILI9341_BLACK);
      tft.print("CRC is not valid!");
      return;
    }

   do{
   // while (! ts.bufferEmpty()) { // while not empty, keep calling buffer to empty.
      TS_Point p = ts.getPoint();  //X & Y points range from 0-4095, Z = 0-255
      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
      delay(100);
   
    // UP button
    if (p.y > 0 && p.y < 60 && p.x > 55 && p.x < 100) {
      tft.fillRect(85, 123, 210, 45, ILI9341_BLACK);
      tft.setCursor(90, 140);
      para = (para + 1);
      if (para > 6) para = 0;
      tft.print(ParaName[para]);
    }
    // Down button
    if (p.y > 0 && p.y < 60 && p.x > 0 && p.x < 45) { //Decrement ParaName
      tft.fillRect(85, 123, 210, 45, ILI9341_BLACK);
      tft.setCursor(90, 140);
      // tft.setTextSize(2);
      para = (para + 1);
      if (para > 6) para = 0;
      tft.print(ParaName[para]);
    }
    // ADD button     // assign captured number to selected parameter
   while (!(p.y > 145 && p.y < 205 && p.x > 0 && p.x < 50)) { //wait for operator to select which parameter is read 
      //  xxx();    // call "Assign" Hex address of OneWire to ParaName
      tft.fillRect(85, 123, 210, 45, ILI9341_BLACK);
      tft.setCursor(90, 140);
      tft.print("ADD Button"); 
      // save the setting
      delay(200);
    }
  }
  }
  // Assign addr to that parameter and save
  //   tft.print("Done");
  oneWire.reset_search();
  return;

}


// **** Screen0: Main Screen fixed background *************************
void Screen0() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_YELLOW);
  tft.print(ParaName[7]);			//Oil PSI
  BarGraph(ILI9341_YELLOW, 6);
  tft.setCursor(0, Row[1]);
  tft.setTextColor(ILI9341_ORANGE);
  tft.print(ParaName[0]);			//Oil F
  BarGraph(ILI9341_ORANGE, 34);
  tft.setCursor(0, Row[2]);
  tft.setTextColor(ILI9341_BLUE);
  tft.print(ParaName[1]); 			//Cool F
  BarGraph(ILI9341_BLUE, 62);
  tft.setCursor(0, Row[3]);
  tft.setTextColor(ILI9341_GREEN);
  tft.print(ParaName[2]);			//Trans F
  BarGraph(ILI9341_GREEN, 90);
  tft.setCursor(0, Row[4]);
  tft.setTextColor(ILI9341_RED);
  tft.print(ParaName[3]); 			//EGT F
  BarGraph(ILI9341_RED, 118);
  tft.setCursor(0, Row[5]);
  tft.setTextColor(ILI9341_DARKCYAN);
  tft.print(ParaName[8]); 			//Fuel PSI
  tft.setCursor(0, Row[6]);
  tft.setTextColor(ILI9341_DARKCYAN);
  tft.print(ParaName[9]);			//Cool PSI
  tft.setCursor(0, Row[7]);
  tft.setTextColor(ILI9341_DARKCYAN);
  tft.print(ParaName[4]);			//In F
  tft.setCursor(0, Row[8]);
  tft.setTextColor(ILI9341_DARKCYAN);
  tft.print(ParaName[5]);			// Out F
  tft.setCursor(170, Row[5]);
  tft.setTextColor(ILI9341_DARKCYAN);
  tft.print(ParaName[6]); 			// Fridge F
  tft.setCursor(170, Row[6]);
  tft.setTextColor(ILI9341_DARKCYAN);
  tft.print(ParaName[10]);			// Air PSI
  tft.setCursor(170, Row[7]);
  tft.setTextColor(ILI9341_DARKCYAN);
  tft.print(ParaName[11]);			// Bat 1
  tft.setCursor(170, Row[8]);
  tft.setTextColor(ILI9341_DARKCYAN);
  tft.print(ParaName[12]);			// Bat 2
}
//***** Screen1: Alt display screen **************************
void Screen1() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_YELLOW);
  tft.print("Settings Screen");
  drawhomeicon(); // draw the home icon
  SetScreen(); // draw the Alt screen
  settingsscr(); // draw the backlight adjust bar
  tft.drawRect(0, 200, 245, 40, ILI9341_WHITE); // message box
}
// **** Screen2: Alarms  ************
void Screen2() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(100, 0);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_YELLOW);
  tft.print("Alarm Screen");
  Tbuttons();
  GTbutOFF();		//function for GreaterThan Button OFF (Grey)
  LTbutOFF();		////function for LessThan Button OFF (Grey)
  Bbuttons();								// Bottom +/- buttons
  tft.drawRect(80, 160, 80, 50, ILI9341_WHITE);
  tft.drawRect(190, 160, 60, 50, ILI9341_WHITE);
  tft.fillRect(190, 160, 60, 50, ILI9341_BLUE);
  tft.setCursor(203, 177);
  tft.setTextSize(2);
  tft.print("Set");
  drawhomeicon(); // draw the home icon
}
// **** Screen3 : New ID  ***********************
void Screen3() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  //tft.print("New ID Screen");
  tft.setTextSize(1);
  tft.setCursor(18, 0);
  tft.print("Connect a single 1-Wire devices and press GET");
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.fillRect(0, 23, 60, 50, ILI9341_ORANGE); // Orange GET button
  tft.drawRect(0, 23, 60, 50, ILI9341_WHITE);
  tft.setCursor(5, 38);
  tft.print("Get");
  tft.drawRect(0, 77, 319, 25, ILI9341_CYAN);  // "ID" box for GET
  Bbuttons();    								// Bottom +/- buttons
  tft.drawRect(80, 120, 220, 50, ILI9341_CYAN);  // "Assign" box
  tft.fillRect(150, 185, 60, 50, ILI9341_BLUE);  // "Add" Button (Assign)
  tft.drawRect(150, 185, 60, 50, ILI9341_WHITE);
  tft.setCursor(155, 198);
  tft.print("Add");
  drawhomeicon(); // draw the home icon
}
//*/
////**** Tachometer *******************************************************************
//// NEED: setup digital input
////float bar= ((OutTempRead*10) / 74); // 7 * 146 =1022
//tft.setTextSize(1);
//tft.fillRect(177, 192, 177 + bar-1, 8, ILI9341_BLACK);
//tft.fillRect (177+ bar, Row[6], 5, 8, ILI9341_YELLOW);
////tft.fillRect(177 + (bar + 5), 192, 320, 8, ILI9341_BLACK);

//return micros() - start;
//}
//*********( THE END )***********


