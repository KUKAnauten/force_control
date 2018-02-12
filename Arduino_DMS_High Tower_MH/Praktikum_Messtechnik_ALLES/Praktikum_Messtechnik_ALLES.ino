// Praktikum Messtechnik, geschrieben von Eike Christmann, TU Darmstadt 2016

#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include <AD5422.h>
#include <ADS1220.h>
#include <SD.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include <ros.h>
#include <std_msgs/Float32.h>

AD5422 myAD5422;
ADS1220 myADS1220;
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

enum State {Einschalten, Messung};
State myState = Einschalten;
unsigned long prev; // Zeit Variable
unsigned long prev_SD; // Zeit Variable
short start;
float Spannung;
float Kraft;
// Aktiviert/ Deaktiviert Ausgabe an SD Karte
boolean enableSD = false;
// Aktiviert/ Deaktiviert Ausgabe an UART
boolean enableUART = false;
// chip select der SD Karte
const int CS3 = 4;
// Zähler für SD Karte
int i = 0;
// Zähler für UART
int j = 0;
// Zähler für schreibe_Display()
int k = 0;

// An dieser Stelle werden char im Flash Speicher gespeichert.
// Diese werden nach Bedarf mit der Funktion schreibe_Display aus dem Speicher geladen und in buffer zischengespeichert
char buffer[30];
const char string_0[] PROGMEM = "Initializing SD card...";
const char string_1[] PROGMEM = "Card failed, or not present";
const char string_2[] PROGMEM = "card initialized.";
const char string_3[] PROGMEM = "error opening datalog.txt";
const char string_4[] PROGMEM = "SD Card enabled";
const char string_5[] PROGMEM = "SD Card disabled";
const char string_6[] PROGMEM = "UART enabled";
const char string_7[] PROGMEM = "UART disabled";
const char string_8[] PROGMEM = "Press 'up' to";
const char string_9[] PROGMEM = "=====START======";
const char string_10[] PROGMEM = "Press 'Reset' to";
const char string_11[] PROGMEM = "======STOP======";

const char* const string_table[] PROGMEM = {string_0, string_1, string_2, string_3, string_4, string_5, string_6, string_7, string_8, string_9, string_10, string_11};

// Einstellungen Messung:
// ===================================================================================================================
// Hier die gewünschte Brückenspannung einstellen. Bei Modus Stromspeisung hier Strom
// einstellen. z.B. 5 für 5 V und 0.02 für 20 mA
const float setU_I = 0.02;
// Spannungsspeisung im Bereich 0 V bis 5 V --> 0
// Stromspeisung im Bereich 4 mA bis 20 mA --> 5
const int range = 5;
// Kanäle AINP = AIN1, AINN = AIN2 --> 3
const int mux = 3;
// Verstärkung: PGA = 128 --> 128
const int gain = 128;
// Mode: Normal mode --> 'N', Turbo mode --> 'T'
const char mode = 'T';
// Datarate: 20 SPS --> 20
const int rate = 2000;
// Continuous mode --> true
const boolean continuous = true;
// ===================================================================================================================

// Einstellungen Kraftmessung:
// Hier werden die Parameter eingestellt, welche für die Kraftmessung nötig sind: f(x)=m*x+b
// Empfindlichkeit m:
const float m = 0.4757;
// Offset b: Dieser Wert mit betätigen der Tara Taste (Select) verändert.
float b = 0.177;



void setup() {
  start = millis();
  Serial.begin(115200);
  digitalWrite(CS1, HIGH);
  digitalWrite(CS2, HIGH);
  digitalWrite(CS3, HIGH);

  lcd.begin(16, 2);
  // Weiße Hintergrundfarbe. Weitere Farben sind möglich.
  lcd.setBacklight(0x7);

  myAD5422.begin();
  // Erläuterungen zum Reset siehe Library
  myAD5422.writeAD5422(cmdReset, 0x00, 0x01);
  myAD5422.setOutRange(range);
  myAD5422.enableOutput(true);
  //  Hier die Spannung in V oder Strom in mA einstellen
  //  An dieser Stelle gibt es noch einen kleinen Bug welcher auch schon im Matlab Skript auftritt.
  //  Die Stromspeisung funktioniert nur wenn die Range vor einschalten des Ausgangs eingestellt wird.
  //  Bei der Spannungsspeisung ist die Reihenfolge unerheblich.
  myAD5422.setVoltage(setU_I);


  // Erläuterungen zu den Einstellungen des ADS1220 siehe Library, default eingestellt sind:
  // PGA = 128, Eingänge: AIN1-AIN2, Rate = 20 SPS
  myADS1220.begin();
  myADS1220.setMUX(mux);
  myADS1220.setGain(gain);
  myADS1220.setOperatingMode(mode);
  myADS1220.setDatarate(rate);
  myADS1220.check_Register();
  myADS1220.startContinuousMeas(continuous);
  myADS1220.check_Register();

  // "Initializing SD card..."
  schreibe_Display(0, 's');

  // see if the card is present and can be initialized:
  if (!SD.begin(CS3)) {
    // "Card failed, or not present"
    schreibe_Display(1, 's');
    // don't do anything more:
    return;
  }
  // "card initialized."
  schreibe_Display(3, 's');
}

ros::NodeHandle nh;
std_msgs::Float32 force_msg;
ros::Publisher pub_force( "/externalForceSensor", &force_msg);

void loop() {

//  force_msg.data = Kraft;

  switch (myState) {
    case Einschalten:
      lcd.setCursor(0, 0); schreibe_Display(8, 'l');
      lcd.setCursor(0, 1); schreibe_Display(9, 'l');
      break;

    case Messung:
      if ((digitalRead(DRDY)) == LOW)
      {
        // Hier wird vor jedem auslesen des ADCs der SPI Mode zurückgesetzt, da dieser durch das schreiben auf die SD Karte geändert wird.
        SPI.setDataMode(SPI_MODE1);
        Spannung = myADS1220.Read_Data();


        if (enableSD == true) {
          File dataFile = SD.open("datalog.txt", FILE_WRITE);

          // if the file is available, write to it:
          if (dataFile) {
            dataFile.print(i);
            dataFile.print(" ");
            dataFile.print(millis() - start);
            dataFile.print(" ");
            dataFile.println(Spannung);
            dataFile.close();
          }
          // if the file isn't open, pop up an error:
          else {
            // "error opening datalog.txt"
            schreibe_Display(4, 's');
          }
          i++;
        }


        if (enableUART == true) {
          Serial.print(millis() - start); Serial.print(" ");
          Serial.print(j); j++;
          Serial.print(" "); Serial.println(Spannung);
        }

        // Um die Abtastrate zu erhöhen wird bei aktivierter SD Karte oder aktiviertem UART die Ausgabe auf dem LCD eingeschränkt
        if (enableSD == false && enableUART == false)
        {
          lcd.setCursor(0, 0);
          lcd.print("Vout mV: ");
          if (Spannung < 10 && Spannung > -10) lcd.print(" ");
          if (Spannung >= 0) {
            lcd.print("+");
            lcd.print(Spannung);
          }
          else {
            lcd.print(Spannung);
          }

          Kraft = ((m * Spannung) - b) * 9.81;
          Kraft = (float)((int)(Kraft * 100)) / 100;
          lcd.setCursor(0, 1);
          lcd.print("Kraft N: ");
          if (Kraft < 10 && Kraft > -10) lcd.print(" ");
          if (Kraft >= 0) {
            lcd.print("+");
            lcd.print(Kraft);
          }
          else {
            lcd.print(Kraft);
          }
        }
      }
      break;
  }

  // Das Abfragen der Knöpfe dauert etwa 4ms, daher werden diese während der Messung nicht abgefragt
  // Um eine Messung auf SD oder UART zu stoppen, muss der Reset Knopf gedrückt werden.
  if ((enableSD == false && enableUART == false) || myState == Einschalten) {
    prev_SD = millis();

    uint8_t buttons = lcd.readButtons();
    if (buttons) {
      lcd.clear();
      lcd.setCursor(0, 0);
      if (buttons & BUTTON_UP) {
        if (enableSD == true || enableUART == true) {
          lcd.setCursor(0, 0); schreibe_Display(10, 'l');
          lcd.setCursor(0, 1); schreibe_Display(11, 'l');

        }
        myState = Messung;
      }
      if (buttons & BUTTON_DOWN) {
        lcd.clear();
        myState = Einschalten;
      }
      if (buttons & BUTTON_LEFT) {
        if (myState == Einschalten) {

          enableSD = !enableSD;
          if (enableSD == true) {
            // "SD Card enabled"
            schreibe_Display(4, 'l');
            delay(1500);
            lcd.clear();
          }
          else {
            // "SD Card disabled"
            schreibe_Display(5, 'l');
            delay(1500);
            lcd.clear();
          }
        }
      }
      if (buttons & BUTTON_RIGHT) {
        if (myState == Einschalten) {

          enableUART = !enableUART;
          if (enableUART == true) {
            // "UART enabled"
            schreibe_Display(6, 'l');
            delay(1500);
            lcd.clear();
          }
          else {
            // "UART disabled"
            schreibe_Display(7, 'l');
            delay(1500);
            lcd.clear();
          }
        }
      }
      if (buttons & BUTTON_SELECT) {
        b = m * Spannung;
      }
    }
  }
}

void schreibe_Display(int number, char output) {
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[number])));
  switch (output) {
    case 's':
      Serial.println(buffer);
      break;
    case 'l':
      lcd.print(buffer);
      break;
  }
  nh.spinOnce();
}

