// Praktikum Messtechnik, geschrieben von Eike Christmann, TU Darmstadt 2016

#include <Wire.h>
#include <AD5422.h>
#include <ADS1220.h>
#include <SPI.h>
#include <ros.h>
#include <std_msgs/Float32.h>

AD5422 myAD5422;
ADS1220 myADS1220;

//set up ROS node
ros::NodeHandle  nh;

std_msgs::Float32 force_msg;
std_msgs::Float32 vlt_msg;

ros::Publisher pub_force( "/sensor/externalForce", &force_msg); //Kraft
ros::Publisher pub_vlt( "/sensor/externalVoltage", &vlt_msg); //Spannung

unsigned long prev; // Zeit Variable
unsigned long prev_SD; // Zeit Variable
short start;
float Spannung;
float Kraft;
int j=0;

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
const float m = 2; 
// Offset b: Dieser Wert mit betätigen der Tara Taste (Select) verändert.
float b = 2.1;
int i=0; //counter for offset calculation
float sensorValue = 0; //Buffer for Sensor Values

void setup() {
  start = millis();
  Serial.begin(57600);
  digitalWrite(CS1, HIGH);
  digitalWrite(CS2, HIGH);

  nh.initNode();
  nh.advertise(pub_force);
  nh.advertise(pub_vlt);
 
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
  
}
void loop() {

 
      if ((digitalRead(DRDY)) == LOW)
      {
        // Hier wird vor jedem auslesen des ADCs der SPI Mode zurückgesetzt, da dieser durch das schreiben auf die SD Karte geändert wird.
        SPI.setDataMode(SPI_MODE1);
        Spannung = myADS1220.Read_Data();


      
          Serial.print(millis() - start); Serial.print(" ");
          Serial.print(j); j++;
          Serial.print(" "); Serial.println(Spannung);
      

          Kraft = m*(Spannung - b) * 9.81;
          Kraft = (float)((int)(Kraft * 100)) / 100;


          force_msg.data = Kraft;
          pub_force.publish(&force_msg);

          vlt_msg.data = Spannung;
          pub_vlt.publish(&vlt_msg);
          
          nh.spinOnce();
          
  }
}


  



