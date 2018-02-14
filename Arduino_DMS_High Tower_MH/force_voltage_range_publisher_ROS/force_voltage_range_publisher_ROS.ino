//ROS Publischer für Kraft- und US-Daten, Laura Bielenberg, TU Darmstadt 2017

//#include <Wire.h>
#include <AD5422.h>
#include <ADS1220.h>
#include <SPI.h>

//To publish force- and US-sensor data to ROS
#include <ros.h>
//forcce
#include <std_msgs/Float32.h>
// US
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#define USE_USBCON

AD5422 myAD5422;
ADS1220 myADS1220;

//set up ROS node
ros::NodeHandle  nh;

std_msgs::Float32 force_msg;
std_msgs::Float32 vlt_msg;
sensor_msgs::Range range_msg;

ros::Publisher pub_force( "/sensor/externalForce", &force_msg); // force
ros::Publisher pub_vlt( "/sensor/externalVoltage", &vlt_msg); // voltage
ros::Publisher pub_range( "/ultrasound", &range_msg); // distance

unsigned long prev; // Zeit Variable
unsigned long prev_SD; // Zeit Variable
short start;
float Spannung = 0;
float Kraft;
int j=0;
float sumVolt = 0; //für kalibrierung
float offsetVolt; //für kalibrierung
int numCalibReadings = 10000; //für kalibrierung


// Ermittlung Offset:
// ===================================================================================================================
void averageForceReadings(){

    for(int x=0; x<numCalibReadings;x++){
      Spannung = myADS1220.Read_Data();
      sumVolt += Spannung;

    }
    
  offsetVolt = sumVolt/numCalibReadings;

}



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
const float m = 0.9152; 
// Offset b: Dieser Wert mit betätigen der Tara Taste (Select) verändert.
//float b = 2.1;
int i=0; //counter for offset calculation
float sensorValue = 0; //Buffer for Sensor Values

// US
int pingPin = 7;
int inPin = 8;
int ledPin = 13;
long range_time;
char frameid[] = "/tool_link_ee"; // set the corresponding Frame 

//----------------------------------------
//----------------------------------------

long microsecondsToCentimeters(long microseconds)
{
// The speed of sound is 340 m/s or 29 microseconds per centimeter.
// The ping travels out and back, so to find the distance of the
// object we take half of the distance travelled.
return microseconds / 29.1 / 2;
}

float getRange()
{
    
    // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration, cm;
  
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);
  
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(inPin, INPUT);
  duration = pulseIn(inPin, HIGH);
  
  // convert the time into a distance
  return microsecondsToCentimeters(duration);
}

void setup() {
  start = millis();
  Serial.begin(57600);
  digitalWrite(CS1, HIGH);
  digitalWrite(CS2, HIGH);

  nh.initNode();
  nh.advertise(pub_force);
  nh.advertise(pub_vlt);
  nh.advertise(pub_range);
  
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.0;
  range_msg.max_range = 6.47;
 
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

  //calculate offset
  averageForceReadings();

  
}

//----------------------------------------
//----------------------------------------

void loop() {

//      if ( millis() >= range_time ){
//          int r =0;  
//          range_msg.range = getRange() / 100;
//          range_msg.header.stamp = nh.now();
//          pub_range.publish(&range_msg);
//          range_time =  millis() + 50;
//        }  
        
      if ((digitalRead(DRDY)) == LOW)
      {
        // Hier wird vor jedem auslesen des ADCs der SPI Mode zurückgesetzt, da dieser durch das schreiben auf die SD Karte geändert wird.
        SPI.setDataMode(SPI_MODE1);
        Spannung = myADS1220.Read_Data();


      
        //  Serial.print(millis() - start); Serial.print(" ");
        //  Serial.print(j); j++;
        //  Serial.print(" "); Serial.println(Spannung);
      

          Kraft = m*(Spannung - offsetVolt - 0.89) * 9.81;
          Kraft = (float)((int)(Kraft * 100)) / 100;


          force_msg.data = Kraft;
          pub_force.publish(&force_msg);

          vlt_msg.data = Spannung - offsetVolt;
          pub_vlt.publish(&vlt_msg);
      }
       nh.spinOnce();
}


  



