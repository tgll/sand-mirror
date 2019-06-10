
// Include Libraries
#include "Arduino.h"
#include "Flex.h"
#include "MPU6050.h"
#include "Wire.h"
#include "I2Cdev.h"
#include <SoftwareSerial.h>


// Pin Definitions
#define WIFI_PIN_RX  11
#define WIFI_PIN_TX 10
#define FLEX5V_PIN_SIG	A0
#define FLEX5V_PIN_SIG_1  A1
#define FLEX5V_PIN_SIG_2  A2
#define TIMEOUT 5000 // mS

// Global variables and defines
String ssid   = "ONEKARO"; // Enter your Wi-Fi name 
String PASSWORD = "drobocik" ; // Enter your Wi-Fi password
int16_t mpu6050Ax, mpu6050Ay, mpu6050Az;
int16_t mpu6050Gx, mpu6050Gy, mpu6050Gz;

// object initialization
//ESP8266 wifi(WIFI_PIN_RX,WIFI_PIN_TX);
//SoftwareSerial mySerial(WIFI_PIN_RX,WIFI_PIN_TX); // RX, TX

Flex flex5v_1(FLEX5V_PIN_SIG);
Flex flex5v_2(FLEX5V_PIN_SIG_1);
Flex flex5v_3(FLEX5V_PIN_SIG_2);
MPU6050 mpu6050;


// values for moving average
const double ol = 0.75, ne = 0.25;

// evergy values flexes
int dE[] = {0,0,0}, dEma[] = {0,0,0}, Eold[] = {0,0,0}, Enew[] = {0,0,0};
int dE_f = 0; // around 600; 0 - 1500
// energy values Acc & Gyro
int dEg[] = {0,0,0}, dEgma[] = {0,0,0}, Egold[] = {0,0,0}, Egnew[] = {0,0,0};
int dEa[] = {0,0,0}, dEama[] = {0,0,0}, Eaold[] = {0,0,0}, Eanew[] = {0,0,0};
int dE_a = 0; // 
int dE_g = 0; // 0 to 1000
// energy compared
double gyrToFlex = 0.3;
int dE_full=0;
Flex *flexes[] = {&flex5v_1, &flex5v_2, &flex5v_3};
int counter = 0;
int count = 4;


// Setup the essentials for your circuit to work. It runs first every time your circuit is powered with electricity.
void setup() 
{
    // Setup Serial which is useful for debugging
    // Use the Serial Monitor to view printed messages
    Serial.begin(9600);
    //mySerial.begin(9600);
    while (!Serial) ; // wait for serial port to connect. Needed for native USB
    Serial.println("start");

    mpu6050.initialize();
     /*SendCommand("AT+RST", "Ready");
     delay(5000);
     Serial.println("after delay");
     SendCommand("AT+CWMODE=1","OK");
     String command = "AT+CWJAP= \"" + ssid + "\",\"" + PASSWORD + "\"";
     SendCommand("AT+CIFSR", "OK");
     SendCommand(command, "OK");
     Serial.println("after wifi");
     SendCommand("AT+CIPMUX=1","OK");
     SendCommand("AT+CIPSERVER=1,80","OK");
     Serial.println("after setup");*/
}

// Main logic of your circuit. It defines the interaction between the components you selected. After setup, it runs over and over again, in an eternal loop.
void loop() 
{
    // reading the flexes value and adding up the dEs
    for(int i = 0; i<3; i++){
       Enew[i] = flexes[i]->read();
       dE[i] += abs(Enew[i]-Eold[i]);
       Eold[i] = Enew[i];
    }

    // read MPU values
    mpu6050.getMotion6(&mpu6050Ax, &mpu6050Ay, &mpu6050Az, &mpu6050Gx, &mpu6050Gy, &mpu6050Gz);   //read accelerometer and gyroscope raw data in three axes
    //double mpu6050Temp = ((double)mpu6050.getTemperature() + 12412.0) / 340.0;
    int Eanew[] = {mpu6050Ax/100, mpu6050Ay/100, mpu6050Az/100};
    int Egnew[] = {mpu6050Gx/100, mpu6050Gy/100, mpu6050Gz/100};
    
    for(int i = 0; i<3; i++){
      // dividing by 100 not to overload integers
       dEa[i] += abs(Eanew[i]-Eaold[i]);
       Eaold[i] = Eanew[i];
       dEg[i] += abs(Egnew[i]-Egold[i]);
       Egold[i] = Egnew[i];
    }
   
    counter += 1;
    if(counter >= count){
      counter = 0;
      dE_f = 0;
      dE_g = 0;
      dE_a = 0;

      // count moving average and add to general energies
      for(int i = 0; i<3; i++){
        //flex 
        dEma[i] = ol*dEma[i] + ne*dE[i];
        dE_f += dEma[i];
        dE[i] = 0;

        //acc
        dEama[i] = ol*dEama[i] + ne*dEa[i];
        dE_a += dEama[i];
        dEa[i] = 0;

        //gyro
        dEgma[i] = ol*dEgma[i] + ne*dEg[i];
        dE_g += dEgma[i];
        dEg[i] = 0;
      }
      dE_g *= gyrToFlex;

      dE_full = dE_g + dE_a + dE_f; 
    }
    
    //Serial.print(F("flex5vVal: ")); 
    /* // showing all flexes
    Serial.print(Enew[0]);Serial.print("\t");
    Serial.print(Enew[1]);Serial.print("\t");
    Serial.print(Enew[2]);Serial.print("\t");
    Serial.print(dEma[0]);Serial.print("\t");
    Serial.print(dEma[1]);Serial.print("\t");
    Serial.print(dEma[2]);Serial.print("\t");
    
    //Serial.print(F("flexdE: ")); 
    //Serial.print(dE);Serial.print("\t");
    //Serial.print(F("flexdEma: ")); */
    Serial.print(dE_f);Serial.print("\t");

     // showing all gyros
    /*Serial.print(Egnew[0]);Serial.print("\t");
    Serial.print(Egnew[1]);Serial.print("\t");
    Serial.print(Egnew[2]);Serial.print("\t");
    Serial.print(dEgma[0]);Serial.print("\t");
    Serial.print(dEgma[1]);Serial.print("\t");
    Serial.print(dEgma[2]);Serial.print("\t");*/
    Serial.print(dE_g);Serial.print("\t");

    
     // showing all acc
    /*Serial.print(Eanew[0]);Serial.print("\t");
    Serial.print(Eanew[1]);Serial.print("\t");
    Serial.print(Eanew[2]);Serial.print("\t");
    Serial.print(dEama[0]);Serial.print("\t");
    Serial.print(dEama[1]);Serial.print("\t");
    Serial.print(dEama[2]);Serial.print("\t");*/
    Serial.print(dE_a);Serial.print("\t");
    
    Serial.println(dE_full);

    
    //mySerial.println("AT+CIPSEND=0,5");
    //mySerial.println(dE_full);
    delay(200); 
    //SendCommand("AT+CIPCLOSE=0","OK");

    //delay(200);
}

/*
boolean SendCommand(String cmd, String ack){
  mySerial.println(cmd); // Send "AT+" command to module
  Serial.println(cmd); // Send "AT+" command to module
  if (!echoFind(ack)) // timed out waiting for ack string
    return true; // ack blank or ack found
}

boolean echoFind(String keyword){
 byte current_char = 0;
 byte keyword_length = keyword.length();
 long deadline = millis() + TIMEOUT;
 while(millis() < deadline){
  if (mySerial.available()){
    char ch = mySerial.read();
    Serial.write(ch);
    if (ch == keyword[current_char])
      if (++current_char == keyword_length){
       Serial.println();
       return true;
    }
   }
  }
  Serial.println("timeout");
 return false; // Timed out
}
*/




