/*
  Current and Voltage Datalogger
  Teensy 3.5 + DFRobot SEN0291 Gravity: I2C Digital Wattmeter
  GNU General Public License
  see https://github.com/LionelBirglen/DataLogger
  
  Lionel Birglen
  2021-02-11
*/

#include <SoftwareSerial.h>

//For SEN0291 sensor
#include <Wire.h>
#include "DFRobot_INA219.h"

//For SD card recording
#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"

//For RTC Timestamp
#include <TimeLib.h>

//SD card definition, see Examples/SDFat/Quickstart
#define SD_FAT_TYPE 3
const int8_t DISABLE_CHIP_SELECT = -1;
#define SPI_SPEED SD_SCK_MHZ(4)
#if SD_FAT_TYPE == 0
SdFat sd;
File file;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile file;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

//INA219 Sensor parameters
DFRobot_INA219_IIC     ina219(&Wire, INA219_I2C_ADDRESS4);
//INA219 Calibration
float ina219Reading_mA = 117.42;
float extMeterReading_mA = 115.63;

//SD card pin
const int chipSelect = 62;

//LED pinout
const int vert = 38;
const int milieu = 37;
const int rouge = 36;

//Paddle switch line
const int record = 2;

//Recording matrix definition
const unsigned long enregistrements_court=2000;
const unsigned long enregistrements_long=20000;
unsigned long enregistrements;
float mesures[2*enregistrements_long];
const unsigned long attente_2sig_rapide=2000;
const unsigned long attente_2sig_lent=200000;
const unsigned long attente_1sig_rapide=1000;
const unsigned long attente_1sig_lent=100000;
unsigned long attente;

//Dipswitch pinout
const int dipswitch1 = 6;
const int dipswitch2 = 7;
const int dipswitch3 = 8;
const int dipswitch4 = 9;
const int dipswitch5 = 10;
const int dipswitch6 = 11;

//Dipswitch states
boolean dipswitch1_etat;    //courant ou tension    - courant et tension
boolean dipswitch2_etat;    //courant               - tension                 ignoré si courant+tension
boolean dipswitch3_etat;    //enregistrement rapide - lent                    definition Fs
boolean dipswitch4_etat;    //enregistrement long   - court
boolean dipswitch5_etat;    //filtrage actif        - pas                     zero phase passe bas 0.1Fs 4e order Butterworth
boolean dipswitch6_etat;
//These states are only read at startup at the moment, so any change needs a reset

//Filter coefficients
const float b2=0.0976;
const float b1=0.1953;
const float b0=0.0976;
const float a2=0.3333;
const float a1=-0.9428;
const float a0=1;

//Filter temporary variables to save memory space
float yfm0,yfm1,yfm2;
float yfp0,yfp1,yfp2;


void setup(void) 
{
  //Teensy RTC provides date and time
  setSyncProvider(getTeensy3Time);
    
  //Serial initialization
  Serial.begin(115200); // with Teensy the actual number does not matter
  delay(100);           // wait 100 ms
  Serial.println("RTC ok");
    
  //Sensor opening
  while(ina219.begin() != true) {
    Serial.println("INA219 begin failed");
    delay(100);
  }
  //Set no averaging, get data as fast as possible, we'll take care of filtering if needed
  ina219.setBADC(eIna219AdcBits_12, eIna219AdcSample_1);
  ina219.setSADC(eIna219AdcBits_12, eIna219AdcSample_1);
  Serial.println("INA219 ok");
    
  //Apply sensor calibration 
  ina219.linearCalibrate(ina219Reading_mA, extMeterReading_mA);
  Serial.println("Sensor calibration ok");
    
  //SD card initialization
  if (!sd.begin(chipSelect, SPI_SPEED)) {
      sd.initErrorHalt();
  }
  Serial.println("SD ok");

  //Bicolor LED initialization
  pinMode(rouge, OUTPUT);
  pinMode(vert, OUTPUT);
  pinMode(milieu, OUTPUT);
  digitalWrite(milieu, HIGH);
  Serial.println("Bicolor LED ok");

  //Dipswitch initialization by setting internal pullup resistor
  pinMode(record,INPUT_PULLUP);
  pinMode(dipswitch1,INPUT_PULLUP);
  pinMode(dipswitch2,INPUT_PULLUP);
  pinMode(dipswitch3,INPUT_PULLUP);
  pinMode(dipswitch4,INPUT_PULLUP);
  pinMode(dipswitch5,INPUT_PULLUP);
  pinMode(dipswitch6,INPUT_PULLUP);
  delay(100);
  Serial.println("Dipswitch ok");

  //Reading of the dipswitch states
  dipswitch1_etat=digitalRead(dipswitch1);
  dipswitch2_etat=digitalRead(dipswitch2);
  dipswitch3_etat=digitalRead(dipswitch3);
  dipswitch4_etat=digitalRead(dipswitch4);
  dipswitch5_etat=digitalRead(dipswitch5);
  dipswitch6_etat=digitalRead(dipswitch6);

  //Setting up the number of measurements to do by checking dipswitch4
  if (dipswitch4_etat==LOW) {enregistrements=enregistrements_long;}
  else {enregistrements=enregistrements_court;}

  //Synchronization of the sampling based on number of signals to acquire and desired speed
  if (dipswitch1_etat==HIGH) {
    //If only one signal sampling freq are 1000Hz (fast) or 10Hz (slow) so periods are 1000us and 100000us
    if (dipswitch3_etat==HIGH) {
      attente=attente_1sig_rapide;
    }
    else {
      attente=attente_1sig_lent;
    }
  }
  else {
    //If two signals sampling freq are 500Hz (fast) or 5Hz (slow) so periods are 2000us and 200000us
    if (dipswitch3_etat==HIGH) {
      attente=attente_2sig_rapide;
    }
    else {
      attente=attente_2sig_lent;
    }
  }  
}

void loop(void)
{
  unsigned int i;
  unsigned long depart,fin,depart2,timepast;
  char filename[64];
  char date[32];
  
  //Checking the state of the paddle switch: record if leaning toward USB port
  if (digitalRead(record)==HIGH){
    //Start the recording
    
    //Opening the file on SD card
    sprintf(date,"%4d-%2d-%2d_%2d-%2d-%2d",year(),month(),day(),hour(),minute(),second());
    strcpy(filename,"DataRecord_");
    strcat(filename,date);
    strcat(filename,".csv");

    //FsDateTime::setCallback(getTeensy3Time);
    //ou
    SdFile::dateTimeCallback(getTeensy3Time);

    //If opening the file failed, stop there
    if (!file.open(filename, O_RDWR | O_CREAT)) sd.errorHalt("open failed");
    
    //Writing the header of the CSV file
    file.print("Date of recording: ");
    file.println(date);

    //Configuration vector: concatenation of dipswitch states
    Serial.print("Configuration: ");
    Serial.print(dipswitch1_etat);Serial.print(" - ");
    Serial.print(dipswitch2_etat);Serial.print(" - ");
    Serial.print(dipswitch3_etat);Serial.print(" - ");
    Serial.print(dipswitch4_etat);Serial.print(" - ");
    Serial.print(dipswitch5_etat);Serial.print(" - ");
    Serial.print(dipswitch6_etat);Serial.println();

    //Explicit interpretation of this configuration vector
    //Output on the serial port
    if (dipswitch1_etat==LOW) {Serial.println("Current and voltage are recorded");}
    else {
      if (dipswitch2_etat==LOW) {Serial.println("Voltage is recorded");}
      else {Serial.println("Current is recorded");}
      }
    if (dipswitch3_etat==LOW) {
      Serial.print("Slow sampling: ");
      if (dipswitch1_etat==LOW) {Serial.print(1000000/attente_2sig_lent);Serial.println(" Hz");}
      else {Serial.print(1000000/attente_1sig_lent);Serial.println(" Hz");}
      }
    else {
      Serial.print("Fast sampling: ");
      if (dipswitch1_etat==LOW) {Serial.print(1000000/attente_2sig_rapide);Serial.println(" Hz");}
      else {Serial.print(1000000/attente_1sig_rapide);Serial.println(" Hz");}
      }
    
    if (dipswitch4_etat==LOW) {Serial.print("Long ");}
    else {Serial.print("Short ");}
    Serial.print("recording: ");
    Serial.print(enregistrements);
    Serial.println(" measurements");
    if (dipswitch5_etat==LOW) {Serial.println("No filtering");}
    else {Serial.println("Active filtering (4th order Butterworth low-pass zero-phase 0.1*Fs)");}

    //On the SD card
    if (dipswitch1_etat==LOW) {file.println("Current and voltage are recorded");}
    else {
      if (dipswitch2_etat==LOW) {file.println("Voltage is recorded");}
      else {file.println("Current is recorded");}
      }
    if (dipswitch3_etat==LOW) {
      file.print("Slow sampling: ");
      if (dipswitch1_etat==LOW) {file.print(1000000/attente_2sig_lent);file.println(" Hz");}
      else {file.print(1000000/attente_1sig_lent);file.println(" Hz");}
      }
    else {
      file.print("Fast sampling: ");
      if (dipswitch1_etat==LOW) {file.print(1000000/attente_2sig_rapide);file.println(" Hz");}
      else {file.print(1000000/attente_1sig_rapide);file.println(" Hz");}
      }
    if (dipswitch4_etat==LOW) {file.print("Long ");}
    else {file.print("Short ");}
    file.print("recording: ");
    file.print(enregistrements);
    file.println(" measurements");
    if (dipswitch5_etat==LOW) {file.println("No filtering");}
    else {file.println("Active filtering (4th order Butterworth low-pass zero-phase 0.1*Fs)");}

    //Header for the data
    if (dipswitch1_etat==LOW) {
      file.println("Current (mA), Voltage (V)");
    }
    else {
      if (dipswitch2_etat==LOW){file.println("Voltage (V)");}
      if (dipswitch2_etat==HIGH){file.println("Current (mA)");}
    }

    //Bicolor LED set to green
    digitalWrite(rouge, HIGH);   // turn off red LED
    digitalWrite(vert, LOW);     // turn on green LED

    //Main loop
    if (dipswitch1_etat==LOW) {
      //Current and voltage are recorded

      //Saving starting time
      depart=millis();
      
      //Record loop
      for (i=0;i<2*enregistrements;i=i+2) {
        //Time at start of the measure point
        timepast=micros();  

        //Reading the sensor
        mesures[i]=ina219.getCurrent_mA();
        mesures[i+1]=ina219.getBusVoltage_V();

        //Synchronization: 
        //if not yet the time for the next sampling, we wait
        while ((micros()-timepast)<attente){}
      }
    }
    else {
      if (dipswitch2_etat==LOW) {
        //Only voltage is recorded

        //Saving starting time
        depart=millis();
        
        //Record loop
        for (i=0;i<enregistrements;i=i+1) {
          //Time at start of the measure point
          timepast=micros();  

          //Reading the sensor
          mesures[i]=ina219.getBusVoltage_V();

          //Synchronization: 
          //if not yet the time for the next sampling, we wait
          while ((micros()-timepast)<attente){}
        }
      }
      else {
      //Only current is recorded

        //Saving starting time
        depart=millis();
        
        //Record loop
        for (i=0;i<enregistrements;i=i+1) {
          //Time at start of the measure point
          timepast=micros();  

          //Reading the sensor
          mesures[i]=ina219.getCurrent_mA();

          //Synchronization: 
          //if not yet the time for the next sampling, we wait
          while ((micros()-timepast)<attente){}
        }
      }
    }
    
    //Saving the end time
    fin=millis();

    //Flashing the green LED for 2s
    digitalWrite(rouge, HIGH);   //turn off red LED
    depart2=millis();
    while ((millis()-depart2)<2000) {
      delay(50);
      digitalWrite(vert, HIGH);
      delay(50);
      digitalWrite(vert, LOW);
    }

    //SD card writing and serial output
    //First, set red LED to solid on
    digitalWrite(rouge, LOW);   //turn on red LED
    digitalWrite(vert, HIGH);   //turn off green LED

    //Adding total recording time to file to doublecheck everything is ok
    file.print("Duree totale de l'enregistrement : ");
    file.print(fin-depart);
    file.println(" ms");

    Serial.print("Duree totale de l'enregistrement : ");
    Serial.print(fin-depart);
    Serial.println(" ms");

    //For filter debugging purposes: outputting raw data
    //file.println("DONNEES BRUTES :");
    //for (i=0;i<2*enregistrements;i=i+2) {    
    //file.print(mesures[i]);
    //file.print(',');
    //file.print(mesures[i+1]);
    //file.println();
    //        }
    //file.println("FIN DONNEES BRUTES");

    //Optional filtering
    if (dipswitch5_etat==HIGH) {
      //If both current and voltage need to be filtered
      if (dipswitch1_etat==LOW) {
        //Filtering of the first column (current)
        //Initialization of the filter
        yfm1=0;
        yfm2=0;
        yfp1=0;
        yfp2=0;

        //Forward filtering
        for (i=4;i<2*enregistrements;i=i+2) {
          yfm0=b2*mesures[i-4]+b1*mesures[i-2]+b0*mesures[i]-a2*yfm2-a1*yfm1;
          mesures[i-4]=yfm2;
          yfm2=yfm1;
          yfm1=yfm0; 
        }

        //Backward filtering
        for (i=2*enregistrements-4;i>0;i=i-2) {
          yfp0=b2*mesures[i+4]+b1*mesures[i+2]+b0*mesures[i]-a2*yfp2-a1*yfp1;
          mesures[i+4]=yfp2;
          yfp2=yfp1;
          yfp1=yfp0; 
        }

        //Filtering of the second column (voltage)
        //Initialization of the filter
        yfm1=0;
        yfm2=0;
        yfp1=0;
        yfp2=0;

        //Forward filtering
        for (i=4;i<2*enregistrements;i=i+2) {
          yfm0=b2*mesures[i-3]+b1*mesures[i-1]+b0*mesures[i+1]-a2*yfm2-a1*yfm1;
          mesures[i-3]=yfm2;
          yfm2=yfm1;
          yfm1=yfm0; 
        }

        //Backward filtering
        for (i=2*enregistrements-4;i>0;i=i-2) {
          yfp0=b2*mesures[i+5]+b1*mesures[i+3]+b0*mesures[i+1]-a2*yfp2-a1*yfp1;
          mesures[i+5]=yfp2;
          yfp2=yfp1;
          yfp1=yfp0; 
        }
        
      }
      //If there is a single column only, it's simpler
      else {
        //Initialization 
        yfm1=0;
        yfm2=0;
        yfp1=0;
        yfp2=0;

        //Forward filtering
        for (i=2;i<enregistrements;i++) {
          yfm0=b2*mesures[i-2]+b1*mesures[i-1]+b0*mesures[i]-a2*yfm2-a1*yfm1;
          mesures[i-2]=yfm2;
          yfm2=yfm1;
          yfm1=yfm0; 
        }

        //Backward filtering
        for (i=enregistrements-2;i>0;i--) {
          yfp0=b2*mesures[i+2]+b1*mesures[i+1]+b0*mesures[i]-a2*yfp2-a1*yfp1;
          mesures[i+2]=yfp2;
          yfp2=yfp1;
          yfp1=yfp0; 
        }
      }      
    }

    //Writing of the recordings on SD card and serial port
    if (dipswitch1_etat==LOW) {
      //Two signals
      
      //Outputing the data
      for (i=0;i<2*enregistrements;i=i+2) {    
        //SD Card
        file.print(mesures[i]);
        file.print(',');
        file.print(mesures[i+1]);
        file.println();

        //Serial port
        Serial.print(i, 1);
        Serial.print(" : ");
        Serial.print(mesures[i], 2);
        Serial.print(" | ");
        Serial.println(mesures[i+1], 3);
      }
    }
    else {
      if (dipswitch2_etat==LOW) {
        //Only voltage
        
        //Outputing the data
        for (i=0;i<enregistrements;i=i+1) {    
        
          //SD Card
          file.print(mesures[i]);
          file.println();

          //Serial port
          Serial.print(i, 1);
          Serial.print(" : ");
          Serial.println(mesures[i], 2);
        }
      }
      else {
      //Current only
        //Outputing the data
        for (i=0;i<enregistrements;i=i+1) {    
        
          //SD card
          file.print(mesures[i]);
          file.println();

          //Serial port
          Serial.print(i, 1);
          Serial.print(" : ");
          Serial.println(mesures[i], 2);
        }
      }
    }

    //Closing the SD card file
    file.close();  

    //Blinking red light for 2s: recording is about to resume
    digitalWrite(vert, HIGH);   // turn off green LED
    depart2=millis();
    while ((millis()-depart2)<2000) {
      delay(50);
      digitalWrite(rouge, HIGH);
      delay(50);
      digitalWrite(rouge, LOW);
    }
  }
  else
  {
    //If no recording required at the moment, alternate blinking of green and red LED to show 
    //the logger is ready and waiting
    digitalWrite(vert, HIGH);
    digitalWrite(rouge, LOW);
    delay(50);
    digitalWrite(vert, LOW);
    digitalWrite(rouge, HIGH);
    delay(200);
  }
}

//Provides RTC from Teensy
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}
