 /*
CVM Board Software
The purpose of this code is to read voltage of each cell fuel cell stack and send a 
signal if the voltage is under a certain threshold or if something is disconnected. 

For Further Documentation and the Board Design see the Github folder CVM-BOARD
https://github.com/UAlberta-EcoCar/CVM-BOARD

 created  Winter 2017 
 modified Fall 2019
 University of Alberta EcoCar Team
 Fuel Cell Subteam
 by Elizabeth Gierl
 
Things to do:
- Test watchdog 
- set signal 2 as high unless disconnected?
- Test new board

 */                                                                                                                                                       
                                                                                                    
                                               
                                                    //   Including Libraries   //
#include <Wire.h>  //Include I^2C Library (Wire Library)
#include "Linduino.h" //librays to make I2C work 
#include "LT_I2C.h"
#include  "LTC2309.h"
#include <avr/wdt.h>

                                                   //   Set Global Variables  //
 
int sigOne = A3;                // sig 1 connected to FCC ethernet pin 5
int sigTwo = A2;                // sig 1 connected to FCC ethernet pin 6



//Set can of cells here: 
int number_of_cells=48;
float CellVoltages[48];
//Code for setting what channel are broken if broken set that channel to zero number to zero..see map in excell
int WorkingChannels[49]={1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1 , 1, 1, 1, 1, 1, 1, 1,1 , 1, 1, 1, 1, 1, 1, 1,1,1, 1, 1, 1, 1, 1, 1,1, 1, 1, 1, 1, 1, 1,1};

// holds average voltage value
int AverageAdcVoltage=0;

// i2c variables
uint8_t ack=0;
uint16_t adc_code= 0;
uint8_t adc_command=0;
float adc_voltage= 0;

//conversion factor from voltage dividers
float conversion[48]= {1,1,1,1,1.221,1.47,1.715,2,2.2121,2.4684,2.6949,2.9569,3.2075,3.4271,3.6737, 3.9411,4.1645,4.4013,4.6496,4.9215,5.1667,5.4248,5.6512,5.8780,6.2356,6.4945,6.6179,6.917,7.1728,7.3291,7.6667,7.9930,8.1429,8.5188,8.6923,8.8740,9.0645,9.333,9.6957,9.8496,10.0909,10.3458,10.5238,10.8035,11,11.4932,0,0};

uint16_t Channels[8] = {0x80,0xC0,0x90,0xD0,0xA0,0xE0,0xB0,0xF0}; 
uint16_t chipAddress[6] = {0x08,0x09,0x0A,0x0B,0x18,0x19}; 
float arrayAdcVoltage[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//time delay varaible 
uint8_t chill_time= 100;


                                                           //  SETUP  //
void setup() {
Serial.begin(9600); 
Wire.begin(); 
pinMode(sigOne, OUTPUT); // sets the digital pin as output
pinMode(sigTwo, OUTPUT); // sets the digital pin as output
digitalWrite(sigOne, LOW);  // turn off signal
digitalWrite(sigTwo, LOW);  // turn off signal
introduction(); //call function introduction 
watchdogSetup();
}


                                                           //  MAIN  //
void loop() 
{ 
collectData(); //call function collectData
calculateCellVoltage();//Calculate voltage of each cell relative to the one before it
writeData();  //call function writeData
wdt_reset();
}




                                                           //FUNCTIONS//
void introduction(void) {
   Serial.print(".");
   delay (1500);
   Serial.print("..");
   delay (1500);
   Serial.print("..");
   delay (1000);
   Serial.print("..");
   delay (800);
   Serial.print(".");
   delay (500);
   Serial.print(".");
   delay (300);
   Serial.print(".");
   delay (300);
   Serial.print("P");
   delay (100);
   Serial.print("0");
   delay (100);
   Serial.print("W");
   delay (50);
   Serial.print("!");
   delay (50);
   Serial.print("!");
   delay (1500);
   Serial.print("\n");
   Serial.print("Welcome!  Eco Car CVM board here , ready to calc. cell voltages B^)!"); 
   Serial.print("\n");
   delay (1500);
}

void watchdogSetup(void)
  {
  cli(); // disable all interrupts
  wdt_reset();
              /*
              WDTCSR configuration:
              WDIE = 1: Interrupt Enable
              WDE = 1 :Reset Enable
              WDP3 = 0, WDP2 = 1,WDP1 = 1,WDP0 = 1:For 2000ms Time-out
              */
  // Configuration mode:
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // Watchdog settings:
  WDTCSR = (1<<WDIE) | (1<<WDE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
  sei();
  }

void collectData(void) 
{
  int CellN = 0;
  for (int chip=0; chip< 6; chip++) 
  {
    for (int Channel_number = 0; Channel_number < 8; Channel_number++) 
    {
        AverageAdcVoltage = 0;
        for (int N = 0; N<20; N ++)
        {
            //Read Channel # in Single-Ended Unipolar mode
            adc_command = Channels[ Channel_number] | LTC2309_UNIPOLAR_MODE;           // Set to read channel #
            ack |= LTC2309_read(chipAddress[chip], adc_command, &adc_code);   // Throws out last reading
            ack |= LTC2309_read(chipAddress[chip], adc_command, &adc_code);   // Obtains the current reading and stores to adc_code variable
    
            // Convert adc_code to voltage
            adc_voltage = LTC2309_code_to_voltage(adc_code, 2.5f, LTC2309_UNIPOLAR_MODE); //accuracy changes as you change internal ref... maybe try and change it if readings are off
            adc_voltage = (adc_voltage* 1.5 ); //voltage correction..change in future adc_voltage = (adc_voltage* correct(N) ) correct having the designated correction factors
     
            //calculating the average of the 20 voltages read on a channel
            AverageAdcVoltage = (AverageAdcVoltage + adc_voltage);     
          }

          CellVoltages[CellN] = ( ((AverageAdcVoltage)/20) * (1.6388*conversion[CellN])  );
          CellN =(CellN + 1);
        }
      }
    }


void calculateCellVoltage(void)
{
  for(int CellNumb=45;CellNumb>=0;CellNumb--)
       {
        if(WorkingChannels[CellNumb]==1 & WorkingChannels[(CellNumb-1)]==1){
        CellVoltages[CellNumb] = CellVoltages[CellNumb] - CellVoltages[CellNumb-1];
             //send signal to show that pin is disconnected 
             if (CellVoltages[CellNumb] <0.1)
                 {
                    digitalWrite(sigTwo, HIGH);    // signals that a cvm probe is disconnected 
                     }
                     
             //purge signal             
             if ((CellVoltages[CellNumb] > 0.1) && (CellVoltages[CellNumb] <0.5)) //purge signal
                   {
                     digitalWrite(sigOne, HIGH);   // sends signal to FCC 
                     delay(500); // waits half a second
                     digitalWrite(sigOne, LOW);   // sends signal to FCC 
                     delay(500);
                   }
        }
        else {
          CellVoltages[CellNumb] = -21;
        }
        }
              
}

void writeData(void) {
    for (int out = 0; out<48; out ++){
      if(0<=out<8){
             Serial.print ("0x08: ");
      }
      else if(8<=out<16){
             Serial.print("0x09: ");
      }
      else if(16<=out<24){
             Serial.print("0x0A: ");
      }
      else if(24<=out<32){
             Serial.print("0x0B: ");
      }
      else if(32<=out<40){
             Serial.print("0x18: ");
      }
      else if(40<=out<48){
             Serial.print("0x19: ");
      }
      Serial.print (out);
      Serial.print (":");
      delay (chill_time);
      Serial.print(CellVoltages[out]);
      Serial.print( ',' ); 
      delay (chill_time);  
      }
    }


  ISR(WDT_vect) // Watchdog timer interrupt.
{
  digitalWrite(sigTwo, HIGH);    // signals that a cvm probe is disconnected 
}


 
