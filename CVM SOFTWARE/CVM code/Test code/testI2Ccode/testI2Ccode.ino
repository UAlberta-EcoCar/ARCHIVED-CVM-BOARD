
#include <Wire.h>  //Include I^2C Library (Wire Library)
#include "Linduino.h" //librays to make I2C work 
#include "LT_I2C.h"
#include  "LTC2309.h"

int sigOne = A3;
int sigTwo = A2;         

//Variable for storing voltage readings
float CellVoltages[48];



uint8_t ack = 0;
uint16_t adc_code = 0;
uint8_t adc_command =0;
float adc_voltage= 0;
float AverageAdcVoltage = 0;
uint8_t chill_time= 50;


int Channels[8] = {0x80,0xC0,0x90,0xD0,0xA0,0xE0,0xB0,0xF0};
uint16_t chipAddress[6] = {0x08,0x09,0x0A,0x0B,0x18,0x19}; 

//conversion factor from voltage dividers
float conversion[48]= {1,1,1,1,1.221,1.47,1.715,2,2.2121,2.4684,2.6949,2.9569,3.2075,3.4271,3.6737, 3.9411,4.1645,4.4013,4.6496,4.9215,5.1667,5.4248,5.6512,5.8780,6.2356,6.4945,6.6179,6.917,7.1728,7.3291,7.6667,7.9930,8.1429,8.5188,8.6923,8.8740,9.0645,9.333,9.6957,9.8496,10.0909,10.3458,10.5238,10.8035,11,11.4932,0,0};



void setup() {
Serial.begin(9600); 

Wire.begin(); 
pinMode(sigOne, OUTPUT);  // sets the digital pin as output
pinMode(sigTwo, OUTPUT); // sets the digital pin as output

digitalWrite(sigOne, LOW);
digitalWrite(sigTwo, LOW);

introduction(); //call function introduction
}
 
void loop() 
{

collectData(); //call function collectData


  //Calculate voltage of each cell relative to the one before it

    
       for(int CellNumb=45;CellNumb>37;CellNumb--)
       {
       
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
                     delay(500); // waits
                     digitalWrite(sigOne, LOW);   // sends signal to FCC 
                     delay(100);

                   }
         }

          
       for(int CellNumb=35 ;CellNumb>19;CellNumb--)
       {
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
                     delay(100);

                   }
         }


        for(int CellNumb= 17 ;CellNumb> 14;CellNumb--)
             {
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
                           delay(100);
      
                         }
               }

       for(int CellNumb=9 ;CellNumb>3;CellNumb--)
       {
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
                     delay(100);

                   }
       }

          for(int CellNumb=1 ;CellNumb>0;CellNumb--)
       {
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
                     delay(100);

                   }
       }


     
       {
      
             if (CellVoltages[0] <0.1)
                 {
                  
                    digitalWrite(sigTwo, HIGH);    // signals that a cvm probe is disconnected 
                     }
                     
             //purge signal             
             if ((CellVoltages[0] > 0.1) && (CellVoltages[0] <0.5)) //purge signal
                   {
                     digitalWrite(sigOne, HIGH);   // sends signal to FCC 
                     delay(500); // waits half a second
                     digitalWrite(sigOne, LOW);   // sends signal to FCC 
                     delay(100);

                   }

        
  
//writeData();  //call function writeData

}
}
                  

void collectData(void) {

  int CellN = 0;
  
    for (int chip=0; chip< 6; chip++) 
  {
 
  for (int Channel_number = 0; Channel_number < 8; Channel_number++) 
    {
        AverageAdcVoltage = 0;
        
        for (int N = 0; N<20; N ++)
        {
         
            //Read Channel # in Single-Ended Unipolar mode
            adc_command = Channels[Channel_number] | LTC2309_UNIPOLAR_MODE;        // Set to read channel #
       
           ack |= LTC2309_read(chipAddress[chip], adc_command, &adc_code);   // Throws out last reading
           ack |= LTC2309_read(chipAddress[chip], adc_command, &adc_code);   // Obtains the current reading and stores to adc_code variable
    
 
            // Convert adc_code to voltage
            adc_voltage = LTC2309_code_to_voltage(adc_code, 2.5f, LTC2309_UNIPOLAR_MODE); //accuracy changes as you change internal ref... maybe try and change it if readings are off
           
            

            //calculating the average of the 20 voltages read on a channel
            AverageAdcVoltage = (AverageAdcVoltage + adc_voltage);
          
           
          }



          CellVoltages[CellN] = ( ((AverageAdcVoltage)/20) * (1.6388*conversion[CellN])  );
         CellN =(CellN + 1);
         
         
    }
  
     
}
}

void writeData(void) {

Serial.print("\n");
Serial.print ("Chip 0x08 ");
 Serial.print ("RED: ");
 Serial.print(CellVoltages[0]);
 Serial.print( ',' ); 
 delay (chill_time);
 Serial.print ("PINK: ");
 Serial.print(CellVoltages[1]);
 Serial.print( ',' ); 
 delay (chill_time);
////Taken out broken channels light blue-light light green 
 Serial.print ("BLUE: ");
 Serial.print(CellVoltages[4]);
 Serial.print( ',' ); 
 delay (chill_time);
 Serial.print ("GREEN: ");
 Serial.print(CellVoltages[5]);
 Serial.print( ',' ); 
 delay (chill_time);
 Serial.print ("LIGHT ORANGE: ");
 Serial.print(CellVoltages[6]);
 Serial.print( ',' ); 
 delay (chill_time);
  Serial.print ("ORANGE: ");
 Serial.print(CellVoltages[7]);
 delay (chill_time);
 Serial.print( ',' ); 


 Serial.print("\n");
 Serial.print ("Chip 0x09 ");
 Serial.print ("RED: ");
 Serial.print(CellVoltages[8]);
 Serial.print( ',' ); 
 delay (chill_time);
 Serial.print ("PINK: ");
 Serial.print(CellVoltages[9]);
 Serial.print( ',' ); 
 delay (chill_time);

////Taken out broken channels light blue-light orange 

Serial.print ("ORANGE: ");
Serial.print(CellVoltages[15]);
delay (chill_time);
Serial.print( ',' ); 


Serial.print("\n");
Serial.print ("Chip 0x0A ");
Serial.print ("RED: ");
Serial.print(CellVoltages[16]);
Serial.print( ',' ); 
delay (chill_time);
Serial.print ("PINK: ");
Serial.print(CellVoltages[17]);
Serial.print( ',' ); 
delay (chill_time);

////Taken out broken channels light blue-light light green 

 Serial.print( ',' ); 
 Serial.print ("BLUE: ");
 Serial.print(CellVoltages[20]);
 Serial.print( ',' ); 
 delay (chill_time);
 Serial.print ("GREEN: ");
 Serial.print(CellVoltages[21]);
 Serial.print( ',' ); 
 delay (chill_time);
 Serial.print ("LIGHT ORANGE: ");
 Serial.print(CellVoltages[22]);
 Serial.print( ',' ); 
 delay (chill_time);
 Serial.print ("ORANGE: ");
 Serial.print(CellVoltages[23]);
 delay (chill_time);
 Serial.print( ',' ); 


 Serial.print("\n");
Serial.print ("Chip 0x0B ");
 Serial.print ("RED: ");
 delay (chill_time);
 Serial.print(CellVoltages[24]);
 Serial.print( ',' ); 
 delay (chill_time);
 Serial.print ("PINK: ");
 Serial.print(CellVoltages[25]);
 Serial.print( ',' ); 
 delay (chill_time);
  Serial.print ("LIGHT BLUE: ");
 Serial.print(CellVoltages[26]);
 Serial.print( ',' ); 
 delay (chill_time);
 Serial.print ("LIGHT GREEN: ");
 Serial.print(CellVoltages[27]);
 delay (chill_time);
 Serial.print( ',' ); 
 Serial.print ("BLUE: ");
 Serial.print(CellVoltages[28]);
 Serial.print( ',' ); 
 delay (chill_time);
 Serial.print ("GREEN: ");
 Serial.print(CellVoltages[29]);
 Serial.print( ',' ); 
  delay (chill_time);
 Serial.print ("LIGHT ORANGE: ");
 Serial.print(CellVoltages[30]);
 Serial.print( ',' ); 
  delay (chill_time);
  Serial.print ("ORANGE: ");
 Serial.print(CellVoltages[31]);
  delay (chill_time);
 Serial.print( ',' ); 

 Serial.print("\n");
 Serial.print ("Chip 0x18 ");
 Serial.print ("RED: ");
 Serial.print(CellVoltages[32]);
 Serial.print( ',' ); 
 delay (chill_time);
 Serial.print ("PINK: ");
 Serial.print(CellVoltages[33]);
 Serial.print( ',' ); 
 delay (chill_time);
  Serial.print ("LIGHT BLUE: ");
 Serial.print(CellVoltages[34]);
 Serial.print( ',' ); 
  delay (chill_time);
 Serial.print ("LIGHT GREEN: ");
 Serial.print(CellVoltages[35]);
 delay (chill_time);
 Serial.print( ',' ); 
 
// Taken out broken channels blue- green
 
 Serial.print ("LIGHT ORANGE: ");
 Serial.print(CellVoltages[38]);
 Serial.print( ',' ); 
 delay (chill_time);
  Serial.print ("ORANGE: ");
 Serial.print(CellVoltages[39]);
  delay (chill_time);
 Serial.print( ',' ); 


 Serial.print("\n");
 Serial.print ("Chip 0x19 ");
 Serial.print ("RED : ");
 Serial.print(CellVoltages[40]);
 Serial.print( ',' ); 
  delay (chill_time);
 Serial.print ("PINK: ");
 Serial.print(CellVoltages[41]);
 Serial.print( ',' ); 
 delay (chill_time);
  Serial.print ("LIGHT BLUE: ");
 Serial.print(CellVoltages[42]);
 Serial.print( ',' ); 
 delay (chill_time);
 Serial.print ("LIGHT GREEN: ");
 Serial.print(CellVoltages[43]);
  delay (chill_time);
 Serial.print( ',' ); 
 Serial.print ("BLUE: ");
 Serial.print(CellVoltages[44]);
 Serial.print( ',' ); 
 delay (chill_time);
 Serial.print ("GREEN: ");
 Serial.print(CellVoltages[45]);
 Serial.print( ',' ); 
 delay (chill_time);



  
}

void introduction(void) {
  Serial.print(".");
 delay (1500);
 Serial.print("..");
 delay (1500);
 Serial.print("..");
 delay (1000);
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
 delay (100);
 Serial.print(".");
 delay (400);
  Serial.print(".");
 delay (1500);
  Serial.print("\n");
  
 Serial.print("Welcome!  Eco Car CVM board here , ready to calc. cell voltages B^)!"); 
delay (1500);


}
 




 
