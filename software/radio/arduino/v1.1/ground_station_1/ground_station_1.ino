    /* RFM69 library and code by Felix Rusu - felix@lowpowerlab.com
    // Get libraries at: https://github.com/LowPowerLab/
    // Make sure you adjust the settings in the configuration section below !!!
    // **********************************************************************************
    // Copyright Felix Rusu, LowPowerLab.com
    // Library and code by Felix Rusu - felix@lowpowerlab.com
    // **********************************************************************************
    // License
    // **********************************************************************************
    // This program is free software; you can redistribute it 
    // and/or modify it under the terms of the GNU General    
    // Public License as published by the Free Software       
    // Foundation; either version 3 of the License, or        
    // (at your option) any later version.                    
    //                                                        
    // This program is distributed in the hope that it will   
    // be useful, but WITHOUT ANY WARRANTY; without even the  
    // implied warranty of MERCHANTABILITY or FITNESS FOR A   
    // PARTICULAR PURPOSE. See the GNU General Public        
    // License for more details.                              
    //                                                        
    // You should have received a copy of the GNU General    
    // Public License along with this program.
    // If not, see <http://www.gnu.org/licenses></http:>.
    //                                                        
    // Licence can be viewed at                               
    // http://www.gnu.org/licenses/gpl-3.0.txt
    //
    // Please maintain this license information along with authorship
    // and copyright notices in any redistribution of this code
    // **********************************************************************************/
     
    #include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
    #include <SPI.h>
    #include <SD.h>
    #include <Adafruit_GPS.h>
     
    //*********************************************************************************************
    // *********** IMPORTANT SETTINGS - YOU MUST CHANGE/ONFIGURE TO FIT YOUR HARDWARE *************
    //*********************************************************************************************
    #define NETWORKID     100  // The same on all nodes that talk to each other
    #define NODEID        5    // The unique identifier of this node
    #define RECEIVER      255  // The recipient of packets
     
    //Match frequency to the hardware version of the radio on your Feather
    #define FREQUENCY     RF69_433MHZ
    //#define FREQUENCY     RF69_868MHZ
    //#define FREQUENCY     RF69_915MHZ
    #define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
    #define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module
    //double rf_freq = 433000000;
    //double rf_freq = 432998200; //Node 1 Freq
    //double rf_freq = 432997300; //Node 2 Freq
    //double rf_freq = 432997500; //Node 3 Freq
    //double rf_freq = 433000000; //Node 4 Freq
    //double rf_freq = 432998300; //Node 5 Freq
    double rf_freq = 433000000; //Node 6 Freq
     
    //*********************************************************************************************
    #define SERIAL_BAUD   9600
     
    // Feather 32u4 w/wing
    #define RFM69_RST     A4   // RFM69 Reset pin
    #define RFM69_CS      12   // "E" RFM69 Chip Select Pin
    #define RFM69_IRQ     2    // "SDA" (only SDA/SCL/RX/TX have IRQ!) RFM69 Interrupt 0
    #define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ)
    #define RFM69_DIO1    11  // RFM69 Interrupt 1
    #define RFM69_DIO2    10  // RFM69 Interrupt 2
    #define RFM69_DIO3    6   // RFM69 Interrupt 3
    #define RFM69_DIO4    5   // RFM69 Interrupt 4
    #define RFM69_DIO5    A5  // RFM69 Interrupt 5

    #define RED_LED       13  // onboard blinky
    #define BATT_MON      9   // Battery Monitor, analog input
    
    #define GPS_RESET     A0  // GPS Reset Pin
    #define GPS_FIX       A1  // GPS Fix Pin
    #define GPS_EN        A2  // GPS Enable Pin ACTIVE LOW
    #define GPS_TX        0  // GPS TX Pin, Serial RX Pin 
    #define GPS_RX        1  // GPS RX Pin, Serial TX Pin
    #define GPS_PPS       3  // "SCL" HW Interrupt pin

    #define SD_CS         4   // uSD Card 'Chip Select' Pin
    #define SD_CD         7   // uSD Card 'Card Detect' Pin
    #define GREEN_LED     8   // uSD Card blinky
     
     
    int16_t packetnum = 0;  // packet counter, we increment per xmission
    String filename = "GS1.txt";
    
    RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);
     
    void setup() {
      while (!Serial); // wait until serial console is open, remove if not tethered to computer
      Serial.begin(SERIAL_BAUD);
      Serial.println("Feather RFM69HCW Receiver");
      //Configure Pin Modes
      pinMode(GREEN_LED, OUTPUT);
      pinMode(RED_LED, OUTPUT);
      
      pinMode(GPS_RESET,OUTPUT);
      digitalWrite(GPS_RESET, HIGH); //Active HIGH, Pull LOW to reset GPS
      pinMode(GPS_FIX,  INPUT);
      pinMode(GPS_EN,   OUTPUT);
      digitalWrite(GPS_EN, HIGH); //Active LOW, Pull HIGH to Disable GPS
      pinMode(GPS_PPS,  INPUT);

      // Hard Reset the RFM module
      pinMode(RFM69_RST, OUTPUT);
      digitalWrite(RFM69_RST, HIGH);
      delay(100);
      digitalWrite(RFM69_RST, LOW);
      delay(100);
     
      // Initialize radio
      radio.initialize(FREQUENCY,NODEID,NETWORKID);
      if (IS_RFM69HCW) {
        radio.setHighPower();    // Only for RFM69HCW & HW!
      }
      radio.setPowerLevel(0); // power output ranges from 0 (5dBm) to 31 (20dBm)
      
      //radio.encrypt(ENCRYPTKEY);
      radio.setFrequency(rf_freq);
      
      Serial.println("Receiving in filtered mode ");
      //radio.promiscuous(true);
      
      pinMode(GREEN_LED, OUTPUT);
      pinMode(RED_LED, OUTPUT);
      Serial.print("\Receiving at ");
      Serial.print(FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
      Serial.println(" MHz");

      // see if the card is present and can be initialized:
      if (!SD.begin(SD_CS)) {
        Serial.println("Card failed, or not present");
        // don't do anything more:
        return;
      }
      else {
        Serial.println("Card Found, Logging Enabled");
      }
      
      delay(1000);
    }
     
     
    void loop() {
      //check if something was received (could be an interrupt from the radio)
      if (radio.receiveDone()) {
        //Serial.print('[');Serial.print(radio.SENDERID);Serial.print("] ");
        //Serial.print('[');Serial.print(radio.TARGETID);Serial.println("] ");
        
        //Serial.print("[RSSI:");Serial.print(radio.RSSI);Serial.print("] ");
        //PrintHex8(radio.DATA, sizeof(radio.DATA)); Serial.println("");
        uint8_t idx = findIndex(radio.DATA, sizeof(radio.DATA));
        unsigned char rx_msg[idx+1];
        memset(rx_msg, 0, sizeof(rx_msg)); 
        //Serial.println(sizeof(rx_msg));
        memcpy(rx_msg, &radio.DATA, idx+1);
        int8_t rssi = (int8_t)radio.RSSI;
        memset(radio.DATA, 0, sizeof(radio.DATA));//flush field?
        //DATA Reconstructed from Transmission Packet
        /* *****DON'T DELETE, YOU NEED THIS IN THE RECEIVER*******
          float lat2 = 0;
          memcpy(&lat2, msg+8, 4);
          */
        uint8_t SRC = 0; //Node ID of transmitting packet
        uint8_t DST = 0; //Intended recipient of PAcket
        uint16_t packetnum = 0;
        uint8_t hour          = 0;
        uint8_t minute        = 0;
        uint8_t second        = 0;
        uint16_t millisecond  = 0;
        float lat   = 0; //
        float lon   = 0;
        float alt   = 0;
        float spd   = 0;
        float crs   = 0;
        uint16_t batt_adc = 0;

        

        memcpy(&SRC, rx_msg, 1);
        memcpy(&DST, rx_msg+1, 1);
        memcpy(&packetnum, rx_msg+2, 2);
        memcpy(&hour, rx_msg+4, 1);
        memcpy(&minute, rx_msg+5, 1);
        memcpy(&second, rx_msg+6, 1);
        memcpy(&millisecond, rx_msg+7, 2);
        memcpy(&lat, rx_msg+9, 4);
        memcpy(&lon, rx_msg+13, 4);
        memcpy(&alt, rx_msg+17, 4);
        memcpy(&spd, rx_msg+21, 4);
        memcpy(&crs, rx_msg+25, 4);
        memcpy(&batt_adc, rx_msg+29, 2);

        float batt_volt = (float)batt_adc;
        batt_volt *= 2;
        batt_volt *= 3.3;
        batt_volt /= 1024;

        String msg = "$,";
        msg += String(SRC) +",";
        msg += String(DST) +",";
        msg += String(packetnum) +",";
        msg += String(hour) +",";
        msg += String(minute) +",";
        msg += String(second) +",";
        msg += String(millisecond) +",";
        msg += String(lat, 7) +",";
        msg += String(lon, 7) +",";
        msg += String(alt, 2) +",";
        msg += String(spd, 1) +",";
        msg += String(crs, 1) +",";
        msg += String(batt_volt, 2) +",";
        msg += String(rssi) +",";

        //Serial.println(sizeof(rx_msg));
        if (sizeof(rx_msg) >= 35){ //at least single hopped packet
          uint8_t hop1_id   = 0;
          int8_t hop1_rssi  = 0;
          memcpy(&hop1_id, rx_msg+32, 1);
          memcpy(&hop1_rssi, rx_msg+33, 1);
          msg += String(hop1_id) +",";
          msg += String(hop1_rssi) +",";
        }
        if (sizeof(rx_msg) == 38){ //double hopped packet
          uint8_t hop2_id   = 0;
          int8_t hop2_rssi  = 0;
          memcpy(&hop2_id, rx_msg+35, 1);
          memcpy(&hop2_rssi, rx_msg+36, 1);
          msg += String(hop2_id) +",";
          msg += String(hop2_rssi) +",";
        }
        Serial.println(msg);
        Blink(RED_LED, 50, 1); //blink LED 3 times, 40ms between blinks
        
        //Log Message
        File dataFile = SD.open(filename, FILE_WRITE);
        if (dataFile){
          dataFile.println(msg);
          dataFile.close();
          Blink(GREEN_LED,50,1);
        }
        
       
      }
      Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU
    }

    uint8_t findIndex(uint8_t *data, uint8_t length){
      uint8_t idx = 0;
      for (int i=4; i<length; i++) { 
        if (data[i]==0x7E) {
          idx = i;
          //Serial.println(idx);
        } 
      }
      return idx;
    }

    void PrintHex8(uint8_t *data, uint8_t length){ // prints 8-bit data in hex with leading zeroes
    //Serial.print("0x"); 
      for (int i=0; i<length; i++) { 
        if (data[i]<0x10) {Serial.print("0");} 
        Serial.print(data[i],HEX); 
        //Serial.print(" "); 
      }
    }

    void Blink(byte PIN, byte DELAY_MS, byte loops)
    {
      for (byte i=0; i<loops; i++)
      {
        digitalWrite(PIN,HIGH);
        delay(DELAY_MS);
        digitalWrite(PIN,LOW);
        delay(DELAY_MS);
      }
    }
