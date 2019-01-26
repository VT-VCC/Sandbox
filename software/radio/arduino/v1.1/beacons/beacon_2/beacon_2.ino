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
    //#define NODEID        1    // The unique identifier of this node
    //#define RECEIVER      3    // The recipient of packets
    uint8_t NODEID   =      2;    // The unique identifier of this node
    uint8_t RECEIVER =      3;    // The recipient of packets
     
    //Match frequency to the hardware version of the radio on your Feather
    #define FREQUENCY     RF69_433MHZ
    #define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module
    //double rf_freq = 433000000;
    //double rf_freq = 432998200; //Node 1 Freq
    double rf_freq = 432997300; //Node 2 Freq
    //double rf_freq = 432997500; //Node 3 Freq
    //double rf_freq = 433000000; //Node 4 Freq
    //double rf_freq = 432998300; //Node 5 Freq
    //double rf_freq = 433000000; //Node 6 Freq
    //*********************************************************************************************
    
    #define SERIAL_BAUD   9600
    #define GPS_BAUD      9600
    #define TX_RATE       5000 //number of milliseconds between TX window
    #define TX_OFFSET     2500  //Absolute Value of random TX Rate offset
    String CALLSIGN =     "B2";
    
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
    
    #define GPS_RESET     A0  // GPS Reset Pin. ACTIVE LOW
    #define GPS_FIX       A1  // GPS Fix Pin
    #define GPS_EN        A2  // GPS Enable Pin ACTIVE LOW
    #define GPS_TX        0  // GPS TX Pin, Serial RX Pin 
    #define GPS_RX        1  // GPS RX Pin, Serial TX Pin
    #define GPS_PPS       3  // "SCL" HW Interrupt pin

    #define SD_CS         4   // uSD Card 'Chip Select' Pin
    #define SD_CD         7   // uSD Card 'Card Detect' Pin
    #define GREEN_LED     8   // uSD Card blinky
    
    #define GPSSerial Serial1
    
    //*********************************************************************************************
    uint16_t packetnum = 0;  // packet counter, we increment per xmission
    RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);
    
    // Connect to the GPS on the hardware port
    Adafruit_GPS GPS(&GPSSerial);
    
    volatile uint32_t timer = millis();

    //String msg;
    bool STATE = false;
    long random_delay = random(-TX_OFFSET, TX_OFFSET);
    String filename ="";
    
    void setup() {
      //configure Interrupts
      //attachInterrupt(digitalPinToInterrupt(GPS_FIX), GPS_FIX_ISR, RISING);

      //Configure Pin Modes
      pinMode(GREEN_LED, OUTPUT);
      pinMode(RED_LED, OUTPUT);
      
      pinMode(GPS_RESET,OUTPUT);
      digitalWrite(GPS_RESET, HIGH); //Active HIGH, Pull LOW to reset GPS
      pinMode(GPS_FIX,  INPUT);
      pinMode(GPS_EN,   OUTPUT);
      digitalWrite(GPS_EN, LOW); //Active LOW, Pull HIGH to Disable GPS
      pinMode(GPS_PPS,  INPUT);

      
      //while (!Serial); // wait until serial console is open, remove if not tethered to computer
      Serial.begin(SERIAL_BAUD);
      Serial.println("GPS Beacon");

      //*****Setup Radio*****
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
      radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
      radio.setFrequency(rf_freq);
      Serial.print("\nTransmitting at ");
      Serial.print(FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
      Serial.println(" MHz");
      
      //*****Setup GPS*****
      GPS.begin(GPS_BAUD);
      // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
      GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
      // Set the update rate
      GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
      // For the parsing code to work nicely and have time to sort thru the data, and
      // print it out we don't suggest using anything higher than 1 Hz

      // see if the card is present and can be initialized:
      if (!SD.begin(SD_CS)) {
        Serial.println("Card failed, or not present");
        // don't do anything more:
        return;
      }
      else {
        Serial.println("Card Found, Logging Enabled");
      }
      Serial.println("Waiting for GPS Fix...");
      delay(500);
    }
    
     
    void loop() {
      if (Serial1.available()) {
        char c = GPS.read();
        //Serial.write(c);
      }
      if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
        //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
          return; // we can fail to parse a sentence in which case we should just wait for another
      }
      if (STATE == false){ //Waiting for initial GPS LOCK
        if (GPS.fix){
          Serial.println("GPS Fix Obtained");
          //Generate Logging filename based on data
          filename = CALLSIGN + String(GPS.year);
          //filename = "20" + String(GPS.year);
          if (GPS.month < 10){filename += "0";}
          filename += String(GPS.month);
          if (GPS.day < 10){filename += "0";}
          filename += String(GPS.day) +".txt";
          Serial.println("Logging to: " + filename);
    
          File dataFile = SD.open(filename, FILE_WRITE);
          if (dataFile){
            unsigned char hdr[5];
            memset(hdr, 0x7E, sizeof(hdr));
            dataFile.write(hdr, sizeof(hdr));
            dataFile.close();
            Serial.println("logged header flags to file...");
          }
          Blink(GREEN_LED,50,6);
          STATE = true;
        }
      }
      else {
        if (millis() - timer > (TX_RATE + random_delay)) {
          if (GPS.fix) {
            uint8_t FLAG = 126; //HEX: 0x7E, BIN: 0111110, HDLC Flag used to mark end of payload.
            
            packetnum++;
            
            uint8_t hour          = GPS.hour;
            uint8_t minute        = GPS.minute;
            uint8_t second        = GPS.seconds;
            uint16_t millisecond  = GPS.milliseconds;
            
            float lat = GPS.latitudeDegrees;//units of degrees
            float lon = GPS.longitudeDegrees;//units of degrees
            float alt = GPS.altitude * 3.28084; //convert meters to feet
            float spd = GPS.speed * 1.150779; //convert knots to mph
            float crs = GPS.angle; //units of degrees
  
            uint16_t batt_adc = analogRead(BATT_MON);
  
            unsigned char msg[32];
            memset(msg, 0, sizeof(msg));
            
            memcpy(msg   , &NODEID, 1);
            memcpy(msg+1 , &RECEIVER, 1);
            memcpy(msg+2 , &packetnum, 2);
            memcpy(msg+4 , &hour, 1);
            memcpy(msg+5 , &minute, 1);
            memcpy(msg+6 , &second, 1);
            memcpy(msg+7 , &millisecond, 2);
            memcpy(msg+9 , &lat, 4);
            memcpy(msg+13, &lon, 4);
            memcpy(msg+17, &alt, 4);
            memcpy(msg+21, &spd, 4);
            memcpy(msg+25, &crs, 4);
            memcpy(msg+29, &batt_adc, 2);
            memcpy(msg+31, &FLAG, 1);
  
            radio.send(RECEIVER, msg, sizeof(msg));
            Blink(RED_LED,50,2);
  
            //Log Message, Binary Format
            File dataFile = SD.open(filename, FILE_WRITE);
            if (dataFile){
              dataFile.write(msg, sizeof(msg));
              dataFile.close();
              Blink(GREEN_LED,50,1);
            }

            //Write message Hexstring to Serial Port
            //PrintHex8(msg, sizeof(msg));

            //Write message Binary to Serial Port
            Serial.write(msg, sizeof(msg));
            //WriteBinary(msg, sizeof(msg));
          }
          random_delay = random(-TX_OFFSET,TX_OFFSET); //reset random tx rate
          timer = millis(); // reset the timer
        }
      }
    }

    void PrintHex8(uint8_t *data, uint8_t length){ // prints 8-bit data in hex with leading zeroes
    //Serial.print("0x"); 
      for (int i=0; i<length; i++) { 
        if (data[i]<0x10) {Serial.print("0");} 
        Serial.print(data[i],HEX); 
        //Serial.print(" "); 
      }
      Serial.print("\n");
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
