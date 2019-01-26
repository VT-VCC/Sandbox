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
    #define NODEID        4    // The unique identifier of this node
    #define RECEIVER      5    // The recipient of packets
    String CALLSIGN =     "D2";
     
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
    double rf_freq = 432999750; //Node 4 Freq
    //double rf_freq = 432998300; //Node 5 Freq
    //double rf_freq = 433000000; //Node 6 Freq
     
    //*********************************************************************************************
    #define SERIAL_BAUD   9600
    #define GPS_BAUD      9600
    #define TX_RATE       5000 //number of milliseconds between TX window
    #define TX_OFFSET     2500  //Absolute Value of random TX Rate offset
    #define DIGI_DELAY    100  //number of milliseconds between TX window
    
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
     
    #define GPSSerial Serial1

    // Connect to the GPS on the hardware port
    Adafruit_GPS GPS(&GPSSerial);
    
    int16_t packetnum = 0;  // packet counter, we increment per xmission
     
    RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

    volatile uint32_t timer = millis();
    long random_delay = random(-TX_OFFSET, TX_OFFSET);

    bool STATE = false;
    String filename = "DIGI2.txt";

    //Message Data for digipeated message
    uint8_t SRC  = NODEID; //Node ID of digipeater
    uint8_t DST  = RECEIVER; //Intended recipient of Packet
    uint8_t FLAG = 126; //HEX: 0x7E, BIN: 0111110, HDLC Flag used to mark end of payload.
    
    void setup() {
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
      Serial.println("RFM69 Digipeater Version 1");

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
      
      //radio.encrypt(ENCRYPTKEY);
      radio.setFrequency(rf_freq);

      //radio.promiscuous(true);
      
      pinMode(GREEN_LED, OUTPUT);
      pinMode(RED_LED, OUTPUT);
      Serial.print("\Receiving at ");
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

      delay(250);
      // see if the card is present and can be initialized:
      Serial.println("Checking SD Card Status...");
      if (!SD.begin(SD_CS)) {
        Serial.println("Card failed, or not present");
        // don't do anything more:
        return;
      }
      else {
        Serial.println("Card Found, Logging Enabled");
      }
      delay(100);
      Serial.println("Logging to: " + filename);
      delay(50);
      File dataFile = SD.open(filename, FILE_WRITE);
      if (dataFile){
        unsigned char hdr[5];
        memset(hdr, 0x7E, sizeof(hdr));
        dataFile.write(hdr, sizeof(hdr));
        dataFile.close();
        Serial.println("logged header flags to file...");
      }
      Serial.println("Waiting for GPS Fix...");
      Serial.println("Begin Digipeating...");
      
      unsigned char hdr[5];
      memset(hdr, 0x7E, sizeof(hdr));
      Serial.write(hdr, sizeof(hdr));
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
      
      //check if something was received (could be an interrupt from the radio)
      if (radio.receiveDone()) {//Packet is received by radio, time to digipeat
        Blink(GREEN_LED, 100, 1); //RX LED indication, blink GREEN LED 1 time, 100ms between blinks
        
        //Extract Packet from Radio
        uint8_t idx = findIndex(radio.DATA, sizeof(radio.DATA));
        unsigned char rx_msg[idx+1];
        memset(rx_msg, 0, sizeof(rx_msg)); 
        memcpy(rx_msg, &radio.DATA, idx+1);
        int8_t rssi = (int8_t)radio.RSSI; //Get RSSI of received packet
        memset(radio.DATA, 0, sizeof(radio.DATA));//flush field?

        //Create digi packet
        unsigned char tx_msg[idx + 1 + 1 + 1 + 1];
        memset(tx_msg, 0, sizeof(rx_msg)); 
        memcpy(tx_msg, rx_msg, sizeof(rx_msg));
        //Serial.println(sizeof(rx_msg));
        if (sizeof(rx_msg) == 32){ //single hop
          memcpy(tx_msg+32, &SRC, 1);
          memcpy(tx_msg+33, &rssi, 1);
          memcpy(tx_msg+34, &FLAG, 1);
        }
        else if (sizeof(rx_msg) == 35){//double hop
          memcpy(tx_msg+35, &SRC, 1);
          memcpy(tx_msg+36, &rssi, 1);
          memcpy(tx_msg+37, &FLAG, 1);
        }
        
        delay(DIGI_DELAY);
        radio.send(RECEIVER, tx_msg, sizeof(tx_msg));
        Blink(RED_LED, 100, 1); //TX LED indication, blink RED LED 2 times, 50ms between blinks

        //Log Message, Binary Format
        File dataFile = SD.open(filename, FILE_WRITE);
        if (dataFile){
          dataFile.write(tx_msg, sizeof(tx_msg));
          dataFile.close();
          Blink(GREEN_LED,50,1);
        }

        //Write message Binary to Serial Port
        Serial.write(tx_msg, sizeof(tx_msg));
        
      }
      
      if (millis() - timer > (TX_RATE + random_delay)) {
        Send_Position_Packet();
      }
      Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU
    }

    void Send_Position_Packet(){
      uint16_t batt_adc = analogRead(BATT_MON);
      
      if (GPS.fix) {
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

        unsigned char msg[32];
        memset(msg, 0, sizeof(msg));
        
        memcpy(msg   , &SRC, 1);
        memcpy(msg+1 , &DST, 1);
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

        //Write message Binary to Serial Port
        Serial.write(msg, sizeof(msg));
        
      } 
      random_delay = random(-TX_OFFSET,TX_OFFSET); //reset random tx rate
      timer = millis(); // reset the timer
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

