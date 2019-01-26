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
    #include <Adafruit_GPS.h>
    
    //*********************************************************************************************
    #define SERIAL_BAUD   9600
    #define GPS_BAUD      9600
    
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
    // Connect to the GPS on the hardware port
    Adafruit_GPS GPS(&GPSSerial);
    
    volatile bool state = false;

    uint32_t timer = millis();
    String msg;
     
    void setup() {
      attachInterrupt(digitalPinToInterrupt(GPS_PPS), PPS_ISR, RISING);
      
      pinMode(GREEN_LED, OUTPUT);
      pinMode(RED_LED, OUTPUT);
      
      pinMode(GPS_RESET,OUTPUT);
      digitalWrite(GPS_RESET, HIGH); //Active HIGH, Pull LOW to reset GPS
      pinMode(GPS_FIX,  INPUT);
      pinMode(GPS_EN,   OUTPUT);
      digitalWrite(GPS_EN, LOW); //Active LOW, Pull HIGH to Disable GPS
      pinMode(GPS_PPS,  INPUT);
      
      while (!Serial); // wait until serial console is open, remove if not tethered to computer
      Serial.begin(SERIAL_BAUD);
     
      Serial.println("GPS Test");
      
      Serial.println("Enabling GPS");
      //digitalWrite(GPS_EN, HIGH); //Active Low, Disable GPS

      //Serial1.begin(SERIAL_BAUD);
      GPS.begin(GPS_BAUD);
      // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
      GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
      // uncomment this line to turn on only the "minimum recommended" data
      //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
      // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
      // the parser doesn't care about other sentences at this time
      // Set the update rate
      GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
      // For the parsing code to work nicely and have time to sort thru the data, and
      // print it out we don't suggest using anything higher than 1 Hz
         
      // Request updates on antenna status, comment out to keep quiet
      //GPS.sendCommand(PGCMD_ANTENNA);
    
      delay(1000);
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
      //if (millis() - timer > 1000) {
      //  timer = millis(); // reset the timer
      if (state) {
        Serial.print("\nTime: ");
        Serial.print(GPS.hour, DEC); Serial.print(':');
        Serial.print(GPS.minute, DEC); Serial.print(':');
        Serial.print(GPS.seconds, DEC); Serial.print('.');
        Serial.println(GPS.milliseconds);
        Serial.print("Date: ");
        Serial.print(GPS.day, DEC); Serial.print('/');
        Serial.print(GPS.month, DEC); Serial.print("/20");
        Serial.println(GPS.year, DEC);
        Serial.print("Fix: "); Serial.print((int)GPS.fix);
        Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
        if (GPS.fix) {
          Serial.print("Location: ");
          Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
          Serial.print(", ");
          Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
          Serial.print("Speed (knots): "); Serial.println(GPS.speed);
          Serial.print("Angle: "); Serial.println(GPS.angle);
          Serial.print("Altitude: "); Serial.println(GPS.altitude);
          Serial.print("Satellites: "); Serial.println((int)GPS.satellites);

          msg = "$," + format_date(String(GPS.year),String(GPS.month),String(GPS.day)) +",";
          msg += format_time(String(GPS.hour), String(GPS.minute),String(GPS.seconds), String(GPS.milliseconds)) + ",";
          //msg += String(GPS.latitude,4) + String(GPS.lat) + "," + String(GPS.longitude,4) + String(GPS.lon) + ",";
          msg += decimal_lat_str(String(GPS.latitude,4), GPS.lat) + ",";
          msg += decimal_lon_str(String(GPS.longitude,4), GPS.lon) + ",";
          msg += alt_ft_str(GPS.altitude) + "," + speed_mph_str(GPS.speed) + "," + String(GPS.angle) + ",";
          //Serial.println(decimal_lat_str(String(GPS.latitude,4), GPS.lat));
          //Serial.println(decimal_lon_str(String(GPS.longitude,4), GPS.lon));
          Serial.println(msg);
          Serial.println(msg.length());
        }
      }
      if (state) {
        Blink(RED_LED,50,2);
        state = false;
      }
    }

    String speed_mph_str(float spd){
      //spd is in knots
      spd = spd * 1.150779; //convert knots to mph
      //Serial.println(alt,4);
      return String(spd,1);
    }

    String alt_ft_str(float alt){
      //alt is in meters
      alt = alt * 3.28084; //convert meters to feet
      //Serial.println(alt,4);
      return String(alt,1);
    }

    
    String decimal_lon_str(String lon, char hemi){
      //Serial.println(lon);
      int idx = lon.indexOf(".");
      float deg = (lon.substring(0,idx-2)).toFloat();
      float dec = (lon.substring(idx-2)).toFloat()/60;
      //Serial.println(deg);
      //Serial.println(dec, 6);
      float lon_f = deg+dec;
      //Serial.println(hemi);
      if (hemi == 'W'){
        lon_f = lon_f*-1;
      }
      //Serial.println(lat_f,6);
      return String(lon_f,6);
    }

    String decimal_lat_str(String lat, char hemi){
      //Serial.println(lat);
      float deg = (lat.substring(0,2)).toFloat();
      float dec = (lat.substring(2)).toFloat()/60;
      //Serial.println(deg);
      //Serial.println(dec, 6);
      float lat_f = deg+dec;
      //Serial.println(hemi);
      if (hemi == 'S'){
        lat_f = lat_f*-1;
      }
      //Serial.println(lat_f,6);
      return String(lat_f,6);
    }
    
    String format_date(String yr, String mo, String day){
      if (mo.length() == 1){
        mo = "0" + mo;
      }
      if (day.length() == 1){
        day = "0" + day;
      }
      String date = yr + mo + day;
      return date;
    }

    String format_time(String hr, String mi, String sec, String ms){
      if (hr.length() == 1){
        hr = "0" + hr;
      }
      if (mi.length() == 1){
        mi = "0" + mi;
      }
      if (sec.length() == 1){
        sec = "0" + sec;
      }
      String time_stamp = hr+mi+sec+"."+ms;
      return time_stamp;
    }
    
      

    void PPS_ISR(){
      state= true;
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
